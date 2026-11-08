"""Command a tilted C1 through a list of angles and keep one scan at each.

With continuous true, the horn travels to the end of the list and back until the node is stopped. Each angle keeps only its latest scan, so the cloud stays one map of the room.
"""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
import serial
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from tf2_ros import TransformBroadcaster

from lidar_tilt.tilt_math import scan_to_points

DEFAULT_ANGLES = [-30.0, -25.0, -20.0, -15.0, -10.0, -5.0, 0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0]


def _default_ply() -> str:
    here = Path(__file__).resolve()
    source_maps = here.parents[1] / "maps"
    if source_maps.is_dir():
        return str(source_maps / "room_cloud.ply")
    share = Path(get_package_share_directory("lidar_tilt")) / "maps" / "room_cloud.ply"
    return str(share)


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _xyz_cloud(points: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    xyz = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    message = PointCloud2()
    message.header.frame_id = frame_id
    message.header.stamp = stamp
    message.height = 1
    message.width = int(xyz.shape[0])
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = 12 * message.width
    message.is_dense = True
    message.data = xyz.tobytes()
    return message


def write_ply(path: Path, points: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    xyz = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    with path.open("w", encoding="ascii") as handle:
        handle.write("ply\nformat ascii 1.0\n")
        handle.write(f"element vertex {xyz.shape[0]}\n")
        handle.write("property float x\nproperty float y\nproperty float z\nend_header\n")
        for x_m, y_m, z_m in xyz:
            handle.write(f"{x_m:.5f} {y_m:.5f} {z_m:.5f}\n")


class Esp32Tilt:
    """Line protocol: send GOTO, wait for AT. The board dwells before it replies."""

    def __init__(self, port: str, baud: int):
        self._ser = serial.Serial(port, baud, timeout=0.2)
        self._buf = bytearray()

    def close(self) -> None:
        self._ser.close()

    def lines_until(self, deadline: float):
        while time.monotonic() < deadline:
            waiting = self._ser.in_waiting
            chunk = self._ser.read(waiting or 1)
            if chunk:
                self._buf.extend(chunk)
            while b"\n" in self._buf:
                raw, _, self._buf = self._buf.partition(b"\n")
                text = raw.decode("ascii", errors="replace").strip()
                if text:
                    yield text

    def wait_ready(self, timeout_s: float = 3.0) -> bool:
        deadline = time.monotonic() + timeout_s
        for line in self.lines_until(deadline):
            if line == "READY":
                return True
        return False

    def goto(self, degrees: float, timeout_s: float = 15.0) -> float:
        self._ser.write(f"GOTO {degrees:.1f}\n".encode("ascii"))
        self._ser.flush()
        deadline = time.monotonic() + timeout_s
        for line in self.lines_until(deadline):
            if line.startswith("ERR"):
                raise RuntimeError(line)
            parts = line.split()
            if len(parts) == 2 and parts[0] == "AT":
                acked = float(parts[1])
                if abs(acked - degrees) > 0.2:
                    raise RuntimeError(f"acked {acked} for commanded {degrees}")
                return acked
        raise TimeoutError(f"no AT for {degrees:.1f}")


class TiltSweep(Node):
    def __init__(self):
        super().__init__("tilt_sweep")
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("angles_deg", DEFAULT_ANGLES)
        self.declare_parameter("tilt_sign", 1.0)
        self.declare_parameter("range_min", 0.15)
        self.declare_parameter("range_max", 12.0)
        self.declare_parameter("dead_min_deg", 180.0)
        self.declare_parameter("dead_max_deg", -180.0)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cloud_topic", "/tilt_cloud")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("output_ply", _default_ply())
        self.declare_parameter("continuous", True)

        self._tilt_sign = float(self.get_parameter("tilt_sign").value)
        self._range_min = float(self.get_parameter("range_min").value)
        self._range_max = float(self.get_parameter("range_max").value)
        self._dead_min = float(self.get_parameter("dead_min_deg").value)
        self._dead_max = float(self.get_parameter("dead_max_deg").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._output = Path(str(self.get_parameter("output_ply").value))
        self._angles = [float(item) for item in self.get_parameter("angles_deg").value]
        continuous = self.get_parameter("continuous").value
        if isinstance(continuous, str):
            self._continuous = continuous.strip().lower() in ("1", "true", "yes")
        else:
            self._continuous = bool(continuous)

        cloud_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self._cloud_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter("cloud_topic").value), cloud_qos
        )
        self._tf = TransformBroadcaster(self)
        self._lock = threading.Lock()
        self._scan: LaserScan | None = None
        self._tilt_deg = 0.0
        self._slices: dict[float, np.ndarray] = {}
        self._points = np.zeros((0, 3), dtype=np.float64)
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._on_scan,
            10,
        )
        self.create_timer(0.05, self._publish_tf)
        self.create_timer(1.0, self._republish_cloud)
        self._worker = threading.Thread(target=self._sweep, daemon=True)
        self._worker.start()

    def _on_scan(self, message: LaserScan) -> None:
        with self._lock:
            self._scan = message

    def _publish_tf(self) -> None:
        beta = math.radians(self._tilt_deg * self._tilt_sign)
        message = TransformStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._base_frame
        message.child_frame_id = "laser"
        message.transform.rotation.y = math.sin(beta / 2.0)
        message.transform.rotation.w = math.cos(beta / 2.0)
        self._tf.sendTransform(message)

    def _republish_cloud(self) -> None:
        with self._lock:
            if self._points.shape[0] == 0:
                return
            points = self._points
        self._cloud_pub.publish(_xyz_cloud(points, self._base_frame, self.get_clock().now().to_msg()))

    def _wait_for_lidar(self, timeout_s: float = 20.0) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and rclpy.ok():
            with self._lock:
                if self._scan is not None:
                    return True
            time.sleep(0.1)
        return False

    def _scan_after(self, ack_ns: int, timeout_s: float = 3.0) -> LaserScan | None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and rclpy.ok():
            with self._lock:
                scan = self._scan
            if scan is not None and _stamp_ns(scan.header.stamp) > ack_ns:
                return scan
            time.sleep(0.02)
        return None

    def _store(self, scan: LaserScan, tilt_deg: float) -> tuple[int, int]:
        window_min = max(self._range_min, float(scan.range_min))
        window_max = min(self._range_max, float(scan.range_max)) if scan.range_max > 0 else self._range_max
        points = scan_to_points(
            scan.ranges,
            float(scan.angle_min),
            float(scan.angle_increment),
            tilt_deg,
            self._tilt_sign,
            window_min,
            window_max,
            self._dead_min,
            self._dead_max,
        )
        key = round(float(tilt_deg), 1)
        with self._lock:
            self._tilt_deg = tilt_deg
            self._slices[key] = points
            chunks = [self._slices[name] for name in sorted(self._slices) if self._slices[name].shape[0]]
            self._points = np.vstack(chunks) if chunks else np.zeros((0, 3), dtype=np.float64)
            total = int(self._points.shape[0])
            cloud = self._points
        self._cloud_pub.publish(_xyz_cloud(cloud, self._base_frame, scan.header.stamp))
        return int(points.shape[0]), total

    def _save(self) -> int:
        with self._lock:
            points = self._points.copy()
        write_ply(self._output, points)
        return int(points.shape[0])

    def _capture_angle(self, link: Esp32Tilt, degrees: float) -> bool:
        last_error: Exception | None = None
        for _attempt in range(2):
            if not rclpy.ok():
                return False
            try:
                acked = link.goto(degrees)
                ack_ns = _stamp_ns(self.get_clock().now().to_msg())
                with self._lock:
                    self._tilt_deg = acked
                scan = self._scan_after(ack_ns)
                if scan is None:
                    raise TimeoutError(f"no /scan after AT {acked:.1f}")
                fresh, total = self._store(scan, acked)
                self.get_logger().info(f"AT {acked:.1f} deg, kept {fresh} points, cloud {total}")
                return True
            except Exception as exc:
                last_error = exc
                self.get_logger().error(f"angle {degrees:.1f} failed: {exc}")
        self.get_logger().error(f"skipped {degrees:.1f} after retry: {last_error}")
        return False

    def _sweep(self) -> None:
        port = str(self.get_parameter("port").value)
        baud = int(self.get_parameter("baud").value)
        link = None
        try:
            link = Esp32Tilt(port, baud)
            if link.wait_ready():
                self.get_logger().info("ESP32 READY")
            else:
                self.get_logger().info("No READY line. The board was already up. Sending GOTO.")
            if not self._wait_for_lidar():
                raise TimeoutError("no /scan before the sweep")
            self.get_logger().info("Lidar is publishing. Starting the sweep.")
            if self._continuous:
                self.get_logger().info("Continuous sweep. Close this window to stop.")
                link = self._sweep_forever(link)
            else:
                link = self._run_sequence(link, self._angles)
                count = self._save()
                self.get_logger().info(f"SWEEP_OK {count} points -> {self._output}")
        except Exception as exc:
            self.get_logger().error(f"SWEEP_FAIL {exc}")
            with self._lock:
                points = self._points.copy()
            if points.shape[0]:
                partial = self._output.with_name(self._output.stem + "_partial.ply")
                write_ply(partial, points)
                self.get_logger().error(f"Wrote partial cloud {partial}")
        finally:
            if link is not None:
                link.close()

    def _run_sequence(self, link: Esp32Tilt, angles: list[float]) -> Esp32Tilt:
        streak = 0
        port = str(self.get_parameter("port").value)
        baud = int(self.get_parameter("baud").value)
        for degrees in angles:
            if not rclpy.ok():
                return link
            if self._capture_angle(link, degrees):
                streak = 0
                continue
            streak += 1
            if streak < 3:
                continue
            self.get_logger().error("Reopening the servo port.")
            try:
                link.close()
            except Exception:
                pass
            time.sleep(1.0)
            link = Esp32Tilt(port, baud)
            streak = 0
        return link

    def _sweep_forever(self, link: Esp32Tilt) -> Esp32Tilt:
        forward = True
        pass_index = 0
        while rclpy.ok():
            sequence = list(self._angles if forward else reversed(self._angles))
            if pass_index:
                sequence = sequence[1:]
            link = self._run_sequence(link, sequence)
            forward = not forward
            pass_index += 1
            count = self._save()
            self.get_logger().info(f"PASS {pass_index} {count} points -> {self._output}")
        return link


def main(args=None):
    rclpy.init(args=args)
    node = TiltSweep()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
