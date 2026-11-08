"""Track an upright rig in a 3D occupancy grid and, when asked, grow that grid."""

from __future__ import annotations

import math
import threading
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Int32
from tf2_ros import TransformBroadcaster

from lidar_tilt.register import (
    INLIER_FRACTION,
    INLIER_RMS_M,
    MapIndex,
    OccupancyGrid,
    Pose,
    accepted,
    apply_pose,
    global_search,
    icp,
    load_ply,
    save_ply,
    score_pose,
)


def _maps_dir() -> Path:
    source = Path(__file__).resolve().parents[1] / "maps"
    if source.is_dir():
        return source
    return Path(get_package_share_directory("lidar_tilt")) / "maps"


def _resolve_map(path_str: str) -> Path | None:
    if not path_str.strip():
        return None
    path = Path(path_str)
    if path.is_file():
        return path
    for folder in (_maps_dir(), Path(get_package_share_directory("lidar_tilt")) / "maps"):
        candidate = folder / path.name
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(path_str)


def _as_bool(value) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes")
    return bool(value)


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


def _cloud_to_xyz(message: PointCloud2) -> np.ndarray:
    if message.width * message.height == 0 or message.point_step == 0:
        return np.zeros((0, 3), dtype=np.float64)
    names = {field.name: field.offset for field in message.fields}
    raw = np.frombuffer(message.data, dtype=np.uint8).reshape(-1, message.point_step)
    points = np.empty((raw.shape[0], 3), dtype=np.float64)
    for column, name in enumerate(("x", "y", "z")):
        points[:, column] = raw[:, names[name] : names[name] + 4].copy().view(np.float32).reshape(-1)
    good = np.isfinite(points).all(axis=1)
    return points[good]


class CloudLocalizer(Node):
    def __init__(self):
        super().__init__("cloud_localize")
        self.declare_parameter("map_path", "")
        self.declare_parameter("extend_map", True)
        self.declare_parameter("voxel_m", 0.05)
        self.declare_parameter("slice_topic", "/tilt_slice")
        self.declare_parameter("pass_topic", "/tilt_pass")
        self.declare_parameter("pose_topic", "/pose_fix")
        self.declare_parameter("map_topic", "/map_cloud")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("output_dir", str(_maps_dir()))

        self._extend = _as_bool(self.get_parameter("extend_map").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._output = Path(str(self.get_parameter("output_dir").value))
        map_path = _resolve_map(str(self.get_parameter("map_path").value))
        voxel = float(self.get_parameter("voxel_m").value)
        self._grid = OccupancyGrid(voxel)
        self._seeded = False
        if map_path is not None and map_path.suffix == ".npz":
            self._grid = OccupancyGrid.load_npz(map_path)
            self._seeded = len(self._grid.occupied_centers()) > 0
            self.get_logger().info(f"Loaded grid {map_path}")
        elif map_path is not None:
            self._grid.occupy_points(load_ply(map_path))
            self._seeded = len(self._grid.occupied_centers()) > 0
            self.get_logger().info(f"Loaded cloud {map_path}")
        else:
            self.get_logger().info("Empty map. The first slice starts the grid at the origin.")

        self._index = MapIndex(self._grid.occupied_centers())
        self._pose = Pose()
        self._lost = self._seeded
        self._misses = 0
        self._lock = threading.Lock()

        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, str(self.get_parameter("pose_topic").value), pose_qos
        )
        self._map_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter("map_topic").value), pose_qos
        )
        self._tf = TransformBroadcaster(self)
        slice_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("slice_topic").value),
            self._on_slice,
            slice_qos,
        )
        self.create_subscription(Int32, str(self.get_parameter("pass_topic").value), self._on_pass, 10)
        self.create_timer(0.05, self._publish_tf)
        self.create_timer(1.0, self._publish_map)
        self._publish_map()

    def _publish_tf(self) -> None:
        with self._lock:
            pose = self._pose.copy()
        message = TransformStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._map_frame
        message.child_frame_id = self._base_frame
        message.transform.translation.x = pose.x
        message.transform.translation.y = pose.y
        message.transform.translation.z = pose.z
        message.transform.rotation.z = math.sin(pose.yaw / 2.0)
        message.transform.rotation.w = math.cos(pose.yaw / 2.0)
        self._tf.sendTransform(message)

    def _publish_pose(self, stamp) -> None:
        with self._lock:
            pose = self._pose.copy()
        message = PoseWithCovarianceStamped()
        message.header.stamp = stamp
        message.header.frame_id = self._map_frame
        message.pose.pose.position.x = pose.x
        message.pose.pose.position.y = pose.y
        message.pose.pose.position.z = pose.z
        message.pose.pose.orientation.z = math.sin(pose.yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(pose.yaw / 2.0)
        covariance = [0.0] * 36
        covariance[0] = 0.04
        covariance[7] = 0.04
        covariance[14] = 0.04
        covariance[21] = 0.01
        covariance[28] = 0.01
        covariance[35] = 0.02
        message.pose.covariance = covariance
        self._pose_pub.publish(message)

    def _publish_map(self) -> None:
        with self._lock:
            centers = self._index.centers
        if centers.shape[0] == 0:
            return
        self._map_pub.publish(_xyz_cloud(centers, self._map_frame, self.get_clock().now().to_msg()))

    def _paint(self, pose: Pose, points: np.ndarray) -> None:
        if not self._extend:
            return
        world = apply_pose(points, pose)
        origin = np.array([pose.x, pose.y, pose.z], dtype=np.float64)
        self._grid.integrate(origin, world)
        self._index = MapIndex(self._grid.occupied_centers())

    def _log_fix(self, pose: Pose, fraction: float, rms: float, kind: str) -> None:
        self.get_logger().info(
            f"{kind} x={pose.x:.2f} y={pose.y:.2f} z={pose.z:.2f} "
            f"yaw={math.degrees(pose.yaw):.1f} inliers={fraction:.2f} rms={rms:.3f}"
        )

    def _on_slice(self, message: PointCloud2) -> None:
        points = _cloud_to_xyz(message)
        if points.shape[0] < 30:
            return
        with self._lock:
            lost = self._lost
            seeded = self._seeded
            guess = self._pose.copy()
            index = self._index
        if not seeded:
            if not self._extend:
                self.get_logger().warning("No map is loaded and extend_map is off.")
                return
            pose = Pose()
            with self._lock:
                self._pose = pose
                self._lost = False
                self._misses = 0
                self._seeded = True
                self._paint(pose, points)
            self._publish_pose(message.header.stamp)
            self._publish_map()
            self._log_fix(pose, 1.0, 0.0, "SEED")
            return
        # A new tilt angle adds points. If the pose already explains the slice,
        # keep it. Otherwise each slice walks the rig even while it sits still.
        held = False
        pose, fraction, rms, count = guess, 0.0, 1.0e9, 0
        kind = "HOLD"
        if not lost and index.tree is not None:
            fraction, rms, count = score_pose(index, points, guess)
            # Full acceptance, not a tighter rms. A new tilt band is a few
            # centimeters off the last one and must not be free to walk.
            held = accepted(fraction, rms, count)
        if not held:
            if lost or index.tree is None:
                pose, fraction, rms, count = global_search(index, points)
                kind = "GLOBAL"
            else:
                pose, fraction, rms, count = icp(index, points, guess)
                kind = "TRACK"
                if not accepted(fraction, rms, count):
                    pose, fraction, rms, count = global_search(index, points)
                    kind = "GLOBAL"
        if not accepted(fraction, rms, count):
            with self._lock:
                self._misses += 1
                if self._misses >= 3:
                    self._lost = True
            self.get_logger().warning(
                f"reject inliers={fraction:.2f} rms={rms:.3f} misses={self._misses}"
            )
            return
        with self._lock:
            self._pose = pose
            self._lost = False
            self._misses = 0
            self._paint(pose, points)
        self._publish_pose(message.header.stamp)
        self._publish_map()
        self._log_fix(pose, fraction, rms, kind)

    def _save(self) -> None:
        if not self._extend:
            return
        with self._lock:
            centers = self._index.centers.copy()
            grid = self._grid
        if centers.shape[0] == 0:
            return
        self._output.mkdir(parents=True, exist_ok=True)
        grid_path = self._output / "room_grid.npz"
        ply_path = self._output / "room_cloud.ply"
        grid.save_npz(grid_path)
        save_ply(ply_path, centers)
        self.get_logger().info(f"MAP {centers.shape[0]} occupied -> {grid_path}")

    def _on_pass(self, _message: Int32) -> None:
        self._save()

    def destroy_node(self):
        try:
            self._save()
        except Exception as exc:
            self.get_logger().error(f"save failed: {exc}")
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CloudLocalizer()
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
