"""Serve a top-down room map and the live pose as JSON on HTTP.

The picture is viewed from above. Column increases with +X, forward of the
first cloud, to the right. Row decreases with +Y, left of the first cloud, so
left is toward the top of the image. The origin pixel is that first cloud.

GET /map is the current document. GET /events is one JSON message each time
the rig moves on the map. The same document is posted to post_url, which the
launch points at the Pi on port 8086.
"""

from __future__ import annotations

import json
import math
import threading
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

from lidar_tilt.register import load_ply


def _as_bool(value) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes")
    return bool(value)


def _yaw(orientation) -> float:
    sin_yaw = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cos_yaw = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    return math.atan2(sin_yaw, cos_yaw)


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


def _load_points(path: Path) -> np.ndarray:
    if path.suffix == ".npz":
        data = np.load(path)
        keys = np.asarray(data["keys"], dtype=np.int32).reshape(-1, 3)
        odds = np.asarray(data["log_odds"], dtype=np.float64).reshape(-1)
        voxel = float(np.asarray(data["voxel"]))
        occupied = keys[odds > 0.0]
        if occupied.size == 0:
            return np.zeros((0, 3), dtype=np.float64)
        return (occupied.astype(np.float64) + 0.5) * voxel
    return np.asarray(load_ply(path), dtype=np.float64).reshape(-1, 3)


def _wall_cells(
    points: np.ndarray,
    pixel_m: float,
    z_min: float,
    z_max: float,
    min_span_m: float,
) -> np.ndarray:
    """XY pixel indices of columns that stand up, so the floor is not the drawing."""
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if points.shape[0] == 0:
        return np.zeros((0, 2), dtype=np.int32)
    band = points[(points[:, 2] > z_min) & (points[:, 2] < z_max)]
    if band.shape[0] == 0:
        band = points
    ix = np.floor(band[:, 0] / pixel_m).astype(np.int32)
    iy = np.floor(band[:, 1] / pixel_m).astype(np.int32)
    order = np.lexsort((band[:, 2], iy, ix))
    ix, iy, z = ix[order], iy[order], band[order, 2]
    change = np.empty(len(ix), dtype=bool)
    change[0] = True
    change[1:] = (ix[1:] != ix[:-1]) | (iy[1:] != iy[:-1])
    starts = np.flatnonzero(change)
    ends = np.r_[starts[1:], len(ix)]
    span = z[ends - 1] - z[starts]
    keep = span >= min_span_m
    if not np.any(keep):
        keep = np.ones(len(starts), dtype=bool)
    return np.column_stack((ix[starts[keep]], iy[starts[keep]])).astype(np.int32)


def build_topdown(
    points: np.ndarray,
    pixel_m: float = 0.10,
    z_min: float = -0.5,
    z_max: float = 0.8,
    min_span_m: float = 0.30,
    margin_px: int = 2,
) -> dict:
    """Raster of the room from above, including the meter origin at (0, 0)."""
    cells = _wall_cells(points, pixel_m, z_min, z_max, min_span_m)
    if cells.shape[0] == 0:
        return {
            "meters_per_pixel": float(pixel_m),
            "width": 1,
            "height": 1,
            "origin_px": 0,
            "origin_py": 0,
            "outline": [],
        }
    x_min = min(int(cells[:, 0].min()) - margin_px, 0)
    x_max = max(int(cells[:, 0].max()) + margin_px, 0)
    y_min = min(int(cells[:, 1].min()) - margin_px, 0)
    y_max = max(int(cells[:, 1].max()) + margin_px, 0)
    # +X to the right, +Y toward the top. Row 0 is the top edge.
    cols = cells[:, 0] - x_min
    rows = y_max - cells[:, 1]
    outline = np.column_stack((cols, rows)).astype(int).tolist()
    return {
        "meters_per_pixel": float(pixel_m),
        "width": int(x_max - x_min + 1),
        "height": int(y_max - y_min + 1),
        "origin_px": int(0 - x_min),
        "origin_py": int(y_max - 0),
        "outline": outline,
    }


def _moved(previous, current, move_m: float, move_yaw_deg: float) -> bool:
    if previous is None:
        return True
    if math.hypot(current[0] - previous[0], current[1] - previous[1]) >= move_m:
        return True
    if abs(current[2] - previous[2]) >= move_m:
        return True
    delta = math.atan2(math.sin(current[3] - previous[3]), math.cos(current[3] - previous[3]))
    return abs(math.degrees(delta)) >= move_yaw_deg


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        return

    def _send(self, status: int, body: bytes, content_type: str) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        node = self.server.node
        if path in ("/", "/map"):
            self._send(200, node.payload(), "application/json")
            return
        if path == "/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Connection", "close")
            self.end_headers()
            seen = -1
            try:
                while True:
                    payload, generation = node.wait_payload(seen, 15.0)
                    if generation == seen:
                        self.wfile.write(b":\n\n")
                    else:
                        self.wfile.write(b"data: " + payload + b"\n\n")
                        seen = generation
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                return
            return
        self._send(404, b'{"error":"not found"}\n', "application/json")


class MapDashboard(Node):
    def __init__(self):
        super().__init__("map_dashboard")
        self.declare_parameter("map_path", "")
        self.declare_parameter("extend_map", False)
        self.declare_parameter("pose_topic", "/pose_fix")
        self.declare_parameter("map_topic", "/map_cloud")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8086)
        self.declare_parameter("post_url", "")
        self.declare_parameter("pixel_m", 0.10)
        self.declare_parameter("move_m", 0.10)
        self.declare_parameter("move_yaw_deg", 2.0)

        self._pixel_m = float(self.get_parameter("pixel_m").value)
        self._move_m = float(self.get_parameter("move_m").value)
        self._move_yaw_deg = float(self.get_parameter("move_yaw_deg").value)
        self._extend = _as_bool(self.get_parameter("extend_map").value)
        self._picture = build_topdown(np.zeros((0, 3)), self._pixel_m)
        self._pose = None
        self._sent = None
        self._seq = 0
        self._generation = 0
        self._payload = b""
        self._post_url = str(self.get_parameter("post_url").value).strip()
        self._post_ok = False
        self._post_warned = False
        self._lock = threading.Condition()

        map_path = str(self.get_parameter("map_path").value).strip()
        if map_path:
            path = Path(map_path)
            if path.is_file():
                self._set_picture(build_topdown(_load_points(path), self._pixel_m))
                self.get_logger().info(
                    f"Top-down outline {len(self._picture['outline'])} px "
                    f"from {path.name} ({self._picture['width']}x{self._picture['height']})"
                )
            else:
                self.get_logger().error(f"Map file not found: {path}")
        self._publish_document()

        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("pose_topic").value),
            self._on_pose,
            pose_qos,
        )
        if self._extend:
            self.create_subscription(
                PointCloud2,
                str(self.get_parameter("map_topic").value),
                self._on_map,
                pose_qos,
            )

        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        self._server = None
        try:
            self._server = ThreadingHTTPServer((host, port), _Handler)
        except OSError as exc:
            self.get_logger().error(f"Dashboard port {host}:{port} is not open: {exc}")
            return
        self._server.node = self
        self._thread = threading.Thread(target=self._server.serve_forever, name="map_dashboard_http", daemon=True)
        self._thread.start()
        self.get_logger().info(f"Dashboard http://{host}:{port}/map")

    def _set_picture(self, picture: dict) -> None:
        with self._lock:
            self._picture = picture

    def _publish_document(self) -> None:
        with self._lock:
            picture = self._picture
            pose = None if self._pose is None else tuple(self._pose)
            seq = self._seq
        pixel_m = picture["meters_per_pixel"]
        origin_px = picture["origin_px"]
        origin_py = picture["origin_py"]
        pose_json = None
        if pose is not None:
            yaw = pose[3]
            pose_json = {
                "x_m": round(pose[0], 3),
                "y_m": round(pose[1], 3),
                "z_m": round(pose[2], 3),
                "yaw_cw_deg": round(-math.degrees(yaw), 1),
                "px": round(origin_px + pose[0] / pixel_m, 2),
                "py": round(origin_py - pose[1] / pixel_m, 2),
                "heading_px": [round(math.cos(yaw), 3), round(-math.sin(yaw), 3)],
            }
        document = {
            "view": "topdown",
            "locked": pose is not None,
            "seq": seq,
            "meters_per_pixel": pixel_m,
            "width": picture["width"],
            "height": picture["height"],
            "col_from_x": "origin.px + x_m / meters_per_pixel",
            "row_from_y": "origin.py - y_m / meters_per_pixel",
            "origin": {
                "x_m": 0.0,
                "y_m": 0.0,
                "z_m": 0.0,
                "px": origin_px,
                "py": origin_py,
            },
            "pose": pose_json,
            "outline": picture["outline"],
        }
        payload = json.dumps(document, separators=(",", ":")).encode("utf-8")
        with self._lock:
            self._payload = payload
            self._generation += 1
            self._lock.notify_all()
        self._post_latest(payload)

    def _post_latest(self, payload: bytes) -> None:
        url = self._post_url
        if not url:
            return

        def send(body: bytes, target: str) -> None:
            try:
                request = urllib.request.Request(
                    target,
                    data=body,
                    headers={"Content-Type": "application/json"},
                    method="POST",
                )
                with urllib.request.urlopen(request, timeout=2.0) as response:
                    response.read()
            except (urllib.error.URLError, TimeoutError, OSError) as exc:
                self._post_ok = False
                if not self._post_warned:
                    self.get_logger().warning(f"Map post failed: {exc}")
                    self._post_warned = True
                return
            self._post_warned = False
            if not self._post_ok:
                self.get_logger().info(f"Posted map to {target}")
                self._post_ok = True

        threading.Thread(target=send, args=(payload, url), name="map_post", daemon=True).start()

    def payload(self) -> bytes:
        with self._lock:
            return self._payload

    def wait_payload(self, seen: int, timeout: float) -> tuple[bytes, int]:
        with self._lock:
            if self._generation == seen:
                self._lock.wait(timeout=timeout)
            return self._payload, self._generation

    def _on_pose(self, message: PoseWithCovarianceStamped) -> None:
        position = message.pose.pose.position
        current = (float(position.x), float(position.y), float(position.z), _yaw(message.pose.pose.orientation))
        with self._lock:
            if not _moved(self._sent, current, self._move_m, self._move_yaw_deg):
                return
            self._pose = current
            self._sent = current
            self._seq += 1
        self._publish_document()

    def _on_map(self, message: PointCloud2) -> None:
        points = _cloud_to_xyz(message)
        if points.shape[0] == 0:
            return
        picture = build_topdown(points, self._pixel_m)
        with self._lock:
            if picture["outline"] == self._picture["outline"]:
                return
            self._picture = picture
            self._seq += 1
        self.get_logger().info(f"Top-down outline {len(picture['outline'])} px")
        self._publish_document()

    def destroy_node(self):
        server = self._server
        if server is not None:
            self._server = None
            server.shutdown()
            server.server_close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MapDashboard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
