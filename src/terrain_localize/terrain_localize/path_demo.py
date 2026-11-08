"""Fly a fixed path and publish the clouds the localizer matches."""

from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Quaternion
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray

from terrain_localize.check_poses import AGL, map_paths
from terrain_localize.cloud import xyz_to_cloud
from terrain_localize.heightmap import load_heightmap
from terrain_localize.raycast import lidar_directions, raycast_cloud
from terrain_localize.route import demo_poses


def _yaw_quaternion(yaw: float) -> Quaternion:
    quat = Quaternion()
    quat.z = math.sin(yaw * 0.5)
    quat.w = math.cos(yaw * 0.5)
    return quat


def _stamp_equal(left, right) -> bool:
    return left is not None and right is not None and left.sec == right.sec and left.nanosec == right.nanosec


class PathDemo(Node):
    def __init__(self):
        super().__init__("path_demo")
        default_yaml, _ = map_paths()
        self.declare_parameter("map_yaml", str(default_yaml))
        self.declare_parameter("samples", 8)
        self.declare_parameter("png_path", "/tmp/terrain_path_demo.png")
        yaml_path = self.get_parameter("map_yaml").get_parameter_value().string_value
        self._heightmap = load_heightmap(yaml_path)
        self._directions = lidar_directions()
        self._poses = demo_poses(self.get_parameter("samples").get_parameter_value().integer_value)
        self._png_path = self.get_parameter("png_path").get_parameter_value().string_value
        self._index = 0
        self._waiting = False
        self._sent_stamp = None
        self._sent_at = 0.0
        self._truth: list[tuple[float, float]] = []
        self._estimate: list[tuple[float, float, float, float]] = []
        self._errors: list[float] = []
        self._done = False

        # Reliable so RViz can subscribe. The localizer's best-effort subscription still connects.
        cloud_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self._cloud_pub = self.create_publisher(PointCloud2, "/lidar/points", cloud_qos)
        self._terrain_pub = self.create_publisher(PointCloud2, "/terrain", 1)
        self._markers_pub = self.create_publisher(MarkerArray, "/demo_markers", 1)
        self.create_subscription(PoseWithCovarianceStamped, "/pose_fix", self._on_fix, 10)
        self._terrain = self._terrain_cloud()
        self.create_timer(1.0, self._tick)
        self.get_logger().info(f"Path has {len(self._poses)} stops. Publishing clouds on /lidar/points.")

    def _terrain_cloud(self) -> PointCloud2:
        grid = self._heightmap.z[::2, ::2]
        rows, cols = grid.shape
        ys = (np.arange(rows) * 2 + 0.5) * self._heightmap.resolution
        xs = (np.arange(cols) * 2 + 0.5) * self._heightmap.resolution
        xx, yy = np.meshgrid(xs, ys)
        points = np.column_stack((xx.ravel(), yy.ravel(), grid.ravel(), grid.ravel())).astype(np.float32)
        msg = PointCloud2()
        msg.header.frame_id = "map"
        msg.height = 1
        msg.width = int(points.shape[0])
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * msg.width
        msg.is_dense = True
        msg.data = points.tobytes()
        return msg

    def _tick(self):
        now = self.get_clock().now().to_msg()
        self._terrain.header.stamp = now
        self._terrain_pub.publish(self._terrain)
        self._markers_pub.publish(self._markers(now))
        if self._done or self._waiting:
            if self._waiting and time.monotonic() - self._sent_at > 25.0:
                self.get_logger().warning("No fix yet. Publishing the same stop again.")
                self._publish_step()
            return
        self._publish_step()

    def _publish_step(self):
        x, y, yaw = self._poses[self._index]
        terrain, on_map = self._heightmap.sample(x, y)
        if not on_map:
            self.get_logger().error(f"Stop {self._index} is off the map.")
            self._done = True
            return
        cloud = raycast_cloud(self._heightmap, x, y, float(terrain) + AGL, yaw, self._directions)
        stamp = self.get_clock().now().to_msg()
        self._sent_stamp = stamp
        self._sent_at = time.monotonic()
        self._waiting = True
        self._cloud_pub.publish(xyz_to_cloud(cloud, "lidar_link", stamp))
        self.get_logger().info(
            f"Stop {self._index + 1}/{len(self._poses)} truth x={x:.0f} y={y:.0f} yaw_deg={math.degrees(yaw):.0f}"
        )

    def _on_fix(self, msg: PoseWithCovarianceStamped):
        if self._done or not _stamp_equal(msg.header.stamp, self._sent_stamp):
            return
        x, y, _yaw = self._poses[self._index]
        est_x = msg.pose.pose.position.x
        est_y = msg.pose.pose.position.y
        est_z = msg.pose.pose.position.z
        est_yaw = 2.0 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        error = math.hypot(est_x - x, est_y - y)
        self._truth.append((x, y))
        self._estimate.append((est_x, est_y, est_z, est_yaw))
        self._errors.append(error)
        self.get_logger().info(f"Stop {self._index + 1} xy_err={error:.1f} m")
        self._index += 1
        self._waiting = False
        self._sent_stamp = None
        if self._index >= len(self._poses):
            self._done = True
            worst = max(self._errors)
            self._save_png()
            status = "PATH_OK" if worst <= 30.0 else "PATH_FAIL"
            self.get_logger().info(f"{status} stops={len(self._errors)} max_xy={worst:.1f} m png={self._png_path}")

    def _markers(self, stamp) -> MarkerArray:
        array = MarkerArray()
        array.markers.append(self._line(3, self._poses_xy(), stamp, 0.35, 0.75, 0.4))
        if len(self._estimate) >= 2:
            array.markers.append(self._line(4, [(x, y) for x, y, _z, _yaw in self._estimate], stamp, 0.95, 0.4, 0.15))
        if not self._done:
            x, y, yaw = self._poses[self._index]
            terrain, on_map = self._heightmap.sample(x, y)
            z = float(terrain) + 40.0 if on_map else 40.0
            array.markers.append(self._arrow(1, x, y, z, yaw, stamp, 0.2, 0.9, 0.3))
        if self._estimate:
            ex, ey, ez, eyaw = self._estimate[-1]
            array.markers.append(self._arrow(2, ex, ey, ez, eyaw, stamp, 0.95, 0.45, 0.1))
        return array

    def _poses_xy(self) -> list[tuple[float, float]]:
        return [(x, y) for x, y, _yaw in self._poses]

    def _line(self, marker_id, points, stamp, red, green, blue) -> Marker:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = stamp
        marker.ns = "path"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 12.0
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        for x, y in points:
            terrain, on_map = self._heightmap.sample(x, y)
            point = Point()
            point.x = float(x)
            point.y = float(y)
            point.z = float(terrain) + 15.0 if on_map else 15.0
            marker.points.append(point)
        return marker

    def _arrow(self, marker_id, x, y, z, yaw, stamp, red, green, blue) -> Marker:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = stamp
        marker.ns = "pod"
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        marker.pose.orientation = _yaw_quaternion(yaw)
        marker.scale.x = 90.0
        marker.scale.y = 24.0
        marker.scale.z = 24.0
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 1.0
        return marker

    def _save_png(self):
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        height = self._heightmap.z
        extent = [
            self._heightmap.origin_x,
            self._heightmap.origin_x + self._heightmap.width * self._heightmap.resolution,
            self._heightmap.origin_y,
            self._heightmap.origin_y + self._heightmap.height * self._heightmap.resolution,
        ]
        figure, axis = plt.subplots(figsize=(8, 8))
        axis.imshow(height, origin="lower", extent=extent, cmap="terrain")
        plan = np.array(self._poses_xy())
        axis.plot(plan[:, 0], plan[:, 1], color="white", linewidth=2, label="plan")
        if self._estimate:
            est = np.array(self._estimate)
            axis.plot(est[:, 0], est[:, 1], color="#e85d04", linewidth=2, marker="o", label="estimate")
        axis.set_title("Planned path and lidar fixes")
        axis.set_xlabel("x (m)")
        axis.set_ylabel("y (m)")
        axis.legend(loc="upper right")
        figure.tight_layout()
        figure.savefig(self._png_path, dpi=120)
        plt.close(figure)


def main(args=None):
    rclpy.init(args=args)
    node = PathDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
