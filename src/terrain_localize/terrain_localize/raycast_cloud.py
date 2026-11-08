"""Publish one synthetic lidar cloud from a known pose on the heightmap."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

from terrain_localize.check_poses import AGL, map_paths
from terrain_localize.cloud import xyz_to_cloud
from terrain_localize.heightmap import load_heightmap
from terrain_localize.raycast import lidar_directions, raycast_cloud


class RaycastCloud(Node):
    def __init__(self):
        super().__init__("raycast_cloud")
        default_yaml, _ = map_paths()
        self.declare_parameter("map_yaml", str(default_yaml))
        self.declare_parameter("topic", "/lidar/points")
        self.declare_parameter("frame_id", "lidar_link")
        self.declare_parameter("x", 1200.0)
        self.declare_parameter("y", 800.0)
        self.declare_parameter("yaw_deg", 110.0)
        self.declare_parameter("agl", AGL)
        yaml_path = self.get_parameter("map_yaml").get_parameter_value().string_value
        self._heightmap = load_heightmap(yaml_path)
        self._directions = lidar_directions()
        self._frame = self.get_parameter("frame_id").get_parameter_value().string_value
        self._x = self.get_parameter("x").get_parameter_value().double_value
        self._y = self.get_parameter("y").get_parameter_value().double_value
        self._yaw = math.radians(self.get_parameter("yaw_deg").get_parameter_value().double_value)
        self._agl = self.get_parameter("agl").get_parameter_value().double_value
        self._pub = self.create_publisher(
            PointCloud2,
            self.get_parameter("topic").get_parameter_value().string_value,
            qos_profile_sensor_data,
        )
        self._timer = self.create_timer(2.0, self._publish)
        self.get_logger().info(
            f"Publishing a cloud at x={self._x:.0f} y={self._y:.0f} yaw_deg={math.degrees(self._yaw):.0f}"
        )

    def _publish(self):
        terrain, on_map = self._heightmap.sample(self._x, self._y)
        if not on_map:
            self.get_logger().error("Pose is outside the heightmap.")
            return
        cloud = raycast_cloud(
            self._heightmap,
            self._x,
            self._y,
            float(terrain) + self._agl,
            self._yaw,
            self._directions,
        )
        self._pub.publish(xyz_to_cloud(cloud, self._frame, self.get_clock().now().to_msg()))


def main(args=None):
    rclpy.init(args=args)
    node = RaycastCloud()
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
