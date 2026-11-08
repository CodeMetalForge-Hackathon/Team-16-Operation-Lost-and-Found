"""Subscribe to a lidar cloud and publish a global terrain fix."""

from __future__ import annotations

import json
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformBroadcaster

from terrain_localize.check_poses import map_paths
from terrain_localize.cloud import cloud_to_xyz
from terrain_localize.heightmap import load_heightmap
from terrain_localize.match import localize


def _yaw_to_quaternion(yaw: float):
    from math import cos, sin

    return 0.0, 0.0, sin(yaw * 0.5), cos(yaw * 0.5)


class TerrainLocalizer(Node):
    def __init__(self):
        super().__init__("terrain_localizer")
        default_yaml, _ = map_paths()
        self.declare_parameter("map_yaml", str(default_yaml))
        self.declare_parameter("cloud_topic", "/lidar/points")
        self.declare_parameter("pose_topic", "/pose_fix")
        self.declare_parameter("jsonl_path", "/tmp/pose_fix.jsonl")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("child_frame", "lidar_link")
        self.declare_parameter("yaw_step_deg", 5.0)

        yaml_path = Path(self.get_parameter("map_yaml").get_parameter_value().string_value)
        self._heightmap = load_heightmap(yaml_path)
        self._map_frame = self.get_parameter("map_frame").get_parameter_value().string_value
        self._child_frame = self.get_parameter("child_frame").get_parameter_value().string_value
        self._yaw_step = self.get_parameter("yaw_step_deg").get_parameter_value().double_value
        self._jsonl = Path(self.get_parameter("jsonl_path").get_parameter_value().string_value)
        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter("pose_topic").get_parameter_value().string_value,
            10,
        )
        self._tf = TransformBroadcaster(self)
        topic = self.get_parameter("cloud_topic").get_parameter_value().string_value
        self.create_subscription(PointCloud2, topic, self._on_cloud, qos_profile_sensor_data)
        self.get_logger().info(f"Matching {topic} against {yaml_path}")

    def _on_cloud(self, msg: PointCloud2):
        fix = localize(self._heightmap, cloud_to_xyz(msg), yaw_step_deg=self._yaw_step)
        if fix is None:
            self.get_logger().warning("Cloud had too few ground returns to match.")
            return
        qx, qy, qz, qw = _yaw_to_quaternion(fix.yaw)
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self._map_frame
        pose.pose.pose.position.x = fix.x
        pose.pose.pose.position.y = fix.y
        pose.pose.pose.position.z = fix.z
        pose.pose.pose.orientation.x = qx
        pose.pose.pose.orientation.y = qy
        pose.pose.pose.orientation.z = qz
        pose.pose.pose.orientation.w = qw
        # 10 m, 10 m, 5 m, and about 5 degrees. Yaw is the weak axis on a smooth hill.
        variance = [100.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 100.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 25.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.05, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.05, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
        pose.pose.covariance = variance
        self._pose_pub.publish(pose)

        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = self._map_frame
        transform.child_frame_id = self._child_frame
        transform.transform.translation.x = fix.x
        transform.transform.translation.y = fix.y
        transform.transform.translation.z = fix.z
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self._tf.sendTransform(transform)

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        record = {"t": stamp, "x": fix.x, "y": fix.y, "z": fix.z, "source": "lidar"}
        self._jsonl.parent.mkdir(parents=True, exist_ok=True)
        with self._jsonl.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record) + "\n")
        self.get_logger().info(
            f"pose_fix x={fix.x:.1f} y={fix.y:.1f} z={fix.z:.1f} yaw_deg={fix.yaw * 57.2958:.1f} rms={fix.rms:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TerrainLocalizer()
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
