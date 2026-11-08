from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("lidar_tilt"))
    sllidar_launch = str(
        Path(get_package_share_directory("sllidar_ros2")) / "launch" / "sllidar_c1_launch.py"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("esp_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("map_path", default_value=""),
            DeclareLaunchArgument("extend_map", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sllidar_launch),
                launch_arguments={
                    "serial_port": LaunchConfiguration("lidar_port"),
                    "serial_baudrate": "460800",
                    "frame_id": "laser",
                    "scan_mode": "Standard",
                }.items(),
            ),
            Node(
                package="lidar_tilt",
                executable="tilt_sweep",
                name="tilt_sweep",
                output="screen",
                parameters=[{"port": LaunchConfiguration("esp_port"), "continuous": True}],
            ),
            Node(
                package="lidar_tilt",
                executable="cloud_localize",
                name="cloud_localize",
                output="screen",
                parameters=[
                    {
                        "map_path": LaunchConfiguration("map_path"),
                        "extend_map": ParameterValue(
                            LaunchConfiguration("extend_map"), value_type=bool
                        ),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", str(share / "rviz" / "localize.rviz")],
                output="screen",
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
