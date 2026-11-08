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
            DeclareLaunchArgument("continuous", default_value="true"),
            DeclareLaunchArgument("start_at_zero", default_value="false"),
            DeclareLaunchArgument("park_at_zero", default_value="false"),
            DeclareLaunchArgument("output_ply", default_value=""),
            DeclareLaunchArgument("passes", default_value="1"),
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
                parameters=[
                    {
                        "port": LaunchConfiguration("esp_port"),
                        "continuous": ParameterValue(
                            LaunchConfiguration("continuous"), value_type=bool
                        ),
                        "start_at_zero": ParameterValue(
                            LaunchConfiguration("start_at_zero"), value_type=bool
                        ),
                        "park_at_zero": ParameterValue(
                            LaunchConfiguration("park_at_zero"), value_type=bool
                        ),
                        "output_ply": LaunchConfiguration("output_ply"),
                        "passes": ParameterValue(
                            LaunchConfiguration("passes"), value_type=int
                        ),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", str(share / "rviz" / "sweep.rviz")],
                output="screen",
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
