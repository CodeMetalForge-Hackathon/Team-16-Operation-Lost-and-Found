from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("terrain_localize"))
    rviz_config = str(share / "rviz" / "demo.rviz")
    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            Node(
                package="terrain_localize",
                executable="localizer",
                name="terrain_localizer",
                output="screen",
            ),
            Node(
                package="terrain_localize",
                executable="path_demo",
                name="path_demo",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
