from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="terrain_localize",
                executable="localizer",
                name="terrain_localizer",
                output="screen",
            ),
        ]
    )
