"""Launch the Week 03 publisher and subscriber together."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="smartcar_intro",
                executable="status_pub",
                name="status_publisher",
                output="screen",
            ),
            Node(
                package="smartcar_intro",
                executable="status_sub",
                name="status_subscriber",
                output="screen",
            ),
        ]
    )
