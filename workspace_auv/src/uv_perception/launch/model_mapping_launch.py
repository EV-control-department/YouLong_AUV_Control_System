"""Launch the latched detector class mapping publisher."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uv_perception',
            executable='model_class_publisher',
            name='model_class_publisher',
            output='both',
            respawn=True,
            respawn_delay=1.0,
        ),
    ])
