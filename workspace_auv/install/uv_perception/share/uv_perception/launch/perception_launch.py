from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uv_perception',
            executable='vision',
            name='vision',
            output='screen',
        ),
        Node(
            package='uv_perception',
            executable='position',
            name='position',
            output='screen',
        ),
    ])
