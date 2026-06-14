from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uv_nav',
            executable='navigator',
            name='navigator',
            output='screen',
        ),
    ])
