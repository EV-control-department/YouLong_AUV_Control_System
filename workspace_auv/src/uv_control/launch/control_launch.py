from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uv_control',
            executable='minimal_control',
            name='minimal_control',
            output='screen',
        ),
        Node(
            package='uv_control',
            executable='basic_motion',
            name='basic_motion',
            output='screen',
        ),
    ])
