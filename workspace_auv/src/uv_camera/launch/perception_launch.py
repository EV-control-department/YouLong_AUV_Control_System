from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uv_camera',
            executable='uv_camera',
            name='uv_camera',
            output='screen',
        ),
        Node(
            package='uv_camera',
            executable='object_localizer',
            name='object_localizer',
            output='screen',
        ),
    ])
