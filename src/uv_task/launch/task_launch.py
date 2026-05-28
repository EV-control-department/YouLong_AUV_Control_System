from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uv_task',
            executable='task_runner',
            name='task_runner',
            output='screen',
        ),
    ])
