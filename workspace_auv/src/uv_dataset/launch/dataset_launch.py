"""Optional raw dataset recorder; it never depends on go2rtc."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('output', default_value='records/datasets'),
        Node(
            package='uv_dataset', executable='dataset_recorder',
            name='dataset_recorder', output='both', respawn=True,
            respawn_delay=1.0,
            arguments=['--output', LaunchConfiguration('output')],
        ),
    ])
