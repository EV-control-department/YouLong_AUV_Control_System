from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uv_hm',
            executable='sim_bridge',
            name='sim_bridge',
            output='screen',
        ),
        Node(
            package='uv_hm',
            executable='hw_manager',
            name='hw_manager',
            output='screen',
        ),
    ])
