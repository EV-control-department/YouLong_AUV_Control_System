"""Launch one multi-camera manager from a collection profile."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='default',
                              description='Collection profile name or YAML path'),
        Node(package='sl_camera', executable='sl_camera', name='sl_camera',
             parameters=[{'profile': LaunchConfiguration('profile')}], output='both'),
    ])
