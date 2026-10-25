"""Start go2rtc and the sl_camera catalog-driven route manager."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('catalog_topic', default_value='/auv/sl_camera/cameras'),
        DeclareLaunchArgument('max_fps', default_value='30.0'),
        DeclareLaunchArgument('bitrate', default_value='2000000'),
        DeclareLaunchArgument('max_bitrate', default_value='3000000'),
        DeclareLaunchArgument('encoder', default_value='auto'),
        DeclareLaunchArgument('api_port', default_value='1984'),
        DeclareLaunchArgument('go2rtc_executable', default_value=''),
        Node(
            package='sl_stream', executable='sl_stream', name='sl_stream',
            output='both', parameters=[{
                'catalog_topic': LaunchConfiguration('catalog_topic'),
                'max_fps': LaunchConfiguration('max_fps'),
                'bitrate': LaunchConfiguration('bitrate'),
                'max_bitrate': LaunchConfiguration('max_bitrate'),
                'encoder': LaunchConfiguration('encoder'),
                'api_port': LaunchConfiguration('api_port'),
                'go2rtc_executable': LaunchConfiguration('go2rtc_executable'),
            }]),
    ])
