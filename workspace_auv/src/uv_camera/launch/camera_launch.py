"""Launch the pure camera acquisition and iceoryx2 publisher."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim_mode', default_value='false'),
        DeclareLaunchArgument('enable_front', default_value='true'),
        DeclareLaunchArgument('enable_down', default_value='true'),
        DeclareLaunchArgument(
            'camera_mode', default_value='auto', choices=['auto', 'real', 'sim']),
        DeclareLaunchArgument('camera_config_dir', default_value=''),
        DeclareLaunchArgument('front_camera_device', default_value=''),
        DeclareLaunchArgument('down_camera_device', default_value=''),
        DeclareLaunchArgument(
            'camera_startup_timeout_sec', default_value='5.0',
            description=(
                'Seconds without a valid frame before reporting a loss and '
                'reopening that camera')),
        DeclareLaunchArgument(
            'camera_reconnect_interval_sec', default_value='1.0',
            description='Delay between camera reopen attempts'),
        Node(
            package='uv_camera', executable='uv_camera', name='uv_camera',
            output='both',
            parameters=[{
                'sim_mode': LaunchConfiguration('sim_mode'),
                'enable_front': LaunchConfiguration('enable_front'),
                'enable_down': LaunchConfiguration('enable_down'),
                'camera_mode': LaunchConfiguration('camera_mode'),
                'camera_config_dir': LaunchConfiguration('camera_config_dir'),
                'front_camera_device': LaunchConfiguration('front_camera_device'),
                'down_camera_device': LaunchConfiguration('down_camera_device'),
                'camera_startup_timeout_sec': LaunchConfiguration(
                    'camera_startup_timeout_sec'),
                'camera_reconnect_interval_sec': LaunchConfiguration(
                    'camera_reconnect_interval_sec'),
            }],
        ),
    ])
