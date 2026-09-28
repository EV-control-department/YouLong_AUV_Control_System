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
        DeclareLaunchArgument('camera_config_profile', default_value='auto'),
        DeclareLaunchArgument('camera_config_dir', default_value=''),
        DeclareLaunchArgument('front_camera_device', default_value=''),
        DeclareLaunchArgument('down_camera_device', default_value=''),
        DeclareLaunchArgument('camera_startup_timeout_sec', default_value='5.0'),
        Node(
            package='uv_camera', executable='uv_camera', name='uv_camera',
            output='both',
            parameters=[{
                'sim_mode': LaunchConfiguration('sim_mode'),
                'enable_front': LaunchConfiguration('enable_front'),
                'enable_down': LaunchConfiguration('enable_down'),
                'camera_config_profile': LaunchConfiguration('camera_config_profile'),
                'camera_config_dir': LaunchConfiguration('camera_config_dir'),
                'front_camera_device': LaunchConfiguration('front_camera_device'),
                'down_camera_device': LaunchConfiguration('down_camera_device'),
                'camera_startup_timeout_sec': LaunchConfiguration(
                    'camera_startup_timeout_sec'),
            }],
        ),
    ])
