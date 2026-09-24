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
        DeclareLaunchArgument('camera_startup_timeout_sec', default_value='5.0'),
        Node(
            package='uv_camera', executable='uv_camera', name='uv_camera',
            output='both', respawn=True, respawn_delay=1.0,
            parameters=[{
                'sim_mode': LaunchConfiguration('sim_mode'),
                'enable_front': LaunchConfiguration('enable_front'),
                'enable_down': LaunchConfiguration('enable_down'),
                'camera_config_profile': LaunchConfiguration('camera_config_profile'),
                'camera_config_dir': LaunchConfiguration('camera_config_dir'),
                'camera_startup_timeout_sec': LaunchConfiguration(
                    'camera_startup_timeout_sec'),
            }],
        ),
    ])
