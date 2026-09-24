"""Compatibility launch for the split camera/perception/streaming graph.

The historical filename is retained so existing operator commands continue to
work. It no longer starts the legacy composed node; the three owning packages
are launched explicitly and the only video listener is go2rtc:1984.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(package, launch_file, arguments, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(package), 'launch', launch_file,
        ])),
        launch_arguments=arguments.items(), condition=condition)


def generate_launch_description():
    enable_ai = LaunchConfiguration('enable_ai')
    return LaunchDescription([
        DeclareLaunchArgument('enable_ai', default_value='true'),
        DeclareLaunchArgument('sim_mode', default_value='false'),
        DeclareLaunchArgument('camera_config_profile', default_value='auto'),
        DeclareLaunchArgument('camera_config_dir', default_value=''),
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('confidence', default_value='0.5'),
        _include('uv_camera', 'camera_launch.py', {
            'sim_mode': LaunchConfiguration('sim_mode'),
            'camera_config_profile': LaunchConfiguration('camera_config_profile'),
            'camera_config_dir': LaunchConfiguration('camera_config_dir'),
        }),
        _include('uv_perception', 'perception_launch.py', {
            'model_path': LaunchConfiguration('model_path'),
            'confidence': LaunchConfiguration('confidence'),
        }, condition=IfCondition(enable_ai)),
        _include('uv_stream', 'stream_launch.py', {}),
    ])
