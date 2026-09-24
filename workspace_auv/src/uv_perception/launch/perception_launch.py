"""Launch detector, geometry localizer, and multi-frame estimator."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('confidence', default_value='0.5'),
        DeclareLaunchArgument('stereo_baseline_m', default_value='0.10'),
        DeclareLaunchArgument('association_distance_m', default_value='2.0'),
        Node(
            package='uv_perception', executable='object_detector',
            name='object_detector', output='both', respawn=True,
            respawn_delay=1.0, parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'confidence': LaunchConfiguration('confidence'),
            }]),
        Node(
            package='uv_perception', executable='object_localizer',
            name='object_localizer', output='both', respawn=True,
            respawn_delay=1.0, parameters=[{
                'stereo_baseline_m': LaunchConfiguration('stereo_baseline_m'),
            }]),
        Node(
            package='uv_perception', executable='object_estimator',
            name='object_estimator', output='both', respawn=True,
            respawn_delay=1.0, parameters=[{
                'association_distance_m': LaunchConfiguration(
                    'association_distance_m'),
            }]),
    ])
