"""Launch detector, geometry localizer, and multi-frame estimator."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('confidence', default_value='0.5'),
        DeclareLaunchArgument('stereo_baseline_m', default_value='0.10'),
        DeclareLaunchArgument('association_distance_m', default_value='1.5'),
        DeclareLaunchArgument('bearing_association_distance_m', default_value='0.35'),
        DeclareLaunchArgument('world_frame', default_value='odom'),
        DeclareLaunchArgument('edge_margin_px', default_value='8.0'),
        DeclareLaunchArgument('edge_margin_ratio', default_value='0.02'),
        DeclareLaunchArgument('stereo_epipolar_tolerance_px', default_value='10.0'),
        DeclareLaunchArgument('max_stereo_range_m', default_value='30.0'),
        DeclareLaunchArgument('min_parallax_deg', default_value='5.0'),
        DeclareLaunchArgument('huber_delta', default_value='2.5'),
        DeclareLaunchArgument('pose_translation_sigma_m', default_value='0.03'),
        DeclareLaunchArgument('pose_rotation_sigma_deg', default_value='1.0'),
        DeclareLaunchArgument('extrinsic_translation_sigma_m', default_value='0.005'),
        DeclareLaunchArgument('extrinsic_rotation_sigma_deg', default_value='0.5'),
        DeclareLaunchArgument('enable_gui', default_value='false'),
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
            respawn_delay=1.0,
            remappings=[('/tf', '/auv/tf'), ('/tf_static', '/auv/tf_static')],
            parameters=[{
                'stereo_baseline_m': LaunchConfiguration('stereo_baseline_m'),
                'world_frame': LaunchConfiguration('world_frame'),
                'edge_margin_px': LaunchConfiguration('edge_margin_px'),
                'edge_margin_ratio': LaunchConfiguration('edge_margin_ratio'),
                'stereo_epipolar_tolerance_px':
                    LaunchConfiguration('stereo_epipolar_tolerance_px'),
                'max_stereo_range_m': LaunchConfiguration('max_stereo_range_m'),
                'pose_translation_sigma_m':
                    LaunchConfiguration('pose_translation_sigma_m'),
                'pose_rotation_sigma_deg':
                    LaunchConfiguration('pose_rotation_sigma_deg'),
                'extrinsic_translation_sigma_m':
                    LaunchConfiguration('extrinsic_translation_sigma_m'),
                'extrinsic_rotation_sigma_deg':
                    LaunchConfiguration('extrinsic_rotation_sigma_deg'),
            }]),
        Node(
            package='uv_perception', executable='object_estimator',
            name='object_estimator', output='both', respawn=True,
            respawn_delay=1.0, parameters=[{
                'world_frame': LaunchConfiguration('world_frame'),
                'association_distance_m': LaunchConfiguration(
                    'association_distance_m'),
                'bearing_association_distance_m':
                    LaunchConfiguration('bearing_association_distance_m'),
                'min_parallax_deg': LaunchConfiguration('min_parallax_deg'),
                'huber_delta': LaunchConfiguration('huber_delta'),
                'pose_translation_sigma_m':
                    LaunchConfiguration('pose_translation_sigma_m'),
                'pose_rotation_sigma_deg':
                    LaunchConfiguration('pose_rotation_sigma_deg'),
                'extrinsic_translation_sigma_m':
                    LaunchConfiguration('extrinsic_translation_sigma_m'),
                'extrinsic_rotation_sigma_deg':
                    LaunchConfiguration('extrinsic_rotation_sigma_deg'),
            }]),
        Node(
            package='uv_perception', executable='perception_gui',
            name='perception_gui', output='both',
            condition=IfCondition(LaunchConfiguration('enable_gui'))),
    ])
