"""Launch detector, per-view ray localizer, and static ray estimator."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_detector', default_value='true'),
        DeclareLaunchArgument('enable_localizer', default_value='true'),
        DeclareLaunchArgument('enable_estimator', default_value='true'),
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('front_model_path', default_value='/home/doc049/dev/UUV/YouLong_AUV_Control_System/workspace_auv/src/uv_perception/weights/HQQ6_aug.pt'),
        DeclareLaunchArgument('down_model_path', default_value='/home/doc049/dev/UUV/YouLong_AUV_Control_System/workspace_auv/src/uv_perception/weights/HQQ7_aug.pt'),
        DeclareLaunchArgument('confidence', default_value='0.5'),
        DeclareLaunchArgument('world_frame', default_value='odom'),
        DeclareLaunchArgument('edge_margin_px', default_value='8.0'),
        DeclareLaunchArgument('edge_margin_ratio', default_value='0.02'),
        DeclareLaunchArgument('stereo_max_ray_gap_m', default_value='0.35'),
        DeclareLaunchArgument('stereo_min_parallax_deg', default_value='0.2'),
        DeclareLaunchArgument('observation_pool_size', default_value='0'),
        DeclareLaunchArgument('candidate_ray_limit', default_value='80'),
        DeclareLaunchArgument('candidate_pair_limit', default_value='2400'),
        DeclareLaunchArgument('max_candidate_clusters', default_value='8'),
        DeclareLaunchArgument('seed_cluster_radius_m', default_value='0.35'),
        DeclareLaunchArgument('max_pair_gap_m', default_value='1.0'),
        DeclareLaunchArgument('min_parallax_deg', default_value='5.0'),
        DeclareLaunchArgument('clutter_prior', default_value='0.08'),
        DeclareLaunchArgument('huber_delta', default_value='2.5'),
        DeclareLaunchArgument('lm_iterations', default_value='10'),
        DeclareLaunchArgument('association_cycles', default_value='3'),
        DeclareLaunchArgument('pose_translation_sigma_m', default_value='0.03'),
        DeclareLaunchArgument('pose_rotation_sigma_deg', default_value='1.0'),
        DeclareLaunchArgument('extrinsic_translation_sigma_m', default_value='0.005'),
        DeclareLaunchArgument('extrinsic_rotation_sigma_deg', default_value='0.5'),
        DeclareLaunchArgument('anchor_sigma_default_m', default_value='0.10'),
        DeclareLaunchArgument('anchor_sigma_collection_frame_m', default_value='0.14'),
        DeclareLaunchArgument('anchor_sigma_target_rack_m', default_value='0.14'),
        DeclareLaunchArgument('stable_covariance_trace_m2', default_value='0.04'),
        DeclareLaunchArgument('instance_association_distance_m', default_value='1.5'),
        DeclareLaunchArgument('enable_gui', default_value='false'),
        Node(
            package='uv_perception', executable='object_detector',
            name='object_detector', output='both', respawn=True,
            respawn_delay=1.0, parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'front_model_path': LaunchConfiguration('front_model_path'),
                'down_model_path': LaunchConfiguration('down_model_path'),
                'confidence': LaunchConfiguration('confidence'),
            }], condition=IfCondition(LaunchConfiguration('enable_detector'))),
        Node(
            package='uv_perception', executable='object_localizer',
            name='object_localizer', output='both', respawn=True,
            respawn_delay=1.0,
            remappings=[('/tf', '/auv/tf'), ('/tf_static', '/auv/tf_static')],
            parameters=[{
                'world_frame': LaunchConfiguration('world_frame'),
                'edge_margin_px': LaunchConfiguration('edge_margin_px'),
                'edge_margin_ratio': LaunchConfiguration('edge_margin_ratio'),
                'stereo_max_ray_gap_m':
                    LaunchConfiguration('stereo_max_ray_gap_m'),
                'stereo_min_parallax_deg':
                    LaunchConfiguration('stereo_min_parallax_deg'),
            }], condition=IfCondition(LaunchConfiguration('enable_localizer'))),
        Node(
            package='uv_perception', executable='object_estimator',
            name='object_estimator', output='both', respawn=True,
            respawn_delay=1.0, parameters=[{
                'world_frame': LaunchConfiguration('world_frame'),
                'observation_pool_size': LaunchConfiguration('observation_pool_size'),
                'candidate_ray_limit': LaunchConfiguration('candidate_ray_limit'),
                'candidate_pair_limit': LaunchConfiguration('candidate_pair_limit'),
                'max_candidate_clusters': LaunchConfiguration('max_candidate_clusters'),
                'seed_cluster_radius_m': LaunchConfiguration('seed_cluster_radius_m'),
                'max_pair_gap_m': LaunchConfiguration('max_pair_gap_m'),
                'min_parallax_deg': LaunchConfiguration('min_parallax_deg'),
                'clutter_prior': LaunchConfiguration('clutter_prior'),
                'huber_delta': LaunchConfiguration('huber_delta'),
                'lm_iterations': LaunchConfiguration('lm_iterations'),
                'association_cycles': LaunchConfiguration('association_cycles'),
                'pose_translation_sigma_m': LaunchConfiguration('pose_translation_sigma_m'),
                'pose_rotation_sigma_deg': LaunchConfiguration('pose_rotation_sigma_deg'),
                'extrinsic_translation_sigma_m': LaunchConfiguration('extrinsic_translation_sigma_m'),
                'extrinsic_rotation_sigma_deg': LaunchConfiguration('extrinsic_rotation_sigma_deg'),
                'anchor_sigma_default_m': LaunchConfiguration('anchor_sigma_default_m'),
                'anchor_sigma_collection_frame_m':
                    LaunchConfiguration('anchor_sigma_collection_frame_m'),
                'anchor_sigma_target_rack_m':
                    LaunchConfiguration('anchor_sigma_target_rack_m'),
                'stable_covariance_trace_m2':
                    LaunchConfiguration('stable_covariance_trace_m2'),
                'instance_association_distance_m':
                    LaunchConfiguration('instance_association_distance_m'),
            }], condition=IfCondition(LaunchConfiguration('enable_estimator'))),
        Node(
            package='uv_perception', executable='perception_gui',
            name='perception_gui', output='both',
            condition=IfCondition(LaunchConfiguration('enable_gui'))),
    ])
