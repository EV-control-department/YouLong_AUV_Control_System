"""Stonefish SIL orchestration owned by ``workspace_sim``."""

from __future__ import annotations

from auv_protocol.topics import (
    DOWN_STITCHED,
    DVL_VELOCITY,
    FRONT_STITCHED,
    IMU,
    SIM_DEGRADED_DOWN_STITCHED,
    SIM_DEGRADED_DVL_VELOCITY,
    SIM_DEGRADED_FRONT_STITCHED,
    SIM_DEGRADED_IMU,
    SIM_DEGRADED_USBL,
    USBL_MEASUREMENT,
)
from launch import LaunchDescription, Substitution
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from uv_sim_bringup.launch_common import (
    configure_simulator_gpu_environment,
    declare_feature_arguments,
    declare_mission_file,
    declare_observability_arguments,
    declare_profile,
    declare_simulation_arguments,
    profile_path,
    validate_profile,
)
from uv_sim_bringup.scene import prepare_scene


class _ConditionalTopic(Substitution):
    """Select a topic when the launch context resolves the feature flag."""

    def __init__(self, enabled, selected, default):
        self._condition = IfCondition(enabled)
        self._selected = selected
        self._default = default

    def perform(self, context):
        return (self._selected if self._condition.evaluate(context)
                else self._default)


def _include(package, launch_file, arguments, *, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(package), 'launch', launch_file,
        ])),
        launch_arguments=arguments.items(),
        condition=condition,
    )


def _profile_include(
    package, launch_file, arguments, *, profile, profile_package,
):
    """Resolve a profile path in the parent launch context."""
    def _create_include(context):
        resolved_arguments = dict(arguments)
        resolved_profile = profile_path(
            profile, profile_package).perform(context)
        resolved_arguments['profile_params'] = resolved_profile
        return [
            LogInfo(msg=[
                'Resolved ', package, ' profile: ', resolved_profile,
            ]),
            _include(package, launch_file, resolved_arguments),
        ]

    return OpaqueFunction(function=_create_include)


def _capture_profile(context):
    """Preserve the simulator profile before child launches reuse ``profile``."""
    context.launch_configurations['sim_profile'] = (
        context.launch_configurations.get('profile', ''))
    return []


def generate_launch_description():
    profile = LaunchConfiguration('profile')
    sim_profile = LaunchConfiguration('sim_profile')
    gpu = LaunchConfiguration('gpu')
    gpu_backend = LaunchConfiguration('gpu_backend')
    scenario = LaunchConfiguration('scenario_desc')
    seed = LaunchConfiguration('scene_seed')
    sim_rate = LaunchConfiguration('simulation_rate')
    sim_width = LaunchConfiguration('sim_window_width')
    sim_height = LaunchConfiguration('sim_window_height')
    render_quality = LaunchConfiguration('render_quality')
    render_fps = LaunchConfiguration('render_fps')
    enable_ai = LaunchConfiguration('enable_ai')
    enable_motion = LaunchConfiguration('enable_motion')
    enable_nav = LaunchConfiguration('enable_nav')
    enable_task = LaunchConfiguration('enable_task')
    estimator = LaunchConfiguration('estimator')
    enable_evaluation = LaunchConfiguration('enable_evaluation')
    enable_degradation = LaunchConfiguration('enable_degradation')
    evaluation_output_dir = LaunchConfiguration('evaluation_output_dir')
    evaluation_run_id = LaunchConfiguration('evaluation_run_id')
    degradation_seed = LaunchConfiguration('degradation_seed')
    dvl_dropout = LaunchConfiguration('dvl_dropout_probability')
    dvl_beam_loss = LaunchConfiguration('dvl_beam_loss_probability')
    dvl_noise = LaunchConfiguration('dvl_noise_stddev')
    dvl_altitude_noise = LaunchConfiguration('dvl_altitude_noise_stddev')
    dvl_bottom_lock_loss = LaunchConfiguration(
        'dvl_bottom_lock_loss_probability')
    dvl_delay = LaunchConfiguration('dvl_delay_ms')
    imu_dropout = LaunchConfiguration('imu_dropout_probability')
    imu_random_walk = LaunchConfiguration('imu_random_walk_stddev')
    visual_dropout = LaunchConfiguration('visual_dropout_probability')
    visual_brightness = LaunchConfiguration('visual_brightness_scale')
    visual_noise = LaunchConfiguration('visual_noise_stddev')
    visual_blur = LaunchConfiguration('visual_blur_kernel')
    usbl_dropout = LaunchConfiguration('usbl_dropout_probability')
    usbl_outlier = LaunchConfiguration('usbl_outlier_probability')
    usbl_outlier_stddev = LaunchConfiguration('usbl_outlier_stddev')
    camera_dir = LaunchConfiguration('camera_config_dir')
    mission_file = LaunchConfiguration('mission_file')

    dvl_input = _ConditionalTopic(
        enable_degradation, SIM_DEGRADED_DVL_VELOCITY, DVL_VELOCITY)
    imu_input = _ConditionalTopic(
        enable_degradation, SIM_DEGRADED_IMU, IMU)
    usbl_input = _ConditionalTopic(
        enable_degradation, SIM_DEGRADED_USBL, USBL_MEASUREMENT)
    front_image_input = _ConditionalTopic(
        enable_degradation, SIM_DEGRADED_FRONT_STITCHED,
        FRONT_STITCHED)
    down_image_input = _ConditionalTopic(
        enable_degradation, SIM_DEGRADED_DOWN_STITCHED, DOWN_STITCHED)

    description = _include('uv_sim_description', 'description.launch.py', {
        'use_sim_time': 'true',
    })
    stonefish_gpu = _include('stonefish_ros2', 'stonefish_simulator.launch.py', {
        'simulation_data': LaunchConfiguration('resolved_simulation_data'),
        'scenario_desc': LaunchConfiguration('resolved_scenario'),
        'simulation_rate': sim_rate, 'window_res_x': sim_width,
        'window_res_y': sim_height, 'rendering_quality': render_quality,
        'render_fps': render_fps,
    }, condition=IfCondition(gpu))
    stonefish_nogpu = _include(
        'stonefish_ros2', 'stonefish_simulator_nogpu.launch.py', {
            'simulation_data': LaunchConfiguration('resolved_simulation_data'),
            'scenario_desc': LaunchConfiguration('resolved_scenario'),
            'simulation_rate': sim_rate,
        }, condition=UnlessCondition(gpu))
    bridge = _profile_include('uv_sim_bridge', 'bridge.launch.py', {
        'hil_mode': 'false',
        'camera_stitch_fps': LaunchConfiguration('camera_stitch_fps'),
        'publish_raw_camera_topics': LaunchConfiguration('publish_raw_camera_topics'),
    }, profile=sim_profile, profile_package='uv_sim_bridge')
    localization = _include('uv_localization', 'localization_launch.py', {
        'sim_mode': 'true', 'publish_tf': 'true',
        'estimator': estimator,
        'dvl_topic': dvl_input, 'imu_topic': imu_input,
        'usbl_topic': usbl_input,
    })
    degradation = _include('uv_sim_bringup', 'degradation.launch.py', {
        'enable_degradation': enable_degradation,
        'degradation_seed': degradation_seed,
        'dvl_dropout_probability': dvl_dropout,
        'dvl_beam_loss_probability': dvl_beam_loss,
        'dvl_noise_stddev': dvl_noise,
        'dvl_altitude_noise_stddev': dvl_altitude_noise,
        'dvl_bottom_lock_loss_probability': dvl_bottom_lock_loss,
        'dvl_delay_ms': dvl_delay,
        'imu_dropout_probability': imu_dropout,
        'imu_random_walk_stddev': imu_random_walk,
        'visual_dropout_probability': visual_dropout,
        'visual_brightness_scale': visual_brightness,
        'visual_noise_stddev': visual_noise,
        'visual_blur_kernel': visual_blur,
        'usbl_dropout_probability': usbl_dropout,
        'usbl_outlier_probability': usbl_outlier,
        'usbl_outlier_stddev': usbl_outlier_stddev,
    })
    control = _include('uv_control', 'control_launch.py', {
        'enable_motion': enable_motion, 'sim_mode': 'true',
        'profile_params': '',
    })
    perception = _profile_include('uv_camera', 'perception_launch.py', {
        'enable_ai': enable_ai, 'sim_mode': 'true',
        'inference_fps': LaunchConfiguration('ai_inference_fps'),
        'inference_threads': LaunchConfiguration('inference_threads'),
        'confidence': LaunchConfiguration('ai_confidence'),
        'gate_feature_mode': LaunchConfiguration('gate_feature_mode'),
        'enable_gortc': LaunchConfiguration('enable_preview'),
        'stream_annotated': LaunchConfiguration('stream_annotated'),
        'camera_config_profile': 'sim', 'camera_config_dir': camera_dir,
        'front_image_topic': front_image_input,
        'down_image_topic': down_image_input,
        'object_localizer_params': PathJoinSubstitution([
            FindPackageShare('uv_camera'), 'config', 'object_localizer_sim.yaml',
        ]),
    }, profile=sim_profile, profile_package='uv_camera')
    planning = _include('uv_planning', 'planning_launch.py', {
        'enable_nav': enable_nav, 'profile_params': '',
    })
    task = _include('uv_task', 'task_launch.py', {
        'enable_task': enable_task,
        'target_id': LaunchConfiguration('target_id'),
        'camera_config_profile': 'sim', 'camera_config_dir': camera_dir,
        'profile_params': '', 'mission_file': mission_file,
    })
    observability = _include('uv_bringup', 'observability.launch.py', {
        'enable_preview': LaunchConfiguration('enable_preview'),
        'preview_port': LaunchConfiguration('preview_port'),
        'record_session': LaunchConfiguration('record_session'),
        'record_root': LaunchConfiguration('record_root'),
    })
    evaluation = Node(
        package='uv_sim_evaluation', executable='evaluator',
        name='uv_sim_evaluation', exec_name='uv_sim_evaluation', output='both',
        parameters=[{
            'seed': seed,
            'output_dir': evaluation_output_dir,
            'run_id': evaluation_run_id,
        }], condition=IfCondition(enable_evaluation),
    )

    return LaunchDescription([
        declare_profile('sim', 'sim_dev'), declare_mission_file(),
        *declare_feature_arguments(
            enable_ai='true', enable_nav='false', enable_task='false',
            enable_motion='true'),
        *declare_simulation_arguments(), *declare_observability_arguments(),
        DeclareLaunchArgument('enable_evaluation', default_value='true'),
        DeclareLaunchArgument(
            'estimator', default_value='bootstrap', choices=['bootstrap'],
            description='Localization backend; V1 currently provides bootstrap'),
        DeclareLaunchArgument('enable_degradation', default_value='false'),
        DeclareLaunchArgument('evaluation_output_dir', default_value=''),
        DeclareLaunchArgument('evaluation_run_id', default_value=''),
        DeclareLaunchArgument('degradation_seed', default_value='0'),
        DeclareLaunchArgument('dvl_dropout_probability', default_value='0.0'),
        DeclareLaunchArgument('dvl_beam_loss_probability', default_value='0.0'),
        DeclareLaunchArgument('dvl_noise_stddev', default_value='0.0'),
        DeclareLaunchArgument(
            'dvl_altitude_noise_stddev', default_value='0.0'),
        DeclareLaunchArgument(
            'dvl_bottom_lock_loss_probability', default_value='0.0'),
        DeclareLaunchArgument('dvl_delay_ms', default_value='0.0'),
        DeclareLaunchArgument('imu_dropout_probability', default_value='0.0'),
        DeclareLaunchArgument('imu_random_walk_stddev', default_value='0.0'),
        DeclareLaunchArgument('visual_dropout_probability', default_value='0.0'),
        DeclareLaunchArgument('visual_brightness_scale', default_value='1.0'),
        DeclareLaunchArgument('visual_noise_stddev', default_value='0.0'),
        DeclareLaunchArgument('visual_blur_kernel', default_value='0'),
        DeclareLaunchArgument('usbl_dropout_probability', default_value='0.0'),
        DeclareLaunchArgument('usbl_outlier_probability', default_value='0.0'),
        DeclareLaunchArgument('usbl_outlier_stddev', default_value='0.0'),
        DeclareLaunchArgument('camera_config_dir', default_value=''),
        configure_simulator_gpu_environment(gpu, gpu_backend),
        validate_profile(profile, 'sim'),
        OpaqueFunction(function=_capture_profile),
        LogInfo(msg=['Simulation profile: ', sim_profile]),
        prepare_scene(
            scenario_desc=scenario, scene_seed=seed, launch_file=__file__,
            start_actions=[description, localization, stonefish_gpu,
                           stonefish_nogpu, bridge, degradation,
                           control, perception, planning, task,
                           observability, evaluation]),
    ])
