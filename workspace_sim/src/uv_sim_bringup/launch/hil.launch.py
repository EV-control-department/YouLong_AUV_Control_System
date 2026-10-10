"""Stonefish HIL orchestration owned by ``workspace_sim``."""

from __future__ import annotations

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from uv_sim_bringup.launch_common import (
    configure_simulator_gpu_environment,
    declare_feature_arguments,
    declare_profile,
    declare_mission_file,
    declare_observability_arguments,
    declare_simulation_arguments,
    release_tasks_after_backend_ready,
)
from uv_sim_bringup.scene import prepare_scene


def _include(package, launch_file, arguments, *, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(package), 'launch', launch_file,
        ])),
        launch_arguments=arguments.items(),
        condition=condition,
    )


def _agent_default():
    candidate = (Path.home() / 'micro_ros_agent_ws' / 'install' /
                 'micro_ros_agent' / 'lib' / 'micro_ros_agent' /
                 'micro_ros_agent')
    return str(candidate) if candidate.is_file() else 'micro_ros_agent'


def generate_launch_description():
    profile = LaunchConfiguration('profile')
    scenario = LaunchConfiguration('scenario_desc')
    seed = LaunchConfiguration('scene_seed')
    camera_dir = LaunchConfiguration('camera_config_dir')
    mission_file = LaunchConfiguration('mission_file')
    enable_stream = LaunchConfiguration('enable_stream')

    model_mapping = _include('uv_perception', 'model_mapping_launch.py', {})
    description = _include('uv_sim_description', 'description.launch.py', {
        'use_sim_time': 'true',
    })
    stonefish_gpu = _include('stonefish_ros2', 'stonefish_simulator.launch.py', {
        'simulation_data': LaunchConfiguration('resolved_simulation_data'),
        'scenario_desc': LaunchConfiguration('resolved_scenario'),
        'simulation_rate': LaunchConfiguration('simulation_rate'),
        'window_res_x': LaunchConfiguration('sim_window_width'),
        'window_res_y': LaunchConfiguration('sim_window_height'),
        'rendering_quality': LaunchConfiguration('render_quality'),
        'render_fps': LaunchConfiguration('render_fps'),
    }, condition=IfCondition(LaunchConfiguration('gpu')))
    stonefish_nogpu = _include(
        'stonefish_ros2', 'stonefish_simulator_nogpu.launch.py', {
            'simulation_data': LaunchConfiguration('resolved_simulation_data'),
            'scenario_desc': LaunchConfiguration('resolved_scenario'),
            'simulation_rate': LaunchConfiguration('simulation_rate'),
        }, condition=UnlessCondition(LaunchConfiguration('gpu')))
    hardware = _include('uv_hm', 'hardware_launch.py', {
        'enable_hardware': 'true',
    })
    localization = _include('uv_localization', 'localization_launch.py', {
        'sim_mode': 'true', 'publish_tf': 'true',
    })
    bridge = _include('uv_sim_bridge', 'bridge.launch.py', {
        'hil_mode': 'true',
        'camera_stitch_fps': LaunchConfiguration('camera_stitch_fps'),
        'publish_raw_camera_topics': 'false',
    })
    control = _include('uv_control', 'control_launch.py', {
        'enable_motion': LaunchConfiguration('enable_motion'),
        'sim_mode': 'true', 'params_file': '',
    })
    camera = _include('uv_camera', 'camera_launch.py', {
        'sim_mode': 'true', 'camera_mode': 'sim',
        'camera_config_dir': camera_dir,
    })
    perception = _include('uv_perception', 'perception_launch.py', {
        'front_model_path': LaunchConfiguration('front_model_path'),
        'down_model_path': LaunchConfiguration('down_model_path'),
    },
                          condition=IfCondition(LaunchConfiguration('enable_ai')))
    stream = _include(
        'uv_stream', 'stream_launch.py', {},
        condition=IfCondition(enable_stream))
    observability = _include('uv_bringup', 'observability.launch.py', {
        'enable_preview': LaunchConfiguration('enable_preview'),
        'preview_port': LaunchConfiguration('preview_port'),
        'record_session': LaunchConfiguration('record_session'),
        'record_root': LaunchConfiguration('record_root'),
        'record_mode': LaunchConfiguration('record_mode'),
        'go2rtc_stream_mode': LaunchConfiguration('go2rtc_stream_mode'),
        'go2rtc_video_format': LaunchConfiguration('go2rtc_video_format'),
        'record_video_fps': LaunchConfiguration('record_video_fps'),
        'record_video_codec': LaunchConfiguration('record_video_codec'),
        'video_segment_seconds': LaunchConfiguration('video_segment_seconds'),
        'bag_segment_seconds': LaunchConfiguration('bag_segment_seconds'),
        'record_bag_storage': LaunchConfiguration('record_bag_storage'),
        'record_use_sim_time': LaunchConfiguration('record_use_sim_time'),
        'record_image_topics': LaunchConfiguration('record_image_topics'),
    })
    planning = _include('uv_planning', 'planning_launch.py', {
        'enable_nav': LaunchConfiguration('enable_nav'), 'params_file': '',
    })
    task = _include('uv_task', 'task_launch.py', {
        'enable_task': LaunchConfiguration('enable_task'),
        'camera_mode': 'sim', 'camera_config_dir': camera_dir,
        'params_file': '', 'mission_file': mission_file,
        'auto_start': 'true',
    })
    task_startup = release_tasks_after_backend_ready(
        task, LaunchConfiguration('enable_task'), LaunchConfiguration('startup_timeout'))
    agent = ExecuteProcess(
        cmd=[LaunchConfiguration('agent_executable'), 'serial', '-D',
             LaunchConfiguration('serial_dev'), '-b',
             LaunchConfiguration('serial_baud'), '-v', '4'],
        name='micro_ros_agent', output='both')

    return LaunchDescription([
        *declare_profile(
            'hil', 'uv_sim_bringup',
            choices=('default', 'record', 'debug', 'task'),
        ),
        declare_mission_file(),
        *declare_feature_arguments(
            enable_ai='false', enable_nav='false', enable_task='false',
            enable_motion='false'),
        *declare_simulation_arguments(
            scenario_default='underwater_xunyun.scn',
            window_width_default='1280', window_height_default='720',
            render_quality_default='high', camera_stitch_fps_default='10.0'),
        *declare_observability_arguments(),
        DeclareLaunchArgument('enable_stream', default_value='true'),
        configure_simulator_gpu_environment(
            LaunchConfiguration('gpu'), LaunchConfiguration('gpu_backend')),
        DeclareLaunchArgument('camera_config_dir', default_value=''),
        DeclareLaunchArgument('front_model_path', default_value=''),
        DeclareLaunchArgument('down_model_path', default_value=''),
        DeclareLaunchArgument('serial_dev', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('serial_baud', default_value='921600'),
        DeclareLaunchArgument('agent_executable', default_value=_agent_default()),
        LogInfo(msg=['HIL profile: ', profile]),
        prepare_scene(
            scenario_desc=scenario, scene_seed=seed, launch_file=__file__,
            start_actions=[model_mapping, description, localization, stonefish_gpu,
                           stonefish_nogpu, bridge,
                           agent, hardware, control, camera, perception, stream,
                           planning, *task_startup, observability]),
    ])
