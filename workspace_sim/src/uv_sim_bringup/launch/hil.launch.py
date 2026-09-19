"""Stonefish HIL orchestration owned by ``workspace_sim``."""

from __future__ import annotations

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from uv_sim_bringup.launch_common import (
    configure_simulator_gpu_environment,
    declare_feature_arguments,
    declare_mission_file,
    declare_profile,
    declare_simulation_arguments,
    profile_path,
    validate_profile,
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
    """Preserve the HIL profile before child launches reuse ``profile``."""
    context.launch_configurations['hil_profile'] = (
        context.launch_configurations.get('profile', ''))
    return []


def _agent_default():
    candidate = (Path.home() / 'micro_ros_agent_ws' / 'install' /
                 'micro_ros_agent' / 'lib' / 'micro_ros_agent' /
                 'micro_ros_agent')
    return str(candidate) if candidate.is_file() else 'micro_ros_agent'


def generate_launch_description():
    profile = LaunchConfiguration('profile')
    hil_profile = LaunchConfiguration('hil_profile')
    scenario = LaunchConfiguration('scenario_desc')
    seed = LaunchConfiguration('scene_seed')
    camera_dir = LaunchConfiguration('camera_config_dir')
    mission_file = LaunchConfiguration('mission_file')

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
    localization = _include('uv_localization', 'localization_launch.py', {
        'sim_mode': 'true', 'publish_tf': 'true',
    })
    bridge = _profile_include('uv_sim_bridge', 'bridge.launch.py', {
        'hil_mode': 'true',
        'camera_stitch_fps': LaunchConfiguration('camera_stitch_fps'),
        'publish_raw_camera_topics': LaunchConfiguration('publish_raw_camera_topics'),
    }, profile=hil_profile, profile_package='uv_sim_bridge')
    control = _include('uv_control', 'control_launch.py', {
        'enable_motion': LaunchConfiguration('enable_motion'),
        'sim_mode': 'true', 'profile_params': '',
    })
    perception = _profile_include('uv_camera', 'perception_launch.py', {
        'enable_ai': LaunchConfiguration('enable_ai'), 'sim_mode': 'true',
        'camera_config_profile': 'sim', 'camera_config_dir': camera_dir,
    }, profile=hil_profile, profile_package='uv_camera')
    planning = _include('uv_planning', 'planning_launch.py', {
        'enable_nav': LaunchConfiguration('enable_nav'), 'profile_params': '',
    })
    task = _include('uv_task', 'task_launch.py', {
        'enable_task': LaunchConfiguration('enable_task'),
        'target_id': LaunchConfiguration('target_id'),
        'camera_config_profile': 'sim', 'camera_config_dir': camera_dir,
        'profile_params': '', 'mission_file': mission_file,
    })
    agent = ExecuteProcess(
        cmd=[LaunchConfiguration('agent_executable'), 'serial', '-D',
             LaunchConfiguration('serial_dev'), '-b',
             LaunchConfiguration('serial_baud'), '-v', '4'],
        name='micro_ros_agent', output='both')

    return LaunchDescription([
        declare_profile('hil', 'hil_lab'), declare_mission_file(),
        *declare_feature_arguments(
            enable_ai='false', enable_nav='false', enable_task='false',
            enable_motion='false'),
        *declare_simulation_arguments(
            scenario_default='underwater_xunyun.scn',
            window_width_default='1280', window_height_default='720',
            render_quality_default='high', camera_stitch_fps_default='10.0'),
        configure_simulator_gpu_environment(
            LaunchConfiguration('gpu'), LaunchConfiguration('gpu_backend')),
        DeclareLaunchArgument('camera_config_dir', default_value=''),
        DeclareLaunchArgument('serial_dev', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('serial_baud', default_value='921600'),
        DeclareLaunchArgument('agent_executable', default_value=_agent_default()),
        validate_profile(profile, 'hil'),
        OpaqueFunction(function=_capture_profile),
        LogInfo(msg=['HIL profile: ', hil_profile]),
        prepare_scene(
            scenario_desc=scenario, scene_seed=seed, launch_file=__file__,
            start_actions=[description, localization, stonefish_gpu,
                           stonefish_nogpu, bridge,
                           agent, control, perception, planning, task]),
    ])
