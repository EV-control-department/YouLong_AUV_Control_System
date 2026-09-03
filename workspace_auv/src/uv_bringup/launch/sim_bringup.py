"""Simulation bringup launch file.

Launches Stonefish simulator + all control/perception/nav/task nodes.
"""

import os
import shutil
import subprocess
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _workspace_python_runtime():
    """Find the repository-local Python runtime, if it was bootstrapped."""
    from pathlib import Path

    candidates = []
    launch_file = Path(__file__).resolve()
    for parent in (launch_file.parent, *launch_file.parents):
        candidates.append(parent / '.venv' / 'bin' / 'python')

    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return str(candidate)
    return None


def _nvidia_available() -> bool:
    """Return True only when an NVIDIA GPU is visible to the installed driver."""
    nvidia_smi = shutil.which('nvidia-smi')
    if nvidia_smi is None:
        return False

    try:
        result = subprocess.run(
            [nvidia_smi, '--query-gpu=name', '--format=csv,noheader'],
            check=True,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
        return bool(result.stdout.strip())
    except (OSError, subprocess.SubprocessError):
        return False


def generate_launch_description():
    # Arguments
    declare_enable_ai = DeclareLaunchArgument(
        'enable_ai', default_value='true',
        description='Enable AI perception nodes'
    )
    declare_enable_nav = DeclareLaunchArgument(
        'enable_nav', default_value='false',
        description='Enable navigation node'
    )
    declare_enable_task = DeclareLaunchArgument(
        'enable_task', default_value='false',
        description='Enable task runner'
    )
    declare_scenario = DeclareLaunchArgument(
        'scenario_desc', default_value='guoshui_2026_cruise_seeded.scn',
        description='Stonefish scenario file name (in Data/ directory); generated from scene_seed by default'
    )
    declare_scene_seed = DeclareLaunchArgument(
        'scene_seed', default_value='0',
        description='Deterministic Guoshui scene seed; 0 keeps the fixed baseline layout'
    )
    declare_target_id = DeclareLaunchArgument(
        'target_id', default_value='yellow_golf',
        description='Competition target metadata: yellow_golf, pink_golf, or red_ring'
    )
    enable_ai = LaunchConfiguration('enable_ai')
    enable_nav = LaunchConfiguration('enable_nav')
    enable_task = LaunchConfiguration('enable_task')
    scenario_desc = LaunchConfiguration('scenario_desc')
    scene_seed = LaunchConfiguration('scene_seed')
    target_id = LaunchConfiguration('target_id')

    # Python ROS nodes that touch images must use the workspace-local NumPy
    # runtime. Otherwise a user-site NumPy 2.x can be selected before ROS 2
    # Jazzy's NumPy 1.x-built cv_bridge and crash sim_bridge.
    workspace_python = _workspace_python_runtime()
    python_node_kwargs = {'prefix': workspace_python} if workspace_python else {}

    # Stonefish simulator paths
    # Use source directory path for Data (simulator needs direct filesystem access)
    from ament_index_python.packages import get_package_share_directory
    stonefish_share = get_package_share_directory('stonefish_ros2')

    # 定位 stonefish_ros2 源码的 Data/ 目录。仓库是"真机workspace_auv + 仿真workspace_sim"
    # 两层布局；不把具体用户目录写死，改为从启动文件和已安装包路径向上查找。
    from pathlib import Path as _Path

    def _find_stonefish_data_dir() -> str:
        candidates = []
        roots = (_Path(__file__).resolve(), _Path(stonefish_share).resolve())
        for root in roots:
            for base in (root, *root.parents):
                candidates.append(base / 'workspace_sim' / 'src' / 'stonefish_ros2' / 'Data')
                candidates.append(base / 'src' / 'stonefish_ros2' / 'Data')

        # 去重，同时保持从当前源码/安装位置向外查找的顺序。
        candidates = list(dict.fromkeys(candidates))
        for c in candidates:
            if c.is_dir() and any(c.glob('*.scn')):
                return str(c)
        raise RuntimeError("无法定位 stonefish_ros2 的 Data 源码目录")

    simulation_data_dir = _find_stonefish_data_dir()
    stonefish_source_dir = str(_Path(simulation_data_dir).parent)

    def _generate_seeded_scene(context):
        seed_text = scene_seed.perform(context)
        try:
            int(seed_text)
        except ValueError as error:
            raise RuntimeError(f'scene_seed must be an integer, got {seed_text!r}') from error

        generator = _Path(simulation_data_dir) / 'generate_guoshui_2026_scene.py'
        template = _Path(simulation_data_dir) / 'guoshui_2026_cruise.scn'
        output = _Path(simulation_data_dir) / 'guoshui_2026_cruise_seeded.scn'
        subprocess.run(
            [
                sys.executable, str(generator),
                '--seed', seed_text,
                '--template', str(template),
                '--output', str(output),
            ],
            check=True,
        )
        return [LogInfo(msg=[
            'Generated Guoshui scene with seed ', seed_text,
            ': ', str(output),
        ])]

    # Build the stonefish simulator node (GPU version — with rendering window)
    # The simulator expects: simulation_data, scenario_desc, rate, res_x, res_y, quality
    stonefish_sim = Node(
        package='stonefish_ros2',
        executable='stonefish_simulator',
        namespace='stonefish_ros2',
        name='stonefish_simulator',
        arguments=[
            simulation_data_dir,
            PathJoinSubstitution([simulation_data_dir, scenario_desc]),
            '100.0',
            '1600',
            '900',
            'high',
        ],
        output='screen',
    )

    # Core nodes
    sim_bridge = Node(
        package='uv_sim',
        executable='sim_bridge',
        name='sim_bridge',
        output='screen',
        **python_node_kwargs,
    )

    basic_motion = Node(
        package='uv_control',
        executable='basic_motion',
        name='basic_motion',
        output='screen',
        **python_node_kwargs,
    )

    # Perception: uv_camera node (uv_sensor + uv_ai, same process) + object_localizer
    vision = Node(
        package='uv_camera',
        executable='uv_camera',
        name='uv_camera',
        output='screen',
        **python_node_kwargs,
        parameters=[{'sim_mode': True}],
        condition=IfCondition(enable_ai),
    )

    object_localizer = Node(
        package='uv_camera',
        executable='object_localizer',
        name='object_localizer',
        output='screen',
        respawn=True,
        respawn_delay=1.0,
        **python_node_kwargs,
        # Build simulated profiles from Stonefish CameraInfo and these camera
        # origins, so the P2 baseline sign follows the simulator optical axes.
        parameters=[{
            'calibration_source': 'sim_camera_info',
            'front_left_camera_info_topic':
                '/sim/front_cam/left/camera_info',
            'front_right_camera_info_topic':
                '/sim/front_cam/right/camera_info',
            'down_left_camera_info_topic':
                '/sim/down_cam/left/camera_info',
            'down_right_camera_info_topic':
                '/sim/down_cam/right/camera_info',
            'front_left_translation': [0.23, -0.05, 0.276],
            'front_right_translation': [0.23, 0.05, 0.276],
            # Stonefish ColorCamera: local +Z is forward, +X is image-right,
            # +Y is image-down.  The front sensor rpy is 1.5708,0,1.5708.
            'front_left_rotation': [0.0, 0.0, 1.0,
                                    1.0, 0.0, 0.0,
                                    0.0, 1.0, 0.0],
            'front_right_rotation': [0.0, 0.0, 1.0,
                                     1.0, 0.0, 0.0,
                                     0.0, 1.0, 0.0],
            'down_left_translation': [-0.13, -0.05, 0.2645],
            'down_right_translation': [-0.13, 0.05, 0.2645],
            'use_rejected_front_pairs_for_multiview': True,
            'front_observation_pool_size': 2000,
            'front_direct_queue_size': 50,
            'front_duplicate_merge_distance_m': 0.25,
            'down_observation_pool_size': 2000,
            'down_direct_queue_size': 50,
            'guide_line_min_spacing_m': 0.5,
            'down_duplicate_merge_distance_m': 0.25,
        }],
        condition=IfCondition(enable_ai),
    )

    # Navigation node (optional)
    navigator = Node(
        package='uv_nav',
        executable='navigator',
        name='navigator',
        output='screen',
        **python_node_kwargs,
        condition=IfCondition(enable_nav),
    )

    # Task runner (optional)
    task_runner = Node(
        package='uv_task',
        executable='task_runner',
        name='task_runner',
        output='screen',
        **python_node_kwargs,
        parameters=[{'target_id': target_id}],
        condition=IfCondition(enable_task),
    )

    render_environment = []
    if _nvidia_available():
        render_environment = [
            SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
            SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
        ]

    return LaunchDescription([
        declare_enable_ai,
        declare_enable_nav,
        declare_enable_task,
        declare_scenario,
        declare_scene_seed,
        declare_target_id,
        # 仅在检测到可用 NVIDIA GPU 时启用 NVIDIA OpenGL/PRIME 渲染。
        *render_environment,
        OpaqueFunction(function=_generate_seeded_scene),
        LogInfo(msg=['Simulation data: ', simulation_data_dir]),
        LogInfo(msg=['Scenario: ', PathJoinSubstitution([simulation_data_dir, scenario_desc])]),
        stonefish_sim,
        sim_bridge,
        basic_motion,
        vision,
        object_localizer,
        navigator,
        task_runner,
    ])
