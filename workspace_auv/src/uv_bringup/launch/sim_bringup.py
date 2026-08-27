"""Simulation bringup launch file.

Launches Stonefish simulator + all control/perception/nav/task nodes.
"""

import os
import shutil
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


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
        'scenario_desc', default_value='guoshui_2026_cruise.scn',
        description='Stonefish scenario file name (in Data/ directory)'
    )
    declare_target_id = DeclareLaunchArgument(
        'target_id', default_value='yellow_golf',
        description='Competition target metadata: yellow_golf, pink_golf, or red_ring'
    )
    enable_ai = LaunchConfiguration('enable_ai')
    enable_nav = LaunchConfiguration('enable_nav')
    enable_task = LaunchConfiguration('enable_task')
    scenario_desc = LaunchConfiguration('scenario_desc')
    target_id = LaunchConfiguration('target_id')

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
    )

    basic_motion = Node(
        package='uv_control',
        executable='basic_motion',
        name='basic_motion',
        output='screen',
    )

    # Perception: uv_camera node (uv_sensor + uv_ai, same process) + position
    vision = Node(
        package='uv_camera',
        executable='uv_camera',
        name='uv_camera',
        output='screen',
        parameters=[{'sim_mode': True}],
        condition=IfCondition(enable_ai),
    )

    position = Node(
        package='uv_camera',
        executable='position',
        name='position',
        output='screen',
        condition=IfCondition(enable_ai),
    )

    # Navigation node (optional)
    navigator = Node(
        package='uv_nav',
        executable='navigator',
        name='navigator',
        output='screen',
        condition=IfCondition(enable_nav),
    )

    # Task runner (optional)
    task_runner = Node(
        package='uv_task',
        executable='task_runner',
        name='task_runner',
        output='screen',
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
        declare_target_id,
        # 仅在检测到可用 NVIDIA GPU 时启用 NVIDIA OpenGL/PRIME 渲染。
        *render_environment,
        LogInfo(msg=['Simulation data: ', simulation_data_dir]),
        LogInfo(msg=['Scenario: ', PathJoinSubstitution([simulation_data_dir, scenario_desc])]),
        stonefish_sim,
        sim_bridge,
        basic_motion,
        vision,
        position,
        navigator,
        task_runner,
    ])
