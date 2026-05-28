"""Simulation bringup launch file.

Launches Stonefish simulator + all control/perception/nav/task nodes.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Arguments
    declare_enable_ai = DeclareLaunchArgument(
        'enable_ai', default_value='true',
        description='Enable AI perception nodes'
    )
    declare_enable_nav = DeclareLaunchArgument(
        'enable_nav', default_value='true',
        description='Enable navigation node'
    )
    declare_enable_task = DeclareLaunchArgument(
        'enable_task', default_value='false',
        description='Enable task runner'
    )
    declare_scenario = DeclareLaunchArgument(
        'scenario_desc', default_value='underwater_xunyun.scn',
        description='Stonefish scenario file name (in Data/ directory)'
    )

    enable_ai = LaunchConfiguration('enable_ai')
    enable_nav = LaunchConfiguration('enable_nav')
    enable_task = LaunchConfiguration('enable_task')

    # Stonefish simulator paths
    # Use source directory path for Data (simulator needs direct filesystem access)
    from ament_index_python.packages import get_package_share_directory
    stonefish_share = get_package_share_directory('stonefish_ros2')
    workspace_root = os.path.abspath(os.path.join(stonefish_share, '..', '..', '..', '..'))
    stonefish_source_dir = os.path.join(workspace_root, 'src', 'stonefish_ros2')
    simulation_data_dir = os.path.join(stonefish_source_dir, 'Data')

    # Build the stonefish simulator node directly with absolute paths
    # The simulator expects: simulation_data, scenario_desc, rate, width, height, quality
    stonefish_sim = Node(
        package='stonefish_ros2',
        executable='stonefish_simulator',
        namespace='stonefish_ros2',
        name='stonefish_simulator',
        arguments=[
            simulation_data_dir,
            os.path.join(simulation_data_dir, 'underwater_xunyun.scn'),
            '100.0',
            '1920',
            '1080',
            'high',
        ],
        output='screen',
    )

    # Core nodes
    sim_bridge = Node(
        package='uv_hm',
        executable='sim_bridge',
        name='sim_bridge',
        output='screen',
    )

    minimal_control = Node(
        package='uv_control',
        executable='minimal_control',
        name='minimal_control',
        output='screen',
    )

    basic_motion = Node(
        package='uv_control',
        executable='basic_motion',
        name='basic_motion',
        output='screen',
    )

    # Perception nodes (optional)
    vision = Node(
        package='uv_perception',
        executable='vision',
        name='vision',
        output='screen',
        condition=IfCondition(enable_ai),
    )

    position = Node(
        package='uv_perception',
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
        condition=IfCondition(enable_task),
    )

    return LaunchDescription([
        declare_enable_ai,
        declare_enable_nav,
        declare_enable_task,
        declare_scenario,
        LogInfo(msg=['Simulation data: ', simulation_data_dir]),
        LogInfo(msg=['Scenario: ', os.path.join(simulation_data_dir, 'underwater_xunyun.scn')]),
        stonefish_sim,
        sim_bridge,
        minimal_control,
        basic_motion,
        vision,
        position,
        navigator,
        task_runner,
    ])
