"""Core simulation bringup — Stonefish + sim_bridge only.

No perception, navigation, control, or task nodes.
Use this to validate the UUV2025 migration (serial→zit6 topics) against simulation.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_scenario = DeclareLaunchArgument(
        'scenario', default_value='underwater_xunyun.scn',
        description='Stonefish scenario file name (in Data/ directory)'
    )
    declare_gpu = DeclareLaunchArgument(
        'gpu', default_value='true',
        description='Enable GPU rendering window (true/false)'
    )

    scenario = LaunchConfiguration('scenario')
    gpu = LaunchConfiguration('gpu')

    # Resolve Stonefish Data directory from the stonefish_ros2 package
    from ament_index_python.packages import get_package_share_directory
    stonefish_share = get_package_share_directory('stonefish_ros2')
    workspace_root = os.path.abspath(os.path.join(stonefish_share, '..', '..', '..', '..'))
    stonefish_source_dir = os.path.join(workspace_root, 'src', 'stonefish_ros2')
    simulation_data_dir = os.path.join(stonefish_source_dir, 'Data')

    stonefish_sim = Node(
        package='stonefish_ros2',
        executable='stonefish_simulator',
        namespace='stonefish_ros2',
        name='stonefish_simulator',
        arguments=[
            simulation_data_dir,
            os.path.join(simulation_data_dir, 'underwater_xunyun.scn'),
            '100.0',
            '1280',
            '720',
            'high',
        ],
        output='screen',
    )

    sim_bridge = Node(
        package='uv_sim',
        executable='sim_bridge',
        name='sim_bridge',
        output='screen',
    )

    return LaunchDescription([
        declare_scenario,
        declare_gpu,
        LogInfo(msg=['Simulation data dir: ', simulation_data_dir]),
        LogInfo(msg=['Scenario: ', scenario]),
        stonefish_sim,
        sim_bridge,
    ])
