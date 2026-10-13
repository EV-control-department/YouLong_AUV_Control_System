"""uv_mapping entry point: the line-laser mapping node.

The node list belongs here so the system presets never declare nodes themselves.
Mapping is deliberately **not** wired into ``sim_bringup``/``real_bringup``: a
survey recorder is something you start for a survey, not part of the standing
vehicle stack, and the offline half of this package (``map_replay``) is a CLI
that needs no graph at all.

``map_mode:=live`` is the only mode that publishes (``/mapping/bathymetry``);
``monitor`` subscribes and prints record-time QA without touching the graph.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_mode', default_value='monitor',
            description=(
                'monitor: subscribe and print record-time QA only; '
                'live: also accumulate and publish /mapping/bathymetry '
                '(not yet validated, see docs)'
            )),
        DeclareLaunchArgument(
            'map_leaf_size', default_value='0.05',
            description='map cell size in metres (0.05 = 5 cm)'),
        DeclareLaunchArgument(
            'output_dir', default_value='',
            description=(
                'directory for the end-of-run QA summary JSON; empty writes '
                'nothing to disk'
            )),

        Node(
            package='uv_mapping',
            executable='laser_map',
            name='laser_map',
            output='screen',
            parameters=[{
                'map_mode': LaunchConfiguration('map_mode'),
                'map_leaf_size': LaunchConfiguration('map_leaf_size'),
                'output_dir': LaunchConfiguration('output_dir'),
            }],
        ),
    ])
