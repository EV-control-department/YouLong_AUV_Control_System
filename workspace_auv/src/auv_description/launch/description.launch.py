"""Publish the fixed AUV sensor tree on the canonical /auv TF topics."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    def _node(context):
        urdf_path = (
            Path(get_package_share_directory('auv_description')) /
            'urdf' / 'auv.urdf')
        return [Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[{
                'robot_description': urdf_path.read_text(encoding='utf-8'),
                'use_sim_time': use_sim_time,
            }],
            remappings=[('/tf', '/auv/tf'), ('/tf_static', '/auv/tf_static')],
        )]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_node),
    ])
