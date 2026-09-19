"""Minimal Stonefish + sensor bridge entry point."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('uv_sim_bringup'), 'launch', 'sim.launch.py'])),
        launch_arguments={
            'scenario_desc': 'underwater_xunyun.scn',
            'enable_ai': 'false', 'enable_motion': 'false',
            'enable_nav': 'false', 'enable_task': 'false',
            'enable_evaluation': 'false', 'enable_preview': 'false',
        }.items(),
    )])
