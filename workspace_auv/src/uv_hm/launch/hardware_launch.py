"""Launch the real vehicle hardware manager."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_hardware = LaunchConfiguration("enable_hardware")
    profile_params = LaunchConfiguration("profile_params")

    def _nodes(context):
        parameters = []
        profile = profile_params.perform(context).strip()
        if profile:
            parameters.append(profile)
        return [Node(
            package="uv_hm",
            executable="hw_manager",
            name="hw_manager",
            exec_name="hw_manager",
            output="both",
            parameters=parameters,
            condition=IfCondition(enable_hardware),
        )]

    return LaunchDescription([
        DeclareLaunchArgument("enable_hardware", default_value="true"),
        DeclareLaunchArgument("profile_params", default_value=""),
        OpaqueFunction(function=_nodes),
    ])
