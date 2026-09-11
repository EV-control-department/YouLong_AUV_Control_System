"""Launch the navigation component."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_nav = LaunchConfiguration("enable_nav")
    profile_params = LaunchConfiguration("profile_params")

    def _nodes(context):
        parameters = []
        profile = profile_params.perform(context).strip()
        if profile:
            parameters.append(profile)
        return [Node(
            package="uv_nav",
            executable="navigator",
            name="navigator",
            exec_name="navigator",
            output="both",
            parameters=parameters,
            condition=IfCondition(enable_nav),
        )]

    return LaunchDescription([
        DeclareLaunchArgument("enable_nav", default_value="true"),
        DeclareLaunchArgument("profile_params", default_value=""),
        OpaqueFunction(function=_nodes),
    ])
