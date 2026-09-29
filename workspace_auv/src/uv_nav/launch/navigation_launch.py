"""Launch the navigation component."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_nav = LaunchConfiguration("enable_nav")
    params_file = LaunchConfiguration("params_file")

    def _nodes(context):
        parameters = []
        parameter_file = params_file.perform(context).strip()
        if parameter_file:
            parameters.append(parameter_file)
        return [Node(
            package="uv_nav",
            executable="navigator",
            name="navigator",
            exec_name="navigator",
            output="both",
            parameters=parameters,
            remappings=[('/tf', '/auv/tf'), ('/tf_static', '/auv/tf_static')],
            condition=IfCondition(enable_nav),
        )]

    return LaunchDescription([
        DeclareLaunchArgument("enable_nav", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=""),
        OpaqueFunction(function=_nodes),
    ])
