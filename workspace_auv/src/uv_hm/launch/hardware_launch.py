"""Launch the real vehicle hardware manager."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enable_hardware = LaunchConfiguration("enable_hardware")
    params_file = LaunchConfiguration("params_file")

    def _nodes(context):
        parameters = []
        parameter_file = params_file.perform(context).strip()
        if parameter_file:
            parameters.append(parameter_file)
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
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("uv_hm"), "config", "default.yaml",
            ]),
            description="Hardware manager default parameters",
        ),
        OpaqueFunction(function=_nodes),
    ])
