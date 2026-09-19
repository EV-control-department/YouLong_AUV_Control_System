"""Launch the vehicle motion-control component."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_motion = LaunchConfiguration("enable_motion")
    profile_params = LaunchConfiguration("profile_params")
    sim_mode = LaunchConfiguration("sim_mode")

    def _nodes(context):
        parameters = []
        profile = profile_params.perform(context).strip()
        if profile:
            parameters.append(profile)
        return [Node(
            package="uv_control",
            executable="basic_motion",
            name="basic_motion",
            exec_name="basic_motion",
            output="both",
            parameters=parameters + [{'sim_mode': sim_mode}],
            remappings=[('/tf', '/auv/tf'), ('/tf_static', '/auv/tf_static')],
            condition=IfCondition(enable_motion),
        )]

    return LaunchDescription([
        DeclareLaunchArgument("enable_motion", default_value="true"),
        DeclareLaunchArgument("sim_mode", default_value="false"),
        DeclareLaunchArgument("profile_params", default_value=""),
        OpaqueFunction(function=_nodes),
    ])
