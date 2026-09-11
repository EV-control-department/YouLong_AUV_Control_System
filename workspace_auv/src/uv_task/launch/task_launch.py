"""Launch the mission task runner component."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enable_task = LaunchConfiguration("enable_task")
    target_id = LaunchConfiguration("target_id")
    profile_params = LaunchConfiguration("profile_params")
    mission_file = LaunchConfiguration("mission_file")

    def _nodes(context):
        parameters = []
        profile = profile_params.perform(context).strip()
        if profile:
            parameters.append(profile)
        parameters.append({
            "target_id": target_id,
            "mission_file": mission_file,
        })
        return [Node(
            package="uv_task",
            executable="task_runner",
            name="task_runner",
            exec_name="task_runner",
            output="both",
            parameters=parameters,
            condition=IfCondition(enable_task),
        )]

    return LaunchDescription([
        DeclareLaunchArgument("enable_task", default_value="true"),
        DeclareLaunchArgument("target_id", default_value="yellow_golf"),
        DeclareLaunchArgument("profile_params", default_value=""),
        DeclareLaunchArgument(
            "mission_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("uv_task"),
                "config",
                "missions",
                "robocup_26.yaml",
            ]),
            description="YAML mission file loaded by task_runner",
        ),
        OpaqueFunction(function=_nodes),
    ])
