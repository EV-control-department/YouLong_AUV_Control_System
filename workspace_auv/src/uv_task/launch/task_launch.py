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
    camera_config_profile = LaunchConfiguration("camera_config_profile")
    camera_config_dir = LaunchConfiguration("camera_config_dir")

    def _nodes(context):
        parameters = []
        profile = profile_params.perform(context).strip()
        if profile:
            parameters.append(profile)
        task_parameters = {
            "target_id": target_id,
            "mission_file": mission_file,
        }
        requested_camera_profile = camera_config_profile.perform(context).strip()
        if requested_camera_profile and requested_camera_profile.lower() != "auto":
            task_parameters["camera_config_profile"] = camera_config_profile
        requested_camera_dir = camera_config_dir.perform(context).strip()
        if requested_camera_dir:
            task_parameters["camera_config_dir"] = camera_config_dir
        parameters.append(task_parameters)
        return [Node(
            package="uv_task",
            executable="task_runner",
            name="task_runner",
            exec_name="task_runner",
            output="both",
            parameters=parameters,
            remappings=[('/tf', '/auv/tf'), ('/tf_static', '/auv/tf_static')],
            condition=IfCondition(enable_task),
        )]

    return LaunchDescription([
        DeclareLaunchArgument("enable_task", default_value="true"),
        DeclareLaunchArgument("target_id", default_value="yellow_golf"),
        DeclareLaunchArgument("profile_params", default_value=""),
        DeclareLaunchArgument("camera_config_profile", default_value="auto"),
        DeclareLaunchArgument("camera_config_dir", default_value=""),
        DeclareLaunchArgument(
            "mission_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("uv_task"),
                "config",
                "missions",
                "robocup_26.yaml",
            ]),
            description="YAML mission or standalone task file loaded by task_runner",
        ),
        OpaqueFunction(function=_nodes),
    ])
