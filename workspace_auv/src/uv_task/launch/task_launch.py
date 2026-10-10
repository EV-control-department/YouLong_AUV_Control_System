"""Launch the mission task runner component."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_task = LaunchConfiguration("enable_task")
    params_file = LaunchConfiguration("params_file")
    mission_file = LaunchConfiguration("mission_file")
    camera_mode = LaunchConfiguration("camera_mode")
    camera_config_dir = LaunchConfiguration("camera_config_dir")
    auto_start = LaunchConfiguration("auto_start")

    def _nodes(context):
        parameters = []
        parameter_file = params_file.perform(context).strip()
        if parameter_file:
            parameters.append(parameter_file)
        task_parameters = {
            "mission_file": mission_file,
            "auto_start": auto_start,
        }
        requested_camera_mode = camera_mode.perform(context).strip()
        if requested_camera_mode and requested_camera_mode.lower() != "auto":
            task_parameters["camera_mode"] = camera_mode
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
        DeclareLaunchArgument("params_file", default_value=""),
        DeclareLaunchArgument("camera_mode", default_value="auto"),
        DeclareLaunchArgument("camera_config_dir", default_value=""),
        DeclareLaunchArgument(
            "auto_start", default_value="true",
            description="Automatically execute the loaded mission on startup",
        ),
        DeclareLaunchArgument(
            "mission_file",
            default_value="",
            description=("YAML mission or task file; empty loads "
                         "src/uv_task/config/missions/robocup_26.yaml"),
        ),
        OpaqueFunction(function=_nodes),
    ])
