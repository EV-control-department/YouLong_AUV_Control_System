"""Launch the camera AI and object-localization component."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_ai = LaunchConfiguration("enable_ai")
    sim_mode = LaunchConfiguration("sim_mode")
    inference_fps = LaunchConfiguration("inference_fps")
    inference_threads = LaunchConfiguration("inference_threads")
    confidence = LaunchConfiguration("confidence")
    gate_feature_mode = LaunchConfiguration("gate_feature_mode")
    enable_gortc = LaunchConfiguration("enable_gortc")
    stream_annotated = LaunchConfiguration("stream_annotated")
    mjpeg_port = LaunchConfiguration("mjpeg_port")
    annotated_max_width = LaunchConfiguration("annotated_max_width")
    profile_params = LaunchConfiguration("profile_params")
    object_localizer_params = LaunchConfiguration("object_localizer_params")

    def _nodes(context):
        profile = profile_params.perform(context).strip()
        localizer_config = object_localizer_params.perform(context).strip()

        vision_parameters = []
        if profile:
            vision_parameters.append(profile)
        vision_parameters.append({
            "sim_mode": sim_mode,
            "inference_fps": inference_fps,
            "inference_threads": inference_threads,
            "confidence": confidence,
            "gate_feature_mode": gate_feature_mode,
            "enable_gortc": enable_gortc,
            "stream_annotated": stream_annotated,
            "mjpeg_port": mjpeg_port,
            "annotated_max_width": annotated_max_width,
        })

        localizer_parameters = []
        if profile:
            localizer_parameters.append(profile)
        if localizer_config:
            localizer_parameters.append(localizer_config)

        return [
            Node(
                package="uv_camera",
                executable="uv_camera",
                name="uv_camera",
                exec_name="uv_camera",
                output="both",
                parameters=vision_parameters,
                condition=IfCondition(enable_ai),
                respawn=True,
                respawn_delay=1.0,
                respawn_max_retries=3,
            ),
            Node(
                package="uv_camera",
                executable="object_localizer",
                name="object_localizer",
                exec_name="object_localizer",
                output="both",
                parameters=localizer_parameters,
                condition=IfCondition(enable_ai),
                respawn=True,
                respawn_delay=1.0,
                respawn_max_retries=3,
            ),
        ]

    return LaunchDescription([
        DeclareLaunchArgument("enable_ai", default_value="true"),
        DeclareLaunchArgument("sim_mode", default_value="false"),
        DeclareLaunchArgument("inference_fps", default_value="5.0"),
        DeclareLaunchArgument("inference_threads", default_value="2"),
        DeclareLaunchArgument("confidence", default_value="0.8"),
        DeclareLaunchArgument("gate_feature_mode", default_value="auto"),
        DeclareLaunchArgument("enable_gortc", default_value="true"),
        DeclareLaunchArgument("stream_annotated", default_value="true"),
        DeclareLaunchArgument("mjpeg_port", default_value="8090"),
        DeclareLaunchArgument("annotated_max_width", default_value="0"),
        DeclareLaunchArgument("profile_params", default_value=""),
        DeclareLaunchArgument("object_localizer_params", default_value=""),
        OpaqueFunction(function=_nodes),
    ])
