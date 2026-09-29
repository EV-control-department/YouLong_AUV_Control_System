"""Public YouLong simulation entry point with a world-only scene selector."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from uv_sim.scenarios import resolve_world
from uv_sim_bringup.launch_common import (
    declare_feature_arguments,
    declare_mission_file,
    declare_observability_arguments,
)


def generate_launch_description():
    world = LaunchConfiguration("world")
    vehicle = LaunchConfiguration("vehicle")

    def _include(context):
        world_value = world.perform(context).strip()
        vehicle_value = vehicle.perform(context).strip()
        if vehicle_value != "youlong":
            raise RuntimeError(
                f"unsupported vehicle {vehicle_value!r}; only 'youlong' is "
                "available in the canonical asset package")
        assets_root = Path(get_package_share_directory("uv_sim_assets"))
        scenario = resolve_world(assets_root, world_value)
        arguments = {
            "scenario_desc": str(scenario),
            "mission_file": LaunchConfiguration("mission_file"),
            "enable_ai": LaunchConfiguration("enable_ai"),
            "enable_motion": LaunchConfiguration("enable_motion"),
            "enable_nav": LaunchConfiguration("enable_nav"),
            "enable_task": LaunchConfiguration("enable_task"),
            "enable_stream": LaunchConfiguration("enable_stream"),
            "enable_evaluation": LaunchConfiguration("enable_evaluation"),
            "camera_config_dir": LaunchConfiguration("camera_config_dir"),
            "scene_seed": LaunchConfiguration("scene_seed"),
            "simulation_rate": LaunchConfiguration("simulation_rate"),
            "sim_window_width": LaunchConfiguration("sim_window_width"),
            "sim_window_height": LaunchConfiguration("sim_window_height"),
            "render_quality": LaunchConfiguration("render_quality"),
            "render_fps": LaunchConfiguration("render_fps"),
            "gpu": LaunchConfiguration("gpu"),
            "gpu_backend": LaunchConfiguration("gpu_backend"),
            "camera_stitch_fps": LaunchConfiguration("camera_stitch_fps"),
            "publish_raw_camera_topics": LaunchConfiguration(
                "publish_raw_camera_topics"),
            "ai_inference_fps": LaunchConfiguration("ai_inference_fps"),
            "inference_threads": LaunchConfiguration("inference_threads"),
            "ai_confidence": LaunchConfiguration("ai_confidence"),
            "gate_feature_mode": LaunchConfiguration("gate_feature_mode"),
            "startup_timeout": LaunchConfiguration("startup_timeout"),
            "enable_perception_gui": LaunchConfiguration("enable_perception_gui"),
            "stream_annotated": LaunchConfiguration("stream_annotated"),
            "enable_preview": LaunchConfiguration("enable_preview"),
            "annotated_max_width": LaunchConfiguration("annotated_max_width"),
            "preview_port": LaunchConfiguration("preview_port"),
            "gortc_http_port": LaunchConfiguration("gortc_http_port"),
            "record_session": LaunchConfiguration("record_session"),
            "record_root": LaunchConfiguration("record_root"),
            "record_mode": LaunchConfiguration("record_mode"),
            "go2rtc_stream_mode": LaunchConfiguration("go2rtc_stream_mode"),
            "go2rtc_video_format": LaunchConfiguration("go2rtc_video_format"),
            "record_video_fps": LaunchConfiguration("record_video_fps"),
            "record_video_codec": LaunchConfiguration("record_video_codec"),
            "record_image_topics": LaunchConfiguration("record_image_topics"),
            "video_segment_seconds": LaunchConfiguration("video_segment_seconds"),
            "bag_segment_seconds": LaunchConfiguration("bag_segment_seconds"),
            "record_bag_storage": LaunchConfiguration("record_bag_storage"),
            "record_use_sim_time": LaunchConfiguration("record_use_sim_time"),
        }
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("uv_sim_bringup"), "launch", "sim.launch.py",
            ])),
            launch_arguments=arguments.items(),
        )]

    return LaunchDescription([
        DeclareLaunchArgument(
            "world", default_value="guoshui_2026/cruise_seeded",
            description="World name, for example sauvc_2026/finals"),
        DeclareLaunchArgument(
            "vehicle", default_value="youlong",
            description="Canonical vehicle name"),
        declare_mission_file(),
        *declare_feature_arguments(
            enable_ai="true", enable_nav="false", enable_task="false",
            enable_motion="true"),
        DeclareLaunchArgument("enable_stream", default_value="true"),
        DeclareLaunchArgument("enable_evaluation", default_value="true"),
        DeclareLaunchArgument("camera_config_dir", default_value=""),
        DeclareLaunchArgument("enable_perception_gui", default_value="false"),
        DeclareLaunchArgument("scene_seed", default_value="0"),
        DeclareLaunchArgument("simulation_rate", default_value="100.0"),
        DeclareLaunchArgument("sim_window_width", default_value="960"),
        DeclareLaunchArgument("sim_window_height", default_value="540"),
        DeclareLaunchArgument("render_quality", default_value="low"),
        DeclareLaunchArgument("render_fps", default_value="30.0"),
        DeclareLaunchArgument("gpu", default_value="true"),
        DeclareLaunchArgument("gpu_backend", default_value="auto",
                              choices=["auto", "nvidia", "system"]),
        DeclareLaunchArgument("camera_stitch_fps", default_value="5.0"),
        *declare_observability_arguments(),
        DeclareLaunchArgument("publish_raw_camera_topics", default_value="false"),
        DeclareLaunchArgument("ai_inference_fps", default_value="3.0"),
        DeclareLaunchArgument("inference_threads", default_value="2"),
        DeclareLaunchArgument("ai_confidence", default_value="0.8"),
        DeclareLaunchArgument("gate_feature_mode", default_value="auto"),
        DeclareLaunchArgument("startup_timeout", default_value="120.0"),
        OpaqueFunction(function=_include),
    ])
