"""Shared launch helpers for the YouLong AUV system bringup.

This module intentionally contains only launch configuration helpers.  Node
implementations and node-specific parameters remain in their owning packages.
"""

from __future__ import annotations

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def declare_profile(runtime, package, *, choices, default="default"):
    """Load a whole-system startup profile before other launch arguments.

    Profile values act as defaults. Explicit launch arguments take precedence.
    """
    choices = tuple(choices)
    if default not in choices:
        raise ValueError(f"default profile {default!r} is not in choices")

    def _load_profile(context):
        selected = str(context.launch_configurations.get("profile", default)).strip()
        if selected not in choices:
            raise RuntimeError(
                f"profile {selected!r} is not valid for runtime {runtime!r}; "
                f"choose one of {', '.join(choices)}")

        profile_file = Path(
            get_package_share_directory(package), "config", "profiles",
            runtime, f"{selected}.yaml")
        if not profile_file.is_file():
            raise RuntimeError(f"bringup profile file does not exist: {profile_file}")
        try:
            payload = yaml.safe_load(profile_file.read_text(encoding="utf-8"))
        except (OSError, yaml.YAMLError) as error:
            raise RuntimeError(
                f"cannot read bringup profile {profile_file}: {error}") from error

        if not isinstance(payload, dict) or payload.get("schema_version") != 1:
            raise RuntimeError(
                f"bringup profile {profile_file} must use schema_version: 1")
        if payload.get("runtime") != runtime:
            raise RuntimeError(
                f"bringup profile {profile_file} is for runtime "
                f"{payload.get('runtime')!r}, expected {runtime!r}")
        if payload.get("profile") != selected:
            raise RuntimeError(
                f"bringup profile {profile_file} must declare profile: {selected}")
        arguments = payload.get("arguments", {})
        if not isinstance(arguments, dict):
            raise RuntimeError(
                f"bringup profile {profile_file} arguments must be a mapping")
        for name, value in arguments.items():
            if not isinstance(name, str) or name == "profile":
                raise RuntimeError(
                    f"bringup profile {profile_file} has invalid argument name {name!r}")
            if name in context.launch_configurations:
                continue
            if isinstance(value, bool):
                value = "true" if value else "false"
            elif isinstance(value, (str, int, float)):
                value = str(value)
            else:
                raise RuntimeError(
                    f"bringup profile {profile_file} argument {name!r} must be a scalar")
            context.launch_configurations[name] = value
        return []

    return [
        DeclareLaunchArgument(
            "profile", default_value=default, choices=list(choices),
            description=(f"Whole-system startup profile for {runtime}: "
                         f"{', '.join(choices)}")),
        OpaqueFunction(function=_load_profile),
    ]


def declare_mission_file():
    """Declare the YAML mission passed to the task component."""
    return DeclareLaunchArgument(
        "mission_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("uv_task"),
            "config",
            "missions",
            "robocup_26.yaml",
        ]),
        description="YAML mission or standalone task file loaded by task_runner",
    )


def declare_feature_arguments(
    *,
    enable_ai="true",
    enable_nav="false",
    enable_task="false",
    enable_motion=None,
):
    """Declare the feature switches shared by the mode launch files."""
    arguments = [
        DeclareLaunchArgument(
            "enable_ai", default_value=enable_ai,
            description="Enable AI perception nodes",
        ),
        DeclareLaunchArgument(
            "enable_nav", default_value=enable_nav,
            description="Enable navigation node",
        ),
    ]
    if enable_task is not None:
        arguments.append(DeclareLaunchArgument(
            "enable_task", default_value=enable_task,
            description="Enable task runner",
        ))
    if enable_motion is not None:
        arguments.append(DeclareLaunchArgument(
            "enable_motion", default_value=enable_motion,
            description="Enable motion control nodes",
        ))
    return arguments


def declare_observability_arguments():
    """Declare go2rtc preview and session recording arguments."""
    from uv_record.session import default_output_root

    return [
        DeclareLaunchArgument(
            "stream_annotated", default_value="true",
            description="Generate annotated go2rtc streams",
        ),
        DeclareLaunchArgument(
            "enable_preview", default_value="true",
            description="Legacy compatibility argument; enable_stream controls go2rtc startup",
        ),
        DeclareLaunchArgument(
            "annotated_max_width", default_value="1280",
            description="Annotated display width limit; 0 means full size",
        ),
        DeclareLaunchArgument(
            "preview_port", default_value="1984",
            description="go2rtc HTTP/WebRTC port",
        ),
        DeclareLaunchArgument(
            "gortc_http_port", default_value="1984",
            description="go2rtc HTTP/WebRTC page port; auto-falls back if occupied",
        ),
        DeclareLaunchArgument(
            "record_session", default_value="false",
            description="Record a crash-resilient ROS/video/log session",
        ),
        DeclareLaunchArgument(
            "record_root", default_value=str(default_output_root()),
            description="Directory under which recording sessions are created",
        ),
        DeclareLaunchArgument(
            "record_mode", default_value="raw", choices=("raw", "go2rtc"),
            description="Image recording path: source frames or go2rtc video",
        ),
        DeclareLaunchArgument(
            "go2rtc_stream_mode", default_value="unannotated",
            choices=("unannotated", "annotated", "both"),
            description="go2rtc stream selection when record_mode is go2rtc",
        ),
        DeclareLaunchArgument(
            "go2rtc_video_format", default_value="jpeg", choices=("jpeg", "ts"),
            description="go2rtc archive format",
        ),
        DeclareLaunchArgument(
            "record_video_fps", default_value=LaunchConfiguration("camera_stitch_fps"),
            description="Recorded video FPS",
        ),
        DeclareLaunchArgument(
            "record_video_codec", default_value="libx264",
            description="FFmpeg video codec for legacy TS recording",
        ),
        DeclareLaunchArgument(
            "record_image_topics", default_value="false",
            description="Deprecated compatibility option",
        ),
        DeclareLaunchArgument(
            "video_segment_seconds", default_value="2.0",
            description="Crash-recoverable video segment length",
        ),
        DeclareLaunchArgument(
            "bag_segment_seconds", default_value="10.0",
            description=(
                "Rosbag segment length when supported by this ROS distro; "
                "Foxy records one sqlite3 bag per supervised child"),
        ),
        DeclareLaunchArgument(
            "record_bag_storage", default_value="auto",
            description=(
                "Bag segment backend: auto, sqlite3, or mcap. auto falls "
                "back to sqlite3 on Foxy; merged session.mcap is optional"),
        ),
        DeclareLaunchArgument(
            "record_use_sim_time", default_value="false",
            description="Use /clock when supported by the active rosbag CLI",
        ),
    ]
