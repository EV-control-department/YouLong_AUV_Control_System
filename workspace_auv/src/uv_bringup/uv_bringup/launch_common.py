"""Shared launch helpers for the YouLong AUV system bringup.

This module intentionally contains only launch configuration helpers.  Node
implementations and node-specific parameters remain in their owning packages.
"""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare


PROFILE_CHOICES = {
    "real": ("real_default", "real_safe"),
}

PROFILE_PACKAGES = {
    "real": ("uv_hm", "uv_camera"),
}


def profile_path(profile, package):
    """Return a package-owned standard ROS parameter file for ``profile``."""
    filename = PythonExpression(["'", profile, "' + '.yaml'"])
    return PathJoinSubstitution([
        FindPackageShare(package),
        "config",
        "profiles",
        filename,
    ])


def validate_profile(profile, mode):
    """Return a launch action that rejects a profile from another mode."""

    def _validate(context):
        value = profile.perform(context).strip()
        choices = PROFILE_CHOICES[mode]
        if value not in choices:
            raise RuntimeError(
                f"profile {value!r} is not valid for mode {mode!r}; "
                f"choose one of {', '.join(choices)}")
        for package in PROFILE_PACKAGES[mode]:
            profile_file = Path(
                get_package_share_directory(package),
                "config",
                "profiles",
                f"{value}.yaml",
            )
            if not profile_file.is_file():
                raise RuntimeError(
                    f"profile parameter file does not exist: {profile_file}")
        return []

    return OpaqueFunction(function=_validate)


def declare_profile(mode, default):
    """Declare a mode-specific profile selector."""
    return DeclareLaunchArgument(
        "profile",
        default_value=default,
        choices=list(PROFILE_CHOICES[mode]),
        description=(
            f"Runtime parameter profile for {mode}: "
            f"{', '.join(PROFILE_CHOICES[mode])}"
        ),
    )


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
        DeclareLaunchArgument(
            "enable_task", default_value=enable_task,
            description="Enable task runner",
        ),
    ]
    if enable_motion is not None:
        arguments.append(DeclareLaunchArgument(
            "enable_motion", default_value=enable_motion,
            description="Enable motion control nodes",
        ))
    return arguments


def declare_observability_arguments():
    """Declare MJPEG/go2rtc and session recording arguments."""
    from uv_log.session import default_output_root

    return [
        DeclareLaunchArgument(
            "stream_annotated", default_value="true",
            description="Generate annotated MJPEG streams",
        ),
        DeclareLaunchArgument(
            "enable_preview", default_value="true",
            description="Enable MJPEG/go2rtc preview and video capture",
        ),
        DeclareLaunchArgument(
            "annotated_max_width", default_value="1280",
            description="Annotated MJPEG width limit; 0 means full size",
        ),
        DeclareLaunchArgument(
            "preview_port", default_value="8090",
            description="uv_camera MJPEG port",
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
            "record_raw_video", default_value="false",
            description="Deprecated compatibility option",
        ),
        DeclareLaunchArgument(
            "record_video_mode", default_value="raw",
            description="Video streams to record: raw, annotated, or both",
        ),
        DeclareLaunchArgument(
            "record_video_format", default_value="jpeg",
            description="Video archive format: jpeg or ts",
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
                "Bag backend: auto, sqlite3, or mcap; auto keeps Foxy on "
                "sqlite3 and uses MCAP when the plugin is installed"),
        ),
        DeclareLaunchArgument(
            "record_use_sim_time", default_value="false",
            description="Use /clock when supported by the active rosbag CLI",
        ),
    ]
