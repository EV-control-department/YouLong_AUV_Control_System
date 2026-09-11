"""Shared launch helpers for the YouLong AUV system bringup.

This module intentionally contains only launch configuration helpers.  Node
implementations and node-specific parameters remain in their owning packages.
"""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare


PROFILE_CHOICES = {
    "sim": ("sim_dev", "sim_ci"),
    "hil": ("hil_lab",),
    "real": ("real_default", "real_safe"),
}

PROFILE_PACKAGES = {
    "sim": ("uv_sim", "uv_camera"),
    "hil": ("uv_sim", "uv_camera"),
    "real": ("uv_hm", "uv_camera"),
}


def configure_simulator_gpu_environment(gpu, gpu_backend):
    """Select the OpenGL provider used by the Stonefish process.

    On hybrid laptops the default GLX provider can be the integrated AMD
    adapter even when an NVIDIA device and X server are available.  Stonefish
    then reports a pair of generic shader link failures while constructing its
    flat-ocean programs.  ``auto`` only enables NVIDIA PRIME offload when the
    NVIDIA device node is present; ``system`` leaves the user's environment
    untouched; ``nvidia`` fails early with an actionable message if the device
    is unavailable.
    """

    def _configure(context):
        use_gpu = gpu.perform(context).strip().lower() in {
            "1", "true", "yes", "on"
        }
        backend = gpu_backend.perform(context).strip().lower()
        if not use_gpu or backend == "system":
            return []

        nvidia_device = Path("/dev/nvidia0").exists()
        if backend == "nvidia" and not nvidia_device:
            raise RuntimeError(
                "gpu_backend:=nvidia requested, but /dev/nvidia0 is not "
                "available; check the NVIDIA driver or use gpu_backend:=system"
            )
        if backend == "auto" and not nvidia_device:
            return [LogInfo(msg="Stonefish GPU backend: system OpenGL provider")]

        return [
            SetEnvironmentVariable("__NV_PRIME_RENDER_OFFLOAD", "1"),
            SetEnvironmentVariable("__GLX_VENDOR_LIBRARY_NAME", "nvidia"),
            LogInfo(msg="Stonefish GPU backend: NVIDIA PRIME offload"),
        ]

    return OpaqueFunction(function=_configure)


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
        description="YAML mission file loaded by task_runner",
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


def declare_simulation_arguments(
    *,
    core=False,
    scenario_default=None,
    window_width_default=None,
    window_height_default=None,
    render_quality_default=None,
    camera_stitch_fps_default=None,
):
    """Declare simulator and SIL/HIL bridge arguments."""
    scenario_default = scenario_default or (
        "underwater_xunyun.scn" if core
        else "guoshui_2026_cruise_seeded.scn")
    window_width_default = window_width_default or ("1280" if core else "960")
    window_height_default = window_height_default or ("720" if core else "540")
    render_quality_default = render_quality_default or (
        "high" if core else "low")
    camera_stitch_fps_default = camera_stitch_fps_default or (
        "10.0" if core else "5.0")
    arguments = [
        DeclareLaunchArgument(
            "scenario_desc", default_value=scenario_default,
            description="Stonefish scenario file name or absolute path",
        ),
        DeclareLaunchArgument(
            "scene_seed", default_value="0",
            description=(
                "Integer seed for the Guoshui generated scene; "
                "the fixed baseline is used for seed 0"
            ),
        ),
        DeclareLaunchArgument(
            "simulation_rate", default_value="100.0",
            description="Stonefish simulation rate in Hz",
        ),
        DeclareLaunchArgument(
            "sim_window_width", default_value=window_width_default,
            description="Stonefish window width in pixels",
        ),
        DeclareLaunchArgument(
            "sim_window_height", default_value=window_height_default,
            description="Stonefish window height in pixels",
        ),
        DeclareLaunchArgument(
            "render_quality", default_value=render_quality_default,
            description="Stonefish rendering quality: low, medium, or high",
        ),
        DeclareLaunchArgument(
            "render_fps", default_value="30.0",
            description="Stonefish display refresh limit",
        ),
        DeclareLaunchArgument(
            "gpu", default_value="true",
            description="Use the GPU Stonefish executable when true",
        ),
        DeclareLaunchArgument(
            "gpu_backend", default_value="auto",
            choices=["auto", "nvidia", "system"],
            description=(
                "OpenGL provider for Stonefish GPU mode: auto detects "
                "NVIDIA PRIME, nvidia forces it, system preserves the environment"
            ),
        ),
        DeclareLaunchArgument(
            "camera_stitch_fps", default_value=camera_stitch_fps_default,
            description="Maximum stitched camera topic rate",
        ),
        DeclareLaunchArgument(
            "publish_raw_camera_topics", default_value="false",
            description="Republish individual raw camera image topics",
        ),
        DeclareLaunchArgument(
            "ai_inference_fps", default_value="3.0",
            description="Maximum AI inference rate per camera",
        ),
        DeclareLaunchArgument(
            "inference_threads", default_value="2",
            description="Maximum PyTorch CPU threads used by AI",
        ),
        DeclareLaunchArgument(
            "ai_confidence", default_value="0.8",
            description="YOLO confidence threshold",
        ),
        DeclareLaunchArgument(
            "gate_feature_mode", default_value="auto",
            description="Front gate anchor: auto, centerline, segmentation, or bbox",
        ),
        DeclareLaunchArgument(
            "target_id", default_value="yellow_golf",
            description="Competition target metadata",
        ),
        DeclareLaunchArgument(
            "startup_timeout", default_value="120.0",
            description="Maximum seconds per readiness stage",
        ),
    ]
    return arguments


def declare_observability_arguments(
    *, preview_width_default="960", preview_height_default="540"
):
    """Declare preview and session recording arguments."""
    from uv_log.session import default_output_root

    return [
        DeclareLaunchArgument(
            "open_annotated_windows", default_value="true",
            description="Open front/down annotated preview windows",
        ),
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
            "preview_width", default_value=preview_width_default,
            description="Annotated preview window width in pixels",
        ),
        DeclareLaunchArgument(
            "preview_height", default_value=preview_height_default,
            description="Annotated preview window maximum height in pixels",
        ),
        DeclareLaunchArgument(
            "preview_port", default_value="8090",
            description="uv_camera MJPEG port",
        ),
        DeclareLaunchArgument(
            "preview_wait_timeout", default_value="60.0",
            description="Seconds before logging a missing preview stream",
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
            description="Rosbag recording segment length",
        ),
        DeclareLaunchArgument(
            "record_use_sim_time", default_value="false",
            description="Use /clock for rosbag timestamps",
        ),
    ]
