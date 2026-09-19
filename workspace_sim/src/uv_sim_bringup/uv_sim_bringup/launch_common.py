"""Simulation-only launch helpers kept outside the AUV workspace."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from uv_bringup import launch_common as _common


def declare_feature_arguments(*args, **kwargs):
    """Forward shared feature switches from the AUV launch package."""
    return _common.declare_feature_arguments(*args, **kwargs)


def declare_mission_file(*args, **kwargs):
    """Forward the mission-file declaration from the AUV package."""
    return _common.declare_mission_file(*args, **kwargs)


def declare_observability_arguments(*args, **kwargs):
    """Forward shared observability switches from the AUV package."""
    return _common.declare_observability_arguments(*args, **kwargs)


def profile_path(*args, **kwargs):
    """Forward package profile lookup from the AUV launch package."""
    return _common.profile_path(*args, **kwargs)


PROFILE_CHOICES = {
    'sim': ('sim_dev', 'sim_ci'),
    'hil': ('hil_lab',),
}

PROFILE_PACKAGES = {
    'sim': ('uv_sim_bridge', 'uv_camera'),
    'hil': ('uv_sim_bridge', 'uv_camera'),
}


def configure_simulator_gpu_environment(gpu, gpu_backend):
    """Select the OpenGL provider used by the Stonefish process."""

    def _configure(context):
        use_gpu = gpu.perform(context).strip().lower() in {
            '1', 'true', 'yes', 'on'
        }
        backend = gpu_backend.perform(context).strip().lower()
        if not use_gpu or backend == 'system':
            return []

        nvidia_device = Path('/dev/nvidia0').exists()
        if backend == 'nvidia' and not nvidia_device:
            raise RuntimeError(
                'gpu_backend:=nvidia requested, but /dev/nvidia0 is not '
                'available; check the NVIDIA driver or use '
                'gpu_backend:=system')
        if backend == 'auto' and not nvidia_device:
            return [LogInfo(
                msg='Stonefish GPU backend: system OpenGL provider')]

        return [
            SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
            SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
            LogInfo(msg='Stonefish GPU backend: NVIDIA PRIME offload'),
        ]

    return OpaqueFunction(function=_configure)


def validate_profile(profile, mode):
    """Reject a simulation profile from another simulation mode."""

    def _validate(context):
        value = profile.perform(context).strip()
        choices = PROFILE_CHOICES[mode]
        if value not in choices:
            raise RuntimeError(
                f'profile {value!r} is not valid for mode {mode!r}; '
                f'choose one of {", ".join(choices)}')
        for package in PROFILE_PACKAGES[mode]:
            profile_file = Path(
                get_package_share_directory(package), 'config', 'profiles',
                f'{value}.yaml')
            if not profile_file.is_file():
                raise RuntimeError(
                    f'profile parameter file does not exist: {profile_file}')
        return []

    return OpaqueFunction(function=_validate)


def declare_profile(mode, default):
    """Declare a simulation or HIL profile selector."""
    return DeclareLaunchArgument(
        'profile',
        default_value=default,
        choices=list(PROFILE_CHOICES[mode]),
        description=(
            f'Runtime parameter profile for {mode}: '
            f'{", ".join(PROFILE_CHOICES[mode])}'),
    )


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
    if scenario_default is None:
        scenario_default = (
            'underwater_xunyun.scn' if core
            else 'worlds/guoshui_2026/guoshui_2026_cruise_seeded.scn')
    if window_width_default is None:
        window_width_default = '1280' if core else '960'
    if window_height_default is None:
        window_height_default = '720' if core else '540'
    if render_quality_default is None:
        render_quality_default = 'high' if core else 'low'
    if camera_stitch_fps_default is None:
        camera_stitch_fps_default = '10.0' if core else '5.0'
    return [
        DeclareLaunchArgument(
            'scenario_desc', default_value=scenario_default,
            description='Stonefish scenario file name or absolute path'),
        DeclareLaunchArgument(
            'scene_seed', default_value='0',
            description='Integer seed for generated scenes'),
        DeclareLaunchArgument(
            'simulation_rate', default_value='100.0',
            description='Stonefish simulation rate in Hz'),
        DeclareLaunchArgument(
            'sim_window_width', default_value=window_width_default,
            description='Stonefish window width in pixels'),
        DeclareLaunchArgument(
            'sim_window_height', default_value=window_height_default,
            description='Stonefish window height in pixels'),
        DeclareLaunchArgument(
            'render_quality', default_value=render_quality_default,
            description='Stonefish rendering quality'),
        DeclareLaunchArgument(
            'render_fps', default_value='30.0',
            description='Stonefish display refresh limit'),
        DeclareLaunchArgument(
            'gpu', default_value='true',
            description='Use the GPU Stonefish executable when true'),
        DeclareLaunchArgument(
            'gpu_backend', default_value='auto',
            choices=['auto', 'nvidia', 'system'],
            description='OpenGL provider for Stonefish GPU mode'),
        DeclareLaunchArgument(
            'camera_stitch_fps', default_value=camera_stitch_fps_default,
            description='Legacy compatibility setting; simulator image rate is set by Stonefish'),
        DeclareLaunchArgument(
            'publish_raw_camera_topics', default_value='false',
            description='Deprecated; simulator images always use shared memory (DDS carries CameraInfo only)'),
        DeclareLaunchArgument(
            'ai_inference_fps', default_value='3.0',
            description='Maximum AI inference rate per camera'),
        DeclareLaunchArgument(
            'inference_threads', default_value='2',
            description='Maximum PyTorch CPU threads used by AI'),
        DeclareLaunchArgument(
            'ai_confidence', default_value='0.8',
            description='YOLO confidence threshold'),
        DeclareLaunchArgument(
            'gate_feature_mode', default_value='auto',
            description='Front gate feature selection mode'),
        DeclareLaunchArgument(
            'target_id', default_value='yellow_golf',
            description='Competition target metadata'),
        DeclareLaunchArgument(
            'startup_timeout', default_value='120.0',
            description='Maximum seconds per readiness stage'),
    ]
