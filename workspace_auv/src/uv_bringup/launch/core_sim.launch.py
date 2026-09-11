"""Minimal SIL preset: Stonefish and the simulation bridge only."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import (
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from uv_bringup.launch_common import (
    configure_simulator_gpu_environment,
    declare_profile,
    declare_simulation_arguments,
    profile_path,
    validate_profile,
)
from uv_bringup.scene import prepare_scene


def _include(package, launch_file, arguments, *, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(package), "launch", launch_file,
        ])),
        launch_arguments=arguments.items(),
        condition=condition,
    )


def generate_launch_description():
    profile = LaunchConfiguration("profile")
    profile_params = profile_path(profile, "uv_sim")
    gpu = LaunchConfiguration("gpu")
    gpu_backend = LaunchConfiguration("gpu_backend")
    simulation_rate = LaunchConfiguration("simulation_rate")
    scenario_desc = LaunchConfiguration("scenario_desc")
    scene_seed = LaunchConfiguration("scene_seed")
    render_quality = LaunchConfiguration("render_quality")
    sim_width = LaunchConfiguration("sim_window_width")
    sim_height = LaunchConfiguration("sim_window_height")

    gpu_sim = _include("stonefish_ros2", "stonefish_simulator.launch.py", {
        "simulation_data": LaunchConfiguration("resolved_simulation_data"),
        "scenario_desc": LaunchConfiguration("resolved_scenario"),
        "simulation_rate": simulation_rate,
        "window_res_x": sim_width,
        "window_res_y": sim_height,
        "rendering_quality": render_quality,
        "render_fps": LaunchConfiguration("render_fps"),
    }, condition=IfCondition(gpu))
    nogpu_sim = _include("stonefish_ros2", "stonefish_simulator_nogpu.launch.py", {
        "simulation_data": LaunchConfiguration("resolved_simulation_data"),
        "scenario_desc": LaunchConfiguration("resolved_scenario"),
        "simulation_rate": simulation_rate,
    }, condition=UnlessCondition(gpu))
    bridge = _include("uv_sim", "bridge.launch.py", {
        "hil_mode": "false",
        "camera_stitch_fps": LaunchConfiguration("camera_stitch_fps"),
        "publish_raw_camera_topics": LaunchConfiguration("publish_raw_camera_topics"),
        "profile_params": profile_params,
    })

    def _critical_exit(event, context):
        if context.is_shutdown:
            return []
        name = str(getattr(event, "process_name", "") or "")
        if any(name == key or name.startswith(f"{key}-") for key in {
            "stonefish_simulator", "stonefish_simulator_nogpu", "sim_bridge",
        }):
            return [EmitEvent(event=Shutdown(
                reason=f"critical core process exited: {name}"))]
        return []

    return LaunchDescription([
        declare_profile("sim", "sim_dev"),
        *declare_simulation_arguments(core=True),
        configure_simulator_gpu_environment(gpu, gpu_backend),
        RegisterEventHandler(OnProcessExit(on_exit=_critical_exit)),
        SetEnvironmentVariable("SDL_VIDEO_WINDOW_POS", "0,0"),
        validate_profile(profile, "sim"),
        LogInfo(msg=["Core simulation profile: ", profile]),
        prepare_scene(
            scenario_desc=scenario_desc,
            scene_seed=scene_seed,
            launch_file=__file__,
            start_actions=[gpu_sim, nogpu_sim, bridge],
        ),
    ])
