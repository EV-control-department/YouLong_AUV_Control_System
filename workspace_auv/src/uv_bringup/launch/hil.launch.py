"""Hardware-in-the-loop runtime preset."""

from __future__ import annotations

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from uv_bringup.launch_common import (
    configure_simulator_gpu_environment,
    declare_feature_arguments,
    declare_mission_file,
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


def _default_agent_executable():
    candidate = Path.home() / "micro_ros_agent_ws" / "install" / "micro_ros_agent" / "lib" / "micro_ros_agent" / "micro_ros_agent"
    return str(candidate) if candidate.is_file() else "micro_ros_agent"


def generate_launch_description():
    profile = LaunchConfiguration("profile")
    sim_profile_params = profile_path(profile, "uv_sim")
    camera_profile_params = profile_path(profile, "uv_camera")
    enable_ai = LaunchConfiguration("enable_ai")
    enable_nav = LaunchConfiguration("enable_nav")
    enable_task = LaunchConfiguration("enable_task")
    mission_file = LaunchConfiguration("mission_file")
    enable_motion = LaunchConfiguration("enable_motion")
    gpu = LaunchConfiguration("gpu")
    gpu_backend = LaunchConfiguration("gpu_backend")
    serial_dev = LaunchConfiguration("serial_dev")
    serial_baud = LaunchConfiguration("serial_baud")
    scenario_desc = LaunchConfiguration("scenario_desc")

    stonefish = _include("stonefish_ros2", "stonefish_simulator.launch.py", {
        "simulation_data": LaunchConfiguration("resolved_simulation_data"),
        "scenario_desc": LaunchConfiguration("resolved_scenario"),
        "simulation_rate": LaunchConfiguration("simulation_rate"),
        "window_res_x": LaunchConfiguration("sim_window_width"),
        "window_res_y": LaunchConfiguration("sim_window_height"),
        "rendering_quality": LaunchConfiguration("render_quality"),
        "render_fps": LaunchConfiguration("render_fps"),
    })
    bridge = _include("uv_sim", "bridge.launch.py", {
        "hil_mode": "true",
        "camera_stitch_fps": LaunchConfiguration("camera_stitch_fps"),
        "publish_raw_camera_topics": LaunchConfiguration("publish_raw_camera_topics"),
        "profile_params": sim_profile_params,
    })
    control = _include("uv_control", "control_launch.py", {
        "enable_motion": enable_motion,
        "profile_params": "",
    })
    perception = _include("uv_camera", "perception_launch.py", {
        "enable_ai": enable_ai,
        "sim_mode": "true",
        "inference_fps": LaunchConfiguration("ai_inference_fps"),
        "inference_threads": LaunchConfiguration("inference_threads"),
        "confidence": LaunchConfiguration("ai_confidence"),
        "gate_feature_mode": LaunchConfiguration("gate_feature_mode"),
        "enable_gortc": "false",
        "stream_annotated": "false",
        "mjpeg_port": "8090",
        "annotated_max_width": "0",
        "profile_params": camera_profile_params,
        "object_localizer_params": PathJoinSubstitution([
            FindPackageShare("uv_camera"), "config", "object_localizer_sim.yaml"
        ]),
    })
    nav = _include("uv_nav", "navigation_launch.py", {
        "enable_nav": enable_nav,
        "profile_params": "",
    })
    task = _include("uv_task", "task_launch.py", {
        "enable_task": enable_task,
        "target_id": LaunchConfiguration("target_id"),
        "profile_params": "",
        "mission_file": mission_file,
    })

    agent_executable = LaunchConfiguration("agent_executable")
    agent = ExecuteProcess(
        cmd=[agent_executable, "serial", "-D", serial_dev, "-b", serial_baud, "-v", "4"],
        name="micro_ros_agent",
        output="both",
    )

    def _critical_exit(event, context):
        if context.is_shutdown:
            return []
        name = str(getattr(event, "process_name", "") or "")
        if any(name == key or name.startswith(f"{key}-") for key in {
            "stonefish_simulator", "sim_bridge", "micro_ros_agent",
            "basic_motion",
        }):
            return [EmitEvent(event=Shutdown(
                reason=f"critical HIL process exited: {name}"))]
        return []

    return LaunchDescription([
        declare_profile("hil", "hil_lab"),
        declare_mission_file(),
        *declare_feature_arguments(
            enable_ai="false", enable_nav="false", enable_task="false",
            enable_motion="false",
        ),
        *declare_simulation_arguments(
            scenario_default="underwater_xunyun.scn",
            window_width_default="1280",
            window_height_default="720",
            render_quality_default="high",
            camera_stitch_fps_default="10.0",
        ),
        configure_simulator_gpu_environment(gpu, gpu_backend),
        DeclareLaunchArgument("serial_dev", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("serial_baud", default_value="921600"),
        DeclareLaunchArgument(
            "agent_executable", default_value=_default_agent_executable(),
            description="micro-ROS Agent executable or absolute path",
        ),
        RegisterEventHandler(OnProcessExit(on_exit=_critical_exit)),
        validate_profile(profile, "hil"),
        LogInfo(msg=["HIL profile: ", profile]),
        prepare_scene(
            scenario_desc=scenario_desc,
            scene_seed=LaunchConfiguration("scene_seed"),
            launch_file=__file__,
            start_actions=[stonefish, bridge, agent, control, perception, nav, task],
        ),
    ])
