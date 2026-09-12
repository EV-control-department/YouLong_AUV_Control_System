"""SIL simulation system preset.

This file is deliberately an orchestrator: component node definitions live in
their owning packages, while optional desktop and recording capabilities live
in ``observability.launch.py``.
"""

from __future__ import annotations

import os
import shutil
import tempfile
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare

from uv_bringup.desktop import default_preview_window, default_sim_window, focused_monitor
from uv_bringup.launch_common import (
    declare_feature_arguments,
    declare_mission_file,
    declare_observability_arguments,
    declare_profile,
    declare_simulation_arguments,
    configure_simulator_gpu_environment,
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


def _auto_configure_container_display():
    """Use the mounted desktop X server without requiring manual exports.

    The NVIDIA-enabled container normally has the host X11 socket mounted,
    but an interactive shell created with ``docker exec`` may not inherit the
    desktop variables.  Prefer the usual X1/X0 socket and the container's
    mounted Xauthority file; leave true headless containers to the Xvfb
    fallback below.
    """

    actions = []
    display = os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
    if not display:
        x11_dir = Path("/tmp/.X11-unix")
        candidates = [x11_dir / "X1", x11_dir / "X0"]
        candidates.extend(sorted(x11_dir.glob("X*")))
        for socket in candidates:
            if socket.exists() and socket.name[1:].isdigit():
                display = f":{socket.name[1:]}"
                os.environ["DISPLAY"] = display
                actions.append(SetEnvironmentVariable("DISPLAY", display))
                break

    if display and not os.environ.get("XAUTHORITY"):
        for candidate in (Path("/root/.Xauthority"),
                          Path("/run/user/1000/gdm/Xauthority")):
            if candidate.exists():
                os.environ["XAUTHORITY"] = str(candidate)
                actions.append(SetEnvironmentVariable("XAUTHORITY", str(candidate)))
                break

    return actions


def generate_launch_description():
    desktop_environment = _auto_configure_container_display()
    sim_width, sim_height = default_sim_window()
    preview_width, preview_height = default_preview_window()
    monitor_x, monitor_y, _, _ = focused_monitor()

    # Stonefish's camera sensors require the graphical executable even when
    # the simulation runs in a container.  A plain Foxy container commonly
    # has neither DISPLAY nor a mounted X11 socket, so provide a software X
    # server as a transparent fallback.  Existing desktop sessions keep their
    # original environment and are not affected.
    headless_graphics = []
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        xvfb = shutil.which("Xvfb")
        if xvfb:
            display = ":99"
            runtime_dir = tempfile.mkdtemp(prefix="uv_bringup_xdg_")
            os.chmod(runtime_dir, 0o700)
            if not os.environ.get("ROS_LOCALHOST_ONLY"):
                # Set this before any child process is created.  Some Foxy
                # RMW implementations read the variable during module
                # initialisation, before a later launch action can affect
                # their DDS participant.
                os.environ["ROS_LOCALHOST_ONLY"] = "1"
            headless_graphics = [
                # A headless container normally shares the default DDS
                # domain with the host.  Isolate it unless the operator
                # explicitly chose a different setting; this prevents mixed
                # ROS distributions from feeding malformed samples to Foxy.
                *([] if os.environ.get("ROS_LOCALHOST_ONLY") else [
                    SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
                ]),
                SetEnvironmentVariable("DISPLAY", display),
                SetEnvironmentVariable("XDG_RUNTIME_DIR", runtime_dir),
                SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1"),
                ExecuteProcess(
                    cmd=[
                        xvfb, display,
                        "-screen", "0", f"{sim_width}x{sim_height}x24",
                        "-ac", "+extension", "GLX", "+render", "-noreset",
                    ],
                    name="uv_xvfb",
                    output="screen",
                ),
                LogInfo(msg=[
                    "No display detected; using headless Xvfb on ", display,
                ]),
            ]
        else:
            headless_graphics = [LogInfo(msg=(
                "No DISPLAY detected and Xvfb is not installed; "
                "GPU Stonefish needs a graphical display (install xvfb or "
                "set DISPLAY)."
            ))]

    enable_ai = LaunchConfiguration("enable_ai")
    enable_motion = LaunchConfiguration("enable_motion")
    enable_nav = LaunchConfiguration("enable_nav")
    enable_task = LaunchConfiguration("enable_task")
    mission_file = LaunchConfiguration("mission_file")
    profile = LaunchConfiguration("profile")
    sim_profile_params = profile_path(profile, "uv_sim")
    camera_profile_params = profile_path(profile, "uv_camera")
    gpu = LaunchConfiguration("gpu")
    gpu_backend = LaunchConfiguration("gpu_backend")
    scenario_desc = LaunchConfiguration("scenario_desc")
    scene_seed = LaunchConfiguration("scene_seed")

    sim_window_width = LaunchConfiguration("sim_window_width")
    sim_window_height = LaunchConfiguration("sim_window_height")
    render_quality = LaunchConfiguration("render_quality")
    simulation_rate = LaunchConfiguration("simulation_rate")
    render_fps = LaunchConfiguration("render_fps")
    camera_stitch_fps = LaunchConfiguration("camera_stitch_fps")
    publish_raw = LaunchConfiguration("publish_raw_camera_topics")
    ai_inference_fps = LaunchConfiguration("ai_inference_fps")
    inference_threads = LaunchConfiguration("inference_threads")
    ai_confidence = LaunchConfiguration("ai_confidence")
    gate_feature_mode = LaunchConfiguration("gate_feature_mode")
    gortc_http_port = LaunchConfiguration("gortc_http_port")
    target_id = LaunchConfiguration("target_id")
    startup_timeout = LaunchConfiguration("startup_timeout")

    stonefish_arguments = {
        "simulation_data": LaunchConfiguration("resolved_simulation_data"),
        "scenario_desc": LaunchConfiguration("resolved_scenario"),
        "simulation_rate": simulation_rate,
        "window_res_x": sim_window_width,
        "window_res_y": sim_window_height,
        "rendering_quality": render_quality,
        "render_fps": render_fps,
    }
    stonefish_gpu = _include(
        "stonefish_ros2", "stonefish_simulator.launch.py", stonefish_arguments,
        condition=IfCondition(gpu))
    stonefish_nogpu = _include(
        "stonefish_ros2", "stonefish_simulator_nogpu.launch.py", {
            "simulation_data": LaunchConfiguration("resolved_simulation_data"),
            "scenario_desc": LaunchConfiguration("resolved_scenario"),
            "simulation_rate": simulation_rate,
        }, condition=UnlessCondition(gpu))

    bridge = _include("uv_sim", "bridge.launch.py", {
        "hil_mode": "false",
        "camera_stitch_fps": camera_stitch_fps,
        "publish_raw_camera_topics": publish_raw,
        "profile_params": sim_profile_params,
    })
    control = _include("uv_control", "control_launch.py", {
        "enable_motion": enable_motion,
        "profile_params": "",
    })
    perception = _include("uv_camera", "perception_launch.py", {
        "enable_ai": enable_ai,
        "sim_mode": "true",
        "inference_fps": ai_inference_fps,
        "inference_threads": inference_threads,
        "confidence": ai_confidence,
        "gate_feature_mode": gate_feature_mode,
        "enable_gortc": LaunchConfiguration("enable_preview"),
        "gortc_http_port": gortc_http_port,
        "stream_annotated": LaunchConfiguration("stream_annotated"),
        "mjpeg_port": LaunchConfiguration("preview_port"),
        "annotated_max_width": LaunchConfiguration("annotated_max_width"),
        "profile_params": camera_profile_params,
        "object_localizer_params": PathJoinSubstitution([
            FindPackageShare("uv_camera"),
            "config",
            "object_localizer_sim.yaml",
        ]),
    }, condition=IfCondition(enable_ai))

    navigation = _include("uv_nav", "navigation_launch.py", {
        "enable_nav": enable_nav,
        "profile_params": "",
    })
    task = _include("uv_task", "task_launch.py", {
        "enable_task": enable_task,
        "target_id": target_id,
        "profile_params": "",
        "mission_file": mission_file,
    })

    readiness_backend = _include("uv_bringup", "readiness.launch.py", {
        "phase": "backend",
        "require_ai": "false",
        "timeout": startup_timeout,
    })
    readiness_control = _include("uv_bringup", "readiness.launch.py", {
        "phase": "control",
        "require_ai": "false",
        "timeout": startup_timeout,
    })
    readiness_sensors = _include("uv_bringup", "readiness.launch.py", {
        "phase": "sensors",
        "require_ai": enable_ai,
        "timeout": startup_timeout,
    })
    readiness_perception = _include("uv_bringup", "readiness.launch.py", {
        "phase": "perception",
        "require_ai": enable_ai,
        "timeout": startup_timeout,
    })

    observability = _include("uv_bringup", "observability.launch.py", {
        "enable_ai": enable_ai,
        "enable_preview": LaunchConfiguration("enable_preview"),
        "open_annotated_windows": LaunchConfiguration("open_annotated_windows"),
        "stream_annotated": LaunchConfiguration("stream_annotated"),
        "preview_port": LaunchConfiguration("preview_port"),
        "preview_width": LaunchConfiguration("preview_width"),
        "preview_height": LaunchConfiguration("preview_height"),
        "preview_wait_timeout": LaunchConfiguration("preview_wait_timeout"),
        "sim_window_width": sim_window_width,
        "record_session": LaunchConfiguration("record_session"),
        "record_root": LaunchConfiguration("record_root"),
        "record_raw_video": LaunchConfiguration("record_raw_video"),
        "record_video_mode": LaunchConfiguration("record_video_mode"),
        "record_video_format": LaunchConfiguration("record_video_format"),
        "record_video_fps": LaunchConfiguration("record_video_fps"),
        "record_video_codec": LaunchConfiguration("record_video_codec"),
        "record_image_topics": LaunchConfiguration("record_image_topics"),
        "video_segment_seconds": LaunchConfiguration("video_segment_seconds"),
        "bag_segment_seconds": LaunchConfiguration("bag_segment_seconds"),
        "record_use_sim_time": LaunchConfiguration("record_use_sim_time"),
        "camera_stitch_fps": camera_stitch_fps,
    })

    def _critical_exit(event, context):
        if context.is_shutdown:
            return []
        critical = {
            "stonefish_simulator",
            "stonefish_simulator_nogpu",
            "sim_bridge",
            "basic_motion",
        }
        name = str(getattr(event, "process_name", "") or "")
        if any(name == key or name.startswith(f"{key}-") for key in critical):
            return [EmitEvent(event=Shutdown(
                reason=f"critical bringup process exited: {name}"))]
        return []

    def _after_readiness(event, context):
        if context.is_shutdown:
            return []
        name = str(getattr(event, "process_name", "") or "")
        if name.startswith("wait_for_sim_backend"):
            if event.returncode != 0:
                return [LogInfo(msg="Backend readiness failed; downstream startup withheld")]
            return [control, readiness_control]
        if name.startswith("wait_for_sim_control"):
            if event.returncode != 0:
                return [LogInfo(msg="Control readiness failed; sensor/perception startup withheld")]
            return [readiness_sensors]
        if name.startswith("wait_for_sim_sensors"):
            if event.returncode != 0:
                return [LogInfo(msg="Sensor readiness failed; downstream startup withheld")]
            return [perception, readiness_perception]
        if name.startswith("wait_for_sim_perception"):
            if event.returncode != 0:
                return [LogInfo(msg="Perception readiness failed; navigation/task startup withheld")]
            return [navigation, task]
        return []

    start_actions = [
        stonefish_gpu,
        stonefish_nogpu,
        bridge,
        readiness_backend,
    ]

    return LaunchDescription([
        declare_profile("sim", "sim_dev"),
        declare_mission_file(),
        *declare_feature_arguments(
            enable_ai="true", enable_nav="false", enable_task="false",
            enable_motion="true",
        ),
        *declare_simulation_arguments(
            window_width_default=str(sim_width),
            window_height_default=str(sim_height),
            render_quality_default="low",
            camera_stitch_fps_default="5.0",
        ),
        *desktop_environment,
        configure_simulator_gpu_environment(gpu, gpu_backend),
        *declare_observability_arguments(
            preview_width_default=str(preview_width),
            preview_height_default=str(preview_height),
        ),
        RegisterEventHandler(OnProcessExit(on_exit=_critical_exit)),
        RegisterEventHandler(OnProcessExit(on_exit=_after_readiness)),
        SetEnvironmentVariable("SDL_VIDEO_WINDOW_POS", f"{monitor_x},{monitor_y}"),
        validate_profile(profile, "sim"),
        *headless_graphics,
        observability,
        LogInfo(msg=["Simulation profile: ", profile]),
        prepare_scene(
            scenario_desc=scenario_desc,
            scene_seed=scene_seed,
            launch_file=__file__,
            start_actions=start_actions,
        ),
    ])
