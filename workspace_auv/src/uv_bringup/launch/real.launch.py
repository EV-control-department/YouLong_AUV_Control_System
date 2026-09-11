"""Real vehicle runtime preset."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
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
    declare_feature_arguments,
    declare_mission_file,
    declare_profile,
    profile_path,
    validate_profile,
)


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
    hardware_profile_params = profile_path(profile, "uv_hm")
    camera_profile_params = profile_path(profile, "uv_camera")
    enable_ai = LaunchConfiguration("enable_ai")
    enable_motion = LaunchConfiguration("enable_motion")
    enable_nav = LaunchConfiguration("enable_nav")
    enable_task = LaunchConfiguration("enable_task")
    mission_file = LaunchConfiguration("mission_file")

    hardware = _include("uv_hm", "hardware_launch.py", {
        "enable_hardware": LaunchConfiguration("enable_hardware"),
        "profile_params": hardware_profile_params,
    })
    control = _include("uv_control", "control_launch.py", {
        "enable_motion": enable_motion,
        "profile_params": "",
    })
    perception = _include("uv_camera", "perception_launch.py", {
        "enable_ai": enable_ai,
        "sim_mode": "false",
        "inference_fps": "5.0",
        "inference_threads": "2",
        "confidence": "0.8",
        "gate_feature_mode": "auto",
        "enable_gortc": "true",
        "stream_annotated": "true",
        "mjpeg_port": "8090",
        "annotated_max_width": "0",
        "profile_params": camera_profile_params,
        "object_localizer_params": "",
    })
    navigation = _include("uv_nav", "navigation_launch.py", {
        "enable_nav": enable_nav,
        "profile_params": "",
    })
    task = _include("uv_task", "task_launch.py", {
        "enable_task": enable_task,
        "target_id": LaunchConfiguration("target_id"),
        "profile_params": "",
        "mission_file": mission_file,
    })

    def _critical_exit(event, context):
        if context.is_shutdown:
            return []
        name = str(getattr(event, "process_name", "") or "")
        if any(name == key or name.startswith(f"{key}-") for key in {
            "hw_manager", "basic_motion",
        }):
            return [EmitEvent(event=Shutdown(
                reason=f"critical real process exited: {name}"))]
        return []

    return LaunchDescription([
        declare_profile("real", "real_default"),
        declare_mission_file(),
        *declare_feature_arguments(
            enable_ai="true", enable_nav="true", enable_task="false",
            enable_motion="true",
        ),
        DeclareLaunchArgument("enable_hardware", default_value="true"),
        DeclareLaunchArgument("target_id", default_value="yellow_golf"),
        RegisterEventHandler(OnProcessExit(on_exit=_critical_exit)),
        validate_profile(profile, "real"),
        LogInfo(msg=["Real vehicle profile: ", profile]),
        hardware,
        control,
        perception,
        navigation,
        task,
    ])
