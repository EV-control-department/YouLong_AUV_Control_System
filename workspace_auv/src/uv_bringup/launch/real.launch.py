"""Real vehicle system bringup."""

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
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from uv_bringup.launch_common import (
    declare_feature_arguments,
    declare_mission_file,
    declare_observability_arguments,
    declare_profile,
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
    enable_ai = LaunchConfiguration("enable_ai")
    enable_motion = LaunchConfiguration("enable_motion")
    enable_nav = LaunchConfiguration("enable_nav")
    enable_task = LaunchConfiguration("enable_task")
    enable_perception_gui = LaunchConfiguration('enable_perception_gui')
    mission_file = LaunchConfiguration("mission_file")
    camera_config_dir = LaunchConfiguration("camera_config_dir")
    enable_stream = LaunchConfiguration("enable_stream")

    model_mapping = _include("uv_perception", "model_mapping_launch.py", {})
    description = _include("auv_description", "description.launch.py", {
        "use_sim_time": "false",
    })
    localization = _include("uv_localization", "localization_launch.py", {
        "sim_mode": "false",
        "publish_tf": "true",
    })

    hardware = _include("uv_hm", "hardware_launch.py", {
        "enable_hardware": LaunchConfiguration("enable_hardware"),
    })
    control = _include("uv_control", "control_launch.py", {
        "enable_motion": enable_motion,
        "sim_mode": "false",
        "params_file": "",
    })
    camera = _include("uv_camera", "camera_launch.py", {
        "sim_mode": "false", "camera_mode": "real",
        "camera_config_dir": camera_config_dir,
    })
    perception = _include("uv_perception", "perception_launch.py", {
        "confidence": "0.8",
        "enable_gui": enable_perception_gui,
    }, condition=IfCondition(enable_ai))
    stream = _include(
        "uv_stream", "stream_launch.py", {},
        condition=IfCondition(enable_stream))
    observability = _include("uv_bringup", "observability.launch.py", {
        "enable_preview": LaunchConfiguration("enable_preview"),
        "preview_port": LaunchConfiguration("preview_port"),
        "record_session": LaunchConfiguration("record_session"),
        "record_root": LaunchConfiguration("record_root"),
        "record_mode": LaunchConfiguration("record_mode"),
        "go2rtc_stream_mode": LaunchConfiguration("go2rtc_stream_mode"),
        "go2rtc_video_format": LaunchConfiguration("go2rtc_video_format"),
        "record_video_fps": LaunchConfiguration("record_video_fps"),
        "record_video_codec": LaunchConfiguration("record_video_codec"),
        "video_segment_seconds": LaunchConfiguration("video_segment_seconds"),
        "bag_segment_seconds": LaunchConfiguration("bag_segment_seconds"),
        "record_bag_storage": LaunchConfiguration("record_bag_storage"),
        "record_use_sim_time": LaunchConfiguration("record_use_sim_time"),
        "record_image_topics": LaunchConfiguration("record_image_topics"),
    })
    navigation = _include("uv_planning", "planning_launch.py", {
        "enable_nav": enable_nav,
        "params_file": "",
    })
    task = _include("uv_task", "task_launch.py", {
        "enable_task": enable_task,
        "params_file": "",
        "camera_mode": "real",
        "camera_config_dir": camera_config_dir,
        "mission_file": mission_file,
    })

    def _critical_exit(event, context):
        if context.is_shutdown:
            return []
        name = str(getattr(event, "process_name", "") or "")
        if any(name == key or name.startswith(f"{key}-") for key in {
            "hw_manager", "basic_motion", "uv_localization",
        }):
            return [EmitEvent(event=Shutdown(
                reason=f"critical real process exited: {name}"))]
        return []

    return LaunchDescription([
        *declare_profile(
            "real", "uv_bringup",
            choices=("default", "record", "debug", "task"),
        ),
        declare_mission_file(),
        *declare_feature_arguments(
            enable_ai="true", enable_nav="false", enable_task="false",
            enable_motion="true",
        ),
        *declare_observability_arguments(),
        DeclareLaunchArgument(
            "camera_stitch_fps", default_value="5.0",
            description="Default source-camera frame rate used by the recorder",
        ),
        DeclareLaunchArgument(
            "enable_stream", default_value="true",
            description="Launch go2rtc preview streams; raw recording can run with this disabled",
        ),
        DeclareLaunchArgument("enable_perception_gui", default_value="false"),
        DeclareLaunchArgument("enable_hardware", default_value="true"),
        DeclareLaunchArgument(
            "camera_config_dir", default_value="",
            description="Optional directory containing front.yaml and down.yaml",
        ),
        RegisterEventHandler(OnProcessExit(on_exit=_critical_exit)),
        LogInfo(msg=["Real vehicle profile: ", profile]),
        model_mapping,
        description,
        localization,
        hardware,
        control,
        camera,
        perception,
        stream,
        observability,
        navigation,
        task,
    ])
