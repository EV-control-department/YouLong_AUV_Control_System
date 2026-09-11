"""Optional preview windows and crash-resilient session recording."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from uv_bringup.desktop import focused_monitor
from uv_bringup.session_logging import session_log_handlers


def generate_launch_description():
    enable_ai = LaunchConfiguration("enable_ai")
    enable_preview = LaunchConfiguration("enable_preview")
    open_windows = LaunchConfiguration("open_annotated_windows")
    stream_annotated = LaunchConfiguration("stream_annotated")
    preview_port = LaunchConfiguration("preview_port")
    preview_width = LaunchConfiguration("preview_width")
    preview_height = LaunchConfiguration("preview_height")
    preview_wait_timeout = LaunchConfiguration("preview_wait_timeout")
    sim_window_width = LaunchConfiguration("sim_window_width")
    record_session = LaunchConfiguration("record_session")
    record_root = LaunchConfiguration("record_root")
    record_raw_video = LaunchConfiguration("record_raw_video")
    record_video_mode = LaunchConfiguration("record_video_mode")
    record_video_format = LaunchConfiguration("record_video_format")
    record_video_fps = LaunchConfiguration("record_video_fps")
    record_video_codec = LaunchConfiguration("record_video_codec")
    record_image_topics = LaunchConfiguration("record_image_topics")
    video_segment_seconds = LaunchConfiguration("video_segment_seconds")
    bag_segment_seconds = LaunchConfiguration("bag_segment_seconds")
    record_use_sim_time = LaunchConfiguration("record_use_sim_time")

    monitor_x, monitor_y, monitor_width, monitor_height = focused_monitor()

    def _recording_actions(context):
        if record_session.perform(context).strip().lower() not in (
            "1", "true", "yes", "on"
        ):
            return []

        from uv_log.session import create_session

        paths = create_session(record_root.perform(context))
        recorder = Node(
            package="uv_log",
            executable="record",
            name="uv_log_recorder",
            output="both",
            arguments=[
                "--session-dir", str(paths.root),
                "--segment-duration", video_segment_seconds.perform(context),
                "--bag-duration", bag_segment_seconds.perform(context),
                "--record-raw", record_raw_video.perform(context),
                "--video-mode", record_video_mode.perform(context),
                "--video-format", record_video_format.perform(context),
                "--video-fps", record_video_fps.perform(context),
                "--video-codec", record_video_codec.perform(context),
                "--record-image-topics", record_image_topics.perform(context),
                "--use-sim-time", record_use_sim_time.perform(context),
                "--port", preview_port.perform(context),
                "--enable-video", enable_preview.perform(context),
            ],
            sigterm_timeout="20",
            sigkill_timeout="5",
        )
        return [
            SetEnvironmentVariable("ROS_LOG_DIR", str(paths.logs / "ros")),
            *session_log_handlers(paths, dict(context.launch_configurations)),
            LogInfo(msg=["uv_log session: ", str(paths.root)]),
            recorder,
        ]

    preview = Node(
        package="uv_bringup",
        executable="annotated_preview",
        name="annotated_preview",
        output="both",
        arguments=[
            "--port", preview_port,
            "--width", preview_width,
            "--height", preview_height,
            "--sim-width", sim_window_width,
            "--monitor-x", str(monitor_x),
            "--monitor-y", str(monitor_y),
            "--monitor-width", str(monitor_width),
            "--monitor-height", str(monitor_height),
            "--sim-title", "Stonefish Simulator",
            "--wait-timeout", preview_wait_timeout,
        ],
        condition=IfCondition(PythonExpression([
            "'", enable_ai, "'.lower() == 'true' and '",
            enable_preview, "'.lower() == 'true' and '",
            open_windows, "'.lower() == 'true' and '",
            stream_annotated, "'.lower() == 'true'",
        ])),
    )

    return LaunchDescription([
        DeclareLaunchArgument("enable_ai", default_value="true"),
        DeclareLaunchArgument("enable_preview", default_value="true"),
        DeclareLaunchArgument("open_annotated_windows", default_value="true"),
        DeclareLaunchArgument("stream_annotated", default_value="true"),
        DeclareLaunchArgument("preview_port", default_value="8090"),
        DeclareLaunchArgument("preview_width", default_value="960"),
        DeclareLaunchArgument("preview_height", default_value="540"),
        DeclareLaunchArgument("preview_wait_timeout", default_value="60.0"),
        DeclareLaunchArgument("sim_window_width", default_value="960"),
        DeclareLaunchArgument("record_session", default_value="false"),
        DeclareLaunchArgument("record_root", default_value="sessions"),
        DeclareLaunchArgument("record_raw_video", default_value="false"),
        DeclareLaunchArgument("record_video_mode", default_value="raw"),
        DeclareLaunchArgument("record_video_format", default_value="jpeg"),
        DeclareLaunchArgument(
            "camera_stitch_fps", default_value="5.0",
            description="Default camera rate used by the recorder",
        ),
        DeclareLaunchArgument(
            "record_video_fps", default_value=LaunchConfiguration("camera_stitch_fps")
        ),
        DeclareLaunchArgument("record_video_codec", default_value="libx264"),
        DeclareLaunchArgument("record_image_topics", default_value="false"),
        DeclareLaunchArgument("video_segment_seconds", default_value="2.0"),
        DeclareLaunchArgument("bag_segment_seconds", default_value="10.0"),
        DeclareLaunchArgument("record_use_sim_time", default_value="false"),
        OpaqueFunction(function=_recording_actions),
        preview,
    ])
