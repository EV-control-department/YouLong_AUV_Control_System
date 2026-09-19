"""Crash-resilient session recording."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from uv_bringup.session_logging import session_log_handlers
from uv_log.session import default_output_root


def generate_launch_description():
    enable_preview = LaunchConfiguration("enable_preview")
    preview_port = LaunchConfiguration("preview_port")
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
    record_bag_storage = LaunchConfiguration("record_bag_storage")
    record_use_sim_time = LaunchConfiguration("record_use_sim_time")

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
                "--bag-storage", record_bag_storage.perform(context),
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

    return LaunchDescription([
        DeclareLaunchArgument("enable_preview", default_value="true"),
        DeclareLaunchArgument("preview_port", default_value="8090"),
        DeclareLaunchArgument("record_session", default_value="false"),
        DeclareLaunchArgument("record_root", default_value=str(default_output_root())),
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
        DeclareLaunchArgument("record_bag_storage", default_value="auto"),
        DeclareLaunchArgument("record_use_sim_time", default_value="false"),
        OpaqueFunction(function=_recording_actions),
    ])
