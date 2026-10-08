"""Staged real-vehicle bringup entry point."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from uv_bringup.launch_common import (
    declare_feature_arguments,
    declare_observability_arguments,
    declare_profile,
)


def generate_launch_description():
    profile_actions = declare_profile(
        'real', 'uv_bringup', choices=('default', 'record', 'debug', 'task'))
    features = declare_feature_arguments(
        enable_ai='true', enable_nav='false', enable_task=None,
        enable_motion='true')
    observability = declare_observability_arguments()
    declarations = [
        *profile_actions,
        *features,
        DeclareLaunchArgument('camera_stitch_fps', default_value='5.0'),
        *observability,
        DeclareLaunchArgument(
            'startup_mode', default_value='auto',
            choices=('auto', 'adopt', 'managed'),
            description=(
                'auto reuses compatible nodes; adopt only observes; '
                'managed refuses all pre-existing component nodes')),
        DeclareLaunchArgument('ready_timeout', default_value='120.0'),
        DeclareLaunchArgument('readiness_max_age', default_value='2.0'),
        DeclareLaunchArgument(
            'check_backend_health', default_value='false',
            description=(
                'Require healthy MCU/localization/odom during startup. '
                'Health is always printed for observation.')),
        DeclareLaunchArgument('enable_hardware', default_value='true'),
        DeclareLaunchArgument(
            'enable_camera', default_value='false',
            description=(
                'Launch camera hardware. False permits core bringup while '
                'camera/AI wait for a separately started camera node.')),
        DeclareLaunchArgument(
            'enable_perception_gate', default_value='false',
            description=(
                'Require healthy perception output before startup advances.')),
        DeclareLaunchArgument('enable_stream', default_value='true'),
        DeclareLaunchArgument('enable_perception_gui', default_value='false'),
        DeclareLaunchArgument('camera_config_dir', default_value=''),
    ]

    manager_arguments = [
        '--profile', LaunchConfiguration('profile'),
        '--startup-mode', LaunchConfiguration('startup_mode'),
        '--ready-timeout', LaunchConfiguration('ready_timeout'),
        '--max-age', LaunchConfiguration('readiness_max_age'),
        '--check-backend-health', LaunchConfiguration('check_backend_health'),
        '--enable-hardware', LaunchConfiguration('enable_hardware'),
        '--enable-motion', LaunchConfiguration('enable_motion'),
        '--enable-camera', LaunchConfiguration('enable_camera'),
        '--enable-ai', LaunchConfiguration('enable_ai'),
        '--enable-perception-gate', LaunchConfiguration('enable_perception_gate'),
        '--enable-nav', LaunchConfiguration('enable_nav'),
        '--enable-stream', LaunchConfiguration('enable_stream'),
        '--enable-perception-gui', LaunchConfiguration('enable_perception_gui'),
        '--camera-config-dir', LaunchConfiguration('camera_config_dir'),
        '--record-session', LaunchConfiguration('record_session'),
        '--record-root', LaunchConfiguration('record_root'),
        '--record-mode', LaunchConfiguration('record_mode'),
        '--go2rtc-stream-mode', LaunchConfiguration('go2rtc_stream_mode'),
        '--go2rtc-video-format', LaunchConfiguration('go2rtc_video_format'),
        '--record-video-fps', LaunchConfiguration('record_video_fps'),
        '--record-video-codec', LaunchConfiguration('record_video_codec'),
        '--video-segment-seconds', LaunchConfiguration('video_segment_seconds'),
        '--bag-segment-seconds', LaunchConfiguration('bag_segment_seconds'),
        '--record-bag-storage', LaunchConfiguration('record_bag_storage'),
        '--record-image-topics', LaunchConfiguration('record_image_topics'),
        '--preview-port', LaunchConfiguration('preview_port'),
    ]
    manager = Node(
        package='uv_bringup', executable='real_startup',
        name='real_startup_manager', exec_name='real_startup_manager',
        arguments=manager_arguments, output='screen',
        # Foxy normalizes these as launch substitutions (an iterable), not
        # numeric Python values. Keep them as strings for Foxy compatibility.
        sigterm_timeout='22', sigkill_timeout='5',
        remappings=[('/tf', '/auv/tf'), ('/tf_static', '/auv/tf_static')],
    )
    return LaunchDescription([*declarations, manager])
