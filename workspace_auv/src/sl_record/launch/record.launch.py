"""Start the structured-light camera and an isolated recording session."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler,
    SetEnvironmentVariable, Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from sl_record.recorder import default_output_root
from uv_record.launch_logging import session_log_handlers
from uv_record.session import create_session


def _start_session(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    paths = create_session(value('output_root'))
    camera = Node(
        package='sl_camera', executable='sl_camera', name='sl_camera',
        parameters=[value('config')], output='both')
    recorder = Node(
        package='sl_record', executable='record', name='sl_record_recorder',
        arguments=[
            '--session-dir', str(paths.root),
            '--video-fps', value('record_video_fps'),
            '--bag-duration', value('bag_duration'),
            '--bag-storage', value('bag_storage'),
        ],
        output='both', sigterm_timeout='20', sigkill_timeout='5')
    return [
        SetEnvironmentVariable('ROS_LOG_DIR', str(paths.logs / 'ros')),
        *session_log_handlers(paths, {
            name: value(name) for name in (
                'config', 'output_root', 'record_video_fps', 'bag_duration', 'bag_storage')
        }),
        LogInfo(msg=['sl_record session: ', str(paths.root)]),
        RegisterEventHandler(OnProcessExit(
            target_action=camera,
            on_exit=[Shutdown(reason='sl_camera exited')])),
        RegisterEventHandler(OnProcessExit(
            target_action=recorder,
            on_exit=[Shutdown(reason='sl_record exited')])),
        camera,
        recorder,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=get_package_share_directory('sl_camera')
            + '/config/default.yaml'),
        DeclareLaunchArgument('output_root', default_value=str(default_output_root())),
        DeclareLaunchArgument('record_video_fps', default_value='30.0'),
        DeclareLaunchArgument('bag_duration', default_value='10.0'),
        DeclareLaunchArgument('bag_storage', default_value='auto'),
        OpaqueFunction(function=_start_session),
    ])
