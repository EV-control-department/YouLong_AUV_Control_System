"""Start exactly one camera manager and its catalog-discovered recorder."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler,
    SetEnvironmentVariable, Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from sl_camera.profile import load_settings
from sl_record.recorder import default_output_root
from uv_record.launch_logging import session_log_handlers
from uv_record.session import create_session


def _start_session(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    path, topic, recording, _ = load_settings(value('profile'))
    settings = {
        'profile': str(path), 'catalog_topic': topic,
        'output_root': value('output_root') or recording['output_root'] or str(default_output_root()),
        'record_video_fps': value('record_video_fps') or str(recording['video_fps']),
        'bag_duration': value('bag_duration') or str(recording['bag_duration']),
        'bag_storage': value('bag_storage') or recording['bag_storage'],
    }
    paths = create_session(settings['output_root'])
    camera = Node(
        package='sl_camera', executable='sl_camera', name='sl_camera',
        parameters=[{'profile': str(path)}], output='both')
    recorder = Node(
        package='sl_record', executable='record', name='sl_record_recorder',
        arguments=[
            '--profile', str(path), '--session-dir', str(paths.root),
            '--catalog-topic', topic, '--video-fps', settings['record_video_fps'],
            '--bag-duration', settings['bag_duration'], '--bag-storage', settings['bag_storage']],
        output='both', sigterm_timeout='20', sigkill_timeout='5')
    return [
        SetEnvironmentVariable('ROS_LOG_DIR', str(paths.logs / 'ros')),
        *session_log_handlers(paths, settings),
        LogInfo(msg=f'SL profile: {path}; discovery topic: {topic}; session: {paths.root}'),
        RegisterEventHandler(OnProcessExit(
            target_action=recorder, on_exit=[Shutdown(reason='sl_record exited')])),
        camera, recorder,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='default'),
        DeclareLaunchArgument('output_root', default_value=''),
        DeclareLaunchArgument('record_video_fps', default_value=''),
        DeclareLaunchArgument('bag_duration', default_value=''),
        DeclareLaunchArgument('bag_storage', default_value=''),
        OpaqueFunction(function=_start_session),
    ])
