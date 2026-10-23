"""Start profile-selected cameras and record any available sl/down frames."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler,
    SetEnvironmentVariable, Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from sl_camera.profile import camera_node_specs, load_profile
from sl_record.recorder import default_output_root
from uv_record.launch_logging import session_log_handlers
from uv_record.session import create_session


def _start_session(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    profile = load_profile(value('profile'))
    settings = {
        'output_root': value('output_root') or profile.recording['output_root']
        or str(default_output_root()),
        'record_video_fps': value('record_video_fps') or str(profile.recording['video_fps']),
        'bag_duration': value('bag_duration') or str(profile.recording['bag_duration']),
        'bag_storage': value('bag_storage') or profile.recording['bag_storage'],
        'profile': str(profile.path), 'config': value('config'),
    }
    paths = create_session(settings['output_root'])
    cameras = [Node(**spec) for spec in camera_node_specs(profile, value('config'))]
    recorder = Node(
        package='sl_record', executable='record', name='sl_record_recorder',
        arguments=[
            '--session-dir', str(paths.root),
            '--profile', str(profile.path),
            '--video-fps', settings['record_video_fps'],
            '--bag-duration', settings['bag_duration'],
            '--bag-storage', settings['bag_storage'],
        ],
        output='both', sigterm_timeout='20', sigkill_timeout='5')
    return [
        SetEnvironmentVariable('ROS_LOG_DIR', str(paths.logs / 'ros')),
        *session_log_handlers(paths, settings),
        LogInfo(msg=f'Camera profile: {profile.path}; '
                f'sl={profile.sl["enabled"]}, down={profile.down["enabled"]}; '
                'recording available Iceoryx2 sl/down frames'),
        LogInfo(msg=['sl_record session: ', str(paths.root)]),
        RegisterEventHandler(OnProcessExit(
            target_action=recorder,
            on_exit=[Shutdown(reason='sl_record exited')])),
        *cameras,
        recorder,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'profile', default_value='default',
            description='Shared collection profile name or YAML path'),
        DeclareLaunchArgument(
            'config', default_value='',
            description='Optional legacy ROS parameter YAML override for sl_camera'),
        DeclareLaunchArgument('output_root', default_value='',
                              description='Override recording.output_root in the profile'),
        DeclareLaunchArgument('record_video_fps', default_value='',
                              description='Override recording.video_fps in the profile'),
        DeclareLaunchArgument('bag_duration', default_value='',
                              description='Override recording.bag_duration in the profile'),
        DeclareLaunchArgument('bag_storage', default_value='',
                              description='Override recording.bag_storage in the profile'),
        OpaqueFunction(function=_start_session),
    ])
