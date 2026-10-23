"""Launch profile-selected structured-light and downward stereo cameras."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from sl_camera.profile import camera_node_specs, load_profile


def _start_cameras(context):
    profile = load_profile(LaunchConfiguration('profile').perform(context))
    config = LaunchConfiguration('config').perform(context)
    return [
        LogInfo(msg=f'Camera profile: {profile.path}; '
                f'sl={profile.sl["enabled"]}, down={profile.down["enabled"]}'),
        *[Node(**spec) for spec in camera_node_specs(profile, config)],
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'profile', default_value='default',
            description='Shared collection profile name or YAML path'),
        DeclareLaunchArgument(
            'config', default_value='',
            description='Optional legacy ROS parameter YAML override for sl_camera'),
        OpaqueFunction(function=_start_cameras),
    ])
