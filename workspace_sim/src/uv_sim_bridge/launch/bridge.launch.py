"""Launch the split Stonefish-to-AUV adapter owned by ``uv_sim_bridge``."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hil_mode = LaunchConfiguration('hil_mode')
    camera_stitch_fps = LaunchConfiguration('camera_stitch_fps')
    publish_raw = LaunchConfiguration('publish_raw_camera_topics')
    profile_params = LaunchConfiguration('profile_params')

    def _nodes(context):
        parameters = []
        profile = profile_params.perform(context).strip()
        if profile:
            parameters.append(profile)
        parameters.append({
            'hil_mode': hil_mode,
            'camera_stitch_fps': camera_stitch_fps,
            'publish_raw_camera_topics': publish_raw,
        })
        return [Node(
            package='uv_sim_bridge',
            executable='sim_bridge',
            name='sim_bridge',
            exec_name='sim_bridge',
            output='both',
            parameters=parameters,
        )]

    return LaunchDescription([
        DeclareLaunchArgument('hil_mode', default_value='false'),
        DeclareLaunchArgument('camera_stitch_fps', default_value='10.0'),
        DeclareLaunchArgument(
            'publish_raw_camera_topics', default_value='false',
            description='Deprecated; simulator images use shared memory'),
        DeclareLaunchArgument('profile_params', default_value=''),
        OpaqueFunction(function=_nodes),
    ])
