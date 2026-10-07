"""Launch the split Stonefish-to-AUV adapter owned by ``uv_sim_bridge``."""

from auv_protocol.topics import DVL_VELOCITY, IMU
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hil_mode = LaunchConfiguration('hil_mode')
    camera_stitch_fps = LaunchConfiguration('camera_stitch_fps')
    publish_raw = LaunchConfiguration('publish_raw_camera_topics')
    params_file = LaunchConfiguration('params_file')

    def _nodes(context):
        parameters = []
        parameter_file = params_file.perform(context).strip()
        if parameter_file:
            parameters.append(parameter_file)
        parameters.append({
            'hil_mode': hil_mode,
            'dvl_topic': LaunchConfiguration('dvl_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
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
        DeclareLaunchArgument('dvl_topic', default_value=DVL_VELOCITY),
        DeclareLaunchArgument('imu_topic', default_value=IMU),
        DeclareLaunchArgument('camera_stitch_fps', default_value='10.0'),
        DeclareLaunchArgument(
            'publish_raw_camera_topics', default_value='false',
            description='Deprecated; simulator images use shared memory'),
        DeclareLaunchArgument('params_file', default_value=''),
        OpaqueFunction(function=_nodes),
    ])
