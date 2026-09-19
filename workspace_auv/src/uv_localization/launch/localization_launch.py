"""Launch the canonical localization boundary."""

from auv_protocol.topics import DVL_VELOCITY, IMU, USBL_MEASUREMENT
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def generate_launch_description():
    sim_mode = LaunchConfiguration('sim_mode')
    publish_tf = LaunchConfiguration('publish_tf')
    estimator = LaunchConfiguration('estimator')
    dvl_topic = LaunchConfiguration('dvl_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    usbl_topic = LaunchConfiguration('usbl_topic')

    def _nodes(context):
        return [Node(
            package='uv_localization',
            executable='estimator',
            name='uv_localization',
            exec_name='uv_localization',
            output='both',
            parameters=[{
                'sim_mode': _as_bool(sim_mode.perform(context)),
                'publish_tf': _as_bool(publish_tf.perform(context)),
                'estimator': estimator.perform(context).strip(),
            }],
            remappings=[
                ('/tf', '/auv/tf'), ('/tf_static', '/auv/tf_static'),
                (DVL_VELOCITY, dvl_topic),
                (IMU, imu_topic),
                (USBL_MEASUREMENT, usbl_topic),
            ],
        )]

    return LaunchDescription([
        DeclareLaunchArgument('sim_mode', default_value='false'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('estimator', default_value='bootstrap'),
        DeclareLaunchArgument('dvl_topic', default_value=DVL_VELOCITY),
        DeclareLaunchArgument('imu_topic', default_value=IMU),
        DeclareLaunchArgument('usbl_topic', default_value=USBL_MEASUREMENT),
        OpaqueFunction(function=_nodes),
    ])
