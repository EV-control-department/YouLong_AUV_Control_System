"""Launch the planning boundary.

The first implementation delegates to the existing ``uv_nav`` navigator so
the competition behavior remains unchanged.  The public launch ownership and
topic contract already belong to ``uv_planning``; A* and Active SLAM can move
behind this boundary without changing bringup files again.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_nav = LaunchConfiguration('enable_nav')
    profile_params = LaunchConfiguration('profile_params')

    def _nodes(context):
        parameters = []
        profile = profile_params.perform(context).strip()
        if profile:
            parameters.append(profile)
        return [Node(
            package='uv_nav',
            executable='navigator',
            name='navigator',
            exec_name='navigator',
            output='both',
            parameters=parameters,
            remappings=[('/tf', '/auv/tf'), ('/tf_static', '/auv/tf_static')],
            condition=IfCondition(enable_nav),
        )]

    return LaunchDescription([
        DeclareLaunchArgument('enable_nav', default_value='true'),
        DeclareLaunchArgument('profile_params', default_value=''),
        OpaqueFunction(function=_nodes),
    ])
