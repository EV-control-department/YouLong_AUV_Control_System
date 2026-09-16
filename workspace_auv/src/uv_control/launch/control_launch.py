"""uv_control entry point: the motion action server.

The node list belongs here so the system presets (``real_bringup`` /
``sim_bringup`` / ``hil_bringup``) never declare nodes themselves -- they only
include this file.  Whether ``basic_motion`` runs at all is the *preset's*
decision, expressed as a condition on this include, so there is no
``enable_motion`` argument to keep in sync here as well.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uv_control',
            executable='basic_motion',
            name='basic_motion',
            output='screen',
        ),
    ])
