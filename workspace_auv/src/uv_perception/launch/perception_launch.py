"""uv_perception entry point: detection (``vision``) and 3D localization (``position``).

The node list belongs here so the system presets never declare nodes themselves.
``sim_mode`` is a *node* option (where the images come from), not a "should I
run" switch -- whether perception starts at all is the preset's decision.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'sim_mode', default_value='false',
            description=(
                'true: subscribe to Stonefish camera topics '
                '(/auv/*_cam/stitched); false: capture from V4L2 devices'
            )),

        Node(
            package='uv_perception',
            executable='vision',
            name='vision',
            output='screen',
            parameters=[{'sim_mode': LaunchConfiguration('sim_mode')}],
        ),
        Node(
            package='uv_perception',
            executable='position',
            name='position',
            output='screen',
        ),
    ])
