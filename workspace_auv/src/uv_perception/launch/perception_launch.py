"""uv_perception entry point: detection (``vision``) and 3D localization (``position``).

The node list belongs here so the system presets never declare nodes themselves.
The arguments are *node* options (where the images come from, which model, whether
to spawn go2rtc) -- whether perception starts at all is the preset's decision.

``model_path`` and ``enable_gortc`` are exposed because the GPU/SIL acceptance run
needs them: without a model the node logs ``YOLO model not found, detection
disabled`` and the whole perception chain silently produces nothing.
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
                'true: subscribe to the simulator camera topics '
                '(/auv/*_cam/stitched); false: capture from V4L2 devices'
            )),
        DeclareLaunchArgument(
            'model_path', default_value='',
            description=(
                'YOLO segmentation weights (.pt); empty searches '
                '~/YouLong_AUV_Control_System/workspace_auv/src/datas and the repo'
            )),
        DeclareLaunchArgument(
            'enable_gortc', default_value='true',
            description=(
                'Spawn the go2rtc forwarding client (video streaming); set false '
                'for headless acceptance runs'
            )),

        Node(
            package='uv_perception',
            executable='vision',
            name='vision',
            output='screen',
            parameters=[{
                'sim_mode': LaunchConfiguration('sim_mode'),
                'model_path': LaunchConfiguration('model_path'),
                'enable_gortc': LaunchConfiguration('enable_gortc'),
            }],
        ),
        Node(
            package='uv_perception',
            executable='position',
            name='position',
            output='screen',
        ),
    ])
