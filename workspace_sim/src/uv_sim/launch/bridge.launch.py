"""Launch the simulation bridge component."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hil_mode = LaunchConfiguration("hil_mode")
    camera_stitch_fps = LaunchConfiguration("camera_stitch_fps")
    publish_raw = LaunchConfiguration("publish_raw_camera_topics")
    profile_params = LaunchConfiguration("profile_params")

    def _nodes(context):
        parameters = []
        profile = profile_params.perform(context).strip()
        if profile:
            parameters.append(profile)
        parameters.append({
            "hil_mode": hil_mode,
            "camera_stitch_fps": camera_stitch_fps,
            "publish_raw_camera_topics": publish_raw,
        })
        return [Node(
            package="uv_sim",
            executable="sim_bridge",
            name="sim_bridge",
            exec_name="sim_bridge",
            output="both",
            parameters=parameters,
        )]

    return LaunchDescription([
        DeclareLaunchArgument(
            "hil_mode", default_value="false",
            description="Use HIL camera passthrough and MCU control mode",
        ),
        DeclareLaunchArgument(
            "camera_stitch_fps", default_value="10.0",
            description="Maximum stitched camera topic rate",
        ),
        DeclareLaunchArgument(
            "publish_raw_camera_topics", default_value="false",
            description="Republish individual raw camera image topics",
        ),
        DeclareLaunchArgument(
            "profile_params", default_value="",
            description="Optional standard ROS 2 parameter file",
        ),
        OpaqueFunction(function=_nodes),
    ])
