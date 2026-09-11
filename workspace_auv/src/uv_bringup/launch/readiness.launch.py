"""Launch one measured startup readiness gate."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    phase = LaunchConfiguration("phase")
    require_ai = LaunchConfiguration("require_ai")
    timeout = LaunchConfiguration("timeout")

    return LaunchDescription([
        DeclareLaunchArgument(
            "phase", choices=["backend", "control", "sensors", "perception"],
            description="Readiness phase to check",
        ),
        DeclareLaunchArgument("require_ai", default_value="true"),
        DeclareLaunchArgument("timeout", default_value="120.0"),
        Node(
            package="uv_bringup",
            executable="wait_for_sim",
            name=["wait_for_sim_", phase],
            exec_name=["wait_for_sim_", phase],
            output="both",
            arguments=[
                "--phase", phase,
                "--require-ai", require_ai,
                "--timeout", timeout,
            ],
        ),
    ])
