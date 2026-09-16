"""uv_sim entry point: the simulated MCU (``sim_bridge``).

SIL: setpoints in -> PID + thruster mixing -> thrusters, plus the ``/zit6/state/*``
telemetry the control stack consumes.  HIL (``hil_mode:=true``) keeps only the
camera relay and the thruster passthrough, because the real MCU owns PID and
mixing there.

Lives here rather than in the preset so that splitting this node up later (the
T5 task) touches only this package.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'hil_mode', default_value='false',
            description=(
                'true: forward camera images and relay thrusters only '
                '(a real MCU owns PID and mixing); false: simulate the MCU'
            )),
        DeclareLaunchArgument(
            'pid_config_file', default_value='',
            description=(
                'YAML file with the PID gains; empty means the packaged '
                'uv_sim/config/sim_pid.yaml'
            )),

        Node(
            package='uv_sim',
            executable='sim_bridge',
            name='sim_bridge',
            output='screen',
            parameters=[{
                'hil_mode': LaunchConfiguration('hil_mode'),
                'pid_config_file': LaunchConfiguration('pid_config_file'),
            }],
        ),
    ])
