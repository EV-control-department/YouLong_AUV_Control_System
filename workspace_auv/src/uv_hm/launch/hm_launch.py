"""Launch file for hardware manager node (real hardware only).

Launches hw_manager with configurable parameters for heartbeat and watchdog.

Usage:
    ros2 launch uv_hm hm_launch.py
    ros2 launch uv_hm hm_launch.py arm_mode:=3 watchdog_timeout:=10.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_arm_mode = DeclareLaunchArgument(
        'arm_mode', default_value='1',
        description='Arm mode: 1=normal (INS required), 3=force (bypass INS).'
    )
    declare_heartbeat_rate = DeclareLaunchArgument(
        'heartbeat_rate', default_value='10.0',
        description='Heartbeat publish rate (Hz).'
    )
    declare_watchdog_timeout = DeclareLaunchArgument(
        'watchdog_timeout', default_value='7.0',
        description='MCU heartbeat timeout before ERROR (seconds).'
    )

    arm_mode = LaunchConfiguration('arm_mode')
    heartbeat_rate = LaunchConfiguration('heartbeat_rate')
    watchdog_timeout = LaunchConfiguration('watchdog_timeout')

    hw_manager = Node(
        package='uv_hm',
        executable='hw_manager',
        name='hw_manager',
        output='screen',
        parameters=[{
            'arm_mode': arm_mode,
            'heartbeat_rate': heartbeat_rate,
            'watchdog_timeout': watchdog_timeout,
        }],
    )

    return LaunchDescription([
        declare_arm_mode,
        declare_heartbeat_rate,
        declare_watchdog_timeout,
        hw_manager,
    ])
