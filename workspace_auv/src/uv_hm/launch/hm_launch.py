"""uv_hm entry point: the hardware manager and the MCU serial link.

Both halves belong to the same subsystem -- the micro-ROS agent
(``MicroXRCEAgent serial``) is the transport to the real STM32, and
``hw_manager`` owns the heartbeat (``/zit6/cmd/agxhbt``) that keeps the MCU
armed.  A real MCU needs both, in HIL and on the vehicle alike, so the two live
in one file instead of being duplicated in each preset.

The node itself stays in ``uv_hm``; only its launch arguments live here.
"""

import os
import shutil
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

#: Where the agent was built on the operator PC during the 2026-09 HIL setup.
#: Only a default: ``agent_bin`` overrides it, and a ``micro_ros_agent`` on
#: ``PATH`` is preferred when the workspace is absent.
DEFAULT_AGENT_WS = Path.home() / 'micro_ros_agent_ws' / 'install'
DEFAULT_AGENT_BIN = (
    DEFAULT_AGENT_WS / 'micro_ros_agent' / 'lib' / 'micro_ros_agent' / 'micro_ros_agent'
)
DEFAULT_AGENT_LIB_DIRS = (
    DEFAULT_AGENT_WS / 'micro_ros_agent' / 'lib',
    DEFAULT_AGENT_WS / 'micro_ros_msgs' / 'lib',
)


def _agent_binary() -> str:
    """Absolute path of the agent: the known workspace build, else ``$PATH``."""
    if DEFAULT_AGENT_BIN.is_file():
        return DEFAULT_AGENT_BIN.as_posix()
    return shutil.which('micro_ros_agent') or DEFAULT_AGENT_BIN.as_posix()


def _agent_env() -> dict:
    """``LD_LIBRARY_PATH`` with the agent's own libraries in front."""
    libs = [p.as_posix() for p in DEFAULT_AGENT_LIB_DIRS]
    existing = os.environ.get('LD_LIBRARY_PATH', '')
    if existing:
        libs.append(existing)
    return {'LD_LIBRARY_PATH': ':'.join(libs)}


def generate_launch_description():
    serial_dev = LaunchConfiguration('serial_dev')
    serial_baud = LaunchConfiguration('serial_baud')

    agent = ExecuteProcess(
        cmd=[
            LaunchConfiguration('agent_bin'),
            'serial', '-D', serial_dev, '-b', serial_baud, '-v', '4',
        ],
        additional_env=_agent_env(),
        output='screen',
        name='micro_ros_agent',
        condition=IfCondition(LaunchConfiguration('enable_agent')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_agent', default_value='true',
            description='Start the micro-ROS agent (MCU serial link)'),
        DeclareLaunchArgument(
            'agent_bin', default_value=_agent_binary(),
            description='micro_ros_agent executable (absolute path)'),
        DeclareLaunchArgument(
            'serial_dev', default_value='/dev/ttyUSB0',
            description='Serial device the MCU is connected to'),
        DeclareLaunchArgument(
            'serial_baud', default_value='921600',
            description='Baud rate of the micro-ROS serial link'),

        LogInfo(msg=['micro-ROS serial device: ', serial_dev, ' @ ', serial_baud]),

        Node(
            package='uv_hm',
            executable='hw_manager',
            name='hw_manager',
            output='screen',
        ),
        agent,
    ])
