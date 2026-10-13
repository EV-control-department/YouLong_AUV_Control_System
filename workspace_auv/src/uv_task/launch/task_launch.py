"""uv_task entry point: the competition task runner.

The node list belongs here so the system presets (``real_bringup`` /
``sim_bringup`` / ``hil_bringup``) never declare nodes themselves -- they only
include this file.  Whether ``task_runner`` runs at all is the *preset's*
decision, expressed as a condition on this include.

``debug_mode`` is declared here (and forwarded by the presets) because it gates
the ``/task/exec`` service, which is how a human drives the vehicle one task at
a time::

    ros2 launch uv_sim_bringup sim_bringup.py enable_task:=true debug_mode:=true

Without the forwarding argument there was **no way** to turn it on from a preset
-- ``ros2 launch`` only accepts declared launch arguments, so
``ros2 launch ... --ros-args -p debug_mode:=true`` is rejected outright (``-p``
is a ``ros2 run`` flag).  Until 2026-10-12 ``task_launch.py`` started the node
with no parameters at all and no preset declared ``debug_mode``, so ``/task/exec``
was unreachable from every bringup path.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # The value is forwarded as the raw string on purpose: `-p debug_mode:=true`
    # is parsed by rclpy with YAML rules, so it arrives as a real bool.  Do not
    # "helpfully" wrap it in bool(value) -- bool('false') is True.  (Same
    # mechanism bridge_launch.py relies on for `hil_mode`.)
    debug_mode = LaunchConfiguration('debug_mode')

    return LaunchDescription([
        DeclareLaunchArgument(
            'debug_mode', default_value='false',
            description=(
                'true: enable the /task/exec service (drive single tasks by '
                'hand) and per-task debug logging'
            ),
        ),
        Node(
            package='uv_task',
            executable='task_runner',
            name='task_runner',
            output='screen',
            parameters=[{'debug_mode': debug_mode}],
        ),
    ])
