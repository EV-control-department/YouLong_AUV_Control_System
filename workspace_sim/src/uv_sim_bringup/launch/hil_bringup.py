"""HIL bringup: Stonefish in the loop, with the *real* MCU doing PID and mixing.

**Status: PAUSED.**  The project is currently software-in-the-loop only, so this
entry point is kept correct and statically checked but is not accepted at
runtime (see ``docs/architecture/sim_real_boundary.md``).  Known unverified
change: the MCU link is now included from ``uv_hm/hm_launch.py`` instead of
being duplicated here, which also means ``hw_manager`` now runs in HIL -- that
is intended (the real MCU needs the ``/zit6/cmd/agxhbt`` heartbeat to arm) but
has not been exercised against hardware.

``sim_bridge`` runs with ``hil_mode:=true``: it forwards camera images and relays
thrusters, leaving PID and mixing to the MCU over micro-ROS.
"""

from launch import LaunchDescription
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration

from uv_bringup.launch_common import arg, include
from uv_sim_bringup.stonefish_common import (
    engine,
    resolve_simulation_data,
    scenario_check,
)


def generate_launch_description():
    data_dir, warnings = resolve_simulation_data()

    return LaunchDescription([
        arg('scenario_desc', 'underwater_xunyun.scn',
            'Stonefish scenario file name (in the Data/ directory)'),
        arg('headless', 'false',
            'Use the console simulator: no rendering window, and no camera images '
            '(which also disables the HIL camera relay)'),
        arg('simulation_rate', '100.0',
            'Physics rate; 0 means real time, which the console simulator needs '
            'before it publishes any ROS topic'),
        arg('window_res_x', '2560', 'Simulator window width (GPU mode only)'),
        arg('window_res_y', '1440', 'Simulator window height (GPU mode only)'),
        arg('rendering_quality', 'high',
            'Rendering quality low/medium/high (GPU mode only)'),

        arg('enable_mcu', 'true',
            'Enable sim_bridge in HIL mode (camera relay + thruster passthrough)'),
        arg('enable_motion', 'false',
            'Enable basic_motion (the motion action server)'),
        arg('enable_ai', 'false', 'Enable AI perception nodes'),
        arg('enable_nav', 'false', 'Enable navigation node'),
        arg('enable_task', 'false', 'Enable task runner'),

        arg('enable_agent', 'true',
            'Start the micro-ROS agent that links the MCU over serial'),
        arg('serial_dev', '/dev/ttyUSB0',
            'Serial device the MCU is connected to'),
        arg('serial_baud', '921600', 'Baud rate of the micro-ROS serial link'),

        LogInfo(msg=['Stonefish asset directory: ', data_dir]),
        *[LogInfo(msg=['[hil] ', warning]) for warning in warnings],
        scenario_check(data_dir),

        engine(data_dir),
        include('uv_sim', 'bridge_launch.py',
                arguments={'hil_mode': 'true'}, when='enable_mcu'),
        include('uv_hm', 'hm_launch.py', arguments={
            'enable_agent': LaunchConfiguration('enable_agent'),
            'serial_dev': LaunchConfiguration('serial_dev'),
            'serial_baud': LaunchConfiguration('serial_baud'),
        }),
        include('uv_control', 'control_launch.py', when='enable_motion'),
        include('uv_perception', 'perception_launch.py',
                arguments={'sim_mode': 'true'}, when='enable_ai'),
        include('uv_nav', 'nav_launch.py', when='enable_nav'),
        include('uv_task', 'task_launch.py', when='enable_task'),
    ])
