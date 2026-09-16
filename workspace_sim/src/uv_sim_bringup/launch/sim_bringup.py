"""Simulation bringup (SIL by default) -- the simulated counterpart of real_bringup.

This preset declares **no nodes**: it orchestrates the entry points owned by each
package (``uv_sim``, ``uv_control``, ``uv_perception``, ``uv_nav``, ``uv_task``,
``uv_hm``) plus the Stonefish engine, which is included through its own launch
file.  Adding or splitting a node therefore never means editing this file.

The retired ``uv_bringup sim_core.py`` (engine + simulated MCU, nothing else) is
now the same entry point with the upper layers switched off::

    ros2 launch uv_sim_bringup sim_bringup.py enable_motion:=false enable_ai:=false

``workspace_auv`` no longer ships any simulation entry point at all.
"""

from launch import LaunchDescription
from launch.actions import LogInfo

from uv_bringup.launch_common import arg, include
from uv_sim_bringup.stonefish_common import (
    engine,
    resolve_simulation_data,
    scenario_check,
)


def generate_launch_description():
    data_dir, warnings = resolve_simulation_data()

    return LaunchDescription([
        arg('scenario_desc', 'wuurc_murc_2026_auv.scn',
            'Stonefish scenario file name (in the Data/ directory)'),
        arg('headless', 'false',
            'Use the console simulator: no rendering window, and no camera images '
            '(the console engine rejects camera sensors)'),
        arg('simulation_rate', '100.0',
            'Physics rate in steps/s (fixed step). Keep it positive: 0 stops the '
            'simulation loop and nothing is published (see stonefish_notes.md)'),
        arg('window_res_x', '2560', 'Simulator window width (GPU mode only)'),
        arg('window_res_y', '1440', 'Simulator window height (GPU mode only)'),
        arg('rendering_quality', 'high',
            'Rendering quality low/medium/high (GPU mode only)'),

        arg('enable_mcu', 'true',
            'Enable the simulated MCU side (uv_sim: sensor relays + nav_sim + '
            'mcu_sim + camera_relay)'),
        arg('enable_motion', 'true',
            'Enable basic_motion (the motion action server)'),
        arg('enable_ai', 'true', 'Enable AI perception nodes'),
        arg('enable_nav', 'false', 'Enable navigation node'),
        arg('enable_task', 'false', 'Enable task runner'),

        LogInfo(msg=['Stonefish asset directory: ', data_dir]),
        *[LogInfo(msg=['[sim] ', warning]) for warning in warnings],
        scenario_check(data_dir),

        engine(data_dir),
        include('uv_sim', 'bridge_launch.py', when='enable_mcu'),
        include('uv_control', 'control_launch.py', when='enable_motion'),
        include('uv_perception', 'perception_launch.py',
                arguments={'sim_mode': 'true'}, when='enable_ai'),
        include('uv_nav', 'nav_launch.py', when='enable_nav'),
        include('uv_task', 'task_launch.py', when='enable_task'),
    ])
