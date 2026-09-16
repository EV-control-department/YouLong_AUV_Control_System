"""Real-hardware bringup -- the only system entry point that ships in workspace_auv.

This file declares **no nodes** of its own.  Every node belongs to the package
that owns it, and this preset only orchestrates those entry points through
``uv_bringup.launch_common``.

Invariant: this file (and ``workspace_auv`` as a whole) must stay free of any
simulation dependency, so the control stack builds and launches with **no**
``workspace_sim`` overlay at all.  The simulation presets live in
``workspace_sim``'s ``uv_sim_bringup`` package.  Both facts are machine-checked
(``scripts/check_docs_contract.py`` and the launch-contract tests).

Note on navigator: it takes its pose from ``/basic_motion/pose_info``, so
``basic_motion`` must be running and must have received a START goal before
``/nav/navigate_to`` will plan.
"""

from launch import LaunchDescription
from launch.actions import LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from uv_bringup.launch_common import arg, include


def _warn_if_nav_enabled(context):
    """Emit a heads-up when the navigator is enabled."""
    if context.launch_configurations.get('enable_nav', 'false').lower() == 'true':
        return [LogInfo(msg=(
            '[real_bringup] navigator enabled: its pose source is '
            '/basic_motion/pose_info -- make sure basic_motion is up and a START '
            'goal has been sent, otherwise /nav/navigate_to will refuse to plan.'
        ))]
    return []


def generate_launch_description():
    return LaunchDescription([
        arg('enable_agent', 'true',
            'Start the micro-ROS agent that links the MCU over serial'),
        arg('enable_motion', 'true',
            'Enable basic_motion (the motion action server)'),
        arg('enable_ai', 'true', 'Enable AI perception nodes'),
        arg('enable_nav', 'false',
            'Enable navigation node (needs basic_motion + START for pose)'),
        arg('enable_task', 'false', 'Enable task runner'),

        OpaqueFunction(function=_warn_if_nav_enabled),

        include('uv_hm', 'hm_launch.py',
                arguments={'enable_agent': LaunchConfiguration('enable_agent')}),
        include('uv_control', 'control_launch.py', when='enable_motion'),
        include('uv_perception', 'perception_launch.py',
                arguments={'sim_mode': 'false'}, when='enable_ai'),
        include('uv_nav', 'nav_launch.py', when='enable_nav'),
        include('uv_task', 'task_launch.py', when='enable_task'),
    ])
