"""Stonefish-specific helpers for the simulation presets.

Deliberately separate from ``uv_bringup.launch_common``: everything here knows
about the engine, and the sim-free package must not (that separation is what
keeps ``workspace_auv`` buildable and launchable without any simulation
overlay).  ``uv_sim_bringup`` is allowed to import *both*.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from uv_bringup.launch_common import include

#: The engine's own launch files.  Included rather than re-declared, so the
#: engine's argument list has exactly one author.
GPU_LAUNCH = 'stonefish_simulator.launch.py'
CONSOLE_LAUNCH = 'stonefish_simulator_nogpu.launch.py'

#: Environment override for the asset directory (meshes, textures, scenarios).
DATA_ENV_VAR = 'STONEFISH_DATA_DIR'

#: A file that must be present for a directory to be the Stonefish asset dir.
DATA_MARKER = 'xunyun_fixed.scn'


def _as_bool(value: str) -> bool:
    """Normalise a launch-argument string to a bool.

    ``IfCondition`` matches strings, so ``headless:=True`` would match *neither*
    branch of a two-way condition and quietly start no engine at all.
    """
    return str(value).strip().lower() in ('true', '1', 'yes', 'on')


def resolve_simulation_data() -> tuple[str, list[str]]:
    """Locate the Stonefish asset directory (``Data/``); return it with warnings.

    ``Data/`` is **not** installed into ``share/`` by the engine's CMakeLists
    (the simulator wants direct filesystem access to the meshes and scenarios),
    so the shipped layout is found by walking up from the installed share
    directory to the workspace root.  If none of the candidates is found the
    best guess is returned along with a warning -- the launch file prints it,
    which is louder than the previous silent path join and still lets
    ``--show-args`` work on a machine without the source tree.
    """
    share = get_package_share_directory('stonefish_ros2')
    workspace_root = os.path.abspath(os.path.join(share, '..', '..', '..', '..'))
    candidates = [
        os.environ.get(DATA_ENV_VAR, ''),
        os.path.join(share, 'Data'),
        os.path.join(workspace_root, 'src', 'stonefish_ros2', 'Data'),
    ]
    tried = [path for path in candidates if path]
    for path in tried:
        if os.path.isfile(os.path.join(path, DATA_MARKER)):
            return path, []
    return tried[-1], [
        f'Stonefish asset directory not found (no {DATA_MARKER} in: '
        f'{", ".join(tried)}); falling back to {tried[-1]}'
    ]


def _scenario_path(data_dir: str, name: str) -> str:
    return name if os.path.isabs(name) else os.path.join(data_dir, name)


def scenario_check(data_dir: str) -> OpaqueFunction:
    """Warn -- listing what is available -- when the scenario file is missing.

    The scenario name only exists once launch arguments are evaluated, so this
    cannot be a build-time check.  A typo used to surface as an engine crash.
    """

    def _check(context: LaunchContext):
        name = context.launch_configurations.get('scenario_desc', '')
        path = _scenario_path(data_dir, name)
        if os.path.isfile(path):
            return []
        available = []
        if os.path.isdir(data_dir):
            available = sorted(f for f in os.listdir(data_dir) if f.endswith('.scn'))
        hint = f'; available: {", ".join(available)}' if available else ''
        return [LogInfo(msg=f'[sim] scenario {name!r} not found at {path}{hint}')]

    return OpaqueFunction(function=_check)


def engine(data_dir: str) -> OpaqueFunction:
    """The Stonefish engine, included through its own launch file.

    GPU (``stonefish_simulator``, six arguments) and console
    (``stonefish_simulator_nogpu``, three arguments) are different executables,
    so the branch is made here after evaluating ``headless``.
    """

    def _engine(context: LaunchContext):
        name = context.launch_configurations.get('scenario_desc', '')
        scenario = (
            LaunchConfiguration('scenario_desc') if os.path.isabs(name)
            else PathJoinSubstitution([data_dir, LaunchConfiguration('scenario_desc')])
        )
        shared = {
            'simulation_data': data_dir,
            'scenario_desc': scenario,
            'simulation_rate': LaunchConfiguration('simulation_rate'),
        }
        if _as_bool(context.launch_configurations.get('headless', 'false')):
            return [include('stonefish_ros2', CONSOLE_LAUNCH, arguments=shared)]
        return [include('stonefish_ros2', GPU_LAUNCH, arguments={
            **shared,
            'window_res_x': LaunchConfiguration('window_res_x'),
            'window_res_y': LaunchConfiguration('window_res_y'),
            'rendering_quality': LaunchConfiguration('rendering_quality'),
        })]

    return OpaqueFunction(function=_engine)
