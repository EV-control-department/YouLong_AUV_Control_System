"""PID gains: one external file is the single source of truth.

The keys are the same paths the ZIT6 parameter service speaks, so the file, the
service and the live controllers share one vocabulary.  Moved verbatim out of
``sim_bridge`` (T5) -- the loader never needed ROS, and keeping it here lets the
controller core and the tests load gains without a node.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import yaml

from uv_sim.pid import Pid

PID_PARAM_PATHS = (
    'chassis.pid.pos.x',
    'chassis.pid.pos.y',
    'chassis.pid.pos.z',
    'chassis.pid.pos.yaw',
    'chassis.pid.pos.yaw_rate',   # inner loop of the yaw cascade
    'chassis.pid.vel.x',
    'chassis.pid.vel.y',
    'chassis.pid.vel.z',
    'chassis.pid.vel.yaw',
)

#: channel path -> attribute name the controller exposes for it
PID_PARAM_ATTRIBUTES = {
    'chassis.pid.pos.x': 'pid_x',
    'chassis.pid.pos.y': 'pid_y',
    'chassis.pid.pos.z': 'pid_z',
    'chassis.pid.pos.yaw': 'pid_yaw',
    'chassis.pid.pos.yaw_rate': 'pid_yaw_rate',
    'chassis.pid.vel.x': 'pid_vx',
    'chassis.pid.vel.y': 'pid_vy',
    'chassis.pid.vel.z': 'pid_vz',
    'chassis.pid.vel.yaw': 'pid_vyaw',
}

PID_REQUIRED_FIELDS = ('kp', 'ki', 'kd', 'i_limit')
PID_OPTIONAL_FIELDS = ('z_thrust_ff',)

# Used when the external file is missing or unreadable; keeps the SIL stack
# runnable instead of failing at startup.
# Last-resort defaults, kept in step with the shipped config/sim_pid.yaml.
# The file always wins when it is readable (see resolve_pid_config_path); these
# values only appear when no file is found at all, in which case the loader logs
# a warning.  They are a **mirror** of the last known tuning, not a second place
# to tune: edit sim_pid.yaml, then refresh these (or leave them slightly stale --
# only a completely missing file reaches them).
DEFAULT_PID_PARAMS = {
    'chassis.pid.pos.x': {'kp': 800.0, 'ki': 130.0, 'kd': 200.0, 'i_limit': 1000.0},
    'chassis.pid.pos.y': {'kp': 600.0, 'ki': 120.0, 'kd': 150.0, 'i_limit': 1000.0},
    'chassis.pid.pos.z': {'kp': 600.0, 'ki': 80.0, 'kd': 800.0, 'i_limit': 6200.0,
                          'z_thrust_ff': 0.0},
    'chassis.pid.pos.yaw': {'kp': 10.0, 'ki': 1.0, 'kd': 0.0, 'i_limit': 30.0},
    'chassis.pid.pos.yaw_rate': {'kp': 5.3, 'ki': 0.0, 'kd': 0.7, 'i_limit': 6.0},
    'chassis.pid.vel.x': {'kp': 600.0, 'ki': 50.0, 'kd': 10.0, 'i_limit': 500.0},
    'chassis.pid.vel.y': {'kp': 300.0, 'ki': 50.0, 'kd': 10.0, 'i_limit': 500.0},
    'chassis.pid.vel.z': {'kp': 300.0, 'ki': 30.0, 'kd': 50.0, 'i_limit': 500.0,
                          'z_thrust_ff': 0.0},
    'chassis.pid.vel.yaw': {'kp': 40.0, 'ki': 20.0, 'kd': 2.0, 'i_limit': 300.0},
}


def resolve_pid_config_path(explicit: str = "") -> Optional[Path]:
    """Locate ``sim_pid.yaml``: explicit parameter > source tree > package share.

    The **source tree wins** in a development checkout so that editing
    ``src/uv_sim/config/sim_pid.yaml`` takes effect on the next node start
    without a rebuild (the file installed under ``share/`` is a copy, not a
    symlink).  An installed deployment without the source tree falls back to
    the packaged copy.
    """
    if explicit:
        return Path(explicit).expanduser()

    source = Path(__file__).resolve().parent.parent / 'config' / 'sim_pid.yaml'
    if source.is_file():
        return source

    try:
        from ament_index_python.packages import get_package_share_directory

        installed = Path(get_package_share_directory('uv_sim')) / 'config' / 'sim_pid.yaml'
        if installed.is_file():
            return installed
    except Exception:  # ament index unavailable (e.g. bare pytest run)
        pass

    return None


def load_pid_params(path: Optional[Path], logger=None) -> dict:
    """Load PID gains from ``path``, falling back to ``DEFAULT_PID_PARAMS``.

    Every key in ``PID_PARAM_PATHS`` is always present in the result: the file
    overrides defaults, so a partial file is valid and unknown keys are only
    reported, never fatal.
    """

    def _warn(message: str) -> None:
        if logger is not None:
            logger.warning(message)

    params = {key: dict(DEFAULT_PID_PARAMS[key]) for key in PID_PARAM_PATHS}

    if path is None:
        _warn(
            "no sim_pid.yaml found (source tree, then package share); using the "
            "built-in PID defaults -- control gains may differ from the tuned file"
        )
        return params
    if not path.is_file():
        _warn(
            f"PID config {path} does not exist; using the built-in PID defaults -- "
            "control gains may differ from the tuned file"
        )
        return params

    try:
        raw = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    except Exception as exc:  # malformed YAML must not take the sim down
        _warn(f"failed to parse PID config {path}: {exc}; using built-in defaults")
        return params

    if not isinstance(raw, dict):
        _warn(f"PID config {path} must be a mapping of path -> gains; ignoring it")
        return params

    for key, gains in raw.items():
        if key not in params:
            _warn(f"PID config {path}: unknown channel '{key}' ignored")
            continue
        if not isinstance(gains, dict):
            _warn(f"PID config {path}: '{key}' must be a mapping; ignored")
            continue
        for field, value in gains.items():
            if field not in PID_REQUIRED_FIELDS + PID_OPTIONAL_FIELDS:
                _warn(f"PID config {path}: unknown field '{key}.{field}' ignored")
                continue
            try:
                params[key][field] = float(value)
            except (TypeError, ValueError):
                _warn(f"PID config {path}: '{key}.{field}'={value!r} is not a number; ignored")

    return params


def build_pids(pid_params: dict) -> dict[str, Pid]:
    """Build one :class:`~uv_sim.pid.Pid` per channel, keyed by path.

    The dict that backs the parameter service and the live controllers is built
    by the *same* call, so the service can never report gains that differ from
    the ones actually in use (the drift this replaced was two hardcoded copies).
    """
    pids: dict[str, Pid] = {}
    for path in PID_PARAM_PATHS:
        gains = pid_params[path]
        pids[path] = Pid(
            gains['kp'],
            gains['ki'],
            gains['kd'],
            i_limit=gains['i_limit'],
            z_thrust_ff=gains.get('z_thrust_ff', 0.0),
        )
    return pids
