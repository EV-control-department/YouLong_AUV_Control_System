"""Shared camera/recording profiles for the structured-light collection setup."""

from dataclasses import dataclass
import math
from pathlib import Path

import yaml

from .capture import CaptureConfig


@dataclass(frozen=True)
class CollectionProfile:
    path: Path
    sl: dict
    down: dict
    recording: dict


def profile_path(selected='default'):
    selected = str(selected).strip()
    path = Path(selected).expanduser()
    if path.suffix in ('.yaml', '.yml') or '/' in selected:
        candidates = [path]
    else:
        candidates = []
        try:
            from ament_index_python.packages import get_package_share_directory
            candidates.append(Path(get_package_share_directory('sl_camera'))
                              / 'config/profiles' / f'{selected}.yaml')
        except LookupError:
            pass
        candidates.append(Path(__file__).resolve().parents[1]
                          / 'config/profiles' / f'{selected}.yaml')
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    raise ValueError(f'camera profile does not exist: {selected!r}')


def _mapping(value, name):
    if not isinstance(value, dict):
        raise ValueError(f'{name} must be a YAML mapping')
    return dict(value)


def _positive(value, name):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value <= 0):
        raise ValueError(f'{name} must be finite and positive')


def load_profile(selected='default'):
    path = profile_path(selected)
    try:
        document = _mapping(yaml.safe_load(path.read_text(encoding='utf-8')), str(path))
    except yaml.YAMLError as error:
        raise ValueError(f'cannot parse camera profile {path}: {error}') from error
    if document.get('schema_version') != 1:
        raise ValueError(f'{path}: schema_version must be 1')
    cameras = _mapping(document.get('cameras'), 'cameras')
    sl = _mapping(cameras.get('sl'), 'cameras.sl')
    down = _mapping(cameras.get('down'), 'cameras.down')
    for name, camera in (('sl', sl), ('down', down)):
        if not isinstance(camera.get('enabled'), bool):
            raise ValueError(f'cameras.{name}.enabled must be true or false')
    if sl['enabled']:
        values = {key: value for key, value in sl.items() if key != 'enabled'}
        try:
            CaptureConfig(**values).validate()
        except TypeError as error:
            raise ValueError(f'cameras.sl: {error}') from error
    if down['enabled']:
        if not isinstance(down.get('device'), str) or not down['device'].strip():
            raise ValueError('cameras.down.device must be a non-empty device path')
        for key in ('startup_timeout_sec', 'reconnect_interval_sec'):
            _positive(down.get(key), f'cameras.down.{key}')
        if not isinstance(down.get('camera_config_dir', ''), str):
            raise ValueError('cameras.down.camera_config_dir must be a string')
    recording = {
        'output_root': '', 'video_fps': 30.0, 'bag_duration': 10.0, 'bag_storage': 'auto',
        **_mapping(document.get('recording', {}), 'recording'),
    }
    for key in ('video_fps', 'bag_duration'):
        _positive(recording[key], f'recording.{key}')
    if recording['bag_storage'] not in ('auto', 'sqlite3', 'mcap'):
        raise ValueError('recording.bag_storage must be auto, sqlite3, or mcap')
    if not isinstance(recording['output_root'], str):
        raise ValueError('recording.output_root must be a string')
    return CollectionProfile(path, sl, down, recording)


def camera_node_specs(profile, sl_config=''):
    """Translate profile switches into node specs without opening devices."""
    specs = []
    if profile.sl['enabled']:
        parameters = {key: value for key, value in profile.sl.items() if key != 'enabled'}
        # ROS parameters declared as doubles also accept integer YAML literals here.
        for key in ('fps', 'startup_timeout_sec', 'reconnect_interval_sec'):
            if key in parameters:
                parameters[key] = float(parameters[key])
        specs.append({
            'package': 'sl_camera', 'executable': 'sl_camera', 'name': 'sl_camera',
            'parameters': [parameters] + ([sl_config] if sl_config else []),
            'output': 'both',
        })
    if profile.down['enabled']:
        down = profile.down
        specs.append({
            'package': 'uv_camera', 'executable': 'uv_camera', 'name': 'uv_camera',
            'parameters': [{
                'sim_mode': False, 'camera_mode': 'real',
                'enable_front': False, 'enable_down': True,
                'down_camera_device': down['device'],
                'camera_config_dir': down.get('camera_config_dir', ''),
                'camera_startup_timeout_sec': float(down['startup_timeout_sec']),
                'camera_reconnect_interval_sec': float(down['reconnect_interval_sec']),
            }],
            'output': 'both',
        })
    return specs
