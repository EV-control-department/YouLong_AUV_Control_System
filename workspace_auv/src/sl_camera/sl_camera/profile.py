"""Configuration for an independent, arbitrarily sized camera collection."""

from dataclasses import dataclass, field
import math
from pathlib import Path
import re

import yaml

from .capture import CaptureConfig

DEFAULT_CATALOG_TOPIC = '/auv/sl_camera/cameras'


@dataclass(frozen=True)
class CameraSpec:
    id: int
    name: str
    enabled: bool
    device: str
    ice_service: str
    frame_id: str
    width: int
    height: int
    fps: float = 30.0
    pixel_format: str = 'MJPG'
    layout: str = 'mono'
    startup_timeout_sec: float = 5.0
    reconnect_interval_sec: float = 1.0
    calibrations: list = field(default_factory=list)

    def capture_config(self):
        return CaptureConfig(
            device=self.device, width=self.width, height=self.height, fps=self.fps,
            startup_timeout_sec=self.startup_timeout_sec,
            reconnect_interval_sec=self.reconnect_interval_sec)

    def validate(self):
        if isinstance(self.id, bool) or not isinstance(self.id, int) or not 0 < self.id < 2**32:
            raise ValueError('camera id must be a nonzero uint32')
        if not isinstance(self.name, str) or not re.fullmatch(r'[A-Za-z0-9_]+', self.name):
            raise ValueError('camera name must contain only letters, digits, and underscores')
        if not isinstance(self.enabled, bool):
            raise ValueError('camera enabled must be true or false')
        for key in ('device', 'ice_service', 'frame_id'):
            value = getattr(self, key)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f'camera {self.name}: {key} must be a nonempty string')
        if (not re.fullmatch(r'[A-Za-z0-9_/-]+', self.ice_service)
                or self.ice_service.startswith('/') or '..' in self.ice_service):
            raise ValueError(f'camera {self.name}: invalid ice_service')
        if self.pixel_format != 'MJPG':
            raise ValueError('this version requires MJPG capture')
        if self.layout not in ('mono', 'stereo_sbs'):
            raise ValueError('layout must be mono or stereo_sbs')
        if self.layout == 'stereo_sbs' and self.width % 2:
            raise ValueError('stereo_sbs requires an even full-image width')
        if not isinstance(self.calibrations, list):
            raise ValueError('calibrations must be a list; use [] when uncalibrated')
        self.capture_config().validate()


@dataclass(frozen=True)
class CollectionProfile:
    path: Path
    cameras: tuple
    catalog_topic: str
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


def positive(value, name):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value <= 0):
        raise ValueError(f'{name} must be finite and positive')


def load_settings(selected='default'):
    """Read shared launch/recording settings without inspecting camera definitions."""
    path = profile_path(selected)
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(document, dict) or document.get('schema_version') != 2:
        raise ValueError(f'{path}: use schema_version: 2 and a cameras list')
    topic = document.get('catalog_topic', DEFAULT_CATALOG_TOPIC)
    if not isinstance(topic, str) or not re.fullmatch(r'/[A-Za-z0-9_/]+', topic):
        raise ValueError('catalog_topic must be an absolute ROS topic')
    recording_raw = document.get('recording', {})
    if not isinstance(recording_raw, dict):
        raise ValueError('recording must be a mapping')
    recording = {
        'output_root': '', 'video_fps': 30.0, 'bag_duration': 10.0,
        'bag_storage': 'auto', **recording_raw,
    }
    for key in ('video_fps', 'bag_duration'):
        positive(recording[key], f'recording.{key}')
    if recording['bag_storage'] not in ('auto', 'sqlite3', 'mcap'):
        raise ValueError('bag_storage must be auto, sqlite3, or mcap')
    if not isinstance(recording['output_root'], str):
        raise ValueError('output_root must be a string')
    return path, topic, recording, document


def load_profile(selected='default'):
    path, topic, recording, document = load_settings(selected)
    entries = document.get('cameras')
    if not isinstance(entries, list):
        raise ValueError('cameras must be a list')
    cameras = []
    seen = {key: set() for key in ('id', 'name', 'ice_service', 'device')}
    for entry in entries:
        if not isinstance(entry, dict):
            raise ValueError('each cameras item must be a mapping')
        try:
            spec = CameraSpec(**entry)
        except TypeError as error:
            raise ValueError(f'invalid camera definition: {error}') from error
        spec.validate()
        for key in ('id', 'name', 'ice_service', 'device'):
            if key == 'device' and not spec.enabled:
                continue
            value = (str(Path(spec.device).resolve()) if key == 'device'
                     else getattr(spec, key))
            if value in seen[key]:
                raise ValueError(f'duplicate camera {key}: {value}')
            seen[key].add(value)
        cameras.append(spec)
    return CollectionProfile(path, tuple(cameras), topic, recording)
