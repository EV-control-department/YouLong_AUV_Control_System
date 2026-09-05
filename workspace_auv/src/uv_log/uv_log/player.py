"""Play video and synchronized non-image ROS topics from a uv_log session."""

from __future__ import annotations

import argparse
import bisect
import json
import re
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from .jpeg_archive import archive_entries, read_payload
from .session import default_output_root


IMAGE_TOPIC_TYPES = {
    'sensor_msgs/msg/Image',
    'sensor_msgs/msg/CompressedImage',
    'stereo_msgs/msg/DisparityImage',
}
NANOSECONDS = 1_000_000_000


def _part_sort_key(path: Path):
    match = re.search(r'(\d+)$', path.stem)
    return (int(match.group(1)) if match else -1, path.name)


def _mcap_files(directory: Path) -> list[Path]:
    try:
        candidates = directory.glob('*.mcap')
        files = []
        for path in candidates:
            try:
                if path.is_file() and path.stat().st_size > 0:
                    files.append(path)
            except OSError:
                continue
    except OSError:
        return []
    return sorted(files, key=_part_sort_key)


def _bag_part_paths(bag_root: Path) -> list[Path]:
    try:
        parts = sorted(
            (path for path in bag_root.glob('part_*') if path.is_dir()),
            key=_part_sort_key,
        )
    except OSError:
        return []
    if not parts:
        return _mcap_files(bag_root)

    # Read MCAP files individually.  This works for normal bags and also for
    # power-loss sessions where metadata.yaml is absent or damaged.  Empty
    # files are ignored because they contain no recoverable messages.
    recovered = []
    for part in parts:
        recovered.extend(_mcap_files(part))
    return recovered


def _rosbag_modules():
    """Import ROS bag support lazily so video-only playback still works."""
    import rclpy
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    return rclpy, rosbag2_py, deserialize_message, get_message


class BagPlayback:
    """Publish recorded non-image topics against the video time axis."""

    def __init__(self, bag_root: Path, node):
        self.bag_root = Path(bag_root)
        self.node = node
        (
            self._rclpy,
            self._rosbag2_py,
            self._deserialize_message,
            self._get_message,
        ) = _rosbag_modules()
        self._parts = _bag_part_paths(self.bag_root)
        self._part_index = 0
        self._reader = None
        self._reader_path = None
        self._next = None
        self._topic_types: dict[str, str] = {}
        self._message_types = {}
        self._publishers = {}
        self._errors: list[str] = []
        self._time_offset_ns = 0
        self._published_count = 0
        self._first_raw_ns = None
        self._last_raw_ns = None
        self._raw_end_ns = None
        self._collect_metadata_bounds()
        self._open_next_part()
        self._next = self._read_next()
        if self._next is not None:
            self._first_raw_ns = self._next[0]

    @property
    def empty(self) -> bool:
        return self._next is None and not self._publishers

    @property
    def topic_count(self) -> int:
        return len(self._publishers)

    @property
    def published_count(self) -> int:
        return self._published_count

    @property
    def errors(self) -> tuple[str, ...]:
        return tuple(self._errors)

    @property
    def start_ns(self) -> int | None:
        if self._first_raw_ns is None:
            return None
        return self._first_raw_ns + self._time_offset_ns

    @property
    def end_ns(self) -> int | None:
        if self._first_raw_ns is None:
            return None
        if self._raw_end_ns is None:
            return self._last_raw_ns
        return self._raw_end_ns + self._time_offset_ns

    def configure_timebase(
        self,
        video_start_ns: int | None,
        use_sim_time: bool = False,
    ) -> None:
        """Align old wall-time bags to video if their clocks are unrelated."""
        if self._first_raw_ns is None or video_start_ns is None or use_sim_time:
            self._time_offset_ns = 0
            return
        # New simulation sessions use /clock and need no correction.  This
        # fallback keeps sessions recorded before that default usable when
        # rosbag used wall time but the camera frames used simulation time.
        difference = int(video_start_ns) - self._first_raw_ns
        self._time_offset_ns = difference if abs(difference) > 60 * NANOSECONDS else 0

    def _update_metadata_bounds(self, metadata) -> None:
        try:
            part_start_ns = int(metadata.starting_time.nanoseconds)
            part_duration_ns = int(metadata.duration.nanoseconds)
            part_end_ns = part_start_ns + part_duration_ns
            self._raw_end_ns = max(
                self._raw_end_ns or part_end_ns, part_end_ns)
        except (AttributeError, TypeError, ValueError):
            pass

    def _collect_metadata_bounds(self) -> None:
        try:
            info = self._rosbag2_py.Info()
        except AttributeError:
            return
        for path in self._parts:
            try:
                metadata = info.read_metadata(str(path), 'mcap')
                self._update_metadata_bounds(metadata)
            except Exception:
                # The reader below remains the source of truth for damaged or
                # older bags whose metadata cannot be read by Info.
                continue

    def _register_topics(self, reader) -> list[str]:
        selected = []
        for metadata in reader.get_all_topics_and_types():
            topic = str(metadata.name)
            topic_type = str(metadata.type)
            if topic_type in IMAGE_TOPIC_TYPES:
                continue
            self._topic_types[topic] = topic_type
            try:
                message_type = self._message_types.get(topic)
                if message_type is None:
                    message_type = self._get_message(topic_type)
                    self._message_types[topic] = message_type
                if topic not in self._publishers:
                    qos = 10
                    profiles = getattr(metadata, 'offered_qos_profiles', [])
                    if profiles:
                        try:
                            converted_qos = (
                                self._rosbag2_py
                                .convert_rclcpp_qos_to_rclpy_qos(profiles[0]))
                            qos = self._safe_replay_qos(converted_qos)
                        except Exception:
                            pass
                    self._publishers[topic] = self.node.create_publisher(
                        message_type, topic, qos)
                selected.append(topic)
            except Exception as error:
                self._errors.append(f'{topic} ({topic_type}): {error}')
        return selected

    @staticmethod
    def _safe_replay_qos(qos):
        """Return a publisher-valid QoS profile for recorded metadata.

        Some rosbag2/MCAP metadata contains UNKNOWN policy values.  They are
        useful as metadata, but RMW rejects UNKNOWN history (and some other
        UNKNOWN policies) when creating a publisher.  A depth-only profile is
        a valid, conservative fallback for replay; the recorded message bytes
        and timestamps are unaffected.
        """
        from rclpy.qos import (
            QoSDurabilityPolicy,
            QoSHistoryPolicy,
            QoSLivelinessPolicy,
            QoSReliabilityPolicy,
        )

        unknown = (
            (getattr(qos, 'history', None), QoSHistoryPolicy.UNKNOWN),
            (getattr(qos, 'reliability', None), QoSReliabilityPolicy.UNKNOWN),
            (getattr(qos, 'durability', None), QoSDurabilityPolicy.UNKNOWN),
            (getattr(qos, 'liveliness', None), QoSLivelinessPolicy.UNKNOWN),
        )
        if any(value == invalid for value, invalid in unknown):
            return 10
        return qos

    def _open_next_part(self) -> bool:
        while self._part_index < len(self._parts):
            path = self._parts[self._part_index]
            self._part_index += 1
            reader = self._rosbag2_py.SequentialReader()
            try:
                reader.open(
                    self._rosbag2_py.StorageOptions(
                        uri=str(path), storage_id='mcap'),
                    self._rosbag2_py.ConverterOptions('', ''),
                )
                self._update_metadata_bounds(reader.get_metadata())
                topics = self._register_topics(reader)
                if topics:
                    reader.set_filter(
                        self._rosbag2_py.StorageFilter(topics=topics))
                self._reader = reader
                self._reader_path = path
                return True
            except Exception as error:
                self._errors.append(f'{path}: {error}')
                try:
                    reader.close()
                except Exception:
                    pass
        self._reader = None
        self._reader_path = None
        return False

    def _read_next(self):
        while True:
            if self._reader is None and not self._open_next_part():
                return None
            try:
                if self._reader.has_next():
                    item = self._reader.read_next()
                    if len(item) < 3:
                        continue
                    topic, payload, timestamp_ns = item[:3]
                    if topic in self._publishers:
                        timestamp_ns = int(timestamp_ns)
                        self._last_raw_ns = timestamp_ns
                        return timestamp_ns, topic, payload
                    continue
                self._reader.close()
            except Exception as error:
                path = self._reader_path or '<unknown MCAP>'
                self._errors.append(f'{path}: read failed: {error}')
                try:
                    self._reader.close()
                except Exception:
                    pass
            self._reader = None
            self._reader_path = None

    def _reset(self):
        if self._reader is not None:
            try:
                self._reader.close()
            except Exception:
                pass
        self._reader = None
        self._reader_path = None
        self._part_index = 0
        self._next = None
        self._open_next_part()
        self._next = self._read_next()

    def publish_until(self, target_ns: int) -> int:
        """Publish every recorded message whose mapped time is due."""
        published = 0
        while self._next is not None:
            raw_timestamp, topic, payload = self._next
            if raw_timestamp + self._time_offset_ns > target_ns:
                break
            try:
                message = self._deserialize_message(
                    payload, self._message_types[topic])
                self._publishers[topic].publish(message)
                self._published_count += 1
                published += 1
            except Exception as error:
                self._errors.append(f'{topic}: {error}')
            self._next = self._read_next()
        return published

    def seek(self, target_ns: int) -> None:
        """Reset and replay bag state up to an absolute timeline position."""
        self._reset()
        self.publish_until(target_ns)

    def close(self):
        if self._reader is not None:
            try:
                self._reader.close()
            except Exception:
                pass
            self._reader = None
            self._reader_path = None


def _probe_duration(path: Path) -> float | None:
    try:
        result = subprocess.run(
            [
                'ffprobe', '-v', 'error', '-show_entries', 'format=duration',
                '-of', 'default=noprint_wrappers=1:nokey=1', str(path),
            ],
            check=True,
            capture_output=True,
            text=True,
            timeout=5.0,
        )
        value = float(result.stdout.strip())
        return value if value > 0.0 else None
    except (OSError, ValueError, subprocess.SubprocessError):
        return None


def _valid_ts(path: Path) -> bool:
    """Accept complete or partially written TS files with a sync byte."""
    try:
        if path.stat().st_size < 188:
            return False
        with path.open('rb') as handle:
            data = handle.read(188 * 2)
        return bool(data) and any(data[offset] == 0x47
                                  for offset in range(0, len(data), 188))
    except OSError:
        return False


def _playlist_durations(directory: Path) -> dict[Path, float]:
    """Read cheap per-segment durations from the recorder's m3u8 indexes."""
    durations = {}
    for playlist in sorted(directory.glob('index_*.m3u8')):
        try:
            lines = playlist.read_text(encoding='utf-8').splitlines()
        except OSError:
            continue
        pending = None
        for line in lines:
            line = line.strip()
            if line.startswith('#EXTINF:'):
                try:
                    pending = float(line[8:].split(',', 1)[0])
                except ValueError:
                    pending = None
            elif pending is not None and line and not line.startswith('#'):
                segment = (directory / line).resolve()
                if segment.parent == directory.resolve():
                    durations[segment] = pending
                pending = None
    return durations


def _session_candidates(value: str) -> list[Path]:
    """Resolve a session path without making it depend on the shell cwd."""
    raw = Path(value).expanduser()
    if raw.is_absolute():
        return [raw.resolve()]

    project_sessions = default_output_root()
    roots = [Path.cwd(), project_sessions.parent]
    # ``sessions/name`` is already relative to the repository root; a bare
    # ``name`` is also accepted as shorthand for ``<repo>/sessions/name``.
    if not raw.parts or raw.parts[0] != project_sessions.name:
        roots.append(project_sessions)
    candidates = []
    for root in roots:
        candidate = (root / raw).resolve()
        if candidate not in candidates:
            candidates.append(candidate)
    return candidates


def _has_valid_ts(session_dir: Path) -> bool:
    return any(
        _valid_ts(path)
        for stream in (
            'front_annotated', 'down_annotated', 'front', 'down')
        for path in (session_dir / 'video' / stream).glob('*.ts')
    )


def _has_valid_jpeg(session_dir: Path) -> bool:
    return any(
        archive_entries(session_dir / 'video' / stream)
        for stream in ('front', 'down', 'front_annotated', 'down_annotated')
    )


def _has_valid_media(session_dir: Path) -> bool:
    return _has_valid_ts(session_dir) or _has_valid_jpeg(session_dir)


def _preferred_stream(session_dir: Path, camera: str) -> str:
    """Prefer raw JPEG/TS video, falling back to annotations."""
    for stream in (camera, f'{camera}_annotated'):
        if archive_entries(session_dir / 'video' / stream):
            return stream
        directory = session_dir / 'video' / stream
        if any(_valid_ts(path) for path in directory.glob('*.ts')):
            return stream
    # Keep the UI useful while a new recording is still being written.
    return f'{camera}_annotated'


class SegmentVideo:
    """Sequentially read a directory of independently recoverable TS files."""

    def __init__(
        self,
        directory: Path,
        fallback_fps: float,
        default_segment_duration: float | None = None,
    ):
        self.directory = directory
        self.fallback_fps = max(1.0, float(fallback_fps))
        self.segments = [
            path for path in sorted(directory.glob('*.ts')) if _valid_ts(path)
        ]
        playlist_durations = _playlist_durations(directory)
        default_duration = (
            max(0.0, float(default_segment_duration))
            if default_segment_duration is not None else None)
        self.durations = []
        for index, path in enumerate(self.segments):
            duration = playlist_durations.get(path)
            if duration is None and default_duration is not None:
                duration = default_duration
                # The last TS is often a short tail after a clean stop. Probe
                # only that one file instead of every high-resolution segment.
                if index == len(self.segments) - 1:
                    duration = _probe_duration(path) or duration
            if duration is None:
                duration = _probe_duration(path) or 0.0
            self.durations.append(max(0.0, duration))
        self.fps = self.fallback_fps
        self.width = 0
        self.height = 0
        self.cap = None
        self.segment_index = -1
        self.frame_index = 0
        self.latest_frame = None
        for index in range(len(self.segments)):
            if self._open_segment(index):
                break

    def _open_segment(self, index: int) -> bool:
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        if index < 0 or index >= len(self.segments):
            self.segment_index = len(self.segments)
            return False
        cap = cv2.VideoCapture(str(self.segments[index]))
        if not cap.isOpened():
            cap.release()
            self.segment_index = index
            return False
        self.cap = cap
        self.segment_index = index
        if index == 0:
            self.width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
            self.height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
        return True

    @property
    def duration(self) -> float:
        return sum(self.durations)

    @property
    def start_ns(self) -> int | None:
        return None

    @property
    def end_ns(self) -> int | None:
        return None

    @property
    def empty(self) -> bool:
        return not self.segments or self.cap is None

    def seek(self, position: float):
        """Seek approximately to a timeline position in seconds."""
        position = max(0.0, min(float(position), self.duration))
        elapsed = 0.0
        index = 0
        for index, duration in enumerate(self.durations):
            if position < elapsed + duration or index == len(self.durations) - 1:
                break
            elapsed += duration
        while index < len(self.segments) and not self._open_segment(index):
            index += 1
        if index >= len(self.segments):
            return
        local_frame = max(0, int(round((position - elapsed) * self.fps)))
        for _ in range(local_frame):
            if self.cap is None or not self.cap.grab():
                break
        self.frame_index = int(round(position * self.fps))
        self.latest_frame = None

    def read_to(self, position: float):
        """Read through the frame due at ``position`` and return the latest."""
        if self.empty:
            return self.latest_frame
        target_frame = max(0, int(position * self.fps))
        while self.frame_index <= target_frame:
            if self.cap is None:
                break
            ok, frame = self.cap.read()
            if ok and frame is not None:
                self.latest_frame = frame
                self.frame_index += 1
                continue
            next_index = self.segment_index + 1
            while next_index < len(self.segments):
                if self._open_segment(next_index):
                    break
                next_index += 1
            else:
                break
        return self.latest_frame

    def close(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None


class JpegArchiveVideo:
    """Read timestamped JPEG archive frames and decode only during playback."""

    def __init__(
        self,
        directory: Path,
        fallback_fps: float,
        _default_segment_duration: float | None = None,
    ):
        self.directory = directory
        self.entries = archive_entries(directory)
        self.fps = max(1.0, float(fallback_fps))
        self.latest_frame = None
        self.frame_index = 0
        self.cursor = 0
        self.width = 0
        self.height = 0
        self._times_ns = [entry.timestamp_ns for entry in self.entries]
        self._start_ns = self._times_ns[0] if self._times_ns else 0
        self._duration = 0.0
        if self._times_ns:
            self._duration = max(
                0.0,
                (self._times_ns[-1] - self._start_ns) / 1_000_000_000,
            ) + 1.0 / self.fps

    @property
    def duration(self) -> float:
        return self._duration

    @property
    def empty(self) -> bool:
        return not self.entries

    @property
    def start_ns(self) -> int | None:
        return self._start_ns if self.entries else None

    @property
    def end_ns(self) -> int | None:
        return self._times_ns[-1] if self.entries else None

    def _decode(self, entry):
        payload = read_payload(entry)
        if payload is None:
            return None
        encoded = np.frombuffer(payload, dtype=np.uint8)
        frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if frame is not None and self.width == 0:
            self.height, self.width = frame.shape[:2]
        return frame

    def seek(self, position: float):
        position = max(0.0, min(float(position), self.duration))
        self.seek_ns(self._start_ns + int(position * NANOSECONDS))

    def seek_ns(self, target_ns: int):
        if not self.entries:
            self.cursor = 0
            self.latest_frame = None
            return
        index = bisect.bisect_right(self._times_ns, target_ns)
        self.cursor = max(0, index - 1)
        self.latest_frame = None
        if index > 0:
            candidate = self._decode(self.entries[index - 1])
            if candidate is not None:
                self.latest_frame = candidate
            self.cursor = index
        self.frame_index = index

    def read_to(self, position: float):
        if self.empty:
            return self.latest_frame
        target_ns = self._start_ns + int(
            max(0.0, float(position)) * NANOSECONDS)
        return self.read_to_ns(target_ns)

    def read_to_ns(self, target_ns: int):
        if self.empty:
            return self.latest_frame
        while self.cursor < len(self.entries):
            entry = self.entries[self.cursor]
            if entry.timestamp_ns > target_ns:
                break
            frame = self._decode(entry)
            if frame is not None:
                self.latest_frame = frame
            self.cursor += 1
            self.frame_index += 1
        return self.latest_frame

    def close(self):
        pass


def _make_video(directory: Path, fps: float, segment_duration: float):
    if any(directory.glob('chunk_*.mjpg')):
        return JpegArchiveVideo(directory, fps, segment_duration)
    return SegmentVideo(directory, fps, segment_duration)


def _load_manifest(session_dir: Path) -> dict:
    try:
        return json.loads((session_dir / 'manifest.json').read_text())
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return {}


def _manifest_fps(manifest: dict, stream: str, fallback: float) -> float:
    try:
        value = float(manifest.get('video', {}).get(stream, {}).get('fps'))
        return value if value > 0.0 else fallback
    except (AttributeError, TypeError, ValueError):
        return fallback


def _manifest_segment_duration(
    manifest: dict, stream: str, fallback: float
) -> float:
    try:
        value = float(
            manifest.get('video', {}).get(stream, {}).get('segment_seconds'))
        return value if value > 0.0 else fallback
    except (AttributeError, TypeError, ValueError):
        return fallback


def _manifest_bool(manifest: dict, key: str, fallback: bool = False) -> bool:
    try:
        value = manifest.get('recorder', {}).get(key, fallback)
    except AttributeError:
        return fallback
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _open_bag_playback(session_dir: Path):
    """Create the optional ROS publisher node for a session's rosbag."""
    if not _bag_part_paths(session_dir / 'bag'):
        return None, None, None, False
    try:
        rclpy, _rosbag2_py, _deserialize, _get_message = _rosbag_modules()
        owns_context = False
        if not rclpy.ok():
            rclpy.init(args=None)
            owns_context = True
        node = rclpy.create_node('uv_log_player')
        try:
            bag = BagPlayback(session_dir / 'bag', node)
        except Exception:
            node.destroy_node()
            if owns_context and rclpy.ok():
                rclpy.shutdown()
            raise
        return bag, node, rclpy, owns_context
    except (ImportError, OSError, RuntimeError, TypeError, ValueError):
        return None, None, None, False


def _build_window(session_dir: Path):
    try:
        from PySide6.QtCore import Qt, QTimer
        from PySide6.QtGui import QImage, QKeyEvent, QPixmap
        from PySide6.QtWidgets import (
            QApplication,
            QHBoxLayout,
            QLabel,
            QMainWindow,
            QPushButton,
            QSlider,
            QVBoxLayout,
            QWidget,
        )
    except ImportError as error:
        raise RuntimeError('PySide6 is required for uv_log player') from error

    class Panel(QLabel):
        def __init__(self, title):
            super().__init__()
            self.setMinimumSize(320, 180)
            self.setAlignment(Qt.AlignCenter)
            self.setStyleSheet('background-color: #202020; color: #dddddd;')
            self.setText(title)
            self._pixmap = None

        def set_frame(self, frame):
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            height, width = rgb.shape[:2]
            image = QImage(
                rgb.data, width, height, rgb.strides[0], QImage.Format_RGB888)
            self._pixmap = QPixmap.fromImage(image.copy())
            self._refresh()

        def resizeEvent(self, event):
            super().resizeEvent(event)
            self._refresh()

        def _refresh(self):
            if self._pixmap is not None:
                self.setPixmap(self._pixmap.scaled(
                    self.size(), Qt.KeepAspectRatio,
                    Qt.SmoothTransformation))

    class PlayerWindow(QMainWindow):
        def __init__(self):
            super().__init__()
            self.setWindowTitle(f'uv_log player - {session_dir.name}')
            self.resize(1280, 760)
            manifest = _load_manifest(session_dir)
            self.front_stream = _preferred_stream(session_dir, 'front')
            self.down_stream = _preferred_stream(session_dir, 'down')
            self.front = _make_video(
                session_dir / 'video' / self.front_stream,
                _manifest_fps(manifest, self.front_stream, 10.0),
                _manifest_segment_duration(
                    manifest, self.front_stream, 2.0))
            self.down = _make_video(
                session_dir / 'video' / self.down_stream,
                _manifest_fps(manifest, self.down_stream, 10.0),
                _manifest_segment_duration(
                    manifest, self.down_stream, 2.0))
            (
                self.bag,
                self.ros_node,
                self.ros_module,
                self.owns_ros_context,
            ) = _open_bag_playback(session_dir)
            timestamped_videos = [
                video for video in (self.front, self.down)
                if video.start_ns is not None
            ]
            if timestamped_videos:
                self.timeline_origin_ns = min(
                    video.start_ns for video in timestamped_videos)
                end_ns = max(
                    video.end_ns for video in timestamped_videos
                    if video.end_ns is not None)
                timestamped_duration = max(
                    0.0,
                    (end_ns - self.timeline_origin_ns) / NANOSECONDS,
                )
            elif self.bag is not None and self.bag.start_ns is not None:
                self.timeline_origin_ns = self.bag.start_ns
                timestamped_duration = 0.0
            else:
                self.timeline_origin_ns = 0
                timestamped_duration = 0.0
            self.bag_timebase = (
                _manifest_bool(manifest, 'use_sim_time')
                if self.bag is not None else False)
            if self.bag is not None:
                self.bag.configure_timebase(
                    self.timeline_origin_ns if timestamped_videos else None,
                    use_sim_time=self.bag_timebase,
                )
            self.duration = max(self.front.duration, self.down.duration)
            if timestamped_videos:
                self.duration = max(
                    self.duration,
                    timestamped_duration
                    + 1.0 / max(video.fps for video in timestamped_videos),
                )
            if self.bag is not None and self.bag.end_ns is not None:
                self.duration = max(
                    self.duration,
                    max(
                        0.0,
                        (self.bag.end_ns - self.timeline_origin_ns)
                        / NANOSECONDS,
                    ),
                )
            self.position = 0.0
            self.speed = 1.0
            self.playing = True
            self.last_tick = time.monotonic()

            self.front_panel = Panel(self.front_stream)
            self.down_panel = Panel(self.down_stream)
            panels = QHBoxLayout()
            panels.addWidget(self.front_panel, 1)
            panels.addWidget(self.down_panel, 1)

            self.status = QLabel()
            self.slider = QSlider(Qt.Horizontal)
            self.slider.setRange(0, 10000)
            self.slider.sliderReleased.connect(self._seek_from_slider)

            controls = QHBoxLayout()
            self.play_button = QPushButton('暂停')
            self.play_button.clicked.connect(self._toggle_play)
            controls.addWidget(self.play_button)
            for value in (0.25, 0.5, 1.0, 2.0):
                button = QPushButton(f'{value:g}x')
                button.clicked.connect(
                    lambda _checked=False, v=value: self._set_speed(v))
                controls.addWidget(button)
            controls.addStretch(1)
            controls.addWidget(self.status)

            central = QWidget()
            layout = QVBoxLayout(central)
            layout.addLayout(panels, 1)
            layout.addWidget(self.slider)
            layout.addLayout(controls)
            self.setCentralWidget(central)
            self.manifest = manifest
            self._set_status()
            self.timer = QTimer(self)
            self.timer.timeout.connect(self._tick)
            self.timer.start(20)

        def _set_status(self):
            status = self.manifest.get('status', 'UNKNOWN')
            suffix = '（异常恢复 session）' if self.manifest.get('unclean') else ''
            topic_suffix = (
                f'  ROS话题:{self.bag.topic_count}'
                if self.bag is not None else '  ROS话题:未加载')
            self.status.setText(
                f'{status}{suffix}  {self.position:.1f}/{self.duration:.1f}s  '
                f'{self.speed:g}x{topic_suffix}')

        def _toggle_play(self):
            self.playing = not self.playing
            self.last_tick = time.monotonic()
            self.play_button.setText('暂停' if self.playing else '播放')

        def _set_speed(self, speed):
            self.speed = speed
            self.last_tick = time.monotonic()

        def _seek_from_slider(self):
            if self.duration <= 0.0:
                return
            self.position = self.slider.value() / 10000.0 * self.duration
            target_ns = self.timeline_origin_ns + int(
                self.position * NANOSECONDS)
            for video in (self.front, self.down):
                if isinstance(video, JpegArchiveVideo):
                    video.seek_ns(target_ns)
                else:
                    video.seek(self.position)
            if self.bag is not None:
                self.bag.seek(target_ns)
            self.last_tick = time.monotonic()

        def _tick(self):
            now = time.monotonic()
            if self.playing:
                self.position += (now - self.last_tick) * self.speed
                if self.position >= self.duration:
                    self.position = self.duration
                    self.playing = False
                    self.play_button.setText('播放')
            self.last_tick = now

            target_ns = self.timeline_origin_ns + int(
                self.position * NANOSECONDS)
            if self.bag is not None:
                self.bag.publish_until(target_ns)
            front = (
                self.front.read_to_ns(target_ns)
                if isinstance(self.front, JpegArchiveVideo)
                else self.front.read_to(self.position))
            down = (
                self.down.read_to_ns(target_ns)
                if isinstance(self.down, JpegArchiveVideo)
                else self.down.read_to(self.position))
            if front is not None:
                self.front_panel.set_frame(front)
            if down is not None:
                self.down_panel.set_frame(down)
            if self.duration > 0.0:
                self.slider.blockSignals(True)
                self.slider.setValue(int(self.position / self.duration * 10000))
                self.slider.blockSignals(False)
            self._set_status()

        def keyPressEvent(self, event: QKeyEvent):
            if event.key() == Qt.Key_Space:
                self._toggle_play()
                return
            super().keyPressEvent(event)

        def closeEvent(self, event):
            self.front.close()
            self.down.close()
            if self.bag is not None:
                self.bag.close()
            if self.ros_node is not None:
                self.ros_node.destroy_node()
            if (self.owns_ros_context and self.ros_module is not None
                    and self.ros_module.ok()):
                self.ros_module.shutdown()
            super().closeEvent(event)

    return PlayerWindow, QApplication


def _parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session_dir')
    return parser.parse_args()


def main():
    args = _parse_args()
    candidates = _session_candidates(args.session_dir)
    existing = [candidate for candidate in candidates if candidate.is_dir()]
    if not existing:
        print(
            f'uv_log player: session does not exist: {candidates[0]}',
            file=sys.stderr,
        )
        return 2
    session_dir = next(
        (_path for _path in existing if _has_valid_media(_path)), existing[0])
    if not _has_valid_media(session_dir):
        print(
            f'uv_log player: no valid video segments in {session_dir}',
            file=sys.stderr,
        )
        return 2
    PlayerWindow, QApplication = _build_window(session_dir)
    app = QApplication.instance() or QApplication(sys.argv)
    window = PlayerWindow()
    if window.front.empty and window.down.empty:
        print(
            f'uv_log player: video archives cannot be decoded in {session_dir}',
            file=sys.stderr,
        )
        return 2
    window.show()
    return app.exec()


if __name__ == '__main__':
    raise SystemExit(main())
