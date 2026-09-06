"""Crash-resilient ROS2 bag, MJPEG video and process-log recorder."""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

from .session import (
    SessionPaths,
    append_event,
    create_session,
    default_output_root,
    finish_session,
    initialise_session,
    load_manifest,
    update_heartbeat,
    update_manifest,
)
from .jpeg_archive import next_chunk_number
from .performance import ProcessSampler


DEFAULT_TOPIC_REGEX = (
    r'^/(clock|tf|tf_static|rosout|parameter_events|diagnostics|sim/.*|'
    r'auv/.*|zit6/.*|perception/.*|basic_motion/.*|task/.*|nav/.*|'
    r'cmd_vel.*)$'
)
# Recording image messages in rosbag duplicates the external video archive and
# is usually the largest source of recorder overhead.  Keep this metadata-only
# default narrow; image message types are excluded independently below even
# when a caller supplies a broader custom topic regex.
METADATA_TOPIC_REGEX = (
    r'^/(clock|tf|tf_static|rosout|parameter_events|diagnostics|sim/performance|'
    r'auv/thrusters_cmd|zit6/.*|perception/.*|basic_motion/.*|task/.*|'
    r'nav/.*|cmd_vel.*)$'
)
IMAGE_TOPIC_TYPES = (
    'sensor_msgs/msg/Image',
    'sensor_msgs/msg/CompressedImage',
    'stereo_msgs/msg/DisparityImage',
)
VIDEO_STREAMS = {
    'front_annotated': '/front_annotated',
    'down_annotated': '/down_annotated',
}
RAW_STREAMS = {
    'front': '/front',
    'down': '/down',
}


def _bool_value(value: str | bool) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _next_segment_number(directory: Path) -> int:
    numbers = []
    for path in directory.glob('*.ts'):
        match = re.search(r'(\d+)(?=\.ts$)', path.name)
        if match:
            numbers.append(int(match.group(1)))
    return max(numbers, default=-1) + 1


def _sync_file(path: Path) -> bool:
    try:
        fd = os.open(path, os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
        return True
    except OSError:
        return False


class SegmentSyncer(threading.Thread):
    """Periodically fsync stable video and rosbag files."""

    def __init__(
        self,
        directories: list[Path],
        patterns: tuple[str, ...] = ('*.ts',),
        period: float = 1.0,
    ):
        super().__init__(daemon=True, name='uv-log-segment-sync')
        self.directories = directories
        self.patterns = patterns
        self.period = max(0.25, period)
        self.stop_event = threading.Event()
        self._observed: dict[Path, tuple[int, int]] = {}
        self._synced: set[Path] = set()
        # Files are discovered recursively only every few seconds. Known
        # files are stat'ed between discoveries so a growing MCAP tail is
        # still synced without walking thousands of historical chunks.
        self._known: set[Path] = set()
        self._next_discovery = 0.0
        self.errors = 0

    def run(self):
        while not self.stop_event.is_set():
            self.sync_once()
            self.stop_event.wait(self.period)
        self.sync_once()

    def sync_once(self):
        now = time.monotonic()
        discover = now >= self._next_discovery
        if discover:
            self._next_discovery = now + 5.0
        for directory in self.directories:
            if not directory.is_dir():
                continue
            changed_directories: set[Path] = set()
            files = {p for p in self._known if p.is_relative_to(directory)}
            if discover:
                files.update(
                    p for p in directory.rglob('*')
                    if any(p.match(pattern) for pattern in self.patterns))
            self._known.update(files)
            for path in files:
                try:
                    stat = path.stat()
                except OSError:
                    continue
                signature = (stat.st_size, stat.st_mtime_ns)
                if self._observed.get(path) != signature:
                    self._observed[path] = signature
                    self._synced.discard(path)
                if path not in self._synced and stat.st_size > 0:
                    if _sync_file(path):
                        self._synced.add(path)
                        changed_directories.add(path.parent)
                    else:
                        self.errors += 1
            for changed_directory in changed_directories:
                try:
                    fd = os.open(
                        changed_directory,
                        os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0),
                    )
                    try:
                        os.fsync(fd)
                    finally:
                        os.close(fd)
                except OSError:
                    pass

    def stop(self):
        self.stop_event.set()

    def sync_all(self):
        """Force every currently present segment/file, including the tail."""
        directories = set()
        for directory in self.directories:
            if not directory.is_dir():
                continue
            files = {
                path for pattern in self.patterns
                for path in directory.rglob(pattern)
            }
            for path in files:
                try:
                    stat = path.stat()
                except OSError:
                    continue
                if (path in self._synced and self._observed.get(path) ==
                        (stat.st_size, stat.st_mtime_ns)):
                    continue
                if not _sync_file(path):
                    self.errors += 1
                directories.add(path.parent)
        for directory in directories:
            try:
                fd = os.open(
                    directory,
                    os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0),
                )
                try:
                    os.fsync(fd)
                finally:
                    os.close(fd)
            except OSError:
                pass


class ChildSupervisor(threading.Thread):
    """Run a child process and restart it after an unexpected exit."""

    def __init__(
        self,
        name: str,
        command_factory,
        log_path: Path,
        stop_signal=signal.SIGTERM,
    ):
        super().__init__(daemon=True, name=f'uv-log-{name}')
        self.name_label = name
        self.command_factory = command_factory
        self.log_path = log_path
        self.stop_signal = stop_signal
        self.stop_event = threading.Event()
        self.process_lock = threading.Lock()
        self.process: subprocess.Popen | None = None
        self.restarts = 0
        self.last_exit = None

    def run(self):
        while not self.stop_event.is_set():
            command = self.command_factory()
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            with self.log_path.open('ab', buffering=0) as logfile:
                logfile.write(
                    (f'\n[{time.strftime("%Y-%m-%d %H:%M:%S")}] '
                     f'starting {self.name_label}: '
                     f'{json.dumps(command, ensure_ascii=False)}\n').encode())
                try:
                    process = subprocess.Popen(
                        command,
                        stdin=subprocess.DEVNULL,
                        stdout=logfile,
                        stderr=subprocess.STDOUT,
                        cwd=str(self.log_path.parent.parent),
                        start_new_session=True,
                    )
                except OSError as error:
                    logfile.write(f'failed to start: {error}\n'.encode())
                    self.last_exit = 127
                    self.stop_event.wait(2.0)
                    continue

                with self.process_lock:
                    self.process = process
                while not self.stop_event.wait(0.25):
                    exit_code = process.poll()
                    if exit_code is not None:
                        self.last_exit = exit_code
                        logfile.write(
                            f'process exited with code {exit_code}\n'.encode())
                        break

                if self.stop_event.is_set() and process.poll() is None:
                    self._terminate(process, self.stop_signal)
                if process.poll() is None:
                    try:
                        process.wait(timeout=6.0)
                    except subprocess.TimeoutExpired:
                        self._kill(process)
                        process.wait(timeout=2.0)
                with self.process_lock:
                    self.process = None

            if not self.stop_event.is_set():
                self.restarts += 1
                self.stop_event.wait(1.0)

    @staticmethod
    def _terminate(process: subprocess.Popen, stop_signal=signal.SIGTERM):
        try:
            os.killpg(process.pid, stop_signal)
        except (ProcessLookupError, PermissionError, OSError):
            try:
                process.terminate()
            except OSError:
                pass

    @staticmethod
    def _kill(process: subprocess.Popen):
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError, OSError):
            try:
                process.kill()
            except OSError:
                pass

    def stop(self):
        self.stop_event.set()
        with self.process_lock:
            process = self.process
        if process is not None and process.poll() is None:
            self._terminate(process, self.stop_signal)

    def snapshot(self) -> dict:
        with self.process_lock:
            process = self.process
            pid = process.pid if process is not None else None
            alive = process is not None and process.poll() is None
        return {
            'pid': pid,
            'alive': alive,
            'restarts': self.restarts,
            'last_exit': self.last_exit,
        }


class Recorder:
    """Own one session and all of its independently recoverable writers."""

    def __init__(self, paths: SessionPaths, args):
        self.paths = paths
        self.args = args
        self.stop_event = threading.Event()
        self.heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop, daemon=True, name='uv-log-heartbeat')
        self.syncer = SegmentSyncer([])
        self.children: list[ChildSupervisor] = []
        self.video_directories: list[Path] = []
        self.performance = ProcessSampler(os.getppid())
        self.health_errors = 0
        self._stopped = False
        self._prepare_video_streams()

    def _recorded_streams(self):
        """Return only the video streams requested for this session.

        The annotated streams are tied to the YOLO inference rate, which is
        intentionally lower than the camera rate.  Recording them as the
        primary video therefore produces a slideshow even when the encoder is
        healthy.  Raw streams are the default; annotated streams remain
        available for sessions that explicitly request them.
        """
        mode = str(getattr(self.args, 'video_mode', 'raw')).strip().lower()
        if not _bool_value(getattr(self.args, 'enable_video', True)):
            return {}
        if mode == 'raw':
            return dict(RAW_STREAMS)
        if mode == 'annotated':
            return dict(VIDEO_STREAMS)
        if mode == 'both':
            streams = dict(RAW_STREAMS)
            streams.update(VIDEO_STREAMS)
            return streams
        raise ValueError(
            f'unsupported video mode {mode!r}; use raw, annotated, or both')

    def _prepare_video_streams(self):
        streams = self._recorded_streams()
        container = (
            'jpeg_archive' if self.args.video_format == 'jpeg' else 'mpegts')
        for name, path in streams.items():
            directory = self.paths.video / name
            directory.mkdir(parents=True, exist_ok=True)
            self.video_directories.append(directory)
            manifest = load_manifest(self.paths.root)
            videos = dict(manifest.get('video', {}))
            videos[name] = {
                'url': f'http://{self.args.host}:{self.args.port}{path}',
                'container': container,
                'segment_seconds': self.args.segment_duration,
                'fps': float(self.args.video_fps),
                'directory': str(directory),
            }
            if self.args.video_format == 'jpeg':
                videos[name]['frame_metadata'] = 'embedded_header+jsonl'
                videos[name]['timestamp_source'] = 'mjpeg_X-Frame-Stamp-Ns'
            update_manifest(self.paths.root, video=videos)

    def _video_command(self, name: str, path: str, directory: Path):
        if self.args.video_format == 'jpeg':
            start_number = next_chunk_number(directory)
        else:
            start_number = _next_segment_number(directory)
        url = f'http://{self.args.host}:{self.args.port}{path}'
        duration = max(0.5, float(self.args.segment_duration))
        command = [
            sys.executable, '-m', 'uv_log.mjpeg_proxy',
            '--url', url,
            '--output-dir', str(directory),
            '--start-number', str(start_number),
            '--segment-duration', str(duration),
            '--fps', str(self.args.video_fps),
            '--output-format', self.args.video_format,
        ]
        if self.args.video_format == 'ts':
            playlist = str(directory / f'index_{start_number:06d}.m3u8')
            command += [
                '--playlist', playlist,
                '--video-codec', self.args.video_codec,
                '--ffmpeg', self.args.ffmpeg,
            ]
        return command

    def _start_video_children(self):
        streams = self._recorded_streams()
        for name, path in streams.items():
            directory = self.paths.video / name
            child = ChildSupervisor(
                name,
                lambda n=name, p=path, d=directory:
                self._video_command(n, p, d),
                self.paths.logs / 'nodes' / f'video_{name}.log',
            )
            self.children.append(child)

    def _bag_topic_regex(self):
        """Return the topic-name filter; message-type exclusion is separate."""
        return self.args.topic_regex

    def _bag_command(self, part: int):
        output = self.paths.bag / f'part_{part:03d}'
        # ros2 bag's --max-bag-duration parser accepts integer seconds only.
        # Keep the launch argument flexible, but round fractional values up
        # so the requested maximum duration is never shortened.
        bag_duration = max(1, math.ceil(float(self.args.bag_duration)))
        command = [
            'ros2', 'bag', 'record',
            '--storage', 'mcap',
            '--output', str(output),
            '--regex', self._bag_topic_regex(),
            '--max-bag-duration', str(bag_duration),
            # Direct writes reduce the amount of data left only in rosbag's
            # memory cache when the machine becomes unresponsive.
            '--max-cache-size', '0',
            '--disable-keyboard-controls',
            # Video is archived outside rosbag.  Keep this type-level guard in
            # place even when a custom topic regex is supplied.
            '--exclude-topic-types', *IMAGE_TOPIC_TYPES,
        ]
        if _bool_value(self.args.use_sim_time):
            command.append('--use-sim-time')
        return command

    def _start_bag_child(self):
        existing = [
            int(match.group(1))
            for path in self.paths.bag.glob('part_*')
            if (match := re.fullmatch(r'part_(\d+)', path.name))
        ]
        next_part = max(existing, default=-1) + 1

        def command_factory():
            nonlocal next_part
            command = self._bag_command(next_part)
            next_part += 1
            return command

        self.children.append(ChildSupervisor(
            'rosbag2', command_factory,
            self.paths.logs / 'nodes' / 'rosbag2.log',
            stop_signal=signal.SIGINT))

    def _heartbeat_loop(self):
        next_sample = 0.0
        previous_children = {}
        while not self.stop_event.wait(1.0):
            try:
                snapshots = {
                    child.name_label: child.snapshot() for child in self.children}
                for name, state in snapshots.items():
                    if state != previous_children.get(name):
                        append_event(self.paths.root, {
                            'event': 'recorder_child_state', 'name': name, **state})
                previous_children = snapshots
                update_heartbeat(self.paths.root, children=snapshots,
                                 sync_errors=self.syncer.errors,
                                 health_errors=self.health_errors)
                if time.monotonic() < next_sample:
                    continue
                next_sample = time.monotonic() + 5.0
                sample = self.performance.sample()
                sample['disk_free_bytes'] = shutil.disk_usage(self.paths.root).free
                sample['children'] = snapshots
                sample['video'] = {}
                for directory in self.video_directories:
                    try:
                        status = json.loads((directory / 'status.json').read_text())
                        received = status.get('last_frame_received_unix_ns', 0)
                        status['frame_age_seconds'] = (
                            (time.time_ns() - received) / 1e9 if received else None)
                        sample['video'][directory.name] = status
                    except (OSError, ValueError):
                        sample['video'][directory.name] = {'status': 'waiting_for_video'}
                with (self.paths.metadata / 'performance.jsonl').open('a') as handle:
                    handle.write(json.dumps(sample, ensure_ascii=False) + '\n')
                    handle.flush()
                    os.fsync(handle.fileno())
            except (OSError, ValueError) as error:
                self.health_errors += 1
                print(f'uv_log: health logging failed: {error}', flush=True)

    def start(self):
        if self._recorded_streams() and self.args.video_format == 'ts' and shutil.which(self.args.ffmpeg) is None:
            raise RuntimeError(f'ffmpeg executable not found: {self.args.ffmpeg}')
        self._start_bag_child()
        self._start_video_children()
        self.syncer = SegmentSyncer(
            # JPEG writers sync their own active/closed chunks. Only legacy
            # TS and rosbag need an external syncer.
            ([*self.video_directories] if self.args.video_format == 'ts' else [])
            + [self.paths.bag],
            patterns=('*.ts', '*.mcap', 'metadata.yaml'),
        )
        self.syncer.start()
        for child in self.children:
            child.start()
        update_manifest(
            self.paths.root,
            bag={'directory': 'bag', 'storage': 'mcap',
                 'segment_seconds': float(self.args.bag_duration)},
            recorder={
                'pid': os.getpid(),
                'topic_regex': self._bag_topic_regex(),
                # Kept for manifest compatibility.  The legacy switch cannot
                # override the type-level image exclusion.
                'record_image_topics': False,
                'record_image_topics_requested': _bool_value(
                    getattr(self.args, 'record_image_topics', False)),
                'bag_segment_seconds': float(self.args.bag_duration),
                'video_segment_seconds': float(self.args.segment_duration),
                'video_mode': self.args.video_mode,
                'video_format': self.args.video_format,
                'video_fps': float(self.args.video_fps),
                'use_sim_time': _bool_value(self.args.use_sim_time),
                'image_topics_excluded': True,
                'enable_video': bool(self._recorded_streams()),
            },
        )
        if _bool_value(getattr(self.args, 'record_image_topics', False)):
            print(
                'uv_log: record_image_topics is deprecated and ignored; '
                'image message types are always excluded from rosbag',
                flush=True)
        append_event(self.paths.root, {'event': 'recorder_started'})
        self.heartbeat_thread.start()

    def stop(self):
        if self._stopped:
            return
        self._stopped = True
        self.stop_event.set()
        for child in self.children:
            child.stop()
        for child in self.children:
            if child.is_alive():
                child.join(timeout=8.0)
        self.syncer.stop()
        if self.syncer.is_alive():
            self.syncer.join(timeout=3.0)
        self.syncer.sync_all()
        if self.heartbeat_thread.is_alive():
            self.heartbeat_thread.join(timeout=2.0)
        update_manifest(self.paths.root, recording_health={
            'sync_errors': self.syncer.errors, 'health_errors': self.health_errors,
            'children': {child.name_label: child.snapshot() for child in self.children},
        })
        append_event(self.paths.root, {'event': 'recorder_stopped'})


def _parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session-dir')
    parser.add_argument('--output-root', default=str(default_output_root()))
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8090)
    parser.add_argument('--enable-video', default='true',
                        help='false records ROS/logs only, without reconnecting video workers')
    parser.add_argument('--segment-duration', type=float, default=2.0)
    parser.add_argument('--bag-duration', type=float, default=10.0)
    parser.add_argument(
        '--video-format', choices=('jpeg', 'ts'), default='jpeg',
        help=(
            'jpeg stores source JPEG frames and decodes on playback; '
            'ts keeps the legacy H.264 transcode path'))
    parser.add_argument(
        '--video-fps', type=float, default=10.0,
        help=(
            'Output cadence for the selected camera stream. Keep this equal '
            'to the source topic rate; default matches the simulator stitch '
            'rate'))
    parser.add_argument(
        '--video-mode', choices=('raw', 'annotated', 'both'), default='raw',
        help=(
            'Video streams to record. raw is recommended because annotated '
            'frames are produced at the lower AI inference rate'))
    parser.add_argument('--topic-regex', default=METADATA_TOPIC_REGEX)
    parser.add_argument(
        '--record-image-topics', default='false',
        help=(
            'Deprecated compatibility argument; image message types are '
            'always excluded because video is stored separately'))
    parser.add_argument(
        '--record-raw', default='false',
        help='deprecated compatibility argument; use --video-mode')
    parser.add_argument('--use-sim-time', default='false')
    parser.add_argument('--video-codec', default='libx264')
    parser.add_argument('--ffmpeg', default='ffmpeg')
    # launch_ros appends ``--ros-args -r __node:=...`` to every Node
    # executable.  This recorder is a supervised process rather than an
    # rclpy Node, so those arguments are intentionally not consumed here.
    args, _ros_args = parser.parse_known_args()
    return args


def _session_from_args(args) -> SessionPaths:
    if args.session_dir:
        return initialise_session(args.session_dir)
    return create_session(args.output_root)


def main():
    args = _parse_args()
    paths = None
    recorder = None
    stop_requested = threading.Event()

    def request_stop(_signum, _frame):
        stop_requested.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        paths = _session_from_args(args)
        recorder = Recorder(paths, args)
        recorder.start()
        print(f'uv_log: recording session {paths.root}', flush=True)
        while not stop_requested.wait(0.5):
            pass
        recorder.stop()
        finish_session(paths.root, 'STOPPED')
        print('uv_log: recording stopped cleanly', flush=True)
        return 0
    except KeyboardInterrupt:
        if recorder is not None:
            recorder.stop()
        if paths is not None:
            finish_session(paths.root, 'STOPPED')
        return 0
    except Exception as error:
        print(f'uv_log: recorder failed: {error}', flush=True)
        if recorder is not None:
            recorder.stop()
        if paths is not None:
            finish_session(paths.root, 'FAILED', error=str(error))
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
