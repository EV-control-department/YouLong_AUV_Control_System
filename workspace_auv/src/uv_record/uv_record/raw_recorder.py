"""Record JPEG payloads verbatim from the Iceoryx2 camera services."""

from __future__ import annotations

import json
import math
import os
import statistics
import sys
import threading
import time
from collections import deque
from pathlib import Path

from auv_protocol.topics import ICEORYX_CAMERA_DOWN, ICEORYX_CAMERA_FRONT
from uv_image_transport.iceoryx2 import (
    ENCODING_JPEG, Iceoryx2Error, Iceoryx2Reader, InvalidFrameError,
    warn_invalid_frame,
)

from .session import append_event


SERVICES = {
    'front': ICEORYX_CAMERA_FRONT,
    'down': ICEORYX_CAMERA_DOWN,
}


class RawFrameRecorder:
    """Own one Iceoryx2 reader thread per selected camera."""

    def __init__(self, session_dir: str | Path, cameras=('front', 'down')):
        self.session_dir = Path(session_dir)
        self.root = self.session_dir / 'camera' / 'raw'
        self.cameras = tuple(cameras)
        invalid = set(self.cameras) - set(SERVICES)
        if invalid:
            raise ValueError(f'unknown camera(s): {", ".join(sorted(invalid))}')
        self.stop_event = threading.Event()
        self.readers: dict[str, Iceoryx2Reader] = {}
        self.threads: list[threading.Thread] = []
        self.lock = threading.Lock()
        self.counts = {camera: 0 for camera in self.cameras}
        self.unaligned = {camera: 0 for camera in self.cameras}
        self.probes = {
            camera: {
                'samples': deque(maxlen=300),
                'last': {},
                'total_jpeg_bytes': 0,
                'active': None,
            }
            for camera in self.cameras
        }
        self.error: Exception | None = None
        self.error_detail: dict | None = None

    def _set_active(self, camera: str, stage: str, frame_sequence=None):
        with self.lock:
            self.probes[camera]['active'] = {
                'stage': stage,
                'frame_sequence': frame_sequence,
                'started_monotonic_ns': time.monotonic_ns(),
                'started_unix_ns': time.time_ns(),
            }

    def start(self):
        self.root.mkdir(parents=True, exist_ok=True)
        for camera in self.cameras:
            thread = threading.Thread(
                target=self._record_camera, args=(camera,),
                name=f'uv-record-raw-{camera}', daemon=True)
            self.threads.append(thread)
            thread.start()

    def _record_camera(self, camera: str):
        service = SERVICES[camera]
        camera_dir = self.root / camera
        reader = None
        last_timestamp_ns = None
        last_capture_id = None
        last_stereo_pair_id = None
        last_receive_monotonic_ns = None
        timestamp_epoch = 0
        frame_sequence = 0
        stage = 'camera_directory_create'
        frame_path = None
        current_probe = {}
        try:
            camera_dir.mkdir(parents=True, exist_ok=True)
            stage = 'reader_open'
            reader = Iceoryx2Reader(service)
            self.readers[camera] = reader
            with (camera_dir / 'frames.jsonl').open('a', encoding='utf-8') as index:
                last_sync = time.monotonic()
                while not self.stop_event.is_set():
                    stage = 'reader_read'
                    self._set_active(camera, stage, frame_sequence)
                    read_started_ns = time.perf_counter_ns()
                    try:
                        packet = reader.read()
                        if packet is not None:
                            packet.validate()
                            if packet.header.encoding != ENCODING_JPEG:
                                raise InvalidFrameError('raw recording requires JPEG source frames')
                    except InvalidFrameError as error:
                        warn_invalid_frame(service, error)
                        continue
                    receive_monotonic_ns = time.perf_counter_ns()
                    read_wait_ms = (receive_monotonic_ns - read_started_ns) / 1e6
                    if packet is None:
                        with self.lock:
                            self.probes[camera]['active'] = None
                        break
                    camera_receive_unix_ns = time.time_ns()
                    reader_interarrival_ms = (
                        (receive_monotonic_ns - last_receive_monotonic_ns) / 1e6
                        if last_receive_monotonic_ns is not None else None)
                    last_receive_monotonic_ns = receive_monotonic_ns
                    header = packet.header
                    timestamp_ns = int(header.timestamp_ns)
                    capture_id = int(header.capture_id)
                    stereo_pair_id = int(header.stereo_pair_id)
                    source_interval_ms = (
                        (timestamp_ns - last_timestamp_ns) / 1e6
                        if last_timestamp_ns is not None else None)
                    capture_id_delta = (
                        capture_id - last_capture_id
                        if last_capture_id is not None else None)
                    stereo_pair_id_delta = (
                        stereo_pair_id - last_stereo_pair_id
                        if last_stereo_pair_id is not None else None)
                    if (last_timestamp_ns is not None
                            and timestamp_ns < last_timestamp_ns):
                        timestamp_epoch += 1
                    last_timestamp_ns = timestamp_ns
                    last_capture_id = capture_id
                    last_stereo_pair_id = stereo_pair_id
                    current_probe = {
                        'frame_sequence': frame_sequence,
                        'camera_receive_unix_ns': camera_receive_unix_ns,
                        'reader_wait_ms': round(read_wait_ms, 3),
                        'reader_interarrival_ms': (
                            round(reader_interarrival_ms, 3)
                            if reader_interarrival_ms is not None else None),
                        'source_interval_ms': (
                            round(source_interval_ms, 3)
                            if source_interval_ms is not None else None),
                        'capture_id_delta': capture_id_delta,
                        'stereo_pair_id_delta': stereo_pair_id_delta,
                    }
                    # capture_id can restart when the camera producer restarts;
                    # use a session-local sequence for unique filenames and
                    # preserve the source capture_id separately in the index.
                    filename = f'frame_{frame_sequence:020d}.jpg'
                    frame_path = camera_dir / filename
                    stage = 'jpeg_write'
                    self._set_active(camera, stage, frame_sequence)
                    write_started_ns = time.perf_counter_ns()
                    written = frame_path.write_bytes(packet.payload)
                    jpeg_write_ms = (time.perf_counter_ns() - write_started_ns) / 1e6
                    current_probe['jpeg_write_ms'] = round(jpeg_write_ms, 3)
                    if written != len(packet.payload):
                        raise OSError(f'failed to write {frame_path}')
                    receive_time_ns = time.time_ns()
                    file_write_complete_unix_ns = receive_time_ns
                    stage = 'jpeg_stat'
                    stat_started_ns = time.perf_counter_ns()
                    file_size_bytes = frame_path.stat().st_size
                    file_stat_ms = (time.perf_counter_ns() - stat_started_ns) / 1e6
                    current_probe = {
                        'frame_sequence': frame_sequence,
                        'camera_receive_unix_ns': camera_receive_unix_ns,
                        'file_write_complete_unix_ns': file_write_complete_unix_ns,
                        'reader_wait_ms': round(read_wait_ms, 3),
                        'reader_interarrival_ms': (
                            round(reader_interarrival_ms, 3)
                            if reader_interarrival_ms is not None else None),
                        'jpeg_write_ms': round(jpeg_write_ms, 3),
                        'file_stat_ms': round(file_stat_ms, 3),
                        'source_interval_ms': (
                            round(source_interval_ms, 3)
                            if source_interval_ms is not None else None),
                        'capture_id_delta': capture_id_delta,
                        'stereo_pair_id_delta': stereo_pair_id_delta,
                        'file_size_bytes': file_size_bytes,
                    }
                    record = {
                        'camera_group': camera,
                        'service': service,
                        'capture_id': capture_id,
                        'frame_sequence': frame_sequence,
                        'stereo_pair_id': stereo_pair_id,
                        'timestamp_epoch': timestamp_epoch,
                        'timestamp_ns': timestamp_ns,
                        'source_timestamp_ns': timestamp_ns,
                        'receive_time_unix_ns': receive_time_ns,
                        'camera_info_version': int(header.camera_info_version),
                        'width': int(header.width),
                        'height': int(header.height),
                        'stride': int(header.stride),
                        'encoding': 'JPEG',
                        'format': 'jpeg',
                        'timestamp_aligned': timestamp_ns > 0,
                        'path': filename,
                        'probe_version': 2,
                        'probe': current_probe,
                    }
                    stage = 'index_write_flush'
                    self._set_active(camera, stage, frame_sequence)
                    index_started_ns = time.perf_counter_ns()
                    index.write(json.dumps(record, ensure_ascii=False) + '\n')
                    index.flush()
                    index_write_flush_ms = (
                        time.perf_counter_ns() - index_started_ns) / 1e6
                    now = time.monotonic()
                    index_fsync_ms = None
                    if now - last_sync >= 1.0:
                        stage = 'index_fsync'
                        self._set_active(camera, stage, frame_sequence)
                        fsync_started_ns = time.perf_counter_ns()
                        os.fsync(index.fileno())
                        index_fsync_ms = (
                            time.perf_counter_ns() - fsync_started_ns) / 1e6
                        last_sync = now
                    with self.lock:
                        self.counts[camera] += 1
                        if timestamp_ns <= 0:
                            self.unaligned[camera] += 1
                        probe = self.probes[camera]
                        timings = {
                            'reader_wait_ms': read_wait_ms,
                            'reader_interarrival_ms': reader_interarrival_ms,
                            'jpeg_write_ms': jpeg_write_ms,
                            'file_stat_ms': file_stat_ms,
                            'index_write_flush_ms': index_write_flush_ms,
                            'index_fsync_ms': index_fsync_ms,
                        }
                        probe['samples'].append(timings)
                        probe['total_jpeg_bytes'] += file_size_bytes
                        probe['last'] = {
                            'frame_sequence': frame_sequence,
                            'capture_id': capture_id,
                            'capture_id_delta': capture_id_delta,
                            'stereo_pair_id': stereo_pair_id,
                            'stereo_pair_id_delta': stereo_pair_id_delta,
                            'source_timestamp_ns': timestamp_ns,
                            'source_interval_ms': (
                                round(source_interval_ms, 3)
                                if source_interval_ms is not None else None),
                            'camera_receive_unix_ns': camera_receive_unix_ns,
                            'file_write_complete_unix_ns': file_write_complete_unix_ns,
                            'file_size_bytes': file_size_bytes,
                            'timings_ms': {
                                key: round(value, 3) if value is not None else None
                                for key, value in timings.items()
                            },
                        }
                        probe['active'] = None
                    frame_sequence += 1
                index.flush()
                os.fsync(index.fileno())
        except Exception as error:
            if not self.stop_event.is_set() or not isinstance(error, Iceoryx2Error):
                filesystem = {}
                try:
                    stat = os.statvfs(camera_dir)
                    filesystem = {
                        'free_bytes': stat.f_bavail * stat.f_frsize,
                        'free_inodes': stat.f_favail,
                    }
                except OSError as stat_error:
                    filesystem = {'stat_error': str(stat_error)}
                detail = {
                    'camera': camera,
                    'stage': stage,
                    'frame_sequence': frame_sequence,
                    'path': str(frame_path) if frame_path is not None else None,
                    'error_type': type(error).__name__,
                    'error': str(error),
                    'errno': getattr(error, 'errno', None),
                    'filesystem': filesystem,
                    'last_probe': current_probe,
                    'time_unix_ns': time.time_ns(),
                }
                with self.lock:
                    self.error = error
                    self.error_detail = detail
                print(
                    'uv_record: raw camera probe error {}'.format(
                        json.dumps(detail, ensure_ascii=False)),
                    file=sys.stderr, flush=True)
                try:
                    append_event(self.session_dir, {
                        'event': 'raw_camera_error', **detail})
                except OSError as log_error:
                    print(
                        'uv_record: could not append raw camera error event: '
                        '{}'.format(log_error), file=sys.stderr, flush=True)
            self.stop_event.set()
        finally:
            if reader is not None:
                reader.close()
                self.readers.pop(camera, None)

    def snapshot(self) -> dict:
        with self.lock:
            counts = dict(self.counts)
            unaligned = dict(self.unaligned)
            probes = {
                camera: {
                    'samples': list(self.probes[camera]['samples']),
                    'last': dict(self.probes[camera]['last']),
                    'total_jpeg_bytes': self.probes[camera]['total_jpeg_bytes'],
                    'active': dict(self.probes[camera]['active'])
                    if self.probes[camera]['active'] else None,
                }
                for camera in self.cameras
            }
            error_detail = dict(self.error_detail) if self.error_detail else None
        diagnostics = {}
        for camera, probe in probes.items():
            samples = probe['samples']
            fields = (
                'reader_wait_ms', 'reader_interarrival_ms', 'jpeg_write_ms',
                'file_stat_ms', 'index_write_flush_ms',
                'index_fsync_ms',
            )
            rolling = {}
            for field in fields:
                values = sorted(
                    float(sample[field]) for sample in samples
                    if sample.get(field) is not None)
                if values:
                    p95_index = max(0, min(
                        len(values) - 1, math.ceil(len(values) * 0.95) - 1))
                    rolling[field] = {
                        'mean': round(statistics.fmean(values), 3),
                        'p95': round(values[p95_index], 3),
                        'max': round(values[-1], 3),
                    }
            diagnostics[camera] = {
                'frames_written': counts[camera],
                'total_jpeg_bytes': probe['total_jpeg_bytes'],
                'window_frames': len(samples),
                'last': probe['last'],
                'last_300_frames_ms': rolling,
                'active': ({
                    'stage': probe['active']['stage'],
                    'frame_sequence': probe['active']['frame_sequence'],
                    'started_unix_ns': probe['active']['started_unix_ns'],
                    'elapsed_ms': round(max(
                        0, time.monotonic_ns()
                        - probe['active']['started_monotonic_ns']) / 1e6, 3),
                } if probe['active'] else None),
            }
        return {
            'directory': 'camera/raw',
            'frames': counts,
            'unaligned_frames': unaligned,
            'probe_version': 2,
            'diagnostics': diagnostics,
            'error_detail': error_detail,
            'timestamp_source': 'iceoryx2_frame_header',
            'alignment_status': (
                'degraded' if (any(unaligned.values()) or any(
                    not counts[camera] for camera in self.cameras)) else 'aligned'),
        }

    def stop(self, timeout: float = 5.0):
        # Iceoryx2 receive polls every few milliseconds. Let each reader thread
        # leave read() and close its own native subscriber; mutating the binding
        # from this thread can deadlock while receive() is in progress.
        self.stop_event.set()
        deadline = time.monotonic() + max(0.0, float(timeout))
        for thread in self.threads:
            if thread.is_alive():
                thread.join(timeout=max(0.0, deadline - time.monotonic()))
        alive = [thread.name for thread in self.threads if thread.is_alive()]
        if alive and self.error is None:
            self.error = TimeoutError(
                'raw camera reader did not stop: {}'.format(', '.join(alive)))
        return not alive
