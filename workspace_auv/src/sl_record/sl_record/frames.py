"""Generic catalog-driven source JPEG writers; no fixed camera names or services."""

import json
import math
from pathlib import Path
import re
import threading
import time
import os

from sl_camera.catalog import as_dict
from uv_image_transport import ENCODING_JPEG, Iceoryx2Reader, InvalidFrameError
from uv_record.session import append_event, write_json_atomic

from .catalog_archive import restore_catalog_archive


class Sequences:
    def __init__(self):
        self.lock = threading.Lock()
        self.values = {}

    def take(self, directory):
        with self.lock:
            key = str(directory)
            if key not in self.values:
                existing = [
                    int(match.group(1)) for path in directory.glob('frame_*.jpg')
                    if (match := re.fullmatch(r'frame_(\d+)\.jpg', path.name))]
                self.values[key] = max(existing, default=-1) + 1
            value = self.values[key]
            self.values[key] += 1
            return value


class CameraWriter:
    def __init__(self, root, definition, instance, version, catalog_sequence, fps, sequences):
        self.definition = dict(definition)
        self.instance = instance
        self.version = version
        self.catalog_sequence = catalog_sequence
        self.key = f'camera_{definition["id"]}_{definition["name"]}'
        self.directory = Path(root) / 'camera/raw' / self.key
        self.interval_ns = max(1, round(1_000_000_000 / fps))
        self.sequences = sequences
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.reader = None
        self.error = None
        self.frames = self.skipped = self.rejected = self.unaligned = 0
        self.last_timestamp_ns = None
        self.thread = threading.Thread(target=self._run, name=f'sl-record-{self.key}', daemon=True)

    def update(self, definition, catalog_sequence):
        with self.lock:
            self.definition = dict(definition)
            self.catalog_sequence = catalog_sequence

    def start(self):
        self.thread.start()

    def _run(self):
        index = None
        last_clock = None
        previous_timestamp = None
        epoch = 0
        try:
            self.reader = Iceoryx2Reader(self.definition['ice_service'])
            while not self.stop_event.is_set():
                try:
                    read_started = time.perf_counter_ns()
                    packet = self.reader.read()
                    read_wait_ms = (time.perf_counter_ns() - read_started) / 1e6
                    if packet is None:
                        break
                    packet.validate()
                except InvalidFrameError:
                    with self.lock:
                        self.rejected += 1
                    continue
                received = time.time_ns()
                with self.lock:
                    definition = dict(self.definition)
                    catalog_sequence = self.catalog_sequence
                header = packet.header
                if (header.camera_group != definition['id']
                        or header.camera_info_version != self.version
                        or header.width != definition['width']
                        or header.height != definition['height']
                        or header.encoding != ENCODING_JPEG):
                    with self.lock:
                        self.rejected += 1
                    continue
                timestamp = int(header.timestamp_ns)
                if previous_timestamp is not None and timestamp < previous_timestamp:
                    epoch += 1
                    last_clock = None
                previous_timestamp = timestamp
                clock = timestamp if timestamp > 0 else time.monotonic_ns()
                if last_clock is not None and 0 <= clock - last_clock < self.interval_ns:
                    with self.lock:
                        self.skipped += 1
                    continue
                last_clock = clock
                if index is None:
                    self.directory.mkdir(parents=True, exist_ok=True)
                    index = (self.directory / 'frames.jsonl').open('a', encoding='utf-8')
                    last_sync = time.monotonic()
                sequence = self.sequences.take(self.directory)
                filename = f'frame_{sequence:020d}.jpg'
                write_started = time.perf_counter_ns()
                if (self.directory / filename).write_bytes(packet.payload) != len(packet.payload):
                    raise OSError('short JPEG write')
                write_finished = time.time_ns()
                record = {
                    'camera_id': definition['id'], 'camera_name': definition['name'],
                    'camera_group': definition['name'], 'service': definition['ice_service'],
                    'frame_id': definition['frame_id'],
                    'source_instance_id': self.instance, 'config_version': self.version,
                    'camera_info_version': self.version, 'catalog_sequence': catalog_sequence,
                    'frame_sequence': sequence, 'capture_id': int(header.capture_id),
                    'stereo_pair_id': int(header.stereo_pair_id),
                    'timestamp_ns': timestamp, 'source_timestamp_ns': timestamp,
                    'timestamp_epoch': epoch, 'timestamp_aligned': timestamp > 0,
                    'camera_receive_unix_ns': received,
                    'receive_time_unix_ns': write_finished,
                    'width': int(header.width), 'height': int(header.height),
                    'stride': 0, 'encoding': 'JPEG', 'format': 'jpeg', 'path': filename,
                    'probe_version': 2,
                    'probe': {
                        'camera_receive_unix_ns': received,
                        'file_write_complete_unix_ns': write_finished,
                        'reader_wait_ms': round(read_wait_ms, 3),
                        'jpeg_write_ms': round((time.perf_counter_ns() - write_started) / 1e6, 3),
                        'file_size_bytes': len(packet.payload),
                    },
                }
                index.write(json.dumps(record, ensure_ascii=False) + '\n')
                index.flush()
                if time.monotonic() - last_sync >= 1:
                    os.fsync(index.fileno())
                    last_sync = time.monotonic()
                with self.lock:
                    self.frames += 1
                    self.unaligned += int(timestamp <= 0)
                    self.last_timestamp_ns = timestamp
        except Exception as error:
            if not self.stop_event.is_set() or isinstance(error, OSError):
                self.error = error
        finally:
            try:
                if index is not None:
                    index.flush()
                    os.fsync(index.fileno())
            except OSError as error:
                self.error = error
            finally:
                if index is not None:
                    index.close()
                if self.reader is not None:
                    self.reader.close()

    def snapshot(self):
        with self.lock:
            return {
                'frames': self.frames, 'fps_skipped_frames': self.skipped,
                'rejected_frames': self.rejected, 'unaligned_frames': self.unaligned,
                'last_timestamp_ns': self.last_timestamp_ns,
            }

    def stop(self, timeout=5.0):
        self.stop_event.set()
        if self.reader is not None:
            self.reader.request_stop()
        if self.thread.ident is not None:
            self.thread.join(timeout)
        if self.thread.is_alive():
            self.error = TimeoutError(f'camera writer {self.key} did not stop')
            return False
        return True


def validate_catalog(catalog):
    if not catalog.source_instance_id or catalog.config_version == 0:
        raise ValueError('catalog requires source_instance_id and nonzero config_version')
    if (catalog.configured_count != len(catalog.cameras)
            or catalog.enabled_count != sum(camera.enabled for camera in catalog.cameras)
            or catalog.online_count != sum(camera.online for camera in catalog.cameras)):
        raise ValueError('catalog camera counts are inconsistent')
    seen = {key: set() for key in ('id', 'name', 'ice_service')}
    for item in catalog.cameras:
        if (not 0 < item.id < 2**32 or not re.fullmatch(r'[A-Za-z0-9_]+', item.name)
                or not item.ice_service or not item.frame_id or not item.width or not item.height
                or item.encoding != 'JPEG' or item.pixel_format != 'MJPG'
                or item.layout not in ('mono', 'stereo_sbs')
                or not math.isfinite(item.requested_fps) or item.requested_fps <= 0):
            raise ValueError('invalid camera identity, dimensions, format, or frame rate')
        for key in seen:
            value = getattr(item, key)
            if value in seen[key]:
                raise ValueError(f'duplicate catalog {key}: {value}')
            seen[key].add(value)


class DiscoveredFrames:
    """Own one writer for every enabled descriptor received over DDS."""

    def __init__(self, session_dir, fps=30.0):
        if not math.isfinite(fps) or fps <= 0:
            raise ValueError('recording fps must be finite and positive')
        self.root = Path(session_dir)
        self.fps = fps
        self.lock = threading.RLock()
        self.sequences = Sequences()
        self.writers = {}
        self.retired = []
        self.retired_instances = set()
        self.current_instance = None
        self.current_version = None
        self.catalog = None
        self.catalog_sequence = restore_catalog_archive(self.root)
        self.closed = False
        self.error = None

    def on_catalog(self, catalog):
        try:
            validate_catalog(catalog)
        except ValueError as error:
            append_event(self.root, {'event': 'catalog_rejected', 'error': str(error)})
            return
        with self.lock:
            if self.closed or catalog.source_instance_id in self.retired_instances:
                return
            data = as_dict(catalog)
            if self.current_instance and catalog.source_instance_id != self.current_instance:
                self.retired_instances.add(self.current_instance)
            self.catalog_sequence += 1
            history = {'catalog_sequence': self.catalog_sequence,
                       'received_unix_ns': time.time_ns(), 'catalog': data}
            try:
                self.root.joinpath('metadata').mkdir(parents=True, exist_ok=True)
                with (self.root / 'metadata/camera_catalogs.jsonl').open('a', encoding='utf-8') as stream:
                    stream.write(json.dumps(history, ensure_ascii=False) + '\n')
                    stream.flush()
                    os.fsync(stream.fileno())
                write_json_atomic(self.root / 'metadata/camera_catalog.json', history)
                enabled = {item['id']: item for item in data['cameras'] if item['enabled']}
                for camera_id, writer in list(self.writers.items()):
                    definition = enabled.get(camera_id)
                    # Runtime online/fps/counter updates do not replace readers.
                    if (definition is None or writer.instance != catalog.source_instance_id
                            or writer.version != catalog.config_version
                            or any(writer.definition[key] != definition[key] for key in (
                                'name', 'ice_service', 'width', 'height', 'frame_id',
                                'layout', 'calibrations'))):
                        writer.stop()
                        self.retired.append(writer)
                        del self.writers[camera_id]
                        if writer.error is not None:
                            raise writer.error
                self.current_instance = catalog.source_instance_id
                self.current_version = catalog.config_version
                self.catalog = data
                for camera_id, definition in enabled.items():
                    if camera_id in self.writers:
                        self.writers[camera_id].update(definition, self.catalog_sequence)
                        continue
                    writer = CameraWriter(
                        self.root, definition, catalog.source_instance_id,
                        catalog.config_version, self.catalog_sequence, self.fps, self.sequences)
                    self.writers[camera_id] = writer
                    writer.start()
            except Exception as error:
                self.error = error

    def snapshot(self):
        with self.lock:
            counts, skipped, rejected, unaligned = {}, {}, {}, {}
            for writer in [*self.retired, *self.writers.values()]:
                status = writer.snapshot()
                for target, field in ((counts, 'frames'), (skipped, 'fps_skipped_frames'),
                                      (rejected, 'rejected_frames'), (unaligned, 'unaligned_frames')):
                    target[writer.key] = target.get(writer.key, 0) + status[field]
                if writer.error is not None:
                    self.error = writer.error
            available = [key for key, count in counts.items() if count]
            waiting = [key for key, count in counts.items() if not count]
            alignment = 'degraded' if any(unaligned.values()) else (
                'aligned' if available else 'waiting')
            return {
                'frames': counts, 'fps_limit': self.fps, 'fps_skipped_frames': skipped,
                'rejected_frames': rejected, 'unaligned_frames': unaligned,
                'available_cameras': available, 'waiting_for_cameras': waiting,
                'alignment_status': alignment, 'catalog_sequence': self.catalog_sequence,
                'source_instance_id': self.current_instance,
                'config_version': self.current_version,
                'error': str(self.error) if self.error is not None else None,
            }

    def stop(self, timeout=5.0):
        with self.lock:
            self.closed = True
            deadline = time.monotonic() + timeout
            stopped = True
            for writer in self.writers.values():
                stopped = writer.stop(max(0.0, deadline - time.monotonic())) and stopped
                if writer.error is not None:
                    self.error = writer.error
            return stopped
