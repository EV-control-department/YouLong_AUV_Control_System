"""Losslessly record source BGR8 frames from the Iceoryx2 camera services."""

from __future__ import annotations

import json
import os
import threading
import time
from pathlib import Path

import cv2

from auv_protocol.topics import ICEORYX_CAMERA_DOWN, ICEORYX_CAMERA_FRONT
from uv_image_transport.iceoryx2 import Iceoryx2Error, Iceoryx2Reader


SERVICES = {
    'front': ICEORYX_CAMERA_FRONT,
    'down': ICEORYX_CAMERA_DOWN,
}


class RawFrameRecorder:
    """Own one Iceoryx2 reader thread per selected camera."""

    def __init__(self, session_dir: str | Path, cameras=('front', 'down')):
        self.root = Path(session_dir) / 'camera' / 'raw'
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
        self.error: Exception | None = None

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
        camera_dir.mkdir(parents=True, exist_ok=True)
        reader = None
        last_timestamp_ns = None
        timestamp_epoch = 0
        frame_sequence = 0
        try:
            reader = Iceoryx2Reader(service)
            self.readers[camera] = reader
            with (camera_dir / 'frames.jsonl').open('a', encoding='utf-8') as index:
                last_sync = time.monotonic()
                while not self.stop_event.is_set():
                    packet = reader.read()
                    if packet is None:
                        break
                    header = packet.header
                    timestamp_ns = int(header.timestamp_ns)
                    if (last_timestamp_ns is not None
                            and timestamp_ns < last_timestamp_ns):
                        timestamp_epoch += 1
                    last_timestamp_ns = timestamp_ns
                    image = packet.bgr()
                    # capture_id can restart when the camera producer restarts;
                    # use a session-local sequence for unique filenames and
                    # preserve the source capture_id separately in the index.
                    filename = f'frame_{frame_sequence:020d}.png'
                    frame_path = camera_dir / filename
                    if not cv2.imwrite(
                            str(frame_path), image,
                            [cv2.IMWRITE_PNG_COMPRESSION, 1]):
                        raise OSError(f'failed to write {frame_path}')
                    receive_time_ns = time.time_ns()
                    record = {
                        'camera_group': camera,
                        'service': service,
                        'capture_id': int(header.capture_id),
                        'frame_sequence': frame_sequence,
                        'stereo_pair_id': int(header.stereo_pair_id),
                        'timestamp_epoch': timestamp_epoch,
                        'timestamp_ns': timestamp_ns,
                        'source_timestamp_ns': timestamp_ns,
                        'receive_time_unix_ns': receive_time_ns,
                        'camera_info_version': int(header.camera_info_version),
                        'width': int(header.width),
                        'height': int(header.height),
                        'stride': int(header.stride),
                        'encoding': 'BGR8',
                        'timestamp_aligned': timestamp_ns > 0,
                        'path': filename,
                    }
                    index.write(json.dumps(record, ensure_ascii=False) + '\n')
                    index.flush()
                    now = time.monotonic()
                    if now - last_sync >= 1.0:
                        os.fsync(index.fileno())
                        last_sync = now
                    with self.lock:
                        self.counts[camera] += 1
                        if timestamp_ns <= 0:
                            self.unaligned[camera] += 1
                    frame_sequence += 1
        except Exception as error:
            if not self.stop_event.is_set() or not isinstance(error, Iceoryx2Error):
                with self.lock:
                    self.error = error
            self.stop_event.set()
        finally:
            if reader is not None:
                reader.close()
                self.readers.pop(camera, None)

    def snapshot(self) -> dict:
        with self.lock:
            counts = dict(self.counts)
            unaligned = dict(self.unaligned)
        return {
            'directory': 'camera/raw',
            'frames': counts,
            'unaligned_frames': unaligned,
            'timestamp_source': 'iceoryx2_frame_header',
            'alignment_status': (
                'degraded' if (any(unaligned.values()) or any(
                    not counts[camera] for camera in self.cameras)) else 'aligned'),
        }

    def stop(self):
        self.stop_event.set()
        for reader in tuple(self.readers.values()):
            reader.close()
        for thread in self.threads:
            if thread.is_alive():
                thread.join(timeout=2.0)
