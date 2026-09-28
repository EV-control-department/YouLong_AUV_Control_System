"""Read one iceoryx2 camera service and write MPEG-TS/H264 to stdout.

This process is deliberately not a ROS image node.  Its only binary output is
the ffmpeg stream consumed by go2rtc's ``exec`` source.  Detection metadata is
small ROS semantic data and is used only for the optional annotated overlay.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import argparse
import os
import subprocess
import sys
import threading
import time
import uuid
from typing import Iterable

import cv2
import numpy as np

from auv_protocol.topics import (
    ICEORYX_CAMERA_DOWN, ICEORYX_CAMERA_FRONT, PERCEPTION_DETECTIONS,
    STREAM_FRAME_INFO,
)
from uv_image_transport.iceoryx2 import (
    FramePacket, Iceoryx2Error, Iceoryx2Reader,
)


DISPLAY_WIDTH = 1280
DISPLAY_HEIGHT = 960
FPS_SAMPLE_FRAMES = 7
OVERLAY_WINDOW_NS = 300_000_000


@dataclass(frozen=True)
class CachedFrame:
    image: np.ndarray
    capture_id: int
    stereo_pair_id: int
    timestamp_ns: int
    arrival_monotonic_ns: int


@dataclass(frozen=True)
class DetectionBatch:
    camera_name: str
    capture_id: int
    stereo_pair_id: int
    timestamp_ns: int
    detections: tuple


class DetectionCache:
    """Thread-safe bounded detection metadata cache."""

    def __init__(self):
        self._lock = threading.Lock()
        self._batches: deque[DetectionBatch] = deque(maxlen=256)

    def add(self, message) -> None:
        stamp = getattr(message.header, 'stamp', None)
        timestamp_ns = 0
        if stamp is not None:
            timestamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        batch = DetectionBatch(
            camera_name=str(getattr(message, 'camera_name', '')).strip().lower(),
            capture_id=int(getattr(message, 'capture_id', 0)),
            stereo_pair_id=int(getattr(message, 'stereo_pair_id', 0)),
            timestamp_ns=timestamp_ns,
            detections=tuple(getattr(message, 'detections', ())),
        )
        if not batch.camera_name:
            return
        with self._lock:
            self._batches.append(batch)

    def for_frame(self, frame: CachedFrame) -> tuple:
        """Return exact metadata, falling back to a nearby frame only."""
        with self._lock:
            batches = tuple(self._batches)
        exact = [batch for batch in batches
                 if batch.camera_name and batch.capture_id == frame.capture_id
                 and batch.stereo_pair_id == frame.stereo_pair_id]
        if exact:
            return tuple(detection for batch in exact for detection in batch.detections)
        nearby = [batch for batch in batches
                  if batch.camera_name and batch.stereo_pair_id == frame.stereo_pair_id
                  and abs(batch.timestamp_ns - frame.timestamp_ns) <= OVERLAY_WINDOW_NS]
        if not nearby:
            return ()
        best = min(nearby, key=lambda batch: abs(batch.timestamp_ns - frame.timestamp_ns))
        return tuple(best.detections)


class StreamMetadataBridge:
    """Publish the exact source-to-encoded-frame map over a small ROS topic."""

    def __init__(self, camera: str, mode: str, stream_instance_id: str):
        self.node = None
        self._thread = None
        self.publisher = None
        self.rclpy = None
        self.executor = None
        self.camera = camera
        self.mode = mode
        self.stream_mode = 'annotated' if mode == 'annotated' else 'unannotated'
        self.stream_instance_id = stream_instance_id
        self.timestamp_epoch = 0
        self.last_source_timestamp_ns = None
        self.detection_cache = DetectionCache()
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import (
                QoSProfile, ReliabilityPolicy, qos_profile_sensor_data)
            from uv_msgs.msg import CameraStreamFrameInfo, DetectionArray
        except Exception as error:  # pragma: no cover - deployment error
            print(f'camera_streamer: frame metadata unavailable: {error}',
                  file=sys.stderr, flush=True)
            return

        self.rclpy = rclpy
        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = rclpy.create_node('camera_streamer_frame_metadata')
        # Keep depth below Fast DDS Foxy's default per-instance sample limit
        # (400); a depth of 8192 makes publisher creation fail on the Edge.
        qos = QoSProfile(depth=128)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.publisher = self.node.create_publisher(
            CameraStreamFrameInfo, STREAM_FRAME_INFO, qos)
        if mode == 'annotated':
            self.node.create_subscription(
                DetectionArray, PERCEPTION_DETECTIONS,
                self.detection_cache.add, qos_profile_sensor_data)
        self.message_type = CameraStreamFrameInfo
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self._thread = threading.Thread(
            target=self._spin, name='camera-streamer-ros', daemon=True)
        self._thread.start()

    def _spin(self):
        try:
            while self.rclpy.ok():
                self.executor.spin_once(timeout_sec=0.05)
        except Exception as error:  # pragma: no cover - shutdown race
            print(f'camera_streamer: metadata subscriber stopped: {error}',
                  file=sys.stderr, flush=True)

    def publish(self, frame: CachedFrame, sequence: int, output_fps: float):
        if self.publisher is None:
            return
        message = self.message_type()
        message.stream_instance_id = self.stream_instance_id
        message.camera_name = self.camera
        message.stream_mode = self.stream_mode
        message.frame_sequence = int(sequence)
        message.presentation_timestamp_ns = int(round(
            sequence * 1_000_000_000 / output_fps))
        stamp_ns = max(0, int(frame.timestamp_ns))
        message.source_stamp.sec = stamp_ns // 1_000_000_000
        message.source_stamp.nanosec = stamp_ns % 1_000_000_000
        message.capture_id = int(frame.capture_id)
        message.stereo_pair_id = int(frame.stereo_pair_id)
        if (self.last_source_timestamp_ns is not None
                and stamp_ns < self.last_source_timestamp_ns):
            self.timestamp_epoch += 1
        self.last_source_timestamp_ns = stamp_ns
        message.timestamp_epoch = self.timestamp_epoch
        message.output_fps = float(output_fps)
        self.publisher.publish(message)

    def close(self):
        if self.node is None:
            return
        try:
            self.executor.remove_node(self.node)
            self.node.destroy_node()
            if self.rclpy.ok():
                self.rclpy.shutdown()
        except Exception:
            pass


def _label(detection) -> str:
    class_id = int(getattr(detection, 'class_id', -1))
    try:
        from uv_perception.model_classes import model_class_name
        return model_class_name(class_id)
    except Exception:
        return f'class_{class_id}'


def _draw_detections(image: np.ndarray, detections: Iterable) -> np.ndarray:
    output = image.copy()
    # The source is a stitched 2560x960 frame.  Each half is compressed to
    # 640x960 in the 1280x960 display frame, so X is scaled by 0.5 and Y is
    # unchanged.
    for detection in detections:
        x1 = float(getattr(detection, 'bbox_x1', 0.0))
        y1 = float(getattr(detection, 'bbox_y1', 0.0))
        x2 = float(getattr(detection, 'bbox_x2', 0.0))
        y2 = float(getattr(detection, 'bbox_y2', 0.0))
        camera_name = str(getattr(detection, 'camera_name', ''))
        # Detection.msg intentionally has no camera_name because the enclosing
        # DetectionArray carries it.  The caller sets the half through a
        # temporary attribute-compatible tuple below when needed.
        half_offset = 0
        if camera_name.endswith('_right'):
            half_offset = DISPLAY_WIDTH // 2
        x1 = int(round(x1 * 0.5 + half_offset))
        x2 = int(round(x2 * 0.5 + half_offset))
        y1 = int(round(y1))
        y2 = int(round(y2))
        cv2.rectangle(output, (x1, y1), (x2, y2), (0, 220, 0), 2)
        text = f'{_label(detection)} {float(getattr(detection, "confidence", 0.0)):.2f}'
        cv2.putText(output, text, (max(0, x1), max(18, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 0), 2,
                    cv2.LINE_AA)
    return output


def _draw_batch(image: np.ndarray, camera_name: str, detections: Iterable) -> np.ndarray:
    """Attach the array-level camera side before drawing."""
    class SideDetection:
        __slots__ = ('source', 'detection')

        def __init__(self, source, detection):
            self.source = source
            self.detection = detection

        def __getattr__(self, name):
            if name == 'camera_name':
                return self.source
            return getattr(self.detection, name)

    return _draw_detections(
        image, (SideDetection(camera_name, detection) for detection in detections))


def _resize_stitched(packet: FramePacket) -> np.ndarray:
    image = packet.bgr()
    if image.shape[1] != 2560 or image.shape[0] != 960:
        return cv2.resize(image, (DISPLAY_WIDTH, DISPLAY_HEIGHT),
                          interpolation=cv2.INTER_AREA)
    return cv2.resize(image, (DISPLAY_WIDTH, DISPLAY_HEIGHT),
                      interpolation=cv2.INTER_AREA)


def _ffmpeg_process(output_fps: float):
    ffmpeg = os.environ.get('UV_STREAM_FFMPEG', 'ffmpeg')
    # Bound the GOP to two seconds at the measured source rate. A fixed
    # 10-fps encoder paired with unconditional 3:1 decimation made simulator
    # streams run at roughly 3 fps and pushed fresh-client IDRs far apart.
    gop_frames = max(1, int(round(output_fps * 2.0)))
    return subprocess.Popen([
        ffmpeg, '-loglevel', 'error',
        '-f', 'rawvideo', '-pix_fmt', 'bgr24',
        '-s:v', f'{DISPLAY_WIDTH}x{DISPLAY_HEIGHT}',
        '-r', f'{output_fps:.3f}', '-i', 'pipe:0',
        '-an', '-c:v', 'libx264', '-preset', 'ultrafast',
        '-tune', 'zerolatency', '-pix_fmt', 'yuv420p',
        '-g', str(gop_frames), '-keyint_min', str(gop_frames),
        '-sc_threshold', '0', '-x264-params', 'repeat-headers=1',
        '-mpegts_copyts', '1', '-f', 'mpegts', 'pipe:1',
    ], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=None, bufsize=0)


def _forward_stdout(process):
    assert process.stdout is not None
    output = sys.stdout.buffer
    try:
        while True:
            chunk = process.stdout.read(64 * 1024)
            if not chunk:
                return
            output.write(chunk)
            output.flush()
    except (BrokenPipeError, OSError):
        return


def _sample_source_rate(reader: Iceoryx2Reader, target_fps: float):
    """Estimate unique captured-frame rate before choosing encoder PTS/GOP."""
    packet = None
    arrivals = []
    timestamps = []
    for _ in range(FPS_SAMPLE_FRAMES):
        packet = reader.read()
        if packet is None:
            return None, max(0.1, float(target_fps)), 0, False
        arrivals.append(time.monotonic_ns())
        timestamp_ns = int(packet.header.timestamp_ns)
        if timestamp_ns > 0 and (not timestamps or timestamp_ns > timestamps[-1]):
            timestamps.append(timestamp_ns)

    use_header_timestamps = len(timestamps) >= 2
    if use_header_timestamps:
        elapsed_ns = timestamps[-1] - timestamps[0]
        measured_fps = ((len(timestamps) - 1) * 1_000_000_000 / elapsed_ns
                        if elapsed_ns > 0 else float(target_fps))
        first_clock_ns = int(packet.header.timestamp_ns)
    else:
        elapsed_ns = arrivals[-1] - arrivals[0]
        measured_fps = ((len(arrivals) - 1) * 1_000_000_000 / elapsed_ns
                        if elapsed_ns > 0 else float(target_fps))
        first_clock_ns = arrivals[-1]

    if not np.isfinite(measured_fps) or measured_fps <= 0.0:
        measured_fps = float(target_fps)
    effective_fps = max(0.1, min(float(target_fps), measured_fps))
    return packet, effective_fps, first_clock_ns, use_header_timestamps


def run(camera: str, mode: str, output_fps: float) -> int:
    service = ICEORYX_CAMERA_FRONT if camera == 'front' else ICEORYX_CAMERA_DOWN
    reader = Iceoryx2Reader(service)
    stream_instance_id = uuid.uuid4().hex
    metadata_bridge = StreamMetadataBridge(camera, mode, stream_instance_id)
    detection_cache = metadata_bridge.detection_cache
    encoder = None
    forwarder = None
    try:
        first_packet, effective_fps, last_frame_clock_ns, use_header_timestamps = (
            _sample_source_rate(reader, output_fps))
        if first_packet is None:
            return 0
        # The same rounded rate is used for rawvideo PTS and the published map.
        effective_fps = float(f'{effective_fps:.3f}')
        print(
            f'camera_streamer: {camera} input rate sampled; '
            f'encoding at {effective_fps:.2f} fps',
            file=sys.stderr, flush=True)
        encoder = _ffmpeg_process(effective_fps)
        forwarder = threading.Thread(target=_forward_stdout, args=(encoder,),
                                     name='camera-streamer-stdout', daemon=True)
        forwarder.start()
        cache: deque[CachedFrame] = deque()
        newest_arrival_ns = 0
        min_frame_period_ns = int(round(1_000_000_000 / effective_fps))
        output_sequence = 0

        def encode_packet(packet):
            nonlocal newest_arrival_ns, output_sequence
            image = _resize_stitched(packet)
            cached = CachedFrame(
                image, packet.header.capture_id, packet.header.stereo_pair_id,
                int(packet.header.timestamp_ns), time.monotonic_ns())
            cache.append(cached)
            newest_arrival_ns = max(
                newest_arrival_ns, cached.arrival_monotonic_ns)
            cutoff = newest_arrival_ns - OVERLAY_WINDOW_NS
            while cache and cache[0].arrival_monotonic_ns <= cutoff:
                ready = cache.popleft()
                frame = ready.image
                if mode == 'annotated':
                    frame = _annotate_from_cache(frame, ready, detection_cache)
                if encoder.stdin is None:
                    return False
                try:
                    encoder.stdin.write(frame.tobytes())
                    encoder.stdin.flush()
                except (BrokenPipeError, OSError):
                    return False
                metadata_bridge.publish(ready, output_sequence, effective_fps)
                output_sequence += 1
            return True

        if not encode_packet(first_packet):
            return 0
        while True:
            packet = reader.read()
            if packet is None:
                return 0
            clock_ns = (int(packet.header.timestamp_ns)
                        if use_header_timestamps else time.monotonic_ns())
            if use_header_timestamps and clock_ns <= 0:
                continue
            if clock_ns == last_frame_clock_ns:
                continue
            if clock_ns < last_frame_clock_ns:
                # Camera/ROS time can reset during a simulator reset.
                last_frame_clock_ns = clock_ns - min_frame_period_ns
            if clock_ns - last_frame_clock_ns < min_frame_period_ns:
                continue
            if not encode_packet(packet):
                return 0
            last_frame_clock_ns = clock_ns
    except KeyboardInterrupt:
        return 0
    except Iceoryx2Error as error:
        if 'Interrupt' in str(error):
            return 0
        raise
    finally:
        reader.close()
        metadata_bridge.close()
        if encoder is not None:
            if encoder.stdin is not None:
                encoder.stdin.close()
            try:
                encoder.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                encoder.kill()


def _annotate_from_cache(image, frame: CachedFrame, cache: DetectionCache):
    # Keep side grouping here rather than adding camera_name to every
    # Detection.msg.  DetectionCache exposes this small snapshot privately so
    # the ROS contract remains one array per camera side.
    with cache._lock:
        batches = tuple(cache._batches)
    exact = [batch for batch in batches
             if batch.capture_id == frame.capture_id
             and batch.stereo_pair_id == frame.stereo_pair_id]
    nearby = exact or [batch for batch in batches
                       if batch.stereo_pair_id == frame.stereo_pair_id
                       and abs(batch.timestamp_ns - frame.timestamp_ns) <= OVERLAY_WINDOW_NS]
    result = image
    for batch in nearby:
        result = _draw_batch(result, batch.camera_name, batch.detections)
    return result


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', choices=('front', 'down'), required=True)
    parser.add_argument('--mode', choices=('raw', 'annotated'), default='raw')
    parser.add_argument('--output-fps', type=float, default=10.0)
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    return run(args.camera, args.mode, max(1.0, args.output_fps))


if __name__ == '__main__':  # pragma: no cover
    raise SystemExit(main())
