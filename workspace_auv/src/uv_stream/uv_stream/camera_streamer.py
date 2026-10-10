"""Read one iceoryx2 camera service and write MPEG-TS/H264 to stdout.

This process is deliberately not a ROS image node.  Its only binary output is
the ffmpeg stream consumed by go2rtc's ``exec`` source.  Detection metadata is
small ROS semantic data and is used only for the optional annotated overlay.
"""

from __future__ import annotations

from collections import deque, OrderedDict
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
    STREAM_FRAME_INFO, PERCEPTION_CAMERA_CALIBRATION,
)
from uv_image_transport.iceoryx2 import (
    FramePacket, Iceoryx2Error, Iceoryx2Reader, InvalidFrameError,
    warn_invalid_frame,
)


from uv_camera.image_geometry import CalibrationCache, EyeUndistorter

from .stream_geometry import (
    DISPLAY_HEIGHT, DISPLAY_WIDTH, resize_stitched_bgr, scale_detection_box,
)


FPS_SAMPLE_FRAMES = 7
OVERLAY_WINDOW_NS = 300_000_000


@dataclass(frozen=True)
class CachedFrame:
    image: np.ndarray
    capture_id: int
    stereo_pair_id: int
    timestamp_ns: int
    arrival_monotonic_ns: int
    camera: str = ''
    calibration_ids: tuple = (0, 0)
    image_space: int = 0


@dataclass(frozen=True)
class DetectionBatch:
    camera_name: str
    capture_id: int
    stereo_pair_id: int
    timestamp_ns: int
    detections: tuple
    image_space: int = 0
    calibration_id: int = 0
    camera_info_version: int = 0


class DetectionCache:
    """Thread-safe bounded detection metadata cache."""

    def __init__(self, camera=None):
        self.camera = camera
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
            image_space=int(getattr(message, 'image_space', 0)),
            calibration_id=int(getattr(message, 'calibration_id', 0)),
            camera_info_version=int(getattr(message, 'camera_info_version', 0)),
        )
        if not batch.camera_name or (self.camera and not batch.camera_name.startswith(self.camera + '_')):
            return
        with self._lock:
            self._batches.append(batch)

    def clear(self):
        with self._lock:
            self._batches.clear()

    def for_frame_batches(self, frame: CachedFrame):
        with self._lock:
            batches = tuple(self._batches)
        def compatible(batch):
            if frame.camera and not batch.camera_name.startswith(frame.camera + '_'):
                return False
            if batch.image_space != frame.image_space:
                return False
            side = 1 if batch.camera_name.endswith('_right') else 0
            return (frame.image_space == 0
                    or batch.calibration_id == frame.calibration_ids[side])
        batches = [batch for batch in batches if compatible(batch)]
        selected = []
        for side in ('left', 'right'):
            candidates = [batch for batch in batches if batch.camera_name.endswith('_' + side)]
            exact = [batch for batch in candidates if batch.capture_id == frame.capture_id
                     and batch.stereo_pair_id == frame.stereo_pair_id
                     and batch.timestamp_ns == frame.timestamp_ns]
            nearby = exact or [batch for batch in candidates
                               if batch.stereo_pair_id == frame.stereo_pair_id
                               and abs(batch.timestamp_ns - frame.timestamp_ns) <= OVERLAY_WINDOW_NS]
            if nearby:
                selected.append(min(nearby, key=lambda item: abs(item.timestamp_ns - frame.timestamp_ns)))
        return tuple(selected)

    def for_frame(self, frame: CachedFrame) -> tuple:
        return tuple(detection for batch in self.for_frame_batches(frame)
                     for detection in batch.detections)


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
        self.detection_cache = DetectionCache(camera)
        self.calibrations = CalibrationCache()
        self._corrections = OrderedDict()
        self._last_calibration_warning = float('-inf')
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import (
                QoSProfile, ReliabilityPolicy, qos_profile_sensor_data)
            from uv_msgs.msg import CameraStreamFrameInfo, DetectionArray, PerceptionCameraInfo
            from rclpy.qos import DurabilityPolicy
        except Exception as error:  # pragma: no cover - deployment error
            print(f'camera_streamer: frame metadata unavailable: {error}',
                  file=sys.stderr, flush=True)
            return

        # Raw video does not need the optional ROS frame-metadata channel.
        # Avoid creating another Fast DDS publisher for every go2rtc exec
        # process; on Foxy this can fail when several simulator processes share
        # the same domain/resources and would otherwise kill the video pipe.
        if mode != 'annotated':
            return

        self.rclpy = rclpy
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            self.node = rclpy.create_node('camera_streamer_frame_metadata')
            qos = QoSProfile(depth=8192)
            qos.reliability = ReliabilityPolicy.RELIABLE
            self.publisher = self.node.create_publisher(
                CameraStreamFrameInfo, STREAM_FRAME_INFO, qos)
            self.node.create_subscription(
                DetectionArray, PERCEPTION_DETECTIONS,
                self.detection_cache.add, qos_profile_sensor_data)
            calibration_qos = QoSProfile(depth=16, reliability=ReliabilityPolicy.RELIABLE,
                                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
            for side in ('left', 'right'):
                self.node.create_subscription(
                    PerceptionCameraInfo, PERCEPTION_CAMERA_CALIBRATION(f'{camera}_{side}'),
                    self._calibration, calibration_qos)
            self.message_type = CameraStreamFrameInfo
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.node)
            self._thread = threading.Thread(
                target=self._spin, name='camera-streamer-ros', daemon=True)
            self._thread.start()
        except Exception as error:
            # Metadata/overlay is optional; the encoded stream must remain
            # available even when DDS cannot allocate another publisher.
            print(
                f'camera_streamer: metadata disabled because ROS publisher '
                f'could not be created: {error}',
                file=sys.stderr, flush=True)
            self.close()

    def _calibration(self, message):
        try:
            self.calibrations.add(message)
        except ValueError as error:
            print(f'camera_streamer: invalid calibration: {error}', file=sys.stderr, flush=True)

    def prepare(self, packet):
        # Only called after frame selection. Never decode again for overlay.
        image = packet.bgr()
        if self.mode != 'annotated':
            return resize_stitched_bgr(image), (0, 0), 0
        entries = [self.calibrations.latest(f'{self.camera}_{side}', packet.header.camera_info_version)
                   for side in ('left', 'right')]
        if any(entry is None for entry in entries):
            now = time.monotonic()
            if now - self._last_calibration_warning >= 5.0:
                self._last_calibration_warning = now
                print('camera_streamer: waiting for calibration; showing unannotated source image',
                      file=sys.stderr, flush=True)
            return resize_stitched_bgr(image), (0, 0), 0
        half = image.shape[1] // 2
        corrected, ids = [], []
        for index, entry in enumerate(entries):
            key = (entry.camera_name, int(entry.calibration_id))
            transform = self._corrections.get(key)
            if transform is None:
                transform = EyeUndistorter(entry.source_info, entry.camera_info_version)
                if transform.calibration_id != entry.calibration_id:
                    raise InvalidFrameError('perception calibration ID does not match its source')
                self._corrections[key] = transform
                while len(self._corrections) > 8:
                    self._corrections.popitem(last=False)
            try:
                corrected.append(transform.apply(image[:, index*half:(index+1)*half]))
            except ValueError as error:
                raise InvalidFrameError(str(error)) from error
            ids.append(entry.calibration_id)
        return resize_stitched_bgr(np.hstack(corrected)), tuple(ids), 1

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
    # The source is a stitched 2560x960 frame. Each half is scaled uniformly
    # to 640x480 in the 1280x480 stream, so both X and Y use a 0.5 scale.
    for detection in detections:
        x1 = float(getattr(detection, 'bbox_x1', 0.0))
        y1 = float(getattr(detection, 'bbox_y1', 0.0))
        x2 = float(getattr(detection, 'bbox_x2', 0.0))
        y2 = float(getattr(detection, 'bbox_y2', 0.0))
        camera_name = str(getattr(detection, 'camera_name', ''))
        # Detection.msg intentionally has no camera_name because the enclosing
        # DetectionArray carries it.  The caller sets the half through a
        # temporary attribute-compatible tuple below when needed.
        x1, y1, x2, y2 = scale_detection_box(
            (x1, y1, x2, y2), camera_name)
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
    return resize_stitched_bgr(packet.bgr())


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


def _open_video_output():
    """Reserve the media pipe and route all other process stdout to stderr.

    Native DDS/iceoryx2 diagnostics can write directly to fd 1, bypassing
    Python's print routing. Keep a duplicate for encoded video only, then
    redirect fd 1 before either runtime is initialized. The CLI leaves this
    redirection in place through shutdown so native teardown logs stay out of
    the video pipe too.
    """
    sys.stdout.flush()
    video_fd = os.dup(sys.stdout.fileno())
    try:
        os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
        return os.fdopen(video_fd, 'wb')
    except BaseException:
        os.close(video_fd)
        raise


def _forward_stdout(process, output):
    assert process.stdout is not None
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
    while len(arrivals) < FPS_SAMPLE_FRAMES:
        try:
            packet = reader.read()
        except InvalidFrameError as error:
            warn_invalid_frame(getattr(reader, 'service', 'camera'), error)
            continue
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


def run(camera: str, mode: str, output_fps: float, *, video_output=None) -> int:
    if video_output is None:
        video_output = sys.stdout.buffer
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
        forwarder = threading.Thread(target=_forward_stdout,
                                     args=(encoder, video_output),
                                     name='camera-streamer-stdout', daemon=True)
        forwarder.start()
        cache: deque[CachedFrame] = deque()
        newest_arrival_ns = 0
        min_frame_period_ns = int(round(1_000_000_000 / effective_fps))
        output_sequence = 0
        last_capture_id = int(first_packet.header.capture_id)
        last_info_version = int(first_packet.header.camera_info_version)

        def encode_packet(packet):
            nonlocal newest_arrival_ns, output_sequence
            try:
                image, calibration_ids, image_space = metadata_bridge.prepare(packet)
            except InvalidFrameError as error:
                warn_invalid_frame(service, error)
                return True
            cached = CachedFrame(
                image, packet.header.capture_id, packet.header.stereo_pair_id,
                int(packet.header.timestamp_ns), time.monotonic_ns(), camera,
                calibration_ids, image_space)
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
            try:
                packet = reader.read()
            except InvalidFrameError as error:
                warn_invalid_frame(service, error)
                continue
            if packet is None:
                return 0
            clock_ns = (int(packet.header.timestamp_ns)
                        if use_header_timestamps else time.monotonic_ns())
            if use_header_timestamps and clock_ns <= 0:
                continue
            capture_id = int(packet.header.capture_id)
            info_version = int(packet.header.camera_info_version)
            if (clock_ns < last_frame_clock_ns or capture_id < last_capture_id
                    or info_version != last_info_version):
                # Do not overlay detections retained from a previous producer,
                # simulator clock epoch, or source calibration.
                cache.clear()
                detection_cache.clear()
                last_frame_clock_ns = clock_ns - min_frame_period_ns
            last_capture_id, last_info_version = capture_id, info_version
            if clock_ns == last_frame_clock_ns:
                continue
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
                encoder.wait(timeout=1.0)
        if forwarder is not None:
            forwarder.join(timeout=1.0)


def _annotate_from_cache(image, frame: CachedFrame, cache: DetectionCache):
    result = image
    for batch in cache.for_frame_batches(frame):
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
    with _open_video_output() as video_output:
        return run(args.camera, args.mode, max(1.0, args.output_fps),
                   video_output=video_output)


if __name__ == '__main__':  # pragma: no cover
    raise SystemExit(main())
