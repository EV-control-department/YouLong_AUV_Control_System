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
from typing import Iterable

import cv2
import numpy as np

from auv_protocol.topics import ICEORYX_CAMERA_DOWN, ICEORYX_CAMERA_FRONT, PERCEPTION_DETECTIONS
from uv_perception.transport.iceoryx2 import FramePacket, Iceoryx2Reader


DISPLAY_WIDTH = 1280
DISPLAY_HEIGHT = 960
DISPLAY_EVERY_N = 3
OVERLAY_WINDOW_NS = 300_000_000


@dataclass(frozen=True)
class CachedFrame:
    image: np.ndarray
    capture_id: int
    stereo_pair_id: int
    timestamp_ns: int


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


class DetectionSubscriber:
    """Optional rclpy subscriber; failure leaves annotated output as raw."""

    def __init__(self, cache: DetectionCache):
        self.cache = cache
        self.node = None
        self._thread = None
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import qos_profile_sensor_data
            from uv_msgs.msg import DetectionArray
        except Exception as error:  # pragma: no cover - exercised outside ROS
            print(f'camera_streamer: ROS detections unavailable: {error}',
                  file=sys.stderr, flush=True)
            return

        self._rclpy = rclpy
        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = rclpy.create_node('camera_streamer_detection_overlay')
        self.node.create_subscription(
            DetectionArray, PERCEPTION_DETECTIONS, self.cache.add,
            qos_profile_sensor_data)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._thread = threading.Thread(
            target=self._spin, name='camera-streamer-ros', daemon=True)
        self._thread.start()

    def _spin(self):
        try:
            while self._rclpy.ok():
                self._executor.spin_once(timeout_sec=0.05)
        except Exception as error:  # pragma: no cover - shutdown race
            print(f'camera_streamer: detection subscriber stopped: {error}',
                  file=sys.stderr, flush=True)

    def close(self):
        if self.node is None:
            return
        try:
            self._executor.remove_node(self.node)
            self.node.destroy_node()
            if self._rclpy.ok():
                self._rclpy.shutdown()
        except Exception:
            pass


def _label(detection) -> str:
    class_id = int(getattr(detection, 'class_id', -1))
    try:
        from uv_camera.model_classes import model_class_name
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
    return subprocess.Popen([
        ffmpeg, '-loglevel', 'error',
        '-f', 'rawvideo', '-pix_fmt', 'bgr24',
        '-s:v', f'{DISPLAY_WIDTH}x{DISPLAY_HEIGHT}',
        '-r', f'{output_fps:.3f}', '-i', 'pipe:0',
        '-an', '-c:v', 'libx264', '-preset', 'ultrafast',
        '-tune', 'zerolatency', '-pix_fmt', 'yuv420p',
        '-f', 'mpegts', 'pipe:1',
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


def run(camera: str, mode: str, output_fps: float) -> int:
    service = ICEORYX_CAMERA_FRONT if camera == 'front' else ICEORYX_CAMERA_DOWN
    reader = Iceoryx2Reader(service)
    detection_cache = DetectionCache()
    detection_subscriber = DetectionSubscriber(detection_cache) if mode == 'annotated' else None
    encoder = _ffmpeg_process(output_fps)
    forwarder = threading.Thread(target=_forward_stdout, args=(encoder,),
                                  name='camera-streamer-stdout', daemon=True)
    forwarder.start()
    cache: deque[CachedFrame] = deque()
    input_count = 0
    newest_timestamp = 0
    try:
        while True:
            packet = reader.read()
            if packet is None:
                return 0
            input_count += 1
            if input_count % DISPLAY_EVERY_N:
                continue
            image = _resize_stitched(packet)
            cached = CachedFrame(image, packet.header.capture_id,
                                 packet.header.stereo_pair_id,
                                 packet.header.timestamp_ns)
            cache.append(cached)
            newest_timestamp = max(newest_timestamp, cached.timestamp_ns)
            cutoff = newest_timestamp - OVERLAY_WINDOW_NS
            while cache and cache[0].timestamp_ns <= cutoff:
                ready = cache.popleft()
                frame = ready.image
                if mode == 'annotated':
                    batches = detection_cache.for_frame(ready)
                    # The detection cache stores individual arrays.  Re-read
                    # the matching side from the cache so left/right are not
                    # ever drawn into the opposite half.
                    # ``for_frame`` returns only detections; matching side is
                    # resolved by the helper below from the latest batches.
                    frame = _annotate_from_cache(frame, ready, detection_cache)
                if encoder.stdin is None:
                    return 0
                try:
                    encoder.stdin.write(frame.tobytes())
                    encoder.stdin.flush()
                except (BrokenPipeError, OSError):
                    return 0
    except KeyboardInterrupt:
        return 0
    finally:
        reader.close()
        if detection_subscriber is not None:
            detection_subscriber.close()
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
