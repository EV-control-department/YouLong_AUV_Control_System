"""Join decoded go2rtc frame PTS values to source Iceoryx2 frame headers."""

from __future__ import annotations

import threading
import time

from auv_protocol.topics import STREAM_FRAME_INFO


class FrameMappingSubscriber:
    """Keep a bounded map of encoded PTS values to camera source frames."""

    def __init__(self, camera: str, stream_mode: str):
        self.camera = camera
        self.stream_mode = stream_mode
        self.lock = threading.Lock()
        self.maps: dict[str, dict[int, dict]] = {}
        self.latest_instance: str | None = None
        self.active_instance: str | None = None
        self.last_pts_ns: int | None = None
        self.awaiting_generation = False
        self.node = None
        self.executor = None
        self.thread = None
        self.rclpy = None
        self.error: str | None = None
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from uv_msgs.msg import CameraStreamFrameInfo

            self.rclpy = rclpy
            if not rclpy.ok():
                rclpy.init(args=None)
            self.node = rclpy.create_node('uv_record_frame_mapping')
            qos = QoSProfile(depth=8192)
            qos.reliability = ReliabilityPolicy.RELIABLE
            self.node.create_subscription(
                CameraStreamFrameInfo, STREAM_FRAME_INFO, self._on_mapping, qos)
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.node)
            self.thread = threading.Thread(
                target=self._spin, name='uv-record-frame-mapping', daemon=True)
            self.thread.start()
        except Exception as error:  # pragma: no cover - ROS install dependent
            self.error = str(error)

    def _spin(self):
        try:
            while self.rclpy.ok():
                self.executor.spin_once(timeout_sec=0.05)
        except Exception as error:  # pragma: no cover - shutdown race
            self.error = str(error)

    def _on_mapping(self, message):
        if (str(message.camera_name) != self.camera
                or str(message.stream_mode) != self.stream_mode):
            return
        instance = str(message.stream_instance_id)
        pts_ns = int(message.presentation_timestamp_ns)
        stamp = message.source_stamp
        source_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        value = {
            'stream_instance_id': instance,
            'camera_name': self.camera,
            'stream_mode': self.stream_mode,
            'frame_sequence': int(message.frame_sequence),
            'presentation_timestamp_ns': pts_ns,
            'source_timestamp_ns': source_ns,
            'capture_id': int(message.capture_id),
            'stereo_pair_id': int(message.stereo_pair_id),
            'timestamp_epoch': int(message.timestamp_epoch),
            'output_fps': float(message.output_fps),
            'mapping_received_unix_ns': time.time_ns(),
        }
        with self.lock:
            self.latest_instance = instance
            mappings = self.maps.setdefault(instance, {})
            mappings[pts_ns] = value
            if len(mappings) > 8192:
                for old_pts in sorted(mappings)[:len(mappings) - 4096]:
                    mappings.pop(old_pts, None)
            # Keep only a few encoder generations in memory across reconnects.
            if len(self.maps) > 4:
                for old_instance in list(self.maps):
                    if old_instance != instance and old_instance != self.active_instance:
                        self.maps.pop(old_instance, None)
                        if len(self.maps) <= 4:
                            break

    def match(self, pts_ns: int, tolerance_ns: int = 5_000_000) -> dict | None:
        """Find the source frame for one decoded PTS; never use wall time."""
        pts_ns = int(pts_ns)
        with self.lock:
            reset = (self.last_pts_ns is not None
                     and pts_ns + tolerance_ns < self.last_pts_ns)
            if self.active_instance is None:
                self.active_instance = self.latest_instance
            elif reset:
                # A timestamp reset should begin a new stream generation. Do
                # not trust callback arrival order: wait for a non-active
                # generation that actually contains this PTS.
                self.awaiting_generation = True
            if self.awaiting_generation:
                candidates = []
                for instance, generation in self.maps.items():
                    if instance == self.active_instance or not generation:
                        continue
                    nearest = min(
                        generation, key=lambda candidate: abs(candidate - pts_ns))
                    if abs(nearest - pts_ns) <= tolerance_ns:
                        value = generation[nearest]
                        candidates.append((
                            int(value.get('mapping_received_unix_ns', 0)),
                            instance, nearest, value))
                if candidates:
                    _, self.active_instance, nearest, value = max(
                        candidates, key=lambda candidate: candidate[0])
                    self.awaiting_generation = False
                    self.last_pts_ns = pts_ns
                    return dict(value)
                self.last_pts_ns = pts_ns
                return None
            self.last_pts_ns = pts_ns
            mappings = self.maps.get(self.active_instance or '', {})
            if not mappings:
                return None
            nearest = min(mappings, key=lambda candidate: abs(candidate - pts_ns))
            if abs(nearest - pts_ns) > tolerance_ns:
                return None
            return dict(mappings[nearest])

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
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=1.0)
