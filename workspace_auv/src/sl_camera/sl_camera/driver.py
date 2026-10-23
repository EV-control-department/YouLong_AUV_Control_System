"""Independent multi-camera acquisition and durable DDS discovery catalog."""

import secrets
import signal
import threading
import time
import uuid
from collections import deque

import rclpy
from rclpy.node import Node
from sl_msgs.msg import CameraCatalog
from uv_image_transport import ENCODING_JPEG, FrameHeader, Iceoryx2Publisher

from .capture import MjpegCapture
from .catalog import catalog_qos, descriptor
from .profile import load_profile


class SlCamera(Node):
    def __init__(self, profile=None, **kwargs):
        super().__init__('sl_camera', **kwargs)
        self.declare_parameter('profile', 'default')
        self.profile = profile or load_profile(self.get_parameter('profile').value)
        self.instance_id = str(uuid.uuid4())
        self.config_version = secrets.randbelow(2**32 - 1) + 1
        self._lock = threading.RLock()
        self._closed = False
        self._specs = {spec.id: spec for spec in self.profile.cameras}
        self._descriptors = {spec.id: descriptor(spec) for spec in self.profile.cameras}
        self._publishers_by_id = {}
        self._captures = {}
        self._last_frame_at = {}
        self._frame_times = {}
        self._catalog = self.create_publisher(
            CameraCatalog, self.profile.catalog_topic, catalog_qos())
        try:
            for spec in self.profile.cameras:
                if not spec.enabled:
                    continue
                self._publishers_by_id[spec.id] = Iceoryx2Publisher(spec.ice_service)
                self._captures[spec.id] = MjpegCapture(
                    spec.capture_config(), lambda: self.get_clock().now().nanoseconds,
                    lambda payload, stamp, key=spec.id: self._on_frame(key, payload, stamp),
                    lambda state, key=spec.id: self._on_status(key, state),
                    lambda fps, key=spec.id: self._on_fps(key, fps))
            # Describe services before any capture worker can produce frames.
            self._publish_catalog()
            self._timer = self.create_timer(1.0, self._publish_catalog)
            for capture in self._captures.values():
                capture.start()
        except Exception:
            self.destroy_node()
            raise
        self.get_logger().info(
            f'profile={self.profile.path}; cameras={len(self._specs)}, '
            f'enabled={len(self._captures)}; catalog={self.profile.catalog_topic}; '
            f'instance={self.instance_id}; source JPEG orientation is preserved')

    def _on_frame(self, camera_id, payload, timestamp_ns):
        with self._lock:
            if self._closed:
                return
            spec = self._specs[camera_id]
            item = self._descriptors[camera_id]
            capture_id = item.frames_published + 1
            self._publishers_by_id[camera_id].publish(FrameHeader(
                capture_id=capture_id, timestamp_ns=timestamp_ns, stereo_pair_id=0,
                camera_group=camera_id, width=spec.width, height=spec.height,
                stride=0, encoding=ENCODING_JPEG,
                camera_info_version=self.config_version), payload)
            item.frames_published = capture_id
            item.last_frame_stamp.sec, item.last_frame_stamp.nanosec = divmod(
                timestamp_ns, 1_000_000_000)
            now = time.monotonic()
            times = self._frame_times.setdefault(camera_id, deque(maxlen=31))
            if times and now - times[-1] > spec.startup_timeout_sec:
                times.clear()
            times.append(now)
            item.actual_fps = ((len(times) - 1) / (times[-1] - times[0])
                               if len(times) > 1 and times[-1] > times[0] else 0.0)
            self._last_frame_at[camera_id] = now

    def _on_fps(self, camera_id, fps):
        with self._lock:
            if not self._closed:
                self._descriptors[camera_id].negotiated_fps = float(fps)

    def _on_status(self, camera_id, state):
        with self._lock:
            if self._closed:
                return
            item = self._descriptors[camera_id]
            changed = item.status != state
            item.status = state
            if state.startswith('error:'):
                item.actual_fps = 0.0
                self._frame_times.pop(camera_id, None)
            if changed and rclpy.ok(context=self.context):
                if state.startswith('error:'):
                    self.get_logger().error(f'{item.name}: {state}')
                else:
                    self.get_logger().info(f'{item.name}: {state}')
        if changed:
            self._publish_catalog()

    def _publish_catalog(self):
        with self._lock:
            if self._closed or not rclpy.ok(context=self.context):
                return
            message = CameraCatalog()
            message.header.stamp = self.get_clock().now().to_msg()
            message.source_instance_id = self.instance_id
            message.config_version = self.config_version
            now = time.monotonic()
            for spec in self.profile.cameras:
                item = self._descriptors[spec.id]
                last = self._last_frame_at.get(spec.id)
                item.online = bool(
                    spec.enabled and last is not None
                    and now - last <= spec.startup_timeout_sec
                    and item.status == 'running')
                if not item.online:
                    item.actual_fps = 0.0
                message.cameras.append(item)
            message.configured_count = len(message.cameras)
            message.enabled_count = sum(item.enabled for item in message.cameras)
            message.online_count = sum(item.online for item in message.cameras)
            self._catalog.publish(message)

    def destroy_node(self):
        with self._lock:
            if self._closed:
                return
            self._closed = True
        for capture in self._captures.values():
            capture.stop_event.set()
        deadline = time.monotonic() + 5.0
        stuck = [self._specs[key].name for key, capture in self._captures.items()
                 if not capture.stop(timeout=max(0.0, deadline - time.monotonic()))]
        with self._lock:
            for publisher in self._publishers_by_id.values():
                publisher.close()
        result = super().destroy_node()
        if stuck:
            raise RuntimeError(f'camera capture threads did not stop: {stuck}')
        return result


def main(args=None):
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    node = None
    try:
        node = SlCamera()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            signal.signal(signal.SIGINT, previous)
