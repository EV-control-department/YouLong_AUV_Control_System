"""Publish one structured-light camera's source JPEG frames over iceoryx2."""

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from auv_protocol.topics import ICEORYX_CAMERA_SL, SL_CAMERA_STATUS
from uv_image_transport import CAMERA_SL, ENCODING_JPEG, FrameHeader, Iceoryx2Publisher

from .capture import CaptureConfig, MjpegCapture


class SlCamera(Node):
    def __init__(self):
        super().__init__('sl_camera')
        defaults = CaptureConfig()
        values = {}
        for name in defaults.__dataclass_fields__:
            self.declare_parameter(name, getattr(defaults, name))
            values[name] = self.get_parameter(name).value
        self.config = CaptureConfig(**values)
        self.config.validate()
        self._lock = threading.Lock()
        self._closed = False
        self._capture_id = 0
        self._state = 'starting'
        self._last_frame_at = None
        self._status = self.create_publisher(
            String, SL_CAMERA_STATUS,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._publisher = Iceoryx2Publisher(ICEORYX_CAMERA_SL)
        self._capture = MjpegCapture(
            self.config, lambda: self.get_clock().now().nanoseconds,
            self._on_frame, self._on_status)
        self._timer = self.create_timer(1.0, self._publish_status)
        self._capture.start()
        self.get_logger().info(
            f'sl_camera: {self.config.device}, {self.config.width}x{self.config.height}, '
            f'requested {self.config.fps:g} fps -> {ICEORYX_CAMERA_SL}; '
            'timestamps use the ROS clock immediately after capture')

    def _on_frame(self, payload, timestamp_ns):
        with self._lock:
            if self._closed:
                return
            self._capture_id += 1
            self._publisher.publish(FrameHeader(
                capture_id=self._capture_id, timestamp_ns=timestamp_ns,
                stereo_pair_id=0, camera_group=CAMERA_SL,
                width=self.config.width, height=self.config.height,
                stride=0, encoding=ENCODING_JPEG, camera_info_version=0,
            ), payload)
            self._last_frame_at = time.monotonic()

    def _on_status(self, value):
        with self._lock:
            if self._closed:
                return
            changed = self._state != value
            self._state = value
            if changed:
                logger = self.get_logger()
                (logger.error if value.startswith('error:') else logger.info)(value)
        self._publish_status()

    def _publish_status(self):
        with self._lock:
            if self._closed or not rclpy.ok(context=self.context):
                return
            age = (time.monotonic() - self._last_frame_at
                   if self._last_frame_at is not None else None)
            message = String()
            message.data = f'{self._state}; frames={self._capture_id}'
            if age is not None:
                message.data += f'; last_frame_age_sec={age:.3f}'
            self._status.publish(message)

    def destroy_node(self):
        stopped = self._capture.stop()
        if not stopped:
            self.get_logger().error('camera read did not return before shutdown timeout')
        self._on_status('stopped')
        with self._lock:
            self._closed = True
            self._publisher.close()
        return super().destroy_node()


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
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
