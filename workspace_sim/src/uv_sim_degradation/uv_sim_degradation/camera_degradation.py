"""Camera frame dropout adapter for visual-degradation experiments."""

from __future__ import annotations

import json
import random
import time

from auv_protocol.topics import (
    FRONT_STITCHED,
    SIM_DEGRADATION_EVENTS,
    SIM_DEGRADED_FRONT_STITCHED,
)
import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


def degrade_image(message: Image, *, rng: random.Random,
                  dropout_probability: float = 0.0,
                  brightness_scale: float = 1.0,
                  gaussian_noise_stddev: float = 0.0,
                  blur_kernel: int = 0) -> Image | None:
    """Apply deterministic dropout and optional optical degradation."""
    if rng.random() < float(dropout_probability):
        return None

    brightness_scale = float(brightness_scale)
    noise_stddev = max(0.0, float(gaussian_noise_stddev))
    kernel = int(blur_kernel)
    if kernel > 0 and kernel % 2 == 0:
        kernel += 1
    if (brightness_scale == 1.0 and noise_stddev == 0.0
            and kernel < 3):
        return message

    channels = {'mono8': 1, '8UC1': 1, 'bgr8': 3, 'rgb8': 3, '8UC3': 3}
    channel_count = channels.get(str(message.encoding).lower())
    expected_step = int(message.width) * (channel_count or 0)
    if channel_count is None or int(message.step) != expected_step:
        # Do not corrupt an encoding or padded image we cannot decode safely.
        return message
    expected_size = int(message.height) * expected_step
    if len(message.data) < expected_size:
        return message
    shape = (int(message.height), int(message.width))
    if channel_count == 3:
        shape += (3,)
    image = np.frombuffer(bytes(message.data), dtype=np.uint8,
                          count=expected_size).reshape(shape)
    result = image.astype(np.float32)
    if brightness_scale != 1.0:
        result *= brightness_scale
    if noise_stddev > 0.0:
        noise_rng = np.random.default_rng(rng.randrange(2 ** 32))
        result += noise_rng.normal(0.0, noise_stddev, size=result.shape)
    result = np.clip(result, 0.0, 255.0).astype(np.uint8)
    if kernel >= 3:
        result = cv2.GaussianBlur(result, (kernel, kernel), 0)

    output = Image()
    output.header = message.header
    output.height = message.height
    output.width = message.width
    output.encoding = message.encoding
    output.is_bigendian = message.is_bigendian
    output.step = message.step
    output.data = result.tobytes()
    return output


def keep_image(message: Image, *, rng: random.Random,
               dropout_probability: float = 0.0) -> Image | None:
    """Compatibility wrapper for callers that only need frame dropout."""
    return degrade_image(
        message, rng=rng, dropout_probability=dropout_probability)


class CameraDegradationNode(Node):
    """Drop complete stitched frames while preserving image metadata."""

    def __init__(self) -> None:
        super().__init__('camera_degradation')
        self.declare_parameter('camera', 'front')
        self.declare_parameter('input_topic', FRONT_STITCHED)
        self.declare_parameter('output_topic', SIM_DEGRADED_FRONT_STITCHED)
        self.declare_parameter('dropout_probability', 0.0)
        self.declare_parameter('brightness_scale', 1.0)
        self.declare_parameter('gaussian_noise_stddev', 0.0)
        self.declare_parameter('blur_kernel', 0)
        self.declare_parameter('delay_ms', 0.0)
        self.declare_parameter('seed', 0)
        self._camera = str(self.get_parameter('camera').value)
        self._dropout = float(self.get_parameter('dropout_probability').value)
        self._brightness = float(
            self.get_parameter('brightness_scale').value)
        self._noise = float(
            self.get_parameter('gaussian_noise_stddev').value)
        self._blur_kernel = int(self.get_parameter('blur_kernel').value)
        self._delay_s = max(
            0.0, float(self.get_parameter('delay_ms').value) / 1000.0)
        self._pending = []
        self._rng = random.Random(int(self.get_parameter('seed').value))
        output_topic = str(self.get_parameter('output_topic').value)
        input_topic = str(self.get_parameter('input_topic').value)
        self._pub = self.create_publisher(
            Image, output_topic, qos_profile_sensor_data)
        self._events = self.create_publisher(String, SIM_DEGRADATION_EVENTS, 10)
        self.create_subscription(
            Image, input_topic, self._callback, qos_profile_sensor_data)
        if self._delay_s > 0.0:
            self.create_timer(0.01, self._flush_pending)

    def _publish_or_delay(self, message: Image) -> None:
        if self._delay_s <= 0.0:
            self._pub.publish(message)
            return
        self._pending.append((time.monotonic() + self._delay_s, message))

    def _flush_pending(self) -> None:
        now = time.monotonic()
        ready = []
        while self._pending and self._pending[0][0] <= now:
            ready.append(self._pending.pop(0)[1])
        for message in ready:
            self._pub.publish(message)

    def _callback(self, message: Image) -> None:
        output = degrade_image(
            message, rng=self._rng,
            dropout_probability=self._dropout,
            brightness_scale=self._brightness,
            gaussian_noise_stddev=self._noise,
            blur_kernel=self._blur_kernel)
        if output is None:
            event = String()
            event.data = json.dumps({
                'event': 'camera_dropout', 'camera': self._camera,
            })
            self._events.publish(event)
            return
        self._publish_or_delay(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraDegradationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
