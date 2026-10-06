"""Standalone pure camera driver: acquisition, CameraInfo, iceoryx2 raw frames."""

from __future__ import annotations

import threading
import time
from dataclasses import replace

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from uv_msgs.msg import SensorHealth

from auv_protocol.topics import (
    CAMERA_HEALTH, ICEORYX_CAMERA_DOWN, ICEORYX_CAMERA_FRONT,
)
from uv_image_transport.iceoryx2 import (
    CAMERA_DOWN, CAMERA_FRONT, FrameHeader, Iceoryx2Publisher,
)

from .camera_config import camera_mode_for_sim_mode, load_camera_config
from .sensor import Sensor


class CameraDriver(Node):
    def __init__(self):
        super().__init__('uv_camera')
        self.declare_parameter('sim_mode', False)
        self.declare_parameter('enable_front', True)
        self.declare_parameter('enable_down', True)
        self.declare_parameter('camera_mode', 'auto')
        self.declare_parameter('camera_config_dir', '')
        self.declare_parameter('front_camera_device', '')
        self.declare_parameter('down_camera_device', '')
        self.declare_parameter('camera_startup_timeout_sec', 5.0)
        self.declare_parameter('camera_reconnect_interval_sec', 1.0)
        self.declare_parameter('camera_info_version', 1)
        sim_mode = bool(self.get_parameter('sim_mode').value)
        enable_front = bool(self.get_parameter('enable_front').value)
        enable_down = bool(self.get_parameter('enable_down').value)
        camera_mode = camera_mode_for_sim_mode(
            sim_mode, str(self.get_parameter('camera_mode').value or 'auto'))
        config_dir = str(self.get_parameter('camera_config_dir').value or '')
        device_overrides = {
            'front': str(self.get_parameter('front_camera_device').value or '').strip(),
            'down': str(self.get_parameter('down_camera_device').value or '').strip(),
        }
        self._configs = {}
        for camera in ('front', 'down'):
            config = load_camera_config(camera, camera_mode, config_dir or None)
            if not sim_mode and device_overrides[camera]:
                config = replace(config, device=device_overrides[camera])
            if sim_mode:
                info_topics = {
                    'left': f'/auv/sensors/camera/{"downward" if camera == "down" else camera}/left/camera_info',
                    'right': f'/auv/sensors/camera/{"downward" if camera == "down" else camera}/right/camera_info',
                }
                config = replace(config, camera_info_topics=info_topics)
            self._configs[camera] = config
        self._ice_publishers = {}
        if enable_front:
            self._ice_publishers['front'] = Iceoryx2Publisher(
                ICEORYX_CAMERA_FRONT)
        if enable_down:
            self._ice_publishers['down'] = Iceoryx2Publisher(
                ICEORYX_CAMERA_DOWN)
        self._capture_id = 0
        self._lock = threading.Lock()
        self._status_lock = threading.Lock()
        self._camera_errors = {}
        self._frame_health_lock = threading.Lock()
        self._camera_frame_count = {'front': 0, 'down': 0}
        self._camera_last_frame_at = {'front': 0.0, 'down': 0.0}
        self._status = self.create_publisher(String, '/auv/camera/status', 10)
        self._health = self.create_publisher(SensorHealth, CAMERA_HEALTH, 10)
        self._health_timer = self.create_timer(0.5, self._publish_camera_health)
        self._sensor = Sensor(
            self, sim_mode=sim_mode, enable_front=enable_front,
            enable_down=enable_down,
            startup_timeout_s=float(self.get_parameter(
                'camera_startup_timeout_sec').value),
            reconnect_interval_s=float(self.get_parameter(
                'camera_reconnect_interval_sec').value),
            camera_configs=self._configs, frame_callback=self._on_frame)
        try:
            self._sensor.start()
        except Exception:
            for publisher in self._ice_publishers.values():
                publisher.close()
            raise
        self.get_logger().info(
            'uv_camera started: two stitched raw services '
            'youlong/camera/front and youlong/camera/down')
        self._publish_camera_status()

    def _on_frame(self, camera, frame, stamp, right_stamp=None,
                  stereo_pair_id=0):
        del right_stamp
        publisher = self._ice_publishers.get(camera)
        if publisher is None:
            return
        image = np.ascontiguousarray(frame)
        if image.ndim != 3 or image.shape[2] != 3:
            self.get_logger().error(f'{camera} frame is not BGR8: {image.shape}')
            return
        with self._frame_health_lock:
            self._camera_frame_count[camera] += 1
            self._camera_last_frame_at[camera] = time.monotonic()
        with self._lock:
            self._capture_id += 1
            capture_id = self._capture_id
        timestamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        pair_id = int(stereo_pair_id or capture_id)
        header = FrameHeader(
            capture_id=capture_id,
            timestamp_ns=timestamp_ns,
            stereo_pair_id=pair_id,
            camera_group=CAMERA_FRONT if camera == 'front' else CAMERA_DOWN,
            width=int(image.shape[1]), height=int(image.shape[0]),
            stride=int(image.strides[0]), camera_info_version=int(
                self.get_parameter('camera_info_version').value))
        try:
            publisher.publish(header, image.tobytes())
        except Exception as error:
            self.get_logger().error(f'{camera} iceoryx2 publish failed: {error}')

    def _publish_status(self, value):
        message = String()
        message.data = str(value)
        self._status.publish(message)

    def report_camera_failure(self, camera, reason):
        with self._status_lock:
            self._camera_errors[camera] = reason
        self._publish_camera_status()
        self.get_logger().error(f'{camera} camera failure: {reason}')

    def report_camera_recovered(self, camera, elapsed_s):
        message = f'{camera} camera recovered after {elapsed_s:.1f}s'
        with self._status_lock:
            self._camera_errors.pop(camera, None)
        self._publish_camera_status()
        self.get_logger().info(message)

    def _publish_camera_status(self):
        with self._status_lock:
            if self._camera_errors:
                value = '; '.join(
                    f'{camera}: error: {reason}'
                    for camera, reason in sorted(self._camera_errors.items()))
            else:
                value = 'running'
        self._publish_status(value)

    def _publish_camera_health(self):
        now = time.monotonic()
        with self._status_lock:
            errors = dict(self._camera_errors)
        with self._frame_health_lock:
            frames = dict(self._camera_frame_count)
            last_frames = dict(self._camera_last_frame_at)
        for camera in ('front', 'down'):
            age = now - last_frames[camera]
            available = (
                camera not in errors and frames[camera] >= 2
                and age <= 2.0)
            message = SensorHealth()
            message.header.stamp = self.get_clock().now().to_msg()
            message.sensor_name = f'camera/{camera}'
            message.available = available
            message.quality = 1.0 if available else 0.0
            message.detail = (
                f'frames={frames[camera]} age={age:.2f}s'
                if camera not in errors else f'error: {errors[camera]}')
            self._health.publish(message)

    def destroy_node(self):
        try:
            self._sensor.shutdown()
        except Exception:
            pass
        for publisher in self._ice_publishers.values():
            publisher.close()
        self._publish_status('stopped')
        return super().destroy_node()


def main(args=None):
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = None
    try:
        node = CameraDriver()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
