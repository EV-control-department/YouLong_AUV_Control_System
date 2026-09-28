"""Standalone pure camera driver: acquisition, CameraInfo, iceoryx2 raw frames."""

from __future__ import annotations

import threading
from dataclasses import replace

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from auv_protocol.topics import ICEORYX_CAMERA_DOWN, ICEORYX_CAMERA_FRONT
from uv_image_transport.iceoryx2 import (
    CAMERA_DOWN, CAMERA_FRONT, FrameHeader, Iceoryx2Publisher,
)

from .camera_config import load_camera_config
from .sensor import Sensor


class CameraDriver(Node):
    def __init__(self):
        super().__init__('uv_camera')
        self.declare_parameter('sim_mode', False)
        self.declare_parameter('enable_front', True)
        self.declare_parameter('enable_down', True)
        self.declare_parameter('camera_config_profile', 'auto')
        self.declare_parameter('camera_config_dir', '')
        self.declare_parameter('front_camera_device', '')
        self.declare_parameter('down_camera_device', '')
        self.declare_parameter('camera_startup_timeout_sec', 5.0)
        self.declare_parameter('camera_info_version', 1)
        sim_mode = bool(self.get_parameter('sim_mode').value)
        enable_front = bool(self.get_parameter('enable_front').value)
        enable_down = bool(self.get_parameter('enable_down').value)
        profile = str(self.get_parameter('camera_config_profile').value or 'auto')
        if profile.strip().lower() == 'auto':
            profile = 'sim' if sim_mode else 'real'
        config_dir = str(self.get_parameter('camera_config_dir').value or '')
        device_overrides = {
            'front': str(self.get_parameter('front_camera_device').value or '').strip(),
            'down': str(self.get_parameter('down_camera_device').value or '').strip(),
        }
        self._configs = {}
        for camera in ('front', 'down'):
            config = load_camera_config(camera, profile, config_dir or None)
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
        self._status = self.create_publisher(String, '/auv/camera/status', 10)
        self._sensor = Sensor(
            self, sim_mode=sim_mode, enable_front=enable_front,
            enable_down=enable_down,
            startup_timeout_s=float(self.get_parameter(
                'camera_startup_timeout_sec').value),
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
        self._publish_status('running')

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
        self._publish_status(f'{camera}: {reason}')
        self.get_logger().error(reason)

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
