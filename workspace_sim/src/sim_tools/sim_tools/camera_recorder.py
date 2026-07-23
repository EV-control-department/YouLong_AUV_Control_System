"""Record left front and left down simulator camera frames at a fixed rate."""

import os
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraRecorder(Node):
    """Save one frame per camera every configured interval."""

    def __init__(self):
        super().__init__('sim_camera_recorder')
        self.declare_parameter('output_dir', '/tmp/auv_camera_records')
        self.declare_parameter('interval_sec', 0.5)
        self.declare_parameter(
            'front_topic', '/sim/front_cam/left/image_color')
        self.declare_parameter(
            'down_topic', '/sim/down_cam/left/image_color')

        output_dir = self.get_parameter('output_dir').value
        self._output_dir = os.path.abspath(os.path.expanduser(str(output_dir)))
        self._interval = max(0.05, float(self.get_parameter('interval_sec').value))
        os.makedirs(self._output_dir, exist_ok=True)

        self._bridge = CvBridge()
        self._last_saved = {'front_left': 0.0, 'down_left': 0.0}
        self._sequence = {'front_left': 0, 'down_left': 0}
        self._save_lock = False

        front_topic = str(self.get_parameter('front_topic').value)
        down_topic = str(self.get_parameter('down_topic').value)
        self.create_subscription(
            Image, front_topic,
            lambda msg: self._image_cb('front_left', msg), 10)
        self.create_subscription(
            Image, down_topic,
            lambda msg: self._image_cb('down_left', msg), 10)
        self.get_logger().info(
            f'Recording front/down-left cameras every {self._interval:.2f}s '
            f'to {self._output_dir}')
        self.get_logger().info(
            f'Topics: front={front_topic}, down={down_topic}')

    def _image_cb(self, camera_name: str, msg: Image):
        now = time.monotonic()
        if now - self._last_saved[camera_name] < self._interval:
            return
        if self._save_lock:
            return
        self._save_lock = True
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if frame is None or frame.size == 0:
                return
            self._sequence[camera_name] += 1
            stamp = time.strftime('%Y%m%d_%H%M%S')
            filename = (
                f'{camera_name}_{stamp}_{self._sequence[camera_name]:06d}.jpg')
            path = os.path.join(self._output_dir, filename)
            if not cv2.imwrite(path, frame):
                self.get_logger().error(f'Failed to save image: {path}')
                return
            self._last_saved[camera_name] = now
        except Exception as exc:
            self.get_logger().error(
                f'Failed to record {camera_name} image: {exc}')
        finally:
            self._save_lock = False


def main(args=None):
    rclpy.init(args=args)
    node = CameraRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
