"""uv_sensor: frame source (Stonefish sim OR real V4L2 camera) + go2rtc preview.

In the SAME process as uv_ai. uv_sensor:
  * picks the source via params (sim_mode: ROS stitched topics  OR  V4L2 /dev/video*);
  * on each frame, updates the raw MJPEG preview cache (fed to go2rtc :1984)
    and hands the BGR frame to uv_ai through an in-memory FrameGate (A3: no ROS
    image topic, no JPEG between sensor and ai).
"""

import threading
import time

import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from .common import (
    DOWN_CAMERA_DEVICE,
    FRONT_CAMERA_DEVICE,
    DOWN_CAPTURE_RESOLUTION,
    FRONT_CAPTURE_RESOLUTION,
    normalize_frame,
)
from sensor_msgs.msg import Image

try:
    from uv_msgs.msg import StereoFrameInfo
except ImportError:  # Older installed interfaces remain usable as a fallback.
    StereoFrameInfo = None


class Sensor:
    """Frame producer: chooses source, updates raw preview, feeds FrameGate."""

    def __init__(self, node, gate, sim_mode, enable_front, enable_down):
        self.node = node                 # composed uv_camera rclpy Node
        self.gate = gate                 # common.FrameGate -> ai consumer
        self._sim_mode = sim_mode
        self._enable_front = enable_front
        self._enable_down = enable_down

        self._capture_stop = threading.Event()
        self._capture_threads = []
        self._front_cap = None
        self._down_cap = None
        self._image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._stereo_info = {camera: {} for camera in ('front', 'down')}
        self._pending_sim_images = {}
        self._pending_sim_lock = threading.Lock()
        self._pending_sim_timer = None

    # ── setup: pick source ──────────────────────────────────────────────
    def start(self):
        if self._sim_mode:
            self._start_sim()
        else:
            self._start_v4l2()

    def _start_sim(self):
        if StereoFrameInfo is not None:
            if self._enable_front:
                self.node.create_subscription(
                    StereoFrameInfo, '/auv/front_cam/stereo_info',
                    self._front_stereo_info_cb, self._image_qos)
            if self._enable_down:
                self.node.create_subscription(
                    StereoFrameInfo, '/auv/down_cam/stereo_info',
                    self._down_stereo_info_cb, self._image_qos)
        if self._enable_front:
            self.node.create_subscription(
                Image, '/auv/front_cam/stitched', self._front_img_cb,
                self._image_qos)
        if self._enable_down:
            self.node.create_subscription(
                Image, '/auv/down_cam/stitched', self._down_img_cb,
                self._image_qos)
        # Metadata and image are published back-to-back, but BEST_EFFORT ROS
        # delivery does not promise callback order.  Hold an image briefly so
        # the right-eye timestamp can normally arrive first; if metadata is
        # missing, the image is still processed with the legacy single stamp.
        if StereoFrameInfo is not None:
            self._pending_sim_timer = self.node.create_timer(
                0.03, self._flush_pending_sim_images)
        self.node.get_logger().info(
            'uv_sensor started (sim mode: ROS stitched topics)')

    def _start_v4l2(self):
        front_path = self.node.get_parameter('front_cam_path').value
        down_path = self.node.get_parameter('down_cam_path').value
        if self._enable_front:
            self._front_cap = self._open_cap(front_path, FRONT_CAPTURE_RESOLUTION)
        if self._enable_down:
            self._down_cap = self._open_cap(down_path, DOWN_CAPTURE_RESOLUTION)
        for cap, camera, path in ((self._front_cap, 'front', front_path),
                                  (self._down_cap, 'down', down_path)):
            if cap is not None and cap.isOpened():
                thread = threading.Thread(target=self._capture_loop,
                                          args=(cap, camera),
                                          name=f'capture-{camera}', daemon=True)
                self._capture_threads.append(thread)
                thread.start()
            elif cap is not None:
                self.node.get_logger().error(
                    f'Cannot open {camera} camera: {path}')
        self.node.get_logger().info(
            f'uv_sensor started (real mode: front={front_path}, down={down_path})')

    @staticmethod
    def _open_cap(path, res):
        cap = cv2.VideoCapture(path)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, res[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, res[1])
        return cap

    # ── sim callbacks (ROS Image -> BGR -> preview + gate) ──────────────
    def _front_img_cb(self, msg):
        self._submit_image(msg, 'front')

    def _down_img_cb(self, msg):
        self._submit_image(msg, 'down')

    @staticmethod
    def _stamp_key(stamp):
        try:
            return int(stamp.sec), int(stamp.nanosec)
        except (AttributeError, TypeError, ValueError):
            return None

    @staticmethod
    def _info_stamps(info):
        try:
            left_stamp = info.left_stamp
            right_stamp = info.right_stamp
            pair_id = int(info.stereo_pair_id)
        except (AttributeError, TypeError, ValueError):
            return None
        left_key = Sensor._stamp_key(left_stamp)
        if left_key is None:
            left_key = Sensor._stamp_key(info.header.stamp)
        if left_key is None:
            return None
        return left_key, right_stamp, pair_id

    def _stereo_info_cb(self, message):
        camera = str(getattr(message, 'camera_name', '')).strip().lower()
        if camera not in self._stereo_info:
            return
        values = self._info_stamps(message)
        if values is None:
            return
        key, right_stamp, pair_id = values
        with self._pending_sim_lock:
            cache = self._stereo_info[camera]
            cache[key] = (right_stamp, pair_id, time.monotonic())
            while len(cache) > 8:
                cache.pop(next(iter(cache)))
            pending = self._pending_sim_images.pop((camera, key), None)
        if pending is not None:
            self.node.submit_image(
                camera, pending, right_stamp=right_stamp,
                stereo_pair_id=pair_id)

    def _front_stereo_info_cb(self, message):
        self._stereo_info_cb(message)

    def _down_stereo_info_cb(self, message):
        self._stereo_info_cb(message)

    def _flush_pending_sim_images(self):
        now = time.monotonic()
        expired = []
        with self._pending_sim_lock:
            for key, (message, arrival) in self._pending_sim_images.items():
                if now - arrival >= 0.025:
                    expired.append((key, message))
            for key, _ in expired:
                self._pending_sim_images.pop(key, None)
        for (camera, _), message in expired:
            self.node.submit_image(camera, message)

    def _submit_image(self, msg, camera):
        if StereoFrameInfo is None:
            self.node.submit_image(camera, msg)
            return
        key = self._stamp_key(msg.header.stamp)
        with self._pending_sim_lock:
            info = self._stereo_info[camera].pop(key, None) if key else None
            if info is None and key is not None:
                self._pending_sim_images[(camera, key)] = (
                    msg, time.monotonic())
                return
        if info is None:
            self.node.submit_image(camera, msg)
        else:
            right_stamp, pair_id, _ = info
            self.node.submit_image(
                camera, msg, right_stamp=right_stamp,
                stereo_pair_id=pair_id)

    # ── v4l2 capture loop ───────────────────────────────────────────────
    def _capture_loop(self, cap, camera):
        read_failures = 0
        while not self._capture_stop.is_set():
            ret, frame = cap.read()
            if not ret:
                read_failures = min(read_failures + 1, 6)
                import time
                time.sleep(min(0.5, 0.01 * (2 ** read_failures)))
                continue
            read_failures = 0
            normalized = normalize_frame(frame)
            if normalized is None:
                self.node.get_logger().warn(f'Invalid frame from {camera} camera')
                continue
            # raw preview at capture rate, then hand to ai via gate
            stamp = self.node.get_clock().now().to_msg()
            self.node.update_raw_preview(camera, normalized, stamp)
            self.node.submit_frame(camera, normalized, stamp)

    # ── shutdown ────────────────────────────────────────────────────────
    def shutdown(self):
        self._capture_stop.set()
        if self._pending_sim_timer is not None:
            self._pending_sim_timer.cancel()
            self._pending_sim_timer = None
        for cap in (self._front_cap, self._down_cap):
            if cap is not None:
                cap.release()
