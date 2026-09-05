"""uv_sensor: frame source (Stonefish sim OR real V4L2 camera) + go2rtc preview.

In the SAME process as uv_ai. uv_sensor:
  * picks the source via params (sim_mode: ROS stitched topics  OR  V4L2 /dev/video*);
  * on each frame, updates the raw MJPEG preview cache (fed to go2rtc :1984)
    and hands the BGR frame to uv_ai through an in-memory FrameGate (A3: no ROS
    image topic, no JPEG between sensor and ai).
"""

import threading

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

    # ── setup: pick source ──────────────────────────────────────────────
    def start(self):
        if self._sim_mode:
            self._start_sim()
        else:
            self._start_v4l2()

    def _start_sim(self):
        if self._enable_front:
            self.node.create_subscription(
                Image, '/auv/front_cam/stitched', self._front_img_cb,
                self._image_qos)
        if self._enable_down:
            self.node.create_subscription(
                Image, '/auv/down_cam/stitched', self._down_img_cb,
                self._image_qos)
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

    def _submit_image(self, msg, camera):
        self.node.submit_image(camera, msg)   # composed node decodes ROS->BGR + preview + gate

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
        for cap in (self._front_cap, self._down_cap):
            if cap is not None:
                cap.release()
