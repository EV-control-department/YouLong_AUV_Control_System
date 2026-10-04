"""uv_sensor: Stonefish raw stereo or real V4L2, then in-process frame handoff.

In the SAME process as uv_ai. uv_sensor:
  * picks the source via params (sim_mode: direct Stonefish views OR V4L2);
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
from .common import image_msg_to_bgr
import numpy as np


class Sensor:
    """Frame producer: chooses source, updates raw preview, feeds FrameGate."""

    def __init__(self, node, gate, sim_mode, enable_front, enable_down,
                 startup_timeout_s=5.0):
        self.node = node                 # composed uv_camera rclpy Node
        self.gate = gate                 # common.FrameGate -> ai consumer
        self._sim_mode = sim_mode
        self._enable_front = enable_front
        self._enable_down = enable_down
        self._startup_timeout_s = max(1.0, float(startup_timeout_s))

        self._capture_stop = threading.Event()
        self._capture_threads = []
        self._front_cap = None
        self._down_cap = None
        self._image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._sim_views = {camera: {'left': None, 'right': None}
                           for camera in ('front', 'down')}
        self._last_sim_pair = {'front': None, 'down': None}
        self._sim_pair_sequence = {'front': 0, 'down': 0}
        self._sim_pair_log_at = {'front': time.monotonic(),
                                 'down': time.monotonic()}
        self._sim_pair_count = {'front': 0, 'down': 0}

    # ── setup: pick source ──────────────────────────────────────────────
    def start(self):
        if self._sim_mode:
            self._start_sim()
        else:
            self._start_v4l2()

    def _start_sim(self):
        # Stonefish still emits ROS Image messages. Subscribe to its original
        # pair directly in uv_camera; avoid a second large DDS hop via sim_bridge.
        for camera, enabled in (('front', self._enable_front),
                                ('down', self._enable_down)):
            if enabled:
                for side in ('left', 'right'):
                    self.node.create_subscription(
                        Image, f'/sim/{camera}_cam/{side}/image_color',
                        lambda msg, camera=camera, side=side:
                            self._sim_view_cb(camera, side, msg), self._image_qos)
        self.node.get_logger().info(
            'uv_sensor started (sim mode: direct Stonefish stereo input)')

    def _sim_view_cb(self, camera, side, message):
        views = self._sim_views[camera]
        views[side] = message
        left, right = views['left'], views['right']
        if left is None or right is None:
            return
        left_stamp = left.header.stamp.sec + left.header.stamp.nanosec * 1e-9
        right_stamp = right.header.stamp.sec + right.header.stamp.nanosec * 1e-9
        if abs(left_stamp - right_stamp) > 0.12:
            return
        pair = (left.header.stamp.sec, left.header.stamp.nanosec,
                right.header.stamp.sec, right.header.stamp.nanosec)
        if pair == self._last_sim_pair[camera]:
            return
        self._last_sim_pair[camera] = pair
        # Never reuse either exposure in a second pair. Reusing the latest
        # opposite eye at low FPS silently creates adjacent-frame stereo.
        views['left'] = None
        views['right'] = None
        self._sim_pair_sequence[camera] += 1
        self._sim_pair_count[camera] += 1
        now = time.monotonic()
        elapsed = now - self._sim_pair_log_at[camera]
        if elapsed >= 5.0:
            self.node.get_logger().info(
                f'仿真双目输入[{camera}] {self._sim_pair_count[camera]/elapsed:.2f}Hz，'
                f'左右采集时间差={abs(left_stamp-right_stamp):.3f}s')
            self._sim_pair_log_at[camera] = now
            self._sim_pair_count[camera] = 0
        try:
            frame = np.hstack((image_msg_to_bgr(left), image_msg_to_bgr(right)))
            self.node.update_raw_preview(camera, frame, left.header.stamp)
            self.node.submit_frame(
                camera, frame, left.header.stamp,
                right_stamp=right.header.stamp,
                stereo_pair_id=self._sim_pair_sequence[camera])
        except (ValueError, cv2.error) as error:
            self.node.get_logger().warning(f'仿真双目帧解码失败[{camera}]：{error}')


    def _start_v4l2(self):
        front_path = str(self.node.get_parameter('front_cam_path').value)
        down_path = str(self.node.get_parameter('down_cam_path').value)
        specs = []
        if self._enable_front:
            specs.append(('front', front_path, FRONT_CAPTURE_RESOLUTION))
        if self._enable_down:
            specs.append(('down', down_path, DOWN_CAPTURE_RESOLUTION))

        # Open and probe every enabled camera before starting either capture
        # thread. This prevents a partial recording containing only one side.
        opened = []
        failures = []
        for camera, path, resolution in specs:
            try:
                cap = self._open_cap(path, resolution)
                if cap is None or not cap.isOpened():
                    failures.append(f'{camera} camera cannot be opened: {path}')
                    if cap is not None:
                        cap.release()
                    continue
                opened.append((camera, path, cap))
            except Exception as error:
                failures.append(
                    f'{camera} camera open failed ({path}): {error}')

        probed = []
        for camera, path, cap in opened:
            try:
                initial_frame = self._probe_first_frame(cap, camera, path)
                if initial_frame.shape[:2] != (480, 1280):
                    raise RuntimeError(
                        f'{camera} 相机实际输出 {initial_frame.shape[1]}x{initial_frame.shape[0]}，'
                        '但双目拼接尺寸统一为 1280x480；'
                        '请调整 V4L2 模式或重新标定，不能复用当前内参')
                probed.append((camera, path, cap, initial_frame))
            except Exception as error:
                failures.append(str(error))

        if failures:
            for _, _, cap in opened:
                cap.release()
            self._front_cap = None
            self._down_cap = None
            message = 'camera preflight failed; recording not started: ' + '; '.join(failures)
            self.node.get_logger().error(message)
            raise RuntimeError(message)

        for camera, path, cap, initial_frame in probed:
            if camera == 'front':
                self._front_cap = cap
            else:
                self._down_cap = cap
            thread = threading.Thread(
                target=self._capture_loop,
                args=(cap, camera, initial_frame, path),
                name=f'capture-{camera}', daemon=True)
            self._capture_threads.append(thread)
            thread.start()

        summary = ', '.join(
            f'{camera}={path} shape={frame.shape}'
            for camera, path, _, frame in probed)
        self.node.get_logger().info(f'uv_sensor preflight passed: {summary}')
        self.node.get_logger().info(
            f'uv_sensor started (real mode: front={front_path}, down={down_path})')

    @staticmethod
    def _open_cap(path, res):
        cap = cv2.VideoCapture(path)
        # These properties are honored by V4L2/GStreamer builds that expose
        # them. Unsupported backends simply ignore the setting.
        for property_name in ('CAP_PROP_OPEN_TIMEOUT_MSEC',
                              'CAP_PROP_READ_TIMEOUT_MSEC'):
            property_id = getattr(cv2, property_name, None)
            if property_id is not None:
                try:
                    cap.set(property_id, 3000)
                except cv2.error:
                    pass
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, res[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, res[1])
        return cap

    def _probe_first_frame(self, cap, camera, path):
        deadline = time.monotonic() + self._startup_timeout_s
        attempts = 0
        while True:
            attempts += 1
            ret, frame = cap.read()
            if ret:
                normalized = normalize_frame(frame)
                if normalized is not None:
                    return normalized
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    f'{camera} camera opened but produced no valid frame '
                    f'within {self._startup_timeout_s:.1f}s '
                    f'(path={path}, attempts={attempts})')
            time.sleep(0.05)

    # ── sim callbacks (ROS Image -> BGR -> preview + gate) ──────────────
    def _capture_loop(self, cap, camera, initial_frame=None, path=''):
        read_failures = 0
        failure_started = None
        last_failure_log = 0.0
        failure_reported = False
        frame = initial_frame
        try:
            while not self._capture_stop.is_set():
                if frame is not None:
                    ret, current = True, frame
                    frame = None
                else:
                    try:
                        ret, current = cap.read()
                    except Exception as error:
                        self._report_camera_failure(
                            camera, f'camera read raised {error} (path={path})')
                        return

                if not ret:
                    now = time.monotonic()
                    if failure_started is None:
                        failure_started = now
                    read_failures = min(read_failures + 1, 6)
                    if now - last_failure_log >= 5.0:
                        self.node.get_logger().error(
                            f'{camera} camera read failed; no valid frame for '
                            f'{now - failure_started:.1f}s '
                            f'(consecutive_failures={read_failures}, path={path})')
                        last_failure_log = now
                    if (not failure_reported
                            and now - failure_started >= self._startup_timeout_s):
                        self._report_camera_failure(
                            camera,
                            f'no valid frame for {now - failure_started:.1f}s '
                            f'(path={path})')
                        failure_reported = True
                    time.sleep(min(0.5, 0.01 * (2 ** read_failures)))
                    continue

                normalized = normalize_frame(current)
                if normalized is None:
                    now = time.monotonic()
                    if failure_started is None:
                        failure_started = now
                    if now - last_failure_log >= 5.0:
                        self.node.get_logger().error(
                            f'Invalid frame from {camera} camera '
                            f'(path={path})')
                        last_failure_log = now
                    if (not failure_reported
                            and now - failure_started >= self._startup_timeout_s):
                        self._report_camera_failure(
                            camera,
                            f'camera returned invalid frames for '
                            f'{now - failure_started:.1f}s (path={path})')
                        failure_reported = True
                    time.sleep(0.05)
                    continue

                if normalized.shape[:2] != (480, 1280):
                    self._report_camera_failure(
                        camera, f'图像尺寸在采集中改变为 '
                        f'{normalized.shape[1]}x{normalized.shape[0]}，'
                        '预期双目拼接 1280x480；停止使用错误内参 '
                        f'(path={path})')
                    return

                if failure_started is not None:
                    self.node.get_logger().info(
                        f'{camera} camera recovered after '
                        f'{time.monotonic() - failure_started:.1f}s')
                    failure_started = None
                    failure_reported = False
                    last_failure_log = 0.0
                read_failures = 0
                # raw preview at capture rate, then hand to ai via gate
                stamp = self.node.get_clock().now().to_msg()
                self.node.update_raw_preview(camera, normalized, stamp)
                self.node.submit_frame(camera, normalized, stamp)
        except Exception as error:
            self._report_camera_failure(
                camera, f'capture loop stopped unexpectedly: {error} (path={path})')

    def _report_camera_failure(self, camera, reason):
        report = getattr(self.node, 'report_camera_failure', None)
        if report is not None:
            report(camera, reason)
        else:
            self.node.get_logger().error(f'{camera} camera failure: {reason}')

    # ── shutdown ────────────────────────────────────────────────────────
    def shutdown(self):
        self._capture_stop.set()
        for cap in (self._front_cap, self._down_cap):
            if cap is not None:
                cap.release()
        current_thread = threading.current_thread()
        for thread in self._capture_threads:
            if thread is not current_thread:
                thread.join(timeout=2.0)
        self._capture_threads.clear()
        self._front_cap = None
        self._down_cap = None
