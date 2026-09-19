"""uv_sensor: frame source (Stonefish sim OR real V4L2 camera) + go2rtc preview.

In the SAME process as uv_ai. uv_sensor:
  * picks the source via params (sim_mode: POSIX shared-memory rings  OR  V4L2 /dev/video*);
  * on each frame, updates the raw MJPEG preview cache (fed to go2rtc :1984)
    and hands the BGR frame to uv_ai through an in-memory FrameGate (A3: no ROS
    image topic, no JPEG between sensor and ai).
"""

import threading
import time

import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from .common import normalize_frame
from .sim_shm import SimStereoShmSource
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header

class Sensor:
    """Frame producer: chooses source, updates raw preview, feeds FrameGate."""

    def __init__(self, node, gate, sim_mode, enable_front, enable_down,
                 startup_timeout_s=5.0, camera_configs=None):
        self.node = node                 # composed uv_camera rclpy Node
        self.gate = gate                 # common.FrameGate -> ai consumer
        self._sim_mode = sim_mode
        self._enable_front = enable_front
        self._enable_down = enable_down
        self._startup_timeout_s = max(1.0, float(startup_timeout_s))
        self._camera_configs = dict(camera_configs or {})
        missing = [camera for camera in ('front', 'down')
                   if camera not in self._camera_configs]
        if missing:
            raise ValueError(f'missing camera configs: {missing}')

        self._capture_stop = threading.Event()
        self._capture_threads = []
        self._front_cap = None
        self._down_cap = None
        self._image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._real_info_publishers = {}
        self._sim_shm_source = None
        self._sim_shm_timer = None

    # ── setup: pick source ──────────────────────────────────────────────
    def start(self):
        if self._sim_mode:
            self._start_sim()
        else:
            self._start_v4l2()

    def _start_sim(self):
        # Stonefish and uv_camera are separate processes. Simulator pixels
        # therefore arrive through POSIX shared-memory rings, while the tiny
        # CameraInfo messages continue to arrive through the metadata bridge.
        self._sim_shm_source = SimStereoShmSource(
            enable_front=self._enable_front,
            enable_down=self._enable_down,
        )
        self._sim_shm_timer = self.node.create_timer(
            0.005, self._poll_sim_shm)
        self.node.get_logger().info(
            'uv_sensor started (sim mode: shared-memory camera rings; '
            'no DDS Image topics)')

    def _poll_sim_shm(self):
        if self._sim_shm_source is None:
            return
        for camera in ('front', 'down'):
            if ((camera == 'front' and not self._enable_front)
                    or (camera == 'down' and not self._enable_down)):
                continue
            try:
                result = self._sim_shm_source.poll(camera)
            except Exception as error:
                self.node.get_logger().error(
                    f'{camera} shared-memory camera read failed: {error}')
                continue
            if result is None:
                continue
            frame, left_stamp, right_stamp, pair_id = result
            stamp = self.node.get_clock().now().to_msg()
            if left_stamp[0] > 0 or left_stamp[1] > 0:
                stamp.sec = int(left_stamp[0])
                stamp.nanosec = int(left_stamp[1])
            right = type(stamp)()
            right.sec = int(right_stamp[0])
            right.nanosec = int(right_stamp[1])
            self.node.submit_frame(
                camera, frame, stamp, right_stamp=right,
                stereo_pair_id=pair_id)

    def _start_v4l2(self):
        self._create_real_sensor_publishers()
        specs = []
        if self._enable_front:
            config = self._camera_configs['front']
            specs.append(('front', config.device, config.capture_resolution))
        if self._enable_down:
            config = self._camera_configs['down']
            specs.append(('down', config.device, config.capture_resolution))
        if any(path is None for _, path, _ in specs):
            raise RuntimeError('real camera profile must define device paths')

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
            'uv_sensor started (real mode: '
            f"front={self._camera_configs['front'].device}, "
            f"down={self._camera_configs['down'].device})")

    def _create_real_sensor_publishers(self):
        # Real camera pixels stay in this process. Only calibration metadata is
        # published, so the ROS graph never carries sensor_msgs/Image payloads.
        self._real_info_publishers = {}
        for camera, enabled in (('front', self._enable_front),
                                ('down', self._enable_down)):
            if not enabled:
                continue
            config = self._camera_configs[camera]
            self._real_info_publishers[camera] = {
                side: self.node.create_publisher(
                    CameraInfo, config.camera_info_topics[side],
                    self._image_qos)
                for side in ('left', 'right')
                if config.camera_info_topics[side]
            }

    def _publish_real_sensor_frame(self, camera, frame, stamp):
        """Publish only small calibration metadata for a local V4L2 frame."""
        del frame
        config = self._camera_configs[camera]
        for side, info_publisher in self._real_info_publishers.get(
                camera, {}).items():
            header = Header()
            header.stamp = stamp
            header.frame_id = f'{camera}_{side}_camera_optical_frame'
            info = CameraInfo()
            info.header = header
            info.width = config.width
            info.height = config.height
            side_config = config.side(side)
            info.k = side_config.matrix.reshape(-1).tolist()
            info.d = side_config.distortion.tolist()
            info_publisher.publish(info)

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

    # ── v4l2 capture loop ───────────────────────────────────────────────
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
                self._publish_real_sensor_frame(camera, normalized, stamp)
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
        if self._sim_shm_timer is not None:
            self._sim_shm_timer.cancel()
            self._sim_shm_timer = None
        if self._sim_shm_source is not None:
            self._sim_shm_source.close()
            self._sim_shm_source = None
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
