"""Camera frame source for Stonefish simulation or real V4L2 cameras.

The camera driver selects a source through parameters, then hands each
stitched BGR frame to its acquisition callback.
"""

import threading
import time

import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from .sim_shm import SimStereoShmSource
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header


def normalize_frame(frame):
    """Return a non-empty frame as BGR, converting grayscale or BGRA input."""
    if not isinstance(frame, np.ndarray) or frame.size == 0:
        return None
    if frame.ndim == 2:
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    if frame.ndim != 3 or frame.shape[2] not in (3, 4):
        return None
    if frame.shape[2] == 4:
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    return frame


class Sensor:
    """Frame producer with a callback-compatible output boundary."""

    def __init__(self, node, sim_mode=False, enable_front=True,
                 enable_down=True, startup_timeout_s=5.0,
                 reconnect_interval_s=1.0, camera_configs=None,
                 frame_callback=None):
        if not callable(frame_callback):
            raise ValueError("frame_callback must be callable")
        self.node = node                 # uv_camera driver rclpy Node
        self._frame_callback = frame_callback
        self._sim_mode = sim_mode
        self._enable_front = enable_front
        self._enable_down = enable_down
        self._startup_timeout_s = max(1.0, float(startup_timeout_s))
        self._reconnect_interval_s = max(0.1, float(reconnect_interval_s))
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
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._published_info_cameras = set()
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
            'uv_camera started (sim mode: shared-memory camera rings; '
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
            self._submit_frame(
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
        # Each camera owns an independent worker. A missing or disconnected
        # camera must not prevent the other camera from publishing frames.
        # Workers keep retrying the device so a cable that is reseated while
        # the vehicle is running can recover without restarting uv_camera.
        for camera, path, resolution in specs:
            if not path:
                self._report_camera_failure(
                    camera, 'camera device path is empty; retry disabled')
                continue
            thread = threading.Thread(
                target=self._capture_worker,
                args=(camera, path, resolution),
                name=f'capture-{camera}', daemon=True)
            self._capture_threads.append(thread)
            thread.start()

        summary = ', '.join(f'{camera}={path}' for camera, path, _ in specs)
        self.node.get_logger().info(
            f'uv_camera capture workers started: {summary or "none"}; '
            f'reconnect_interval={self._reconnect_interval_s:.1f}s')
        self.node.get_logger().info(
            'uv_camera started (real mode: '
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
        if camera in self._published_info_cameras:
            return
        config = self._camera_configs[camera]
        frame_prefix = 'downward' if camera == 'down' else camera
        for side, info_publisher in self._real_info_publishers.get(
                camera, {}).items():
            header = Header()
            header.stamp = stamp
            header.frame_id = f'{frame_prefix}_{side}_camera_optical_frame'
            info = CameraInfo()
            info.header = header
            info.width = config.width
            info.height = config.height
            side_config = config.side(side)
            info.k = side_config.matrix.reshape(-1).tolist()
            info.d = side_config.distortion.tolist()
            info_publisher.publish(info)
        self._published_info_cameras.add(camera)

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
            if self._capture_stop.is_set():
                raise RuntimeError('camera capture stopped during probe')
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
            self._capture_stop.wait(0.05)

    # ── v4l2 capture and reconnect loop ─────────────────────────────────
    def _capture_worker(self, camera, path, resolution):
        """Capture one camera and reconnect it without affecting its peer."""
        cap = None
        initial_frame = None
        read_failures = 0
        failure_started = None
        last_failure_log = 0.0
        failure_reported = False
        try:
            while not self._capture_stop.is_set():
                if cap is None:
                    try:
                        cap = self._open_cap(path, resolution)
                        if cap is None or not cap.isOpened():
                            raise RuntimeError(
                                f'camera cannot be opened (path={path})')
                        initial_frame = self._probe_first_frame(
                            cap, camera, path)
                        if camera == 'front':
                            self._front_cap = cap
                        else:
                            self._down_cap = cap
                        self.node.get_logger().info(
                            f'{camera} camera connected: path={path}, '
                            f'shape={initial_frame.shape}')
                    except Exception as error:
                        if cap is not None:
                            try:
                                cap.release()
                            except Exception:
                                pass
                        cap = None
                        initial_frame = None
                        if camera == 'front':
                            self._front_cap = None
                        else:
                            self._down_cap = None
                        now = time.monotonic()
                        if failure_started is None:
                            failure_started = now
                        if (not failure_reported
                                or now - last_failure_log >= 5.0):
                            self._report_camera_failure(
                                camera,
                                f'camera open/probe failed; retrying every '
                                f'{self._reconnect_interval_s:.1f}s: {error} '
                                f'(path={path})')
                            failure_reported = True
                            last_failure_log = now
                        self._capture_stop.wait(self._reconnect_interval_s)
                        continue

                    if failure_started is not None:
                        self._report_camera_recovered(
                            camera, time.monotonic() - failure_started)
                        failure_started = None
                        failure_reported = False
                        last_failure_log = 0.0

                if initial_frame is not None:
                    ret, current = True, initial_frame
                    initial_frame = None
                else:
                    try:
                        ret, current = cap.read()
                    except Exception as error:
                        self._report_camera_failure(
                            camera, f'camera read raised {error} (path={path})')
                        try:
                            cap.release()
                        except Exception:
                            pass
                        cap = None
                        if camera == 'front':
                            self._front_cap = None
                        else:
                            self._down_cap = None
                        failure_started = time.monotonic()
                        failure_reported = True
                        last_failure_log = failure_started
                        self._capture_stop.wait(self._reconnect_interval_s)
                        continue

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
                    if (now - failure_started >= self._startup_timeout_s):
                        try:
                            cap.release()
                        except Exception:
                            pass
                        cap = None
                        if camera == 'front':
                            self._front_cap = None
                        else:
                            self._down_cap = None
                        read_failures = 0
                        self._capture_stop.wait(self._reconnect_interval_s)
                    else:
                        self._capture_stop.wait(
                            min(0.5, 0.01 * (2 ** read_failures)))
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
                    if (now - failure_started >= self._startup_timeout_s):
                        try:
                            cap.release()
                        except Exception:
                            pass
                        cap = None
                        if camera == 'front':
                            self._front_cap = None
                        else:
                            self._down_cap = None
                        read_failures = 0
                        self._capture_stop.wait(self._reconnect_interval_s)
                    else:
                        self._capture_stop.wait(0.05)
                    continue

                if failure_started is not None:
                    self._report_camera_recovered(
                        camera, time.monotonic() - failure_started)
                    failure_started = None
                    failure_reported = False
                    last_failure_log = 0.0
                read_failures = 0
                # hand the latest normalized capture directly to the camera driver
                stamp = self.node.get_clock().now().to_msg()
                self._publish_real_sensor_frame(camera, normalized, stamp)
                self._submit_frame(camera, normalized, stamp)
        except Exception as error:
            self._report_camera_failure(
                camera, f'capture loop stopped unexpectedly: {error} (path={path})')
        finally:
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass
            if camera == 'front':
                self._front_cap = None
            else:
                self._down_cap = None

    def _report_camera_failure(self, camera, reason):
        report = getattr(self.node, 'report_camera_failure', None)
        if report is not None:
            report(camera, reason)
        else:
            self.node.get_logger().error(f'{camera} camera failure: {reason}')

    def _report_camera_recovered(self, camera, elapsed_s):
        report = getattr(self.node, 'report_camera_recovered', None)
        if report is not None:
            report(camera, elapsed_s)
        else:
            self.node.get_logger().info(
                f'{camera} camera recovered after {elapsed_s:.1f}s')

    def _submit_frame(self, camera, frame, stamp, right_stamp=None,
                      stereo_pair_id=0):
        if camera in ('front', 'down') and not self._sim_mode:
            # Rotate the full stereo frame so each eye is corrected and the
            # reversed input eye order is restored to left-then-right.
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        self._frame_callback(
            camera, frame, stamp, right_stamp=right_stamp,
            stereo_pair_id=stereo_pair_id)

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
