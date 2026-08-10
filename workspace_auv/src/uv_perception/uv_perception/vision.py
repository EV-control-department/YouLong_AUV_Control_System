"""Vision node: camera capture, YOLO detection and go2rtc video streaming.

Two input modes are supported:
  sim_mode=true (default): subscribe to /auv/*/stitched ROS topics.
  sim_mode=false: capture stitched frames directly from V4L2 with OpenCV.

The real-hardware path deliberately does not create or publish ROS Image
messages. Frames stay in-process for detection and are exposed to go2rtc via
the local MJPEG server. Only detection, line-state and ArUco metadata use ROS.

Parameters:
    model_path (str): Path to YOLO segmentation model .pt file.
    sim_mode (bool): true=ROS images, false=V4L2 capture (default: false).
    front_cam_path (str): V4L2 device path for front camera (real mode only).
    down_cam_path (str): V4L2 device path for down camera (real mode only).
"""

import math
import os
import shutil
import subprocess
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header, Int32MultiArray

from uv_msgs.msg import Detection, DetectionArray, LineState

CvBridge = None


# ── Runtime switches ───────────────────────────────────────────────
# 真实硬件模式下的 V4L2 设备绑定。
FRONT_CAMERA_DEVICE = '/dev/video2'
DOWN_CAMERA_DEVICE = '/dev/video0'
ENABLE_FRONT_CAMERA = True
ENABLE_DOWN_CAMERA = True

# 每个 stitched 相机拆分后的单目图像分辨率。
# stitched 图像应为：(width * 2, height)，例如前/下相机均为 1280x480。
FRONT_CAMERA_RESOLUTION = (640, 480)  # width, height
DOWN_CAMERA_RESOLUTION = (640, 480)   # width, height

# 真实 V4L2 设备的实际采集分辨率；由 v4l2-ctl 检测得到。
FRONT_CAPTURE_RESOLUTION = (1280, 480)
DOWN_CAPTURE_RESOLUTION = (1280, 480)

# 相机标定常量：K 为 3x3 内参矩阵，D 的顺序为
# (k1, k2, p1, p2, k3)。请用实际标定结果替换 D。
# 480p (单目 640×480)：原 1280×960 标定按 0.5 缩放。
FRONT_CAMERA_MATRIX = (
    1079.2, 0.0, 320.0,
    0.0, 1079.2, 240.0,
    0.0, 0.0, 1.0,
)
FRONT_DIST_COEFFS = (0.0, 0.0, 0.0, 0.0, 0.0)

DOWN_CAMERA_MATRIX = (
    1153.8, 0.0, 320.0,
    0.0, 1153.8, 240.0,
    0.0, 0.0, 1.0,
)
DOWN_DIST_COEFFS = (0.0, 0.0, 0.0, 0.0, 0.0)

CONFIDENCE = 0.5
PIPE_CLASS_ID = 3                # 实机模型: pipe=3
PIPE_CONFIDENCE = 0.7            # pipe 单独的高置信度阈值 (巡线目标需可靠)

ENABLE_UNDISTORT = True              # 是否执行相机去畸变
ENABLE_GORTC = True                 # 启动 go2rtc 转发客户端视频
GORTC_HTTP_PORT = 1984               # go2rtc 对外 HTTP/WebRTC API 端口
VISION_MJPEG_PORT = 8090             # vision 提供给 go2rtc 的本地 MJPEG 端口
GORTC_EXECUTABLE = 'go2rtc'
STREAM_ANNOTATED = True              # 通过 go2rtc 额外提供带检测框的视频

# ROS 参数默认值。
SIM_MODE = False                      # False=V4L2，True=ROS stitched 话题
SAVE_DATASET = False                  # 是否保存训练数据帧
DATASET_DIR = ''                      # 空字符串=自动使用工程下的 img/
DEFAULT_MODEL_FILENAME = 'WUURC2026REAL11nano--001.pt'


class _ScalarKalman:
    """不依赖第三方库的一维 Kalman 滤波器 (用于平滑管道中心和角度)。"""
    def __init__(self, initial_value, process_noise=1.0, measurement_noise=3.0):
        self.x = float(initial_value)
        self.p = 100.0
        self.q = max(1e-6, float(process_noise))
        self.r = max(1e-6, float(measurement_noise))

    def predict(self):
        self.p += self.q
        return self.x

    def update(self, measurement):
        gain = self.p / (self.p + self.r)
        self.x += gain * (float(measurement) - self.x)
        self.p = (1.0 - gain) * self.p
        return self.x


class _LineFilterState:
    """保存单个相机的图像宽度和中心/方向滤波器。"""
    def __init__(self, image_width, process_noise, measurement_noise):
        self.image_width = int(image_width)
        self.kalman_center = _ScalarKalman(
            self.image_width / 2.0,
            process_noise,
            measurement_noise,
        )
        self.kalman_heading = _ScalarKalman(
            0.0,
            max(1e-6, process_noise * 0.5),
            max(1e-6, measurement_noise * 0.5),
        )


class _MjpegHandler(BaseHTTPRequestHandler):
    """Very small MJPEG source used by go2rtc (and directly by a browser)."""
    node = None

    def do_GET(self):  # noqa: N802 - required by BaseHTTPRequestHandler
        stream = self.path.strip('/').split('?', 1)[0]
        valid_streams = {
            'front': ('front', False),
            'down': ('down', False),
            'front_annotated': ('front', True),
            'down_annotated': ('down', True),
        }
        if stream not in valid_streams:
            self.send_error(
                404, 'use /front, /down, /front_annotated or /down_annotated')
            return
        camera, annotated = valid_streams[stream]
        if annotated and (self.node is None or not self.node._stream_annotated):
            self.send_error(404, 'annotated streams are disabled')
            return

        self.send_response(200)
        self.send_header('Cache-Control', 'no-cache, private')
        self.send_header('Connection', 'close')
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        try:
            last_sequence = 0
            while rclpy.ok() and self.node is not None:
                result = self.node._wait_for_stream_frame(
                    camera, annotated, last_sequence)
                if result is None:
                    break
                payload, last_sequence = result
                self.wfile.write(
                    b'--frame\r\nContent-Type: image/jpeg\r\n'
                    + f'Content-Length: {len(payload)}\r\n\r\n'.encode('ascii'))
                self.wfile.write(payload)
                self.wfile.write(b'\r\n')
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass

    def log_message(self, *_args):
        pass


class _MjpegServer(ThreadingHTTPServer):
    """MJPEG server whose client threads cannot keep node shutdown alive."""

    daemon_threads = True
    allow_reuse_address = True


def _import_cv_bridge():
    global CvBridge
    if CvBridge is None:
        try:
            from cv_bridge import CvBridge as _CB
            CvBridge = _CB
        except Exception as e:
            raise ImportError(f'cv_bridge not available: {e}')


class VisionNode(Node):
    """Vision node: YOLO segmentation detection on stitched camera images, split into L/R."""

    def __init__(self):
        super().__init__('vision')

        self.declare_parameter('sim_mode', SIM_MODE)
        self._sim_mode = self.get_parameter('sim_mode').get_parameter_value().bool_value

        self.declare_parameter('enable_gortc', ENABLE_GORTC)
        self._enable_gortc = self.get_parameter(
            'enable_gortc').get_parameter_value().bool_value
        self.declare_parameter('gortc_executable', GORTC_EXECUTABLE)
        self.declare_parameter('gortc_http_port', GORTC_HTTP_PORT)
        self.declare_parameter('mjpeg_port', VISION_MJPEG_PORT)
        self.declare_parameter('stream_annotated', STREAM_ANNOTATED)
        self._stream_annotated = self.get_parameter(
            'stream_annotated').get_parameter_value().bool_value
        self._mjpeg_port = self.get_parameter(
            'mjpeg_port').get_parameter_value().integer_value
        gortc_port = self.get_parameter(
            'gortc_http_port').get_parameter_value().integer_value
        if not 1 <= self._mjpeg_port <= 65535:
            raise ValueError(f'mjpeg_port out of range: {self._mjpeg_port}')
        if not 1 <= gortc_port <= 65535:
            raise ValueError(f'gortc_http_port out of range: {gortc_port}')

        self.declare_parameter('enable_front_camera', ENABLE_FRONT_CAMERA)
        self.declare_parameter('enable_down_camera', ENABLE_DOWN_CAMERA)
        self._enable_front_camera = self.get_parameter('enable_front_camera').get_parameter_value().bool_value
        self._enable_down_camera = self.get_parameter('enable_down_camera').get_parameter_value().bool_value

        # --- 巡线参数 (来自仿真节点) ---
        self.declare_parameter('line_contour_min_area', 200)
        self._line_contour_min_area = int(self.get_parameter('line_contour_min_area').value)
        self.declare_parameter('line_filter_process_noise', 1.0)
        self._line_filter_process_noise = float(self.get_parameter('line_filter_process_noise').value)
        self.declare_parameter('line_filter_measurement_noise', 3.0)
        self._line_filter_measurement_noise = float(self.get_parameter('line_filter_measurement_noise').value)
        self._line_filters = {}

        self.declare_parameter('save_dataset', SAVE_DATASET)
        self._save_dataset = self.get_parameter('save_dataset').get_parameter_value().bool_value
        self._dataset_dir = DATASET_DIR
        self._dataset_last_s = {}  
        self._dataset_count_s = {} 
        if self._save_dataset:
            if not self._dataset_dir:
                self._dataset_dir = os.path.join(
                    os.path.dirname(os.path.dirname(os.path.dirname(__file__))), 'img')
            os.makedirs(self._dataset_dir, exist_ok=True)
            self.get_logger().info(f'Dataset saving enabled: {self._dataset_dir}')

        self.bridge = None
        self._model = None
        self._confidence = CONFIDENCE
        self._model_loaded = False
        self._cv_bridge_ok = False
        self._inference_lock = threading.Lock()
        self._front_cap = None
        self._down_cap = None
        self._gortc_process = None
        self._gortc_config = None
        self._active_channels = set()
        self._mjpeg_server = None

        # 线程锁及流媒体相关
        self._vision_pool = ThreadPoolExecutor(max_workers=2, thread_name_prefix='vision')
        self._processing_lock = threading.Lock()
        self._processing = {'front': False, 'down': False}
        # 每个相机只保留最新待处理帧，避免推理慢时积压旧帧。
        self._pending_work = {'front': None, 'down': None}
        self._stream_lock = threading.Lock()
        self._stream_jpegs = {'front': None, 'down': None}
        self._stream_annotated_jpegs = {'front': None, 'down': None}
        self._stream_sequence = {'front': 0, 'down': 0}
        self._stream_annotated_sequence = {'front': 0, 'down': 0}
        self._stream_condition = threading.Condition(self._stream_lock)
        self._stream_stop = threading.Event()
        self._stream_frame_count = {'front': 0, 'down': 0}
        self._capture_frame_logged = {'front': False, 'down': False}
        self._capture_stop = threading.Event()
        self._capture_threads = []

        if self._enable_gortc:
            if self._start_mjpeg_server():
                self._start_gortc()

        # Real hardware frames remain as OpenCV arrays. cv_bridge is only
        # needed when simulation supplies sensor_msgs/Image input.
        if self._sim_mode:
            try:
                _import_cv_bridge()
                self.bridge = CvBridge()
                self._cv_bridge_ok = True
            except Exception as e:
                self.get_logger().error(f'cv_bridge not available in sim mode: {e}')

        self._init_undistort()
        self._load_model()

        # Image source: ROS topics (sim) or V4L2 devices (real)
        if self._sim_mode:
            if self._enable_front_camera:
                self.create_subscription(Image, '/auv/front_cam/stitched', self._front_img_cb, 10)
            if self._enable_down_camera:
                self.create_subscription(Image, '/auv/down_cam/stitched', self._down_img_cb, 10)
            self.get_logger().info('Vision node started (sim mode: ROS topics)')
            active_cams = []
            if self._enable_front_camera:
                active_cams.append('front')
            if self._enable_down_camera:
                active_cams.append('down')
        else:
            self.declare_parameter('front_cam_path', FRONT_CAMERA_DEVICE)
            self.declare_parameter('down_cam_path', DOWN_CAMERA_DEVICE)
            front_path = self.get_parameter('front_cam_path').get_parameter_value().string_value
            down_path = self.get_parameter('down_cam_path').get_parameter_value().string_value

            if self._enable_front_camera:
                self._front_cap = cv2.VideoCapture(front_path)
                self._front_cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                self._front_cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                self._front_cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRONT_CAPTURE_RESOLUTION[0])
                self._front_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRONT_CAPTURE_RESOLUTION[1])
            if self._enable_down_camera:
                self._down_cap = cv2.VideoCapture(down_path)
                self._down_cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                self._down_cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                self._down_cap.set(cv2.CAP_PROP_FRAME_WIDTH, DOWN_CAPTURE_RESOLUTION[0])
                self._down_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, DOWN_CAPTURE_RESOLUTION[1])

            active_cams = []
            for cap, camera, path in ((self._front_cap, 'front', front_path),
                                      (self._down_cap, 'down', down_path)):
                if cap is not None and cap.isOpened():
                    active_cams.append(camera)
                    thread = threading.Thread(
                        target=self._capture_camera_loop,
                        args=(cap, camera),
                        name=f'capture-{camera}', daemon=True)
                    self._capture_threads.append(thread)
                    thread.start()
                elif cap is not None:
                    self.get_logger().error(f'Cannot open {camera} camera: {path}')
            self.get_logger().info(
                f'Vision node started (real mode: front={front_path}, down={down_path}, active={active_cams})')

        # Detection and state publishers — only for active cameras.
        # Image publishers are intentionally absent: video is served through
        # the local MJPEG endpoint / go2rtc.
        for cam in active_cams:
            self._active_channels.add(f'{cam}_left')
            self._active_channels.add(f'{cam}_right')
        self.pub_det = {}
        self.pub_line = {}
        for ch in sorted(self._active_channels):
            self.pub_det[ch] = self.create_publisher(DetectionArray, f'/perception/detection/{ch}', 10)
            self.pub_line[ch] = self.create_publisher(LineState, f'/perception/line/{ch}', 10)

        # ── ArUco detection (迁移自仿真节点，仅当前视相机激活时启用) ──────────────
        self._aruco_detector = None
        self._aruco_lock = threading.Lock()
        self._aruco_frames = None
        self._aruco_stop = threading.Event()
        self._aruco_thread = None
        self._aruco_pub = None
        if 'front' in active_cams:
            try:
                aruco = getattr(cv2, 'aruco')
                aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
                aruco_params = aruco.DetectorParameters()
                self._aruco_detector = aruco.ArucoDetector(aruco_dict, aruco_params)
            except (AttributeError, cv2.error) as e:
                self.get_logger().warn(f'OpenCV ArUco unavailable, marker detection disabled: {e}')
            self._aruco_pub = self.create_publisher(Int32MultiArray, '/perception/aruco/ids', 10)
            if self._aruco_detector is not None:
                self._aruco_thread = threading.Thread(target=self._aruco_loop, daemon=True)
                self._aruco_thread.start()
        else:
            self.get_logger().warn('ArUco detection disabled (no front camera)')

    def _aruco_loop(self):
        """Background thread: detect ArUco markers on front camera halves."""
        while rclpy.ok() and not self._aruco_stop.is_set():
            with self._aruco_lock:
                frames = self._aruco_frames
                self._aruco_frames = None
            if frames is not None:
                try:
                    all_ids = set()
                    for img in frames:
                        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                        _, ids, _ = self._aruco_detector.detectMarkers(gray)
                        if ids is not None:
                            all_ids.update(int(m) for m in ids.flatten() if 1 <= int(m) <= 6)
                    msg = Int32MultiArray()
                    msg.data = sorted(all_ids)
                    self._aruco_pub.publish(msg)
                except Exception as e:
                    self.get_logger().warn(f'ArUco detection error: {e}')
            self._aruco_stop.wait(0.05)  # ~20 Hz

    def _start_mjpeg_server(self):
        try:
            _MjpegHandler.node = self
            self._mjpeg_server = _MjpegServer(
                ('0.0.0.0', self._mjpeg_port), _MjpegHandler)
            threading.Thread(
                target=self._mjpeg_server.serve_forever,
                name='vision-mjpeg', daemon=True).start()
            self.get_logger().info(f'Local MJPEG server listening on port {self._mjpeg_port}')
            return True
        except (OSError, ValueError) as e:
            self._mjpeg_server = None
            _MjpegHandler.node = None
            self.get_logger().error(f'Cannot open MJPEG port {self._mjpeg_port}: {e}')
            return False

    def _start_gortc(self):
        executable = self._find_gortc()
        if executable is None:
            self.get_logger().warn(
                'go2rtc executable not found; local MJPEG remains available')
            return

        port = self.get_parameter('gortc_http_port').get_parameter_value().integer_value
        self._gortc_config = os.path.join('/tmp', f'vision_go2rtc_{os.getpid()}.yaml')
        config = (
            'api:\n'
            f'  listen: ":{port}"\n'
            'streams:\n'
            f'  front: "http://127.0.0.1:{self._mjpeg_port}/front"\n'
            f'  down: "http://127.0.0.1:{self._mjpeg_port}/down"\n'
        )
        if self._stream_annotated:
            config += (
                f'  front_annotated: "http://127.0.0.1:{self._mjpeg_port}/front_annotated"\n'
                f'  down_annotated: "http://127.0.0.1:{self._mjpeg_port}/down_annotated"\n'
            )
        try:
            with open(self._gortc_config, 'w', encoding='utf-8') as config_file:
                config_file.write(config)
            self._gortc_process = subprocess.Popen(
                [executable, '-config', self._gortc_config],
                stdin=subprocess.DEVNULL,
                stdout=None,
                stderr=None,
            )
            if self._gortc_process.poll() is not None:
                raise RuntimeError(
                    f'go2rtc exited immediately with code {self._gortc_process.returncode}')
            streams = 'front, down'
            if self._stream_annotated:
                streams += ', front_annotated, down_annotated'
            self.get_logger().info(
                f'go2rtc started on port {port}; web UI streams: {streams}')
        except (OSError, ValueError, RuntimeError) as e:
            self.get_logger().error(f'Failed to start go2rtc: {e}')

    def _find_gortc(self):
        configured = self.get_parameter('gortc_executable').get_parameter_value().string_value.strip()
        candidates = [os.environ.get('GO2RTC_BIN'), configured]
        try:
            from ament_index_python.packages import get_package_share_directory
            candidates.append(os.path.join(get_package_share_directory('uv_perception'), 'bin', 'go2rtc'))
        except Exception:
            pass
        repo_dir = os.path.abspath(os.path.join(os.path.dirname(os.path.dirname(__file__)), '..', '..', '..'))
        candidates.append(os.path.join(repo_dir, 'third_party', 'go2rtc', 'go2rtc'))
        candidates.append('go2rtc')

        for candidate in candidates:
            if candidate:
                resolved = shutil.which(candidate)
                if resolved and os.path.isfile(resolved) and os.access(resolved, os.X_OK):
                    return resolved
        return None

    def destroy_node(self):
        self._stream_stop.set()
        with self._stream_condition:
            self._stream_condition.notify_all()
        _MjpegHandler.node = None

        if self._mjpeg_server is not None:
            self._mjpeg_server.shutdown()
            self._mjpeg_server.server_close()
        self._capture_stop.set()
        for thread in self._capture_threads:
            thread.join(timeout=1.0)
        self._aruco_stop.set()
        if self._aruco_thread is not None:
            self._aruco_thread.join(timeout=1.0)
        self._vision_pool.shutdown(wait=True, cancel_futures=True)
        for cap in (self._front_cap, self._down_cap):
            if cap is not None:
                cap.release()
        if self._gortc_process is not None and self._gortc_process.poll() is None:
            self._gortc_process.terminate()
            try:
                self._gortc_process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._gortc_process.kill()
        if self._gortc_config:
            try:
                os.unlink(self._gortc_config)
            except OSError:
                pass
        super().destroy_node()

    def _init_undistort(self):
        self.declare_parameter('front_camera_matrix', list(FRONT_CAMERA_MATRIX))
        self.declare_parameter('front_dist_coeffs', list(FRONT_DIST_COEFFS))
        self.declare_parameter('down_camera_matrix', list(DOWN_CAMERA_MATRIX))
        self.declare_parameter('down_dist_coeffs', list(DOWN_DIST_COEFFS))

        def _load_calib(prefix):
            try:
                k_values = self.get_parameter(
                    f'{prefix}_camera_matrix').get_parameter_value().double_array_value
                d_values = self.get_parameter(
                    f'{prefix}_dist_coeffs').get_parameter_value().double_array_value
                if len(k_values) != 9:
                    raise ValueError(f'camera matrix has {len(k_values)} values, expected 9')
                if len(d_values) not in (4, 5, 8, 12, 14):
                    raise ValueError(
                        f'distortion coefficients have {len(d_values)} values')
                return (
                    np.array(k_values, dtype=np.float32).reshape(3, 3),
                    np.array(d_values, dtype=np.float32),
                )
            except (TypeError, ValueError) as e:
                self.get_logger().error(
                    f'Invalid {prefix} camera calibration ({e}); undistortion disabled for it')
                return None, None

        self._front_K, self._front_D = _load_calib('front')
        self._down_K, self._down_D = _load_calib('down')

    def _load_model(self):
        self.declare_parameter('model_path', '')
        model_path = self.get_parameter('model_path').get_parameter_value().string_value.strip()
        try:
            from ultralytics import YOLO
            if not model_path:
                repo_dir = os.path.abspath(os.path.join(
                    os.path.dirname(os.path.dirname(__file__)), '..', '..', '..'))
                candidates = [
                    os.path.expanduser(
                        f'~/YouLong_AUV_Control_System/workspace_auv/src/datas/{DEFAULT_MODEL_FILENAME}'),
                    os.path.join(
                        repo_dir, 'workspace_auv', 'src', 'datas', DEFAULT_MODEL_FILENAME),
                    os.path.join(repo_dir, 'datas', DEFAULT_MODEL_FILENAME),
                ]
                for candidate in candidates:
                    if os.path.exists(candidate):
                        model_path = candidate
                        break

            if model_path and os.path.exists(model_path):
                self._model = YOLO(model_path)
                self._model_loaded = True
                self.get_logger().info(f'YOLO model loaded: {model_path}')
            else:
                self.get_logger().warn(f'YOLO model not found, detection disabled')
        except ImportError:
            self.get_logger().warn('ultralytics not installed, detection disabled')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model, detection disabled: {e}')

    def _front_img_cb(self, msg: Image):
        self._submit_image(msg, 'front')

    def _down_img_cb(self, msg: Image):
        self._submit_image(msg, 'down')

    def _submit_image(self, msg: Image, camera: str):
        """Queue the latest ROS image without allowing executor backlog."""
        if not self._cv_bridge_ok:
            return
        self._submit_work(camera, ('ros_image', msg))

    def _submit_frame(self, frame, camera: str, stamp=None, raw_streamed=False):
        """Queue a native OpenCV frame; no ROS Image is created."""
        self._submit_work(camera, ('opencv', frame, stamp, raw_streamed))

    def _submit_work(self, camera: str, work):
        if camera not in self._processing:
            self.get_logger().error(f'Unknown camera source: {camera}')
            return

        with self._processing_lock:
            self._pending_work[camera] = work
            if self._processing[camera]:
                return
            self._processing[camera] = True

        def worker():
            while True:
                with self._processing_lock:
                    work_item = self._pending_work[camera]
                    self._pending_work[camera] = None
                    if work_item is None:
                        self._processing[camera] = False
                        return
                try:
                    self._process_work(work_item, camera)
                except Exception as e:
                    self.get_logger().error(
                        f'Frame processing failed ({camera}): {e}')

        try:
            self._vision_pool.submit(worker)
        except RuntimeError as e:
            with self._processing_lock:
                self._pending_work[camera] = None
                self._processing[camera] = False
            self.get_logger().warn(f'Cannot schedule {camera} frame: {e}')

    def _process_work(self, work, camera: str):
        source = work[0]
        if source == 'ros_image':
            msg = work[1]
            try:
                cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            except Exception as e:
                self.get_logger().warn(f'Image conversion failed ({camera}): {e}')
                return
            self._process_frame(msg.header, cv_img, camera)
            return

        if source == 'opencv':
            frame, stamp, raw_streamed = work[1], work[2], work[3]
            header = Header()
            header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
            self._process_frame(header, frame, camera, not raw_streamed)
            return

        raise ValueError(f'unknown frame source {source!r}')

    @staticmethod
    def _normalize_frame(frame):
        if not isinstance(frame, np.ndarray) or frame.size == 0:
            return None
        if frame.ndim == 2:
            return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        if frame.ndim != 3 or frame.shape[2] not in (3, 4):
            return None
        if frame.shape[2] == 4:
            return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        return frame

    def _process_frame(self, header, frame, camera: str, update_raw_stream=True):
        # 未激活的相机通道没有发布者，直接跳过避免 KeyError
        if f'{camera}_left' not in self._active_channels:
            return
        cv_img = self._normalize_frame(frame)
        if cv_img is None:
            self.get_logger().warn(f'Invalid {camera} frame, dropping it')
            return
        if cv_img.shape[0] < 2 or cv_img.shape[1] < 2:
            self.get_logger().warn(f'{camera} frame is too small: {cv_img.shape}')
            return

        # 给客户端缓存最新原图，不依赖 YOLO 是否加载成功。
        if update_raw_stream:
            self._update_stream_frame(camera, cv_img)

        if not self._model_loaded:
            self._update_annotated_stream(camera, cv_img)
            return

        K = self._front_K if camera == 'front' else self._down_K
        D = self._front_D if camera == 'front' else self._down_D

        h, w = cv_img.shape[:2]
        mid = w // 2
        if ENABLE_UNDISTORT and K is not None and D is not None:
            left_img = cv2.undistort(cv_img[:, :mid], K, D)
            right_img = cv2.undistort(cv_img[:, mid:], K, D)
        else:
            left_img = cv_img[:, :mid]
            right_img = cv_img[:, mid:]

        # 提供给 ArUco 线程
        if camera == 'front' and self._aruco_detector is not None:
            with self._aruco_lock:
                self._aruco_frames = (left_img.copy(), right_img.copy())

        left_name = f'{camera}_left'
        right_name = f'{camera}_right'

        if self._save_dataset:
            self._save_frame(left_img, left_name)
            self._save_frame(right_img, right_name)

        # Run detection & filtering on left half
        det_left, polys_left, line_left, debug_info_left = self._detect(header, left_name, left_img)
        self.pub_det[left_name].publish(det_left)
        self.pub_line[left_name].publish(line_left)
        annotated_left = self._draw_boxes(
            left_img, det_left, polys_left, line_left, debug_info_left)

        # Run detection & filtering on right half
        det_right, polys_right, line_right, debug_info_right = self._detect(header, right_name, right_img)
        self.pub_det[right_name].publish(det_right)
        self.pub_line[right_name].publish(line_right)
        annotated_right = self._draw_boxes(
            right_img, det_right, polys_right, line_right, debug_info_right)

        # go2rtc 的标注流使用与输入相同的拼接布局。检测完成后再替换
        # 缓存，因此客户端拿到的是“识别+画框”后的帧。
        self._update_annotated_stream(
            camera, np.hstack((annotated_left, annotated_right)))

    def _update_stream_frame(self, camera: str, frame):
        """Update the raw MJPEG frame cache consumed by go2rtc."""
        ok, encoded = cv2.imencode(
            '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            self.get_logger().warn(f'JPEG encoding failed for {camera} raw stream')
            return
        with self._stream_condition:
            self._stream_jpegs[camera] = encoded.tobytes()
            self._stream_sequence[camera] += 1
            self._stream_condition.notify_all()

    def _update_annotated_stream(self, camera: str, frame):
        """Update the annotated MJPEG frame cache consumed by go2rtc."""
        if not self._stream_annotated:
            return
        ok, encoded = cv2.imencode(
            '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            self.get_logger().warn(
                f'JPEG encoding failed for {camera} annotated stream')
            return
        with self._stream_condition:
            self._stream_annotated_jpegs[camera] = encoded.tobytes()
            self._stream_annotated_sequence[camera] += 1
            self._stream_condition.notify_all()

    def _wait_for_stream_frame(self, camera: str, annotated: bool, last_sequence: int):
        """Wait for a newer encoded frame, returning None during shutdown."""
        sequence_cache = (
            self._stream_annotated_sequence if annotated else self._stream_sequence)
        jpeg_cache = (
            self._stream_annotated_jpegs if annotated else self._stream_jpegs)
        with self._stream_condition:
            while not self._stream_stop.is_set():
                sequence = sequence_cache[camera]
                payload = jpeg_cache[camera]
                if payload is not None and sequence != last_sequence:
                    return payload, sequence
                self._stream_condition.wait(timeout=0.5)
        return None

    def _detect(self, header, camera_name: str, cv_img) -> tuple:
        """运行 YOLO 并在同一模型上提取 LineState 及调试信息。"""
        det_array = DetectionArray()
        det_array.header = header
        det_array.camera_name = camera_name
        
        line_state = LineState()
        line_state.stamp = header.stamp
        line_state.camera_name = camera_name
        line_state.detected = False
        
        polygons = []
        debug_info = {}

        try:
            # Ultralytics/PyTorch models are shared by the front and down
            # workers; serialize inference to avoid backend races.
            with self._inference_lock:
                results = self._model(cv_img, conf=self._confidence, verbose=False)
        except Exception as e:
            self.get_logger().error(f'YOLO inference failed ({camera_name}): {e}')
            return det_array, polygons, line_state, debug_info

        best_pipe_poly = None
        max_pipe_area = 0.0

        for result in results:
            if result.boxes is None:
                continue
            
            boxes = result.boxes
            masks = getattr(result, 'masks', None)

            for i in range(len(boxes)):
                det = Detection()
                det.class_id = int(boxes.cls[i])
                det.confidence = float(boxes.conf[i])
                # pipe 单独高置信度阈值 — 低置信度 pipe 丢弃,避免误检
                if det.class_id == PIPE_CLASS_ID and det.confidence < PIPE_CONFIDENCE:
                    continue
                x1, y1, x2, y2 = boxes.xyxy[i].tolist()
                det.bbox_x1 = x1
                det.bbox_y1 = y1
                det.bbox_x2 = x2
                det.bbox_y2 = y2
                det.pixel_x = (x1 + x2) / 2.0
                det.pixel_y = (y1 + y2) / 2.0
                det_array.detections.append(det)

                if det.class_id == 3 and masks is not None and len(masks.xy) > i:
                    poly = masks.xy[i].astype(np.float32)
                    polygons.append(poly)
                    
                    # 寻找面积最大的 pipe contour 作为主导巡线目标
                    area = cv2.contourArea(poly)
                    if area > max_pipe_area and area >= self._line_contour_min_area:
                        max_pipe_area = area
                        best_pipe_poly = poly
                else:
                    polygons.append(None)

        # ── 管道状态数学提取及 Kalman 滤波 (迁移自仿真) ──
        height, width = cv_img.shape[:2]
        
        if best_pipe_poly is not None:
            # 1. 轮廓重心矩 (moments)
            M = cv2.moments(best_pipe_poly)
            if M['m00'] > 0:
                measured_center = float(M['m10'] / M['m00'])
                cx, cy = int(measured_center), int(M['m01'] / M['m00'])
                debug_info['centroid'] = (cx, cy)
            else:
                measured_center = None

            # 2. 最小外接矩形 (minAreaRect)
            rect_center, rect_size, angle = cv2.minAreaRect(best_pipe_poly)
            rw, rh = rect_size
            debug_info['box_pts'] = cv2.boxPoints((rect_center, rect_size, angle)).astype(np.int32)
            
            measured_heading = None
            if max(rw, rh) > 2.0:
                if rh > rw:
                    long_deg = angle + 90.0
                else:
                    long_deg = angle
                measured_heading = long_deg - 90.0
                measured_heading = (measured_heading + 90.0) % 180.0 - 90.0

            # 3. 一维 Kalman 滤波
            filter_state = self._line_filters.get(camera_name)
            if filter_state is None or filter_state.image_width != width:
                filter_state = _LineFilterState(
                    width, self._line_filter_process_noise, self._line_filter_measurement_noise)
                self._line_filters[camera_name] = filter_state

            filtered_center = filter_state.kalman_center.predict()
            filtered_heading = filter_state.kalman_heading.predict()
            
            if measured_center is not None:
                filtered_center = filter_state.kalman_center.update(measured_center)
            if measured_heading is not None:
                filtered_heading = filter_state.kalman_heading.update(measured_heading)

            # 4. 生成 LineState
            line_state.detected = True
            line_state.center_error = float(np.clip((filtered_center - (width / 2.0)) / (width / 2.0), -1.0, 1.0))
            line_state.heading_error_deg = float(filtered_heading)
            line_state.area_ratio = float(max_pipe_area / (height * width))

        return det_array, polygons, line_state, debug_info

    def _draw_boxes(self, cv_img, det_array: DetectionArray, polygons: list, line_state: LineState, debug_info: dict):
        """混合绘制：检测框 + 分割掩码 + 滤波巡线状态。"""
        annotated = cv_img.copy()
        overlay = cv_img.copy()
        height, width = cv_img.shape[:2]

        for i, det in enumerate(det_array.detections):
            x1, y1 = int(det.bbox_x1), int(det.bbox_y1)
            x2, y2 = int(det.bbox_x2), int(det.bbox_y2)
            color = (0, 165, 255) if det.class_id == 3 else (0, 255, 0)

            if det.class_id == 3 and polygons and i < len(polygons) and polygons[i] is not None:
                poly = polygons[i].astype(np.int32)
                cv2.fillPoly(overlay, [poly], color)
                cv2.polylines(annotated, [poly], isClosed=True, color=color, thickness=2)
            else:
                cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)

            label = f'{det.class_id}:{det.confidence:.2f}'
            cv2.putText(annotated, label, (x1, max(y1 - 5, 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        cv2.addWeighted(overlay, 0.4, annotated, 0.6, 0, annotated)

        # ── 绘制巡线状态 (迁移自仿真) ──
        cv2.line(annotated, (width // 2, 0), (width // 2, height), (255, 255, 255), 1)
        
        if line_state.detected:
            if 'box_pts' in debug_info:
                cv2.polylines(annotated, [debug_info['box_pts']], isClosed=True, color=(255, 128, 0), thickness=2)
            if 'centroid' in debug_info:
                cv2.circle(annotated, debug_info['centroid'], 7, (0, 0, 255), -1)

            center_x = int((0.5 + 0.5 * line_state.center_error) * width)
            draw_y = debug_info.get('centroid', (0, height // 2))[1]
            angle = math.radians(line_state.heading_error_deg)
            half_length = height // 4
            dx = int(math.sin(angle) * half_length)
            dy = int(math.cos(angle) * half_length)
            cv2.line(annotated, (center_x - dx, draw_y + dy), (center_x + dx, draw_y - dy), (0, 0, 255), 3)
            
            line_text = (
                f'pipe center={line_state.center_error:+.3f} '
                f'heading={line_state.heading_error_deg:+.1f}deg '
                f'area={line_state.area_ratio:.4f}')
        else:
            line_text = 'pipe not detected'
            
        cv2.putText(annotated, line_text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2, cv2.LINE_AA)

        return annotated

    def _save_frame(self, img, channel: str):
        now_s = int(time.time())
        count = self._dataset_count_s.get(channel, 0)
        last_s = self._dataset_last_s.get(channel, 0)

        if now_s != last_s:
            self._dataset_count_s[channel] = 0
            self._dataset_last_s[channel] = now_s
            count = 0

        if count >= 5:
            return

        self._dataset_count_s[channel] = count + 1
        ts = time.strftime('%Y%m%d%H%M%S') + f'{time.time() % 1:.6f}'[2:5]
        fname = os.path.join(self._dataset_dir, f'{channel}_{ts}.jpg')
        cv2.imwrite(fname, img)

    def _capture_camera_loop(self, cap, camera: str):
        read_failures = 0
        while not self._capture_stop.is_set():
            ret, frame = cap.read()
            if not ret:
                read_failures = min(read_failures + 1, 6)
                time.sleep(min(0.5, 0.01 * (2 ** read_failures)))
                continue
            read_failures = 0

            normalized = self._normalize_frame(frame)
            if normalized is None:
                self.get_logger().warn(f'Invalid frame received from {camera} camera')
                continue

            # Raw video is updated at capture rate, independently of YOLO speed.
            self._update_stream_frame(camera, normalized)
            with self._stream_lock:
                self._stream_frame_count[camera] += 1
                frame_count = self._stream_frame_count[camera]

            if not self._capture_frame_logged[camera]:
                self._capture_frame_logged[camera] = True
                self.get_logger().info(f'{camera} camera frame received: shape={frame.shape}, stream_frames={frame_count}')

            self._submit_frame(
                normalized,
                camera,
                self.get_clock().now().to_msg(),
                raw_streamed=True,
            )


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
