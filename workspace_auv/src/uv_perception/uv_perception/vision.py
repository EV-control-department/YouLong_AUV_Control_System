"""Vision node: camera image processing and YOLO detection.

Two modes (sim_mode parameter):
  sim_mode=true  (default): subscribe to /auv/*/stitched ROS topics
  sim_mode=false (real HW): capture from V4L2 devices via OpenCV

Stitched stereo images are split into left/right halves, YOLO runs on each
independently. Output: 4 detection channels + 4 debug image channels.

Parameters:
    model_path (str): Path to YOLO model .pt file.
    publish_images (bool): Whether to forward split images (default: false).
    sim_mode (bool): true=ROS images, false=V4L2 capture (default: true).
    front_cam_path (str): V4L2 device path for front camera (real mode only).
    down_cam_path (str): V4L2 device path for down camera (real mode only).
"""

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

from uv_msgs.msg import Detection, DetectionArray

CvBridge = None


# ── Runtime switches ───────────────────────────────────────────────
# 真实硬件模式下的 V4L2 设备绑定。
FRONT_CAMERA_DEVICE = '/dev/video0'
DOWN_CAMERA_DEVICE = '/dev/video2'
ENABLE_FRONT_CAMERA = True
ENABLE_DOWN_CAMERA = True

# 每个 stitched 相机拆分后的单目图像分辨率。
# stitched 图像应为：(width * 2, height)，例如前/下相机均为 2560x960。
FRONT_CAMERA_RESOLUTION = (1280, 960)  # width, height
DOWN_CAMERA_RESOLUTION = (1280, 960)   # width, height

# 真实 V4L2 设备的实际采集分辨率；由 v4l2-ctl 检测得到。
FRONT_CAPTURE_RESOLUTION = (1280, 720)
DOWN_CAPTURE_RESOLUTION = (2560, 960)

# 相机标定常量：K 为 3x3 内参矩阵，D 的顺序为
# (k1, k2, p1, p2, k3)。请用实际标定结果替换 D。
FRONT_CAMERA_MATRIX = (
    2158.4, 0.0, 640.0,
    0.0, 2158.4, 480.0,
    0.0, 0.0, 1.0,
)
FRONT_DIST_COEFFS = (0.0, 0.0, 0.0, 0.0, 0.0)

DOWN_CAMERA_MATRIX = (
    2307.6, 0.0, 640.0,
    0.0, 2307.6, 480.0,
    0.0, 0.0, 1.0,
)
DOWN_DIST_COEFFS = (0.0, 0.0, 0.0, 0.0, 0.0)

ENABLE_UNDISTORT = True              # 是否执行相机去畸变
ENABLE_GORTC = True                 # 启动 go2rtc 转发客户端视频
GORTC_HTTP_PORT = 1984               # go2rtc 对外 HTTP/WebRTC API 端口
VISION_MJPEG_PORT = 8090             # vision 提供给 go2rtc 的本地 MJPEG 端口
GORTC_EXECUTABLE = 'go2rtc'

# ROS 参数默认值。
PUBLISH_IMAGES = False               # 发布拆分后的左右原图
PUBLISH_ANNOTATED = False             # 发布带检测框的图像
SIM_MODE = False                      # False=V4L2，True=ROS stitched 话题
SAVE_DATASET = False                  # 是否保存训练数据帧
DATASET_DIR = ''                      # 空字符串=自动使用工程下的 img/

class _MjpegHandler(BaseHTTPRequestHandler):
    """Very small MJPEG source used by go2rtc (and directly by a browser)."""

    node = None

    def do_GET(self):  # noqa: N802 - required by BaseHTTPRequestHandler
        camera = self.path.strip('/').split('?', 1)[0]
        if camera not in ('front', 'down'):
            self.send_error(404, 'use /front or /down')
            return

        self.send_response(200)
        self.send_header('Cache-Control', 'no-cache, private')
        self.send_header('Connection', 'close')
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        try:
            while rclpy.ok() and self.node is not None:
                with self.node._stream_lock:
                    payload = self.node._stream_jpegs.get(camera)
                if payload is None:
                    time.sleep(0.02)
                    continue
                self.wfile.write(
                    b'--frame\r\nContent-Type: image/jpeg\r\n'
                    + f'Content-Length: {len(payload)}\r\n\r\n'.encode('ascii'))
                self.wfile.write(payload)
                self.wfile.write(b'\r\n')
                self.wfile.flush()
                time.sleep(0.03)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass

    def log_message(self, *_args):
        pass


def _import_cv_bridge():
    global CvBridge
    if CvBridge is None:
        try:
            from cv_bridge import CvBridge as _CB
            CvBridge = _CB
        except Exception as e:
            raise ImportError(f'cv_bridge not available: {e}')


class VisionNode(Node):
    """Vision node: YOLO detection on stitched camera images, split into L/R."""

    def __init__(self):
        super().__init__('vision')

        self.declare_parameter('publish_images', PUBLISH_IMAGES)
        self._publish_images = self.get_parameter('publish_images').get_parameter_value().bool_value

        self.declare_parameter('publish_annotated', PUBLISH_ANNOTATED)
        self._publish_annotated = self.get_parameter('publish_annotated').get_parameter_value().bool_value

        self.declare_parameter('sim_mode', SIM_MODE)
        self._sim_mode = self.get_parameter('sim_mode').get_parameter_value().bool_value

        self.declare_parameter('gortc_executable', GORTC_EXECUTABLE)
        self.declare_parameter('gortc_http_port', GORTC_HTTP_PORT)

        self.declare_parameter('enable_front_camera', ENABLE_FRONT_CAMERA)
        self.declare_parameter('enable_down_camera', ENABLE_DOWN_CAMERA)
        self._enable_front_camera = self.get_parameter(
            'enable_front_camera').get_parameter_value().bool_value
        self._enable_down_camera = self.get_parameter(
            'enable_down_camera').get_parameter_value().bool_value

        self.declare_parameter('save_dataset', SAVE_DATASET)
        self._save_dataset = self.get_parameter('save_dataset').get_parameter_value().bool_value
        self._dataset_dir = DATASET_DIR
        self._dataset_last_s = {}  # channel -> last save time (second bucket)
        self._dataset_count_s = {}  # channel -> count in current second
        if self._save_dataset:
            # img/ at same level as src/
            if not self._dataset_dir:
                self._dataset_dir = os.path.join(
                    os.path.dirname(os.path.dirname(os.path.dirname(__file__))), 'img')
            os.makedirs(self._dataset_dir, exist_ok=True)
            self.get_logger().info(f'Dataset saving enabled: {self._dataset_dir}')

        self.bridge = None
        self._model = None
        self._confidence = 0.9
        self._model_loaded = False
        self._cv_bridge_ok = False
        self._front_cap = None
        self._down_cap = None
        self._gortc_process = None
        self._gortc_config = None

        # ROS 回调只负责收图；前、下相机各自占用一个视觉工作线程，
        # 避免一个相机的 YOLO 推理阻塞另一个相机和客户端视频流。
        self._vision_pool = ThreadPoolExecutor(max_workers=2, thread_name_prefix='vision')
        self._processing_lock = threading.Lock()
        self._processing = {'front': False, 'down': False}
        self._stream_lock = threading.Lock()
        self._stream_frames = {'front': None, 'down': None}
        self._stream_jpegs = {'front': None, 'down': None}
        self._stream_frame_count = {'front': 0, 'down': 0}
        self._capture_frame_logged = {'front': False, 'down': False}
        self._capture_stop = threading.Event()
        self._capture_threads = []

        # 本地 MJPEG 源由 go2rtc 拉取；即使 go2rtc 不存在，原 ROS 功能仍可用。
        self._mjpeg_server = None
        if ENABLE_GORTC:
            self._start_mjpeg_server()
            self._start_gortc()

        # Try to import cv_bridge
        try:
            _import_cv_bridge()
            self.bridge = CvBridge()
            self._cv_bridge_ok = True
        except Exception as e:
            self.get_logger().warn(f'cv_bridge not available: {e}')

        # Distortion correction — per camera pair (applied to each half after split)
        self._init_undistort()

        # Load YOLO model
        self._load_model()

        # Image source: ROS topics (sim) or V4L2 devices (real)
        if self._sim_mode:
            if self._enable_front_camera:
                self.create_subscription(Image, '/auv/front_cam/stitched', self._front_img_cb, 10)
            if self._enable_down_camera:
                self.create_subscription(Image, '/auv/down_cam/stitched', self._down_img_cb, 10)
            self.get_logger().info('Vision node started (sim mode: ROS topics)')
        else:
            self.declare_parameter('front_cam_path', FRONT_CAMERA_DEVICE)
            self.declare_parameter('down_cam_path', DOWN_CAMERA_DEVICE)
            front_path = self.get_parameter('front_cam_path').get_parameter_value().string_value
            down_path = self.get_parameter('down_cam_path').get_parameter_value().string_value

            if self._enable_front_camera:
                self._front_cap = cv2.VideoCapture(front_path)
                self._front_cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                self._front_cap.set(
                    cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                self._front_cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRONT_CAPTURE_RESOLUTION[0])
                self._front_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRONT_CAPTURE_RESOLUTION[1])
            if self._enable_down_camera:
                self._down_cap = cv2.VideoCapture(down_path)
                self._down_cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                self._down_cap.set(
                    cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                self._down_cap.set(cv2.CAP_PROP_FRAME_WIDTH, DOWN_CAPTURE_RESOLUTION[0])
                self._down_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, DOWN_CAPTURE_RESOLUTION[1])
            if self._enable_front_camera and not self._front_cap.isOpened():
                self.get_logger().error(f'Failed to open front camera: {front_path}')
            if self._enable_down_camera and not self._down_cap.isOpened():
                self.get_logger().error(f'Failed to open down camera: {down_path}')

            # 独立连续采集线程始终读取最新帧，避免 10Hz timer 造成 V4L2 缓冲积压。
            for cap, camera in (
                    (self._front_cap, 'front'), (self._down_cap, 'down')):
                if cap is not None and cap.isOpened():
                    thread = threading.Thread(
                        target=self._capture_camera_loop,
                        args=(cap, camera),
                        name=f'capture-{camera}', daemon=True)
                    self._capture_threads.append(thread)
                    thread.start()
            self.get_logger().info(
                f'Vision node started (real mode: front={front_path}, down={down_path})')

        # Publishers — 4 detection + 4 image (common to both modes)
        self.pub_det = {
            'front_left':  self.create_publisher(DetectionArray, '/perception/detection/front_left', 10),
            'front_right': self.create_publisher(DetectionArray, '/perception/detection/front_right', 10),
            'down_left':   self.create_publisher(DetectionArray, '/perception/detection/down_left', 10),
            'down_right':  self.create_publisher(DetectionArray, '/perception/detection/down_right', 10),
        }
        self.pub_img = {
            'front_left':  self.create_publisher(Image, '/perception/image/front_left', 10),
            'front_right': self.create_publisher(Image, '/perception/image/front_right', 10),
            'down_left':   self.create_publisher(Image, '/perception/image/down_left', 10),
            'down_right':  self.create_publisher(Image, '/perception/image/down_right', 10),
        }
        self.pub_annotated = {
            'front_left':  self.create_publisher(Image, '/perception/annotated/front_left', 10),
            'front_right': self.create_publisher(Image, '/perception/annotated/front_right', 10),
            'down_left':   self.create_publisher(Image, '/perception/annotated/down_left', 10),
            'down_right':  self.create_publisher(Image, '/perception/annotated/down_right', 10),
        }

    def _start_mjpeg_server(self):
        """Open a local MJPEG port for the optional go2rtc bridge."""
        try:
            _MjpegHandler.node = self
            self._mjpeg_server = ThreadingHTTPServer(
                ('0.0.0.0', VISION_MJPEG_PORT), _MjpegHandler)
            threading.Thread(
                target=self._mjpeg_server.serve_forever,
                name='vision-mjpeg', daemon=True).start()
            self.get_logger().info(
                f'MJPEG source opened: http://0.0.0.0:{VISION_MJPEG_PORT}/front|down')
        except OSError as e:
            self.get_logger().error(f'Cannot open MJPEG port {VISION_MJPEG_PORT}: {e}')

    def _start_gortc(self):
        """Start go2rtc and expose front/down streams on its HTTP/WebRTC port."""
        executable = self._find_gortc()
        if executable is None:
            self.get_logger().warn(
                'go2rtc not found; local MJPEG source remains available. '
                'Run third_party/go2rtc/download_go2rtc.sh or set GO2RTC_BIN.')
            return

        port = self.get_parameter('gortc_http_port').get_parameter_value().integer_value
        self._gortc_config = os.path.join('/tmp', f'vision_go2rtc_{os.getpid()}.yaml')
        config = (
            'api:\n'
            f'  listen: ":{port}"\n'
            'streams:\n'
            f'  front: "http://127.0.0.1:{VISION_MJPEG_PORT}/front"\n'
            f'  down: "http://127.0.0.1:{VISION_MJPEG_PORT}/down"\n'
        )
        try:
            with open(self._gortc_config, 'w', encoding='utf-8') as config_file:
                config_file.write(config)
            self._gortc_process = subprocess.Popen(
                [executable, '-config', self._gortc_config],
                stdout=None, stderr=None)
            self.get_logger().info(
                f'go2rtc started on port {port}; '
                f'web UI: /stream.html?src=front or /stream.html?src=down')
        except (OSError, ValueError) as e:
            self.get_logger().error(f'Failed to start go2rtc: {e}')

    def _find_gortc(self):
        """Find a deployable go2rtc binary without assuming a user's home path."""
        configured = self.get_parameter(
            'gortc_executable').get_parameter_value().string_value.strip()
        candidates = []
        if os.environ.get('GO2RTC_BIN'):
            candidates.append(os.environ['GO2RTC_BIN'])
        if configured:
            candidates.append(configured)

        # Installed package: share/uv_perception/bin/go2rtc.
        try:
            from ament_index_python.packages import get_package_share_directory
            share_dir = get_package_share_directory('uv_perception')
            candidates.append(os.path.join(share_dir, 'bin', 'go2rtc'))
        except Exception:
            pass

        # Source checkout: repository third_party/go2rtc/go2rtc.
        package_dir = os.path.dirname(os.path.dirname(__file__))
        repo_dir = os.path.abspath(os.path.join(package_dir, '..', '..', '..'))
        candidates.append(os.path.join(repo_dir, 'third_party', 'go2rtc', 'go2rtc'))
        candidates.append('go2rtc')

        for candidate in candidates:
            resolved = shutil.which(candidate)
            if resolved and os.path.isfile(resolved) and os.access(resolved, os.X_OK):
                self.get_logger().info(f'Using go2rtc: {resolved}')
                return resolved
        return None

    def destroy_node(self):
        """Stop workers, camera devices, MJPEG server and optional go2rtc."""
        if self._mjpeg_server is not None:
            self._mjpeg_server.shutdown()
            self._mjpeg_server.server_close()
        self._capture_stop.set()
        for thread in self._capture_threads:
            thread.join(timeout=1.0)
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
        """Load camera matrix and distortion coefficients from parameters.

        Each camera pair (front/down) has a 3x3 camera matrix and distortion
        coefficients (k1,k2,p1,p2,k3).  Applied to each half after splitting.
        Defaults: constants defined at the top of this file.
        """
        # 常量是默认配置；ROS 参数仍可覆盖，方便不改代码时加载标定文件。
        self.declare_parameter('front_camera_matrix', list(FRONT_CAMERA_MATRIX))
        self.declare_parameter('front_dist_coeffs', list(FRONT_DIST_COEFFS))
        self.declare_parameter('down_camera_matrix', list(DOWN_CAMERA_MATRIX))
        self.declare_parameter('down_dist_coeffs', list(DOWN_DIST_COEFFS))

        def _load_calib(prefix):
            K = np.array(self.get_parameter(f'{prefix}_camera_matrix')
                         .get_parameter_value().double_array_value,
                         dtype=np.float32).reshape(3, 3)
            D = np.array(self.get_parameter(f'{prefix}_dist_coeffs')
                         .get_parameter_value().double_array_value,
                         dtype=np.float32)
            return K, D

        self._front_K, self._front_D = _load_calib('front')
        self._down_K, self._down_D = _load_calib('down')

        self.get_logger().info(
            f'Undistort: front K={self._front_K.tolist()}, D={self._front_D.tolist()}')

    def _load_model(self):
        """Load YOLO model from parameter path or default locations."""
        try:
            from ultralytics import YOLO

            self.declare_parameter('model_path', '')
            model_path = self.get_parameter('model_path').get_parameter_value().string_value

            if not model_path:
                for candidate in [
                    os.path.expanduser('~/YouLong_AUV_Control_System/workspace_auv/src/datas/WUURC2026_sim_719.pt'),
                ]:
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

    def _front_img_cb(self, msg: Image):
        self._submit_image(msg, 'front')

    def _down_img_cb(self, msg: Image):
        self._submit_image(msg, 'down')

    def _submit_image(self, msg: Image, camera: str):
        """Submit at most one frame per camera, keeping ROS callbacks responsive."""
        with self._processing_lock:
            if self._processing[camera]:
                return
            self._processing[camera] = True

        def worker():
            try:
                self._process_image(msg, camera)
            finally:
                with self._processing_lock:
                    self._processing[camera] = False

        self._vision_pool.submit(worker)

    def _process_image(self, msg: Image, camera: str):
        """Split stitched image into left/right halves, run YOLO on each."""
        if not self._cv_bridge_ok:
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        # 给客户端缓存最新 JPEG，不依赖 YOLO 是否加载成功。
        ok, encoded = cv2.imencode(
            '.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        with self._stream_lock:
            self._stream_frames[camera] = cv_img
            if ok:
                self._stream_jpegs[camera] = encoded.tobytes()

        # 即使没有加载模型，也继续提供客户端视频，但不发布检测结果。
        if not self._model_loaded:
            return

        K = self._front_K if camera == 'front' else self._down_K
        D = self._front_D if camera == 'front' else self._down_D

        h, w = cv_img.shape[:2]
        mid = w // 2
        if ENABLE_UNDISTORT:
            left_img = cv2.undistort(cv_img[:, :mid], K, D)
            right_img = cv2.undistort(cv_img[:, mid:], K, D)
        else:
            left_img = cv_img[:, :mid]
            right_img = cv_img[:, mid:]

        left_name = f'{camera}_left'
        right_name = f'{camera}_right'

        # Save dataset frames (5 per second per channel)
        if self._save_dataset:
            self._save_frame(left_img, left_name)
            self._save_frame(right_img, right_name)

        # Run detection on left half
        det_left = self._detect(msg.header, left_name, left_img)
        self.pub_det[left_name].publish(det_left)
        if self._publish_images:
            self.pub_img[left_name].publish(self.bridge.cv2_to_imgmsg(left_img, 'bgr8'))
        if self._publish_annotated:
            annotated = self._draw_boxes(left_img, det_left)
            self.pub_annotated[left_name].publish(self.bridge.cv2_to_imgmsg(annotated, 'bgr8'))

        # Run detection on right half
        det_right = self._detect(msg.header, right_name, right_img)
        self.pub_det[right_name].publish(det_right)
        if self._publish_images:
            self.pub_img[right_name].publish(self.bridge.cv2_to_imgmsg(right_img, 'bgr8'))
        if self._publish_annotated:
            annotated = self._draw_boxes(right_img, det_right)
            self.pub_annotated[right_name].publish(self.bridge.cv2_to_imgmsg(annotated, 'bgr8'))

    def _draw_boxes(self, cv_img, det_array: DetectionArray):
        """Draw YOLO detection boxes and labels on image."""
        annotated = cv_img.copy()
        for det in det_array.detections:
            x1, y1 = int(det.bbox_x1), int(det.bbox_y1)
            x2, y2 = int(det.bbox_x2), int(det.bbox_y2)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f'{det.class_id}:{det.confidence:.2f}'
            cv2.putText(annotated, label, (x1, max(y1 - 5, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        return annotated

    def _detect(self, header, camera_name: str, cv_img) -> DetectionArray:
        """Run YOLO on a single image, return DetectionArray."""
        det_array = DetectionArray()
        det_array.header = header
        det_array.camera_name = camera_name

        try:
            results = self._model(cv_img, conf=self._confidence, verbose=False)
        except Exception as e:
            self.get_logger().error(f'YOLO inference failed ({camera_name}): {e}')
            return det_array

        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                det = Detection()
                det.class_id = int(box.cls[0])
                det.confidence = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                det.bbox_x1 = x1
                det.bbox_y1 = y1
                det.bbox_x2 = x2
                det.bbox_y2 = y2
                det.pixel_x = (x1 + x2) / 2.0
                det.pixel_y = (y1 + y2) / 2.0
                det_array.detections.append(det)

        return det_array


    def _save_frame(self, img, channel: str):
        """Save a dataset frame at up to 5 FPS per channel."""
        now_s = int(time.time())
        count = self._dataset_count_s.get(channel, 0)
        last_s = self._dataset_last_s.get(channel, 0)

        # Reset counter on new second
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
        """Continuously capture one V4L2 camera and keep only its newest frame."""
        while not self._capture_stop.is_set():
            ret, frame = cap.read()
            if not ret:
                self.get_logger().warn(
                    f'{camera} camera read failed', throttle_duration_sec=5.0)
                time.sleep(0.01)
                continue

            ok, encoded = cv2.imencode(
                '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            with self._stream_lock:
                self._stream_frames[camera] = frame
                if ok:
                    self._stream_jpegs[camera] = encoded.tobytes()
                self._stream_frame_count[camera] += 1
                frame_count = self._stream_frame_count[camera]

            if not self._capture_frame_logged[camera]:
                self._capture_frame_logged[camera] = True
                self.get_logger().info(
                    f'{camera} camera frame received: shape={frame.shape}, '
                    f'dtype={frame.dtype}, stream_frames={frame_count}')

            if self._cv_bridge_ok:
                img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                img_msg.header.stamp = self.get_clock().now().to_msg()
                self._submit_image(img_msg, camera)


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
