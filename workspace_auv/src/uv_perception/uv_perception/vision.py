"""Vision node: camera image processing and YOLO detection.

Two modes (sim_mode parameter):
  sim_mode=true  (default): subscribe to /auv/*/stitched ROS topics
  sim_mode=false (real HW): capture from V4L2 devices via OpenCV

Stitched stereo images are split into left/right halves, YOLO runs on each
independently. Output: 4 detection channels + 4 debug image channels.

Parameters:
    model_path (str): Path to YOLO segmentation model .pt file.
    publish_images (bool): Whether to forward split images (default: false).
    sim_mode (bool): true=ROS images, false=V4L2 capture (default: true).
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
from std_msgs.msg import Int32MultiArray

from uv_msgs.msg import Detection, DetectionArray, LineState

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

CONFIDENCE = 0.6

ENABLE_UNDISTORT = True              # 是否执行相机去畸变
ENABLE_GORTC = True                 # 启动 go2rtc 转发客户端视频
GORTC_HTTP_PORT = 1984               # go2rtc 对外 HTTP/WebRTC API 端口
VISION_MJPEG_PORT = 8090             # vision 提供给 go2rtc 的本地 MJPEG 端口
GORTC_EXECUTABLE = 'go2rtc'

# ROS 参数默认值。
PUBLISH_IMAGES = False               # 发布拆分后的左右原图
PUBLISH_ANNOTATED = True             # 发布带检测框的图像
SIM_MODE = False                      # False=V4L2，True=ROS stitched 话题
SAVE_DATASET = False                  # 是否保存训练数据帧
DATASET_DIR = ''                      # 空字符串=自动使用工程下的 img/


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

    def do_GET(self):
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
    """Vision node: YOLO segmentation detection on stitched camera images, split into L/R."""

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
        self._front_cap = None
        self._down_cap = None
        self._gortc_process = None
        self._gortc_config = None

        # 线程锁及流媒体相关
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

        if ENABLE_GORTC:
            self._start_mjpeg_server()
            self._start_gortc()

        try:
            _import_cv_bridge()
            self.bridge = CvBridge()
            self._cv_bridge_ok = True
        except Exception as e:
            self.get_logger().warn(f'cv_bridge not available: {e}')

        self._init_undistort()
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
                self._front_cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                self._front_cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRONT_CAPTURE_RESOLUTION[0])
                self._front_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRONT_CAPTURE_RESOLUTION[1])
            if self._enable_down_camera:
                self._down_cap = cv2.VideoCapture(down_path)
                self._down_cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                self._down_cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                self._down_cap.set(cv2.CAP_PROP_FRAME_WIDTH, DOWN_CAPTURE_RESOLUTION[0])
                self._down_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, DOWN_CAPTURE_RESOLUTION[1])

            for cap, camera in ((self._front_cap, 'front'), (self._down_cap, 'down')):
                if cap is not None and cap.isOpened():
                    thread = threading.Thread(
                        target=self._capture_camera_loop,
                        args=(cap, camera),
                        name=f'capture-{camera}', daemon=True)
                    self._capture_threads.append(thread)
                    thread.start()
            self.get_logger().info(f'Vision node started (real mode: front={front_path}, down={down_path})')

        # Publishers
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
        # 新增巡线状态 Publisher
        self.pub_line = {
            name: self.create_publisher(LineState, f'/perception/line/{name}', 10)
            for name in self.pub_det
        }

        # ── ArUco detection (迁移自仿真节点) ─────────────────────────────────────────
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
        self._aruco_params = cv2.aruco.DetectorParameters()
        self._aruco_detector = cv2.aruco.ArucoDetector(self._aruco_dict, self._aruco_params)
        self._aruco_pub = self.create_publisher(Int32MultiArray, '/perception/aruco/ids', 10)
        self._aruco_lock = threading.Lock()
        self._aruco_frames = None  # (left_img, right_img) for ArUco thread
        self._aruco_thread = threading.Thread(target=self._aruco_loop, daemon=True)
        self._aruco_thread.start()

    def _aruco_loop(self):
        """Background thread: detect ArUco markers on front camera halves."""
        while rclpy.ok():
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
            time.sleep(0.05)  # ~20 Hz

    def _start_mjpeg_server(self):
        try:
            _MjpegHandler.node = self
            self._mjpeg_server = ThreadingHTTPServer(('0.0.0.0', VISION_MJPEG_PORT), _MjpegHandler)
            threading.Thread(target=self._mjpeg_server.serve_forever, name='vision-mjpeg', daemon=True).start()
        except OSError as e:
            self.get_logger().error(f'Cannot open MJPEG port {VISION_MJPEG_PORT}: {e}')

    def _start_gortc(self):
        executable = self._find_gortc()
        if executable is None:
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
            self._gortc_process = subprocess.Popen([executable, '-config', self._gortc_config], stdout=None, stderr=None)
        except (OSError, ValueError) as e:
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
            K = np.array(self.get_parameter(f'{prefix}_camera_matrix').get_parameter_value().double_array_value, dtype=np.float32).reshape(3, 3)
            D = np.array(self.get_parameter(f'{prefix}_dist_coeffs').get_parameter_value().double_array_value, dtype=np.float32)
            return K, D

        self._front_K, self._front_D = _load_calib('front')
        self._down_K, self._down_D = _load_calib('down')

    def _load_model(self):
        try:
            from ultralytics import YOLO
            self.declare_parameter('model_path', '/home/nvidia/YouLong_AUV_Control_System/workspace_auv/src/datas/WUURC2026REAL11nano--001.pt')
            model_path = self.get_parameter('model_path').get_parameter_value().string_value

            if not model_path:
                for candidate in [os.path.expanduser('~/YouLong_AUV_Control_System/workspace_auv/src/datas/WUURC2026REAL11nano--001.pt')]:
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
        if not self._cv_bridge_ok:
            return
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            return

        ok, encoded = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        with self._stream_lock:
            self._stream_frames[camera] = cv_img
            if ok:
                self._stream_jpegs[camera] = encoded.tobytes()

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

        # 提供给 ArUco 线程
        if camera == 'front':
            with self._aruco_lock:
                self._aruco_frames = (left_img.copy(), right_img.copy())

        left_name = f'{camera}_left'
        right_name = f'{camera}_right'

        if self._save_dataset:
            self._save_frame(left_img, left_name)
            self._save_frame(right_img, right_name)

        # Run detection & filtering on left half
        det_left, polys_left, line_left, debug_info_left = self._detect(msg.header, left_name, left_img)
        self.pub_det[left_name].publish(det_left)
        self.pub_line[left_name].publish(line_left)
        if self._publish_images:
            self.pub_img[left_name].publish(self.bridge.cv2_to_imgmsg(left_img, 'bgr8'))
        if self._publish_annotated:
            annotated = self._draw_boxes(left_img, det_left, polys_left, line_left, debug_info_left)
            self.pub_annotated[left_name].publish(self.bridge.cv2_to_imgmsg(annotated, 'bgr8'))

        # Run detection & filtering on right half
        det_right, polys_right, line_right, debug_info_right = self._detect(msg.header, right_name, right_img)
        self.pub_det[right_name].publish(det_right)
        self.pub_line[right_name].publish(line_right)
        if self._publish_images:
            self.pub_img[right_name].publish(self.bridge.cv2_to_imgmsg(right_img, 'bgr8'))
        if self._publish_annotated:
            annotated = self._draw_boxes(right_img, det_right, polys_right, line_right, debug_info_right)
            self.pub_annotated[right_name].publish(self.bridge.cv2_to_imgmsg(annotated, 'bgr8'))

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
        while not self._capture_stop.is_set():
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.01)
                continue

            ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            with self._stream_lock:
                self._stream_frames[camera] = frame
                if ok:
                    self._stream_jpegs[camera] = encoded.tobytes()
                self._stream_frame_count[camera] += 1
                frame_count = self._stream_frame_count[camera]

            if not self._capture_frame_logged[camera]:
                self._capture_frame_logged[camera] = True
                self.get_logger().info(f'{camera} camera frame received: shape={frame.shape}, stream_frames={frame_count}')

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