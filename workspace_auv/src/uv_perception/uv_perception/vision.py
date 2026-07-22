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

import math
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from uv_msgs.msg import Detection, DetectionArray, LineState

CvBridge = None


def _import_cv_bridge():
    global CvBridge
    if CvBridge is None:
        try:
            from cv_bridge import CvBridge as _CB
            CvBridge = _CB
        except Exception as e:
            raise ImportError(f'cv_bridge not available: {e}')


class _ScalarKalman:
    """不依赖第三方库的一维 Kalman 滤波器。

    管道 mask 在单帧中通常是正确的，但边缘、反光和遮挡会使轮廓中心/
    方向偶尔跳变。这里只跟踪单变量（中心 x 或角度），不对速度或世界
    位置做估计。
    """

    def __init__(self, initial_value, process_noise=1.0, measurement_noise=3.0):
        self.x = float(initial_value)
        self.p = 100.0
        self.q = max(1e-6, float(process_noise))
        self.r = max(1e-6, float(measurement_noise))

    def predict(self):
        """执行无控制量预测，并返回当前估计值。"""
        self.p += self.q
        return self.x

    def update(self, measurement):
        """融合一个扫描行观测值，并返回滤波后的中心位置。"""
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


class VisionNode(Node):
    """Vision node: YOLO detection on stitched camera images, split into L/R."""

    def __init__(self):
        super().__init__('vision')

        self.declare_parameter('publish_images', False)
        self._publish_images = self.get_parameter(
            'publish_images').get_parameter_value().bool_value

        self.declare_parameter('publish_annotated', False)
        self._publish_annotated = self.get_parameter(
            'publish_annotated').get_parameter_value().bool_value

        # 巡线使用轮廓矩 + minAreaRect 提取管道中心/方向，无需相机安装偏置。
        self.declare_parameter('segment_model_path', '')
        self.declare_parameter('line_contour_min_area', 200)
        self._line_contour_min_area = int(self.get_parameter(
            'line_contour_min_area').value)
        self.declare_parameter('line_filter_process_noise', 1.0)
        self._line_filter_process_noise = float(self.get_parameter(
            'line_filter_process_noise').value)
        self.declare_parameter('line_filter_measurement_noise', 3.0)
        self._line_filter_measurement_noise = float(self.get_parameter(
            'line_filter_measurement_noise').value)
        self._line_filters = {}
        self.get_logger().info(
            f'Pipe line contour: min_area={self._line_contour_min_area}px, '
            f'no camera heading offset')

        self.declare_parameter('sim_mode', True)
        self._sim_mode = self.get_parameter('sim_mode').get_parameter_value().bool_value

        self.declare_parameter('save_dataset', False)
        self._save_dataset = self.get_parameter('save_dataset').get_parameter_value().bool_value
        self._dataset_dir = ''
        self._dataset_last_s = {}  # channel -> last save time (second bucket)
        self._dataset_count_s = {}  # channel -> count in current second
        if self._save_dataset:
            # img/ at same level as src/
            self._dataset_dir = os.path.join(
                os.path.dirname(os.path.dirname(os.path.dirname(__file__))), 'img')
            os.makedirs(self._dataset_dir, exist_ok=True)
            self.get_logger().info(f'Dataset saving enabled: {self._dataset_dir}')

        self.bridge = None
        self._model = None
        self._confidence = 0.9
        self._model_loaded = False
        self._segment_model = None
        self._segment_model_loaded = False
        self._cv_bridge_ok = False
        self._front_cap = None
        self._down_cap = None

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
            self.create_subscription(Image, '/auv/front_cam/stitched', self._front_img_cb, 10)
            self.create_subscription(Image, '/auv/down_cam/stitched', self._down_img_cb, 10)
            self.get_logger().info('Vision node started (sim mode: ROS topics)')
        else:
            self.declare_parameter('front_cam_path', '/dev/video0')
            self.declare_parameter('down_cam_path', '/dev/video2')
            front_path = self.get_parameter('front_cam_path').get_parameter_value().string_value
            down_path = self.get_parameter('down_cam_path').get_parameter_value().string_value

            self._front_cap = cv2.VideoCapture(front_path)
            self._down_cap = cv2.VideoCapture(down_path)
            if not self._front_cap.isOpened():
                self.get_logger().error(f'Failed to open front camera: {front_path}')
            if not self._down_cap.isOpened():
                self.get_logger().error(f'Failed to open down camera: {down_path}')

            # 10Hz capture timer
            self.create_timer(0.1, self._capture_real_cams)
            self.get_logger().info(
                f'Vision node started (real mode: front={front_path}, down={down_path})')

        # Publishers — 4 detection + 4 image (common to both modes)
        self.pub_det = {
            'front_left': self.create_publisher(
                DetectionArray, '/perception/detection/front_left', 10),
            'front_right': self.create_publisher(
                DetectionArray, '/perception/detection/front_right', 10),
            'down_left': self.create_publisher(
                DetectionArray, '/perception/detection/down_left', 10),
            'down_right': self.create_publisher(
                DetectionArray, '/perception/detection/down_right', 10),
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
        self.pub_line = {
            name: self.create_publisher(LineState, f'/perception/line/{name}', 10)
            for name in self.pub_det
        }

    def _init_undistort(self):
        """Load camera matrix and distortion coefficients from parameters.

        Each camera pair (front/down) has a 3x3 camera matrix and distortion
        coefficients (k1,k2,p1,p2,k3).  Applied to each half after splitting.
        Defaults: identity matrix, zero distortion.
        """
        identity_3x3 = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        zero_5 = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.declare_parameter('front_camera_matrix', identity_3x3)
        self.declare_parameter('front_dist_coeffs', zero_5)
        self.declare_parameter('down_camera_matrix', identity_3x3)
        self.declare_parameter('down_dist_coeffs', zero_5)

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

            def valid_file(path: str, parameter_name: str) -> str:
                """Accept only model files; reject accidental directories."""
                path = os.path.expanduser(path.strip()) if path else ''
                if not path:
                    return ''
                if os.path.isfile(path):
                    return path
                self.get_logger().warn(
                    f'{parameter_name}={path!r} is not a model file; '
                    'using the package fallback')
                return ''

            self.declare_parameter('model_path', '')
            model_path = valid_file(
                self.get_parameter('model_path').get_parameter_value().string_value,
                'model_path')

            if not model_path:
                for candidate in [
                    os.path.expanduser(
                        '~/YouLong_AUV_Control_System/workspace_auv/src/'
                        'datas/WUURC2026_sim_719.pt'),
                ]:
                    if os.path.isfile(candidate):
                        model_path = candidate
                        break
                if not model_path:
                    # The competition box model is shipped with this package.
                    # Keep an installed-package fallback for non-symlink builds.
                    resource_box = os.path.join(
                        os.path.dirname(os.path.dirname(__file__)),
                        'resource', 'box_best.pt')
                    if os.path.isfile(resource_box):
                        model_path = resource_box
                    else:
                        try:
                            from ament_index_python.packages import get_package_share_directory
                            share_box = os.path.join(
                                get_package_share_directory('uv_perception'),
                                'resource', 'box_best.pt')
                            if os.path.isfile(share_box):
                                model_path = share_box
                        except Exception:
                            pass

            if model_path and os.path.isfile(model_path):
                self._model = YOLO(model_path)
                self._model_loaded = True
                self.get_logger().info(f'YOLO model loaded: {model_path}')
            else:
                self.get_logger().warn('YOLO model not found, detection disabled')

            # --- Segment model (YOLO-Seg) for line following ---
            seg_path = valid_file(
                self.get_parameter(
                    'segment_model_path').get_parameter_value().string_value,
                'segment_model_path')
            if not seg_path:
                for candidate in [
                    os.path.expanduser(
                        '~/YouLong_AUV_Control_System/workspace_auv/src/'
                        'datas/WUURC2026simSegment.pt'),
                ]:
                    if os.path.isfile(candidate):
                        seg_path = candidate
                        break
                if not seg_path:
                    # Try resource/seg_best.pt next to this file
                    resource_seg = os.path.join(
                        os.path.dirname(os.path.dirname(__file__)),
                        'resource', 'seg_best.pt')
                    if os.path.isfile(resource_seg):
                        seg_path = resource_seg
                    else:
                        # Try ament_index_python fallback
                        try:
                            from ament_index_python.packages import get_package_share_directory
                            share_seg = os.path.join(
                                get_package_share_directory('uv_perception'),
                                'resource', 'seg_best.pt')
                            if os.path.isfile(share_seg):
                                seg_path = share_seg
                        except Exception:
                            pass

            if seg_path and os.path.isfile(seg_path):
                self._segment_model = YOLO(seg_path)
                self._segment_model_loaded = True
                self.get_logger().info(f'Segment model loaded: {seg_path}')
            else:
                self.get_logger().warn(
                    'Segment model not found, line following disabled')
        except ImportError:
            self.get_logger().warn('ultralytics not installed, detection disabled')

    def _front_img_cb(self, msg: Image):
        self._process_image(msg, 'front')

    def _down_img_cb(self, msg: Image):
        self._process_image(msg, 'down')

    def _process_image(self, msg: Image, camera: str):
        """Split stitched image into left/right halves, run YOLO on each."""
        if not self._cv_bridge_ok:
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        K = self._front_K if camera == 'front' else self._down_K
        D = self._front_D if camera == 'front' else self._down_D

        h, w = cv_img.shape[:2]
        mid = w // 2
        left_img = cv2.undistort(cv_img[:, :mid], K, D)
        right_img = cv2.undistort(cv_img[:, mid:], K, D)

        left_name = f'{camera}_left'
        right_name = f'{camera}_right'

        # Save dataset frames (5 per second per channel)
        if self._save_dataset:
            self._save_frame(left_img, left_name)
            self._save_frame(right_img, right_name)

        # Run detection and segmentation on the left half.  They are kept as
        # separate models: box classes drive mission events, while the Seg
        # model supplies only the pipe line state and debug mask.
        det_left = self._detect(msg.header, left_name, left_img)
        self.pub_det[left_name].publish(det_left)
        line_left, mask_left = self._segment_line(
            msg.header, left_name, left_img, return_mask=True)
        self.pub_line[left_name].publish(line_left)
        if self._publish_images:
            raw_left = self.bridge.cv2_to_imgmsg(left_img, 'bgr8')
            raw_left.header = msg.header
            self.pub_img[left_name].publish(raw_left)
        if self._publish_annotated:
            debug_left = self._annotate_frame(left_img, det_left, line_left, mask_left)
            debug_msg = self.bridge.cv2_to_imgmsg(debug_left, 'bgr8')
            debug_msg.header = msg.header
            self.pub_annotated[left_name].publish(debug_msg)

        # Run detection on right half
        det_right = self._detect(msg.header, right_name, right_img)
        self.pub_det[right_name].publish(det_right)
        line_right, mask_right = self._segment_line(
            msg.header, right_name, right_img, return_mask=True)
        self.pub_line[right_name].publish(line_right)
        if self._publish_images:
            raw_right = self.bridge.cv2_to_imgmsg(right_img, 'bgr8')
            raw_right.header = msg.header
            self.pub_img[right_name].publish(raw_right)
        if self._publish_annotated:
            debug_right = self._annotate_frame(
                right_img, det_right, line_right, mask_right)
            debug_msg = self.bridge.cv2_to_imgmsg(debug_right, 'bgr8')
            debug_msg.header = msg.header
            self.pub_annotated[right_name].publish(debug_msg)

    def _detect(self, header, camera_name: str, cv_img) -> DetectionArray:
        """Run YOLO on a single image, return DetectionArray."""
        det_array = DetectionArray()
        det_array.header = header
        det_array.camera_name = camera_name

        if not self._model_loaded or self._model is None:
            return det_array

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

    def _segment_line(self, header, camera_name: str, cv_img, return_mask=False):
        """从 YOLO-Seg mask 提取管道中心位置和图像内方向。

        在最大轮廓上计算 cv2.moments 重心和 cv2.minAreaRect 长轴角度，
        经过两个独立的一维 Kalman 滤波后输出 center_error 和 heading_error_deg。

        heading_error_deg 只描述管道相对于图像竖直方向的几何角度，
        不包含相机安装偏置，也不是世界位姿估计。
        """
        state = LineState()
        state.stamp = header.stamp
        state.camera_name = camera_name
        if not self._segment_model_loaded:
            return (state, None) if return_mask else state

        try:
            results = self._segment_model(cv_img, conf=self._confidence, verbose=False)
            masks = []
            for result in results:
                if result.masks is None:
                    continue
                masks.extend(result.masks.data.cpu().numpy())
            if not masks:
                return (state, None) if return_mask else state

            combined = np.max(np.asarray(masks), axis=0)
            mask = (combined > 0.5).astype(np.uint8)
            mask = cv2.resize(mask, (cv_img.shape[1], cv_img.shape[0]),
                              interpolation=cv2.INTER_NEAREST)
            ys, xs = np.nonzero(mask > 0)
            if len(xs) < self._line_contour_min_area:
                return (state, mask) if return_mask else state

            height, width = cv_img.shape[:2]

            # ── 轮廓分析：用整块 mask 的全局几何代替固定扫描行 ──
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                return (state, mask) if return_mask else state

            max_ctr = max(contours, key=cv2.contourArea)
            if cv2.contourArea(max_ctr) < self._line_contour_min_area:
                return (state, mask) if return_mask else state

            # moments → 重心（观测位置）
            M = cv2.moments(max_ctr)
            if M['m00'] > 0:
                measured_center = float(M['m10'] / M['m00'])
            else:
                measured_center = None

            # minAreaRect → 长轴方向（观测角度）
            rect_center, rect_size, angle = cv2.minAreaRect(max_ctr)
            rw, rh = rect_size
            measured_heading = None
            if max(rw, rh) > 2.0:
                # angle ∈ [-90, 0) 是短边与水平轴的夹角
                if rh > rw:
                    long_deg = angle + 90.0
                else:
                    long_deg = angle
                # 转为与图像竖直方向（↓）的夹角，归一到 [-90, 90]
                measured_heading = long_deg - 90.0
                measured_heading = (measured_heading + 90.0) % 180.0 - 90.0

            # ── Kalman 滤波 ──
            filter_state = self._line_filters.get(camera_name)
            if filter_state is None or filter_state.image_width != width:
                filter_state = _LineFilterState(
                    width,
                    self._line_filter_process_noise,
                    self._line_filter_measurement_noise,
                )
                self._line_filters[camera_name] = filter_state

            filtered_center = filter_state.kalman_center.predict()
            filtered_heading = filter_state.kalman_heading.predict()
            if measured_center is not None:
                filtered_center = filter_state.kalman_center.update(measured_center)
            if measured_heading is not None:
                filtered_heading = filter_state.kalman_heading.update(measured_heading)

            state.detected = True
            state.center_error = float(
                np.clip((filtered_center - (width / 2.0)) /
                        (width / 2.0), -1.0, 1.0))
            state.heading_error_deg = float(filtered_heading)
            state.area_ratio = float(len(xs) / (height * width))

            return (state, mask) if return_mask else state
        except Exception as e:
            self.get_logger().error(
                f'YOLO line segmentation failed ({camera_name}): {e}')
        return (state, None) if return_mask else state

    def _annotate_frame(self, cv_img, detections, line_state, mask):
        """在 BGR 调试图上绘制检测框、轮廓矩形、滤波中心和 LineState。"""
        debug = cv_img.copy()
        height, width = debug.shape[:2]
        if mask is not None:
            mask_bool = mask > 0
            overlay = np.zeros_like(debug)
            overlay[:, :, 1] = np.where(mask_bool, 220, 0).astype(np.uint8)
            debug = cv2.addWeighted(debug, 0.72, overlay, 0.28, 0.0)

        for detection in detections.detections:
            x1 = int(round(detection.bbox_x1))
            y1 = int(round(detection.bbox_y1))
            x2 = int(round(detection.bbox_x2))
            y2 = int(round(detection.bbox_y2))
            cv2.rectangle(debug, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f'{self._class_name(detection.class_id)} '
            label += f'{detection.confidence:.2f} id={detection.class_id}'
            cv2.putText(
                debug, label, (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2,
                cv2.LINE_AA)

        # 竖直中线
        cv2.line(debug, (width // 2, 0), (width // 2, height), (255, 255, 255), 1)

        # 从 mask 提取轮廓，绘制 minAreaRect 和 centroid
        box_pts = None
        centroid = None
        if mask is not None and line_state.detected:
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                max_ctr = max(contours, key=cv2.contourArea)
                # 最小外接矩形（蓝色）
                box_pts = cv2.boxPoints(cv2.minAreaRect(max_ctr)).astype(np.int32)
                cv2.polylines(debug, [box_pts], isClosed=True, color=(255, 128, 0), thickness=2)
                # moments 重心（红色圆点）
                M = cv2.moments(max_ctr)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    centroid = (cx, cy)
                    cv2.circle(debug, centroid, 7, (0, 0, 255), -1)

        if line_state.detected:
            # 从 center_error 反算投影中心 x，沿 heading 绘制方向线段
            center_x = int((0.5 + 0.5 * line_state.center_error) * width)
            draw_y = centroid[1] if centroid else height // 2
            angle = math.radians(line_state.heading_error_deg)
            half_length = height // 4
            dx = int(math.sin(angle) * half_length)
            dy = int(math.cos(angle) * half_length)
            cv2.line(
                debug,
                (center_x - dx, draw_y + dy),
                (center_x + dx, draw_y - dy),
                (0, 0, 255), 3)
            line_text = (
                f'pipe center={line_state.center_error:+.3f} '
                f'heading={line_state.heading_error_deg:+.1f}deg '
                f'area={line_state.area_ratio:.4f}')
        else:
            line_text = 'pipe not detected'
        cv2.putText(
            debug, line_text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX,
            0.65, (0, 255, 255), 2, cv2.LINE_AA)
        return debug

    def _class_name(self, class_id):
        """Resolve the trained YOLO class name without assuming dict/list form."""
        names = getattr(self._model, 'names', {}) if self._model is not None else {}
        if isinstance(names, dict):
            return str(names.get(class_id, 'unknown'))
        if isinstance(names, (list, tuple)) and 0 <= class_id < len(names):
            return str(names[class_id])
        return 'unknown'

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

    def _capture_real_cams(self):
        """Real mode: capture from V4L2 devices at 10Hz, feed to processing pipeline."""
        now = self.get_clock().now().to_msg()
        for cap, camera in [(self._front_cap, 'front'), (self._down_cap, 'down')]:
            if cap is None or not cap.isOpened():
                continue
            ret, frame = cap.read()
            if not ret:
                self.get_logger().warn(f'{camera} camera read failed', throttle_duration_sec=5.0)
                continue
            img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            img_msg.header.stamp = now
            self._process_image(img_msg, camera)


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
