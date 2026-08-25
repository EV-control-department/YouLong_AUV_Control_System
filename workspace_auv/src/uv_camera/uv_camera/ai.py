"""uv_ai: YOLO detection + draw + detection-metadata publish + annotated stream.

Runs in the SAME process as uv_sensor. uv_ai is the FrameGate consumer:
* receives BGR frames handed over in-memory by uv_sensor (A3 — no ROS image,
  no JPEG between the two);
* runs YOLO segmentation, line-state extraction and ArUco detection;
* publishes /perception/detection/{cam}, /perception/line/{cam},
  /perception/aruco/ids (unchanged transport contract for position/task/nav);
* updates the annotated MJPEG cache fed to go2rtc preview.
"""

import os
import threading
import time

import cv2
import numpy as np

from std_msgs.msg import Int32MultiArray

from uv_msgs.msg import Detection, DetectionArray, LineState

from .common import (
    CONFIDENCE,
    DATASET_DIR,
    DEFAULT_MODEL_FILENAME,
    DOWN_CAMERA_MATRIX,
    DOWN_DIST_COEFFS,
    ENABLE_UNDISTORT,
    FRONT_CAMERA_MATRIX,
    FRONT_DIST_COEFFS,
    SAVE_DATASET,
    _LineFilterState,
)


class Ai:
    """Detection consumer for the A3 frame gate."""

    def __init__(self, node, update_annotated_fn, cameras=('front', 'down')):
        self.node = node                     # composed uv_camera rclpy Node
        self._update_annotated = update_annotated_fn  # node.update_annotated_stream
        self._active_cams = list(cameras)

        # line-state / dataset / model config (declared as node params)
        self._line_contour_min_area = int(
            node.get_parameter('line_contour_min_area').value)
        self._line_filter_process_noise = float(
            node.get_parameter('line_filter_process_noise').value)
        self._line_filter_measurement_noise = float(
            node.get_parameter('line_filter_measurement_noise').value)
        self._line_filters = {}

        self._save_dataset = bool(
            node.get_parameter('save_dataset').value)
        self._dataset_dir = DATASET_DIR
        self._dataset_last_s = {}
        self._dataset_count_s = {}
        if self._save_dataset:
            if not self._dataset_dir:
                self._dataset_dir = os.path.join(
                    os.path.dirname(os.path.dirname(
                        os.path.dirname(os.path.dirname(__file__)))), 'img')
            os.makedirs(self._dataset_dir, exist_ok=True)
            node.get_logger().info(f'Dataset saving enabled: {self._dataset_dir}')

        self._model = None
        self._confidence = CONFIDENCE
        self._model_loaded = False
        self._inference_lock = threading.Lock()

        self._front_K, self._front_D = self._load_calib('front')
        self._down_K, self._down_D = self._load_calib('down')

        self._active_channels = set()
        for cam in self._active_cams:
            self._active_channels.add(f'{cam}_left')
            self._active_channels.add(f'{cam}_right')
        self._pub_det = {}
        self._pub_line = {}
        for ch in sorted(self._active_channels):
            self._pub_det[ch] = node.create_publisher(
                DetectionArray, f'/perception/detection/{ch}', 10)
            self._pub_line[ch] = node.create_publisher(
                LineState, f'/perception/line/{ch}', 10)

        # ArUco
        self._aruco_detector = None
        self._aruco_lock = threading.Lock()
        self._aruco_frames = None
        self._aruco_stop = threading.Event()
        self._aruco_thread = None
        self._aruco_pub = None
        self._start_aruco()

    # ── config / model / undistort ──────────────────────────────────────
    def _load_calib(self, prefix):
        try:
            k_values = self.node.get_parameter(
                f'{prefix}_camera_matrix').get_parameter_value().double_array_value
            d_values = self.node.get_parameter(
                f'{prefix}_dist_coeffs').get_parameter_value().double_array_value
            if len(k_values) != 9:
                raise ValueError(f'camera matrix has {len(k_values)} values, expected 9')
            if len(d_values) not in (4, 5, 8, 12, 14):
                raise ValueError(f'distortion coefficients have {len(d_values)} values')
            return (np.array(k_values, dtype=np.float32).reshape(3, 3),
                    np.array(d_values, dtype=np.float32))
        except (TypeError, ValueError) as e:
            self.node.get_logger().error(
                f'Invalid {prefix} calibration ({e}); undistortion disabled')
            return None, None

    def load_model(self, model_path=''):
        try:
            from ultralytics import YOLO
            if not model_path:
                repo_dir = os.path.abspath(
                    os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
                candidates = [
                    os.path.expanduser(
                        f'~/_UUV_AV/workspace_auv/src/datas/{DEFAULT_MODEL_FILENAME}'),
                    os.path.expanduser(
                        f'~/YouLong_AUV_Control_System/workspace_auv/src/datas/{DEFAULT_MODEL_FILENAME}'),
                    os.path.join(repo_dir, 'workspace_auv', 'src', 'datas', DEFAULT_MODEL_FILENAME),
                    os.path.join(repo_dir, 'datas', DEFAULT_MODEL_FILENAME),
                ]
                for candidate in candidates:
                    if os.path.exists(candidate):
                        model_path = candidate
                        break
            if model_path and os.path.exists(model_path):
                self._model = YOLO(model_path)
                self._model_loaded = True
                self.node.get_logger().info(f'YOLO model loaded: {model_path}')
            else:
                self.node.get_logger().warn('YOLO model not found, detection disabled')
        except ImportError:
            self.node.get_logger().warn('ultralytics not installed, detection disabled')
        except Exception as e:
            self.node.get_logger().error(
                f'Failed to load YOLO model, detection disabled: {e}')

    def _start_aruco(self):
        if 'front' not in self._active_cams:
            self.node.get_logger().warn('ArUco disabled (no front camera)')
            return
        try:
            aruco = getattr(cv2, 'aruco')
            aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
            params = aruco.DetectorParameters()
            self._aruco_detector = aruco.ArucoDetector(aruco_dict, params)
        except (AttributeError, cv2.error) as e:
            self.node.get_logger().warn(
                f'OpenCV ArUco unavailable, marker detection disabled: {e}')
        self._aruco_pub = self.node.create_publisher(
            Int32MultiArray, '/perception/aruco/ids', 10)
        if self._aruco_detector is not None:
            self._aruco_thread = threading.Thread(
                target=self._aruco_loop, daemon=True)
            self._aruco_thread.start()

    def _aruco_loop(self):
        while self.node._rclpy_ok() and not self._aruco_stop.is_set():
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
                            all_ids.update(int(m) for m in ids.flatten()
                                           if 1 <= int(m) <= 6)
                    msg = Int32MultiArray()
                    msg.data = sorted(all_ids)
                    self._aruco_pub.publish(msg)
                except Exception as e:
                    self.node.get_logger().warn(f'ArUco detection error: {e}')
            self._aruco_stop.wait(0.05)

    # ── FrameGate consumer (was _process_work + _process_frame) ─────────
    def process(self, camera, work):
        from std_msgs.msg import Header
        from .common import import_cv_bridge
        source = work[0]
        if source == 'ros_image':
            msg = work[1]
            try:
                cb = import_cv_bridge()
                cv_img = cb().imgmsg_to_cv2(msg, 'bgr8')
            except Exception as e:
                self.node.get_logger().warn(
                    f'Image conversion failed ({camera}): {e}')
                return
            self._process_frame(msg.header, cv_img, camera)
            return
        if source == 'opencv':
            frame, stamp, _ = work[1], work[2], work[3]
            header = Header()
            header.stamp = (stamp if stamp is not None
                            else self.node.get_clock().now().to_msg())
            self._process_frame(header, frame, camera)
            return
        raise ValueError(f'unknown frame source {source!r}')

    def _process_frame(self, header, frame, camera):
        if f'{camera}_left' not in self._active_channels:
            return
        from .common import normalize_frame
        cv_img = normalize_frame(frame)
        if cv_img is None or cv_img.shape[0] < 2 or cv_img.shape[1] < 2:
            self.node.get_logger().warn(f'Invalid {camera} frame, dropping')
            return
        # (raw preview was already updated by uv_sensor at capture/arrival time)

        if not self._model_loaded:
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

        if camera == 'front' and self._aruco_detector is not None:
            with self._aruco_lock:
                self._aruco_frames = (left_img.copy(), right_img.copy())

        left_name = f'{camera}_left'
        right_name = f'{camera}_right'
        if self._save_dataset:
            self._save_frame(left_img, left_name)
            self._save_frame(right_img, right_name)

        det_l, polys_l, line_l, dbg_l = self._detect(header, left_name, left_img)
        self._pub_det[left_name].publish(det_l)
        self._pub_line[left_name].publish(line_l)
        ann_l = self._draw_boxes(left_img, det_l, polys_l, line_l, dbg_l)

        det_r, polys_r, line_r, dbg_r = self._detect(header, right_name, right_img)
        self._pub_det[right_name].publish(det_r)
        self._pub_line[right_name].publish(line_r)
        ann_r = self._draw_boxes(right_img, det_r, polys_r, line_r, dbg_r)

        self._update_annotated(camera, np.hstack((ann_l, ann_r)))

    # ── detection (unchanged logic) ────────────────────────────────────
    def _detect(self, header, camera_name, cv_img):
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
            with self._inference_lock:
                results = self._model(cv_img, conf=self._confidence, verbose=False)
        except Exception as e:
            self.node.get_logger().error(
                f'YOLO inference failed ({camera_name}): {e}')
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
                det.bbox_x1, det.bbox_y1 = x1, y1
                det.bbox_x2, det.bbox_y2 = x2, y2
                det.pixel_x = (x1 + x2) / 2.0
                det.pixel_y = (y1 + y2) / 2.0
                det_array.detections.append(det)
                if det.class_id == 3 and masks is not None and len(masks.xy) > i:
                    poly = masks.xy[i].astype(np.float32)
                    polygons.append(poly)
                    area = cv2.contourArea(poly)
                    if area > max_pipe_area and area >= self._line_contour_min_area:
                        max_pipe_area = area
                        best_pipe_poly = poly
                else:
                    polygons.append(None)

        height, width = cv_img.shape[:2]
        if best_pipe_poly is not None:
            M = cv2.moments(best_pipe_poly)
            measured_center = None
            if M['m00'] > 0:
                measured_center = float(M['m10'] / M['m00'])
                debug_info['centroid'] = (int(measured_center), int(M['m01'] / M['m00']))
            rect_center, rect_size, angle = cv2.minAreaRect(best_pipe_poly)
            rw, rh = rect_size
            debug_info['box_pts'] = cv2.boxPoints(
                (rect_center, rect_size, angle)).astype(np.int32)
            measured_heading = None
            if max(rw, rh) > 2.0:
                long_deg = angle + 90.0 if rh > rw else angle
                measured_heading = (long_deg - 90.0 + 90.0) % 180.0 - 90.0
            fs = self._line_filters.get(camera_name)
            if fs is None or fs.image_width != width:
                fs = _LineFilterState(width, self._line_filter_process_noise,
                                      self._line_filter_measurement_noise)
                self._line_filters[camera_name] = fs
            fc = fs.kalman_center.predict()
            fh = fs.kalman_heading.predict()
            if measured_center is not None:
                fc = fs.kalman_center.update(measured_center)
            if measured_heading is not None:
                fh = fs.kalman_heading.update(measured_heading)
            line_state.detected = True
            line_state.center_error = float(np.clip(
                (fc - (width / 2.0)) / (width / 2.0), -1.0, 1.0))
            line_state.heading_error_deg = float(fh)
            line_state.area_ratio = float(max_pipe_area / (height * width))

        return det_array, polygons, line_state, debug_info

    def _draw_boxes(self, cv_img, det_array, polygons, line_state, debug_info):
        import math
        annotated = cv_img.copy()
        overlay = cv_img.copy()
        h, w = cv_img.shape[:2]
        for i, det in enumerate(det_array.detections):
            x1, y1 = int(det.bbox_x1), int(det.bbox_y1)
            x2, y2 = int(det.bbox_x2), int(det.bbox_y2)
            color = (0, 165, 255) if det.class_id == 3 else (0, 255, 0)
            if det.class_id == 3 and polygons and i < len(polygons) and polygons[i] is not None:
                poly = polygons[i].astype(np.int32)
                cv2.fillPoly(overlay, [poly], color)
                cv2.polylines(annotated, [poly], True, color, 2)
            else:
                cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            label = f'{det.class_id}:{det.confidence:.2f}'
            cv2.putText(annotated, label, (x1, max(y1 - 5, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        cv2.addWeighted(overlay, 0.4, annotated, 0.6, 0, annotated)

        cv2.line(annotated, (w // 2, 0), (w // 2, h), (255, 255, 255), 1)
        if line_state.detected:
            if 'box_pts' in debug_info:
                cv2.polylines(annotated, [debug_info['box_pts']], True, (255, 128, 0), 2)
            if 'centroid' in debug_info:
                cv2.circle(annotated, debug_info['centroid'], 7, (0, 0, 255), -1)
            center_x = int((0.5 + 0.5 * line_state.center_error) * w)
            draw_y = debug_info.get('centroid', (0, h // 2))[1]
            angle = math.radians(line_state.heading_error_deg)
            half = h // 4
            dx = int(math.sin(angle) * half)
            dy = int(math.cos(angle) * half)
            cv2.line(annotated, (center_x - dx, draw_y + dy),
                     (center_x + dx, draw_y - dy), (0, 0, 255), 3)
            line_text = (f'pipe center={line_state.center_error:+.3f} '
                         f'heading={line_state.heading_error_deg:+.1f}deg '
                         f'area={line_state.area_ratio:.4f}')
        else:
            line_text = 'pipe not detected'
        cv2.putText(annotated, line_text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (0, 255, 255), 2, cv2.LINE_AA)
        return annotated

    def _save_frame(self, img, channel):
        if not self._save_dataset:
            return
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
        cv2.imwrite(os.path.join(self._dataset_dir, f'{channel}_{ts}.jpg'), img)

    def shutdown(self):
        self._aruco_stop.set()
        if self._aruco_thread is not None:
            self._aruco_thread.join(timeout=1.0)
