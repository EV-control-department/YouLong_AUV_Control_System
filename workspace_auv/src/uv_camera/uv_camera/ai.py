"""uv_ai: YOLO detection + draw + detection-metadata publish + annotated stream.

Runs in the SAME process as uv_sensor. uv_ai is the FrameGate consumer:
* receives BGR frames handed over in-memory by uv_sensor (A3 — no ROS image,
  no JPEG between the two);
* runs YOLO detection (with optional segmentation-mask assistance for line
  visualization), line-state extraction and ArUco detection;
* publishes /perception/detection/{cam}, /perception/line/{cam},
  /perception/aruco/ids (unchanged transport contract for position/task/nav);
* updates the annotated MJPEG cache fed to go2rtc preview.
"""

import os
import threading
import time
from pathlib import Path

import cv2
import numpy as np

from builtin_interfaces.msg import Time
from std_msgs.msg import Header, Int32MultiArray

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
    _LineFilterState,
    image_msg_to_bgr,
    normalize_frame,
    unrotate_points_180,
)
from .model_classes import DEFAULT_CLASS_NAMES, MODEL_MAPPING_PATH, model_class_id
from .dataset_recorder import DatasetRecorder


# Values mirror uv_msgs/msg/Detection.msg.  Keeping the local constants avoids
# importing generated message constants in the image-processing hot path.
FEATURE_BBOX_CENTER = 0
FEATURE_GATE_CENTERLINE = 1
FEATURE_GATE_SEGMENTATION = 2
GATE_FRONT_CLASS_ID = model_class_id('gate_front', required=False)


def _ros_time(value):
    """Convert a ROS Time or a sec/nanosec-compatible test value to Time."""
    if value is None:
        return None
    if isinstance(value, Time):
        return value
    return Time(sec=int(value.sec), nanosec=int(value.nanosec))


class Ai:
    """Detection consumer for the A3 frame gate."""

    def __init__(
        self,
        node,
        update_annotated_fn,
        cameras=('front', 'down'),
        inference_fps=0.0,
        dataset_fps=5.0,
        dataset_debug=False,
        dataset_debug_period_s=1.0,
        dataset_submit_timeout_s=1.0,
        dataset_writer_workers=4,
        dataset_webp_method=0,
        inference_threads=2,
        gate_feature_mode='auto',
        confidence=CONFIDENCE,
        device='auto',
        mapping_callback=None,
        turntable_callback=None,
    ):
        self.node = node                     # composed uv_camera rclpy Node
        self._sim_mode = bool(node.get_parameter('sim_mode').value)
        self._mapping_callback = mapping_callback
        self._turntable_callback = turntable_callback
        self._update_annotated = update_annotated_fn  # node.update_annotated_stream
        self._active_cams = list(cameras)
        self._inference_period_s = (
            0.0 if float(inference_fps) <= 0.0 else 1.0 / float(inference_fps)
        )
        self._inference_timing_lock = threading.Lock()
        self._last_inference_s = {camera: float('-inf') for camera in cameras}
        self._dataset_period_s = (
            0.0 if float(dataset_fps) <= 0.0 else 1.0 / float(dataset_fps)
        )
        self._dataset_timing_lock = threading.Lock()
        self._last_dataset_s = {camera: float('-inf') for camera in cameras}
        self._dataset_debug = bool(dataset_debug)
        self._dataset_debug_period_s = max(0.1, float(dataset_debug_period_s))
        self._dataset_submit_timeout_s = max(0.1, float(dataset_submit_timeout_s))
        self._dataset_debug_lock = threading.Lock()
        self._dataset_debug_last_capture_s = {}
        self._dataset_debug_last_log_s = {}
        self._inference_threads = max(1, int(inference_threads))
        self._requested_device = str(device).strip().lower() or 'auto'
        self._device = 'cpu'
        self._gate_feature_mode = str(gate_feature_mode).strip().lower()
        if self._gate_feature_mode not in {
                'auto', 'bbox', 'centerline', 'segmentation'}:
            self._gate_feature_mode = 'auto'

        # line-state / dataset / model config (declared as node params)
        self._line_contour_min_area = int(
            node.get_parameter('line_contour_min_area').value)
        self._line_filter_process_noise = float(
            node.get_parameter('line_filter_process_noise').value)
        self._line_filter_measurement_noise = float(
            node.get_parameter('line_filter_measurement_noise').value)
        self._line_filters = {}

        self._save_dataset = bool(node.get_parameter('save_dataset').value)
        self._dataset_dir = str(
            node.get_parameter('dataset_dir').value or DATASET_DIR).strip()
        self._dataset_recorder = None
        if self._save_dataset:
            if not self._dataset_dir:
                self._dataset_dir = DATASET_DIR
            self._dataset_recorder = DatasetRecorder(
                self._dataset_dir,
                queue_size=int(node.get_parameter('dataset_queue_size').value),
                png_compression=int(
                    node.get_parameter('dataset_png_compression').value),
                image_format=str(
                    node.get_parameter('dataset_format').value),
                logger=node.get_logger(),
                debug=self._dataset_debug,
                debug_period_s=self._dataset_debug_period_s,
                submit_timeout_s=self._dataset_submit_timeout_s,
                writer_workers=dataset_writer_workers,
                webp_method=dataset_webp_method)
            node.get_logger().info(
                f'Lossless YOLO dataset recording enabled: '
                f'{self._dataset_recorder.session_dir} '
                f'(format={self._dataset_recorder.image_format})')

        self._model = None
        self._confidence = min(1.0, max(0.05, float(confidence)))
        self._model_loaded = False
        self._inference_lock = threading.Lock()
        self._last_inference_diagnostic = {}

        self._front_K, self._front_D = self._load_calib('front')
        self._down_K, self._down_D = self._load_calib('down')
        self._down_right_K, self._down_right_D = self._down_K, self._down_D
        if not self._sim_mode:
            from .down_calibration import load_real_down_json, real_down_calibration_path
            path = real_down_calibration_path()
            _, _, self._down_K, self._down_D, self._down_right_K, \
                self._down_right_D, _, _ = load_real_down_json(path)
            node.get_logger().info(f'下视双目内参已加载：{path}（Camera1/Camera2 独立）')

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
            # Ultralytics delegates CPU inference to PyTorch, whose default
            # thread count is often the full machine.  Cap it so Stonefish,
            # ROS and the control path retain CPU time.
            try:
                import torch
            except ImportError:
                self._device = 'cpu'
                torch = None
            if torch is not None:
                # These calls can raise after another library has initialized
                # PyTorch; that should not disable CUDA selection.
                try:
                    torch.set_num_threads(self._inference_threads)
                    torch.set_num_interop_threads(1)
                except RuntimeError:
                    pass
                requested = self._requested_device
                if requested == 'auto':
                    self._device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
                elif requested.startswith('cuda') and not torch.cuda.is_available():
                    self._device = 'cpu'
                    self.node.get_logger().warn(
                        f'YOLO requested device={requested}, but CUDA is unavailable; '
                        'falling back to CPU')
                else:
                    self._device = requested
                if self._device.startswith('cuda'):
                    self.node.get_logger().info(
                        f'YOLO CUDA enabled: {torch.cuda.get_device_name(0)} '
                        f'({self._device})')
            from ultralytics import YOLO
            if model_path:
                model_path = os.path.expanduser(str(model_path))
            else:
                # ``__file__`` can point into colcon's build-space symlink.
                # Resolve it before walking up and search both the source
                # package and the installed package share directory.  The
                # old ``workspace_auv/src/datas`` path did not contain the
                # deployed model, which made Edge silently run without AI.
                module_path = Path(__file__).resolve()
                filename = DEFAULT_MODEL_FILENAME if self._sim_mode else 'last.pt'
                candidates = []
                for parent in (module_path.parent, *module_path.parents):
                    candidates.extend((
                        parent / 'resource' / filename,
                        parent / 'weights' / filename,
                        parent / 'workspace_auv' / 'src' / 'uv_camera' /
                        'resource' / filename,
                    ))
                candidates.append(
                    Path.cwd() / 'workspace_auv' / 'src' / 'uv_camera' /
                    'resource' / filename)
                try:
                    from ament_index_python.packages import (
                        get_package_share_directory)
                    candidates.append(
                        Path(get_package_share_directory('uv_camera')) /
                        'resource' / filename)
                except Exception:
                    # Keep model discovery usable for offline/unit-test imports.
                    pass

                for candidate in candidates:
                    if not self._sim_mode:
                        compatible = candidate.with_name('last_inference.pt')
                        if compatible.is_file():
                            model_path = str(compatible)
                            break
                    if candidate.is_file():
                        model_path = str(candidate)
                        break

            if model_path and os.path.isfile(model_path):
                self._model = YOLO(str(model_path))
                names = self._model.names
                actual = tuple(str(names[index]) for index in range(len(names)))
                if not self._sim_mode and actual != DEFAULT_CLASS_NAMES:
                    raise ValueError(
                        f'YOLO类别{actual}与映射{DEFAULT_CLASS_NAMES}不一致；'
                        f'检查 UV_MODEL_MAPPING_FILE={MODEL_MAPPING_PATH}')
                # Move the model explicitly so the first inference cannot
                # silently initialize on CPU.  The call site also passes the
                # device because Ultralytics may recreate its predictor.
                self._model.to(self._device)
                self._model_loaded = True
                self.node.get_logger().info(
                    f'YOLO class mapping loaded: {MODEL_MAPPING_PATH}')
                self.node.get_logger().info(f'YOLO model loaded: {model_path}')
            else:
                self.node.get_logger().warn('YOLO model not found, detection disabled')
        except ImportError:
            self.node.get_logger().warn('ultralytics not installed, detection disabled')
        except Exception as e:
            self._model_loaded = False
            self._model = None
            self.node.get_logger().error(
                f'Failed to load YOLO model, detection disabled: {e}')
            if 'ultralytics.utils.loss' in str(e):
                self.node.get_logger().error(
                    '权重包含当前 Ultralytics 不支持的训练损失对象。'
                    '请对可信 last.pt 运行 scripts/convert_yolo_inference.py，'
                    '并使用 last_inference.pt；不要替换 Jetson 的 NVIDIA PyTorch')

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
        source = work[0]
        if source == 'ros_image':
            msg = work[1]
            try:
                cv_img = image_msg_to_bgr(msg)
            except Exception as e:
                self.node.get_logger().warn(
                    f'Image conversion failed ({camera}): {e}')
                return
            self._process_frame(msg.header, cv_img, camera)
            return
        if source == 'opencv':
            frame, stamp, _ = work[1], work[2], work[3]
            right_stamp = work[4] if len(work) > 4 else None
            stereo_pair_id = int(work[5]) if len(work) > 5 else 0
            header = Header()
            header.stamp = (_ros_time(stamp) if stamp is not None
                            else self.node.get_clock().now().to_msg())
            self._process_frame(
                header, frame, camera, right_stamp, stereo_pair_id)
            return
        raise ValueError(f'unknown frame source {source!r}')

    def _process_frame(self, header, frame, camera, right_stamp=None,
                       stereo_pair_id=0):
        if f'{camera}_left' not in self._active_channels:
            return
        prepared = self._prepare_stereo_views(camera, frame)
        if prepared is None:
            self.node.get_logger().warn(f'Invalid {camera} frame, dropping')
            return
        cv_img, left_img, right_img = prepared

        if camera == 'front' and self._aruco_detector is not None:
            with self._aruco_lock:
                # These arrays are read-only in the ArUco worker.  Retaining
                # the views avoids two full-resolution copies per inference.
                self._aruco_frames = (left_img, right_img)

        left_name = f'{camera}_left'
        right_name = f'{camera}_right'

        right_header = Header()
        right_header.frame_id = header.frame_id
        right_header.stamp = (_ros_time(right_stamp)
                             if right_stamp is not None else header.stamp)
        # Keep the ROS detection topics alive with explicit empty results when
        # optional YOLO is unavailable.  This lets readiness finish for data
        # collection without pretending that detections were produced.
        if not self._model_loaded:
            self._publish_empty_results(
                left_name, header, stereo_pair_id)
            self._publish_empty_results(
                right_name, right_header, stereo_pair_id)
            if camera == 'down' and self._mapping_callback is not None:
                self._mapping_callback(cv_img, header.stamp,
                                       DetectionArray(), DetectionArray())
            if camera == 'front' and self._turntable_callback is not None:
                empty_left, empty_right = DetectionArray(), DetectionArray()
                empty_left.header = header
                empty_right.header = right_header
                self._turntable_callback(cv_img, empty_left, empty_right)
            return
        if not self._allow_inference(camera):
            return

        det_l, polys_l, line_l, dbg_l = self._detect(
            header, left_name, left_img, stereo_pair_id)
        self._pub_det[left_name].publish(det_l)
        self._pub_line[left_name].publish(line_l)
        annotate = self.node.stream_requested(camera, True)
        if annotate:
            ann_l = self._draw_boxes(left_img, det_l, polys_l, line_l, dbg_l)

        det_r, polys_r, line_r, dbg_r = self._detect(
            right_header, right_name, right_img, stereo_pair_id)
        self._pub_det[right_name].publish(det_r)
        self._pub_line[right_name].publish(line_r)
        if camera == 'down' and self._mapping_callback is not None:
            self._mapping_callback(cv_img, header.stamp, det_l, det_r)
        if camera == 'front' and self._turntable_callback is not None:
            self._turntable_callback(cv_img, det_l, det_r)
        if annotate:
            ann_r = self._draw_boxes(right_img, det_r, polys_r, line_r, dbg_r)

        if annotate:
            self._update_annotated(
                camera, np.hstack((ann_l, ann_r)), header.stamp)

    def _prepare_stereo_views(self, camera, frame):
        """Normalize and split one stitched frame into YOLO input views."""
        cv_img = normalize_frame(frame)
        if cv_img is None or cv_img.shape[0] < 2 or cv_img.shape[1] < 2:
            return None

        K = self._front_K if camera == 'front' else self._down_K
        D = self._front_D if camera == 'front' else self._down_D
        right_K = self._front_K if camera == 'front' else self._down_right_K
        right_D = self._front_D if camera == 'front' else self._down_right_D
        mid = cv_img.shape[1] // 2
        distortion_active = (
            ENABLE_UNDISTORT and K is not None and D is not None
            and bool(np.any(np.abs(D) > 1e-12))
            # Detection coordinates must match the raw stitched image.
            # Mapping, localizer and turntable each rectify with per-eye K/D.
            and camera not in ('front', 'down')
        )
        if distortion_active:
            left_img = cv2.undistort(cv_img[:, :mid], K, D)
            right_img = cv2.undistort(cv_img[:, mid:], right_K, right_D)
        else:
            left_img = cv_img[:, :mid]
            right_img = cv_img[:, mid:]
        return cv_img, left_img, right_img

    def _allow_dataset(self, camera):
        """Rate-limit dataset sampling independently from YOLO inference."""
        if self._dataset_period_s <= 0.0:
            return True
        now = time.monotonic()
        with self._dataset_timing_lock:
            last = self._last_dataset_s.get(camera, float('-inf'))
            if now - last < self._dataset_period_s:
                return False
            self._last_dataset_s[camera] = now
        return True

    def record_capture_frame(self, camera, frame, header=None,
                             right_stamp=None, stereo_pair_id=0):
        """Record a sampled sensor frame before it enters the AI FrameGate.

        This must be called by the sensor path, not by ``_process_frame``:
        FrameGate intentionally drops stale frames when inference is slower
        than capture, while dataset recording should retain the requested
        capture cadence.
        """
        if self._dataset_recorder is None:
            return
        started = time.monotonic()
        sampled = False
        try:
            if not self._allow_dataset(camera):
                return
            prepared = self._prepare_stereo_views(camera, frame)
            if prepared is None:
                return
            _, left_img, right_img = prepared

            left_header = Header()
            if header is not None:
                left_header.frame_id = str(getattr(header, 'frame_id', ''))
                left_header.stamp = header.stamp
            else:
                left_header.stamp = self.node.get_clock().now().to_msg()

            right_header = Header()
            right_header.frame_id = left_header.frame_id
            right_header.stamp = (
                right_stamp if right_stamp is not None else left_header.stamp)
            left_ok = self._save_frame(
                left_img, f'{camera}_left', left_header, stereo_pair_id)
            if not left_ok:
                return
            right_ok = self._save_frame(
                right_img, f'{camera}_right', right_header, stereo_pair_id)
            sampled = bool(right_ok)
        finally:
            self._maybe_log_dataset_debug(camera, started, sampled)

    def _maybe_log_dataset_debug(self, camera, started, sampled):
        if not self._dataset_debug:
            return
        now = time.monotonic()
        duration_ms = (now - started) * 1000.0
        with self._dataset_debug_lock:
            previous = self._dataset_debug_last_capture_s.get(camera)
            interval_ms = (
                (now - previous) * 1000.0 if previous is not None else 0.0)
            self._dataset_debug_last_capture_s[camera] = now
            last_log = self._dataset_debug_last_log_s.get(camera, 0.0)
            if now - last_log < self._dataset_debug_period_s:
                return
            self._dataset_debug_last_log_s[camera] = now
        snapshot = self._dataset_recorder.debug_snapshot()
        self.node.get_logger().info(
            'dataset-debug capture: '
            f'camera={camera} interval_ms={interval_ms:.1f} '
            f'record_call_ms={duration_ms:.1f} sampled={int(sampled)} '
            f"state={snapshot['state']} "
            f"queue={snapshot['queue_depth']}/{snapshot['queue_capacity']} "
            f"submitted={snapshot['submitted']} written={snapshot['written']} "
            f"blocked_submits={snapshot['blocked_submits']} "
            f"blocked_wait_ms={snapshot['blocked_wait_ms']:.1f} "
            f"last_write_ms={snapshot['last_write_ms']:.1f}")

    def _publish_empty_results(self, camera_name, header, stereo_pair_id):
        """Publish an empty detection/line result when AI is unavailable."""
        detections = DetectionArray()
        detections.header = header
        detections.camera_name = camera_name
        if hasattr(detections, 'stereo_pair_id'):
            detections.stereo_pair_id = int(stereo_pair_id or 0)

        line = LineState()
        line.stamp = header.stamp
        line.camera_name = camera_name
        line.detected = False
        self._pub_det[camera_name].publish(detections)
        self._pub_line[camera_name].publish(line)

    def _allow_inference(self, camera):
        """Rate-limit expensive inference while keeping the newest frame."""
        if self._inference_period_s <= 0.0:
            return True
        now = time.monotonic()
        with self._inference_timing_lock:
            last = self._last_inference_s.get(camera, float('-inf'))
            if now - last < self._inference_period_s:
                return False
            self._last_inference_s[camera] = now
        return True

    # ── detection (unchanged logic) ────────────────────────────────────
    def _detect(self, header, camera_name, cv_img, stereo_pair_id=0):
        det_array = DetectionArray()
        det_array.header = header
        det_array.camera_name = camera_name
        if hasattr(det_array, 'stereo_pair_id'):
            det_array.stereo_pair_id = int(stereo_pair_id or 0)

        line_state = LineState()
        line_state.stamp = header.stamp
        line_state.camera_name = camera_name
        line_state.detected = False

        polygons = []
        debug_info = {}

        # YOLO sees the upright installed view; all published pixels and masks
        # return to the raw calibration space before stereo/geometry processing.
        # Do NOT swap left/right or modify K/D/R/T for a display transform.
        rotate = getattr(self.node, 'camera_rotate_180', lambda camera: False)(
            camera_name.rsplit('_', 1)[0])
        inference_img = cv2.rotate(cv_img, cv2.ROTATE_180) if rotate else cv_img

        try:
            queued_at = time.monotonic()
            with self._inference_lock:
                started_at = time.monotonic()
                results = self._model(
                    inference_img, conf=self._confidence, device=self._device,
                    verbose=False)
                finished_at = time.monotonic()
            if finished_at - self._last_inference_diagnostic.get(camera_name, float('-inf')) >= 5.0:
                self._last_inference_diagnostic[camera_name] = finished_at
                device = getattr(getattr(self._model, 'predictor', None), 'device', 'unknown')
                self.node.get_logger().info(
                    f'YOLO管线[{camera_name}] device={device}，'
                    f'等待模型锁={(started_at-queued_at)*1000:.1f}ms，'
                    f'推理调用={(finished_at-started_at)*1000:.1f}ms，'
                    f'输入={cv_img.shape[1]}x{cv_img.shape[0]}')
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
                if rotate:
                    # Bounding boxes are continuous edge coordinates [0,w]/[0,h],
                    # unlike polygon/sample pixel coordinates [0,w-1]/[0,h-1].
                    height, width = cv_img.shape[:2]
                    x1, y1, x2, y2 = width-x2, height-y2, width-x1, height-y1
                det.bbox_x1, det.bbox_y1 = x1, y1
                det.bbox_x2, det.bbox_y2 = x2, y2
                det.pixel_x = (x1 + x2) / 2.0
                det.pixel_y = (y1 + y2) / 2.0
                poly = None
                if masks is not None and len(masks.xy) > i:
                    poly = masks.xy[i].astype(np.float32)
                    if rotate:
                        poly = unrotate_points_180(poly, cv_img.shape[1], cv_img.shape[0])
                    polygons.append(poly)
                    if det.class_id == GATE_FRONT_CLASS_ID:
                        area = cv2.contourArea(poly)
                        if area > max_pipe_area and area >= self._line_contour_min_area:
                            max_pipe_area = area
                            best_pipe_poly = poly
                else:
                    polygons.append(None)
                if poly is not None and hasattr(det, 'mask_x'):
                    det.mask_x = [float(point[0]) for point in poly]
                    det.mask_y = [float(point[1]) for point in poly]
                self._set_gate_feature(det, poly, cv_img)
                det_array.detections.append(det)

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

    @staticmethod
    def _segmentation_center(poly):
        """Return the center of an oriented segmentation envelope.

        A gate mask is usually a hollow frame rather than a solid object.  Its
        centroid can move when one pipe is occluded, while the center of the
        oriented envelope is a better approximation of the opening center.
        """
        if poly is None:
            return None
        points = np.asarray(poly, dtype=np.float32).reshape(-1, 2)
        if len(points) < 4 or not np.all(np.isfinite(points)):
            return None
        center, size, _ = cv2.minAreaRect(points)
        if (not np.all(np.isfinite(center))
                or min(float(size[0]), float(size[1])) < 2.0):
            return None
        return float(center[0]), float(center[1])

    @staticmethod
    def _centerline_from_red_pipes(cv_img, bbox):
        """Estimate the gate opening center from visible red frame pipes.

        The current checkpoint is a detection model, not a segmentation model,
        so this lightweight image cue is the useful fallback in ``auto`` mode.
        It is deliberately restricted to the predicted gate bbox and requires
        red pixels spanning both axes; a single red bar is not promoted to a
        geometric anchor and falls back to the bbox center.
        """
        x1, y1, x2, y2 = (float(value) for value in bbox)
        height, width = cv_img.shape[:2]
        ix1 = max(0, min(width - 1, int(np.floor(x1))))
        iy1 = max(0, min(height - 1, int(np.floor(y1))))
        ix2 = max(ix1 + 1, min(width, int(np.ceil(x2))))
        iy2 = max(iy1 + 1, min(height, int(np.ceil(y2))))
        roi = cv_img[iy1:iy2, ix1:ix2]
        if roi.size == 0:
            return None
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_red = cv2.inRange(
            hsv, np.array([0, 45, 30], dtype=np.uint8),
            np.array([18, 255, 255], dtype=np.uint8))
        upper_red = cv2.inRange(
            hsv, np.array([165, 45, 30], dtype=np.uint8),
            np.array([180, 255, 255], dtype=np.uint8))
        mask = cv2.bitwise_or(lower_red, upper_red)
        kernel = np.ones((3, 3), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        ys, xs = np.where(mask > 0)
        if len(xs) < 24:
            return None
        x_span = float(np.percentile(xs, 95) - np.percentile(xs, 5))
        y_span = float(np.percentile(ys, 95) - np.percentile(ys, 5))
        roi_width = max(float(ix2 - ix1), 1.0)
        roi_height = max(float(iy2 - iy1), 1.0)
        if (x_span < max(8.0, 0.25 * roi_width)
                or y_span < max(8.0, 0.25 * roi_height)):
            return None
        points = np.column_stack((xs + ix1, ys + iy1)).astype(np.float32)
        center, size, _ = cv2.minAreaRect(points)
        if (not np.all(np.isfinite(center))
                or min(float(size[0]), float(size[1])) < 2.0):
            return None
        return float(center[0]), float(center[1])

    def _set_gate_feature(self, detection, polygon, cv_img):
        """Attach one repeatable gate anchor without changing bbox fields."""
        if (int(detection.class_id) != GATE_FRONT_CLASS_ID
                or self._gate_feature_mode == 'bbox'):
            return
        feature = None
        feature_type = FEATURE_BBOX_CENTER
        if self._gate_feature_mode in {'auto', 'segmentation'}:
            feature = self._segmentation_center(polygon)
            if feature is not None:
                feature_type = FEATURE_GATE_SEGMENTATION
        if feature is None and self._gate_feature_mode in {
                'auto', 'centerline'}:
            feature = self._centerline_from_red_pipes(
                cv_img,
                (detection.bbox_x1, detection.bbox_y1,
                 detection.bbox_x2, detection.bbox_y2))
            if feature is not None:
                feature_type = FEATURE_GATE_CENTERLINE
        if feature is None or not np.all(np.isfinite(feature)):
            return
        if hasattr(detection, 'feature_type'):
            detection.feature_type = int(feature_type)
            detection.feature_pixel_x = float(feature[0])
            detection.feature_pixel_y = float(feature[1])

    def _draw_boxes(self, cv_img, det_array, polygons, line_state, debug_info):
        import math
        annotated = cv_img.copy()
        overlay = cv_img.copy()
        h, w = cv_img.shape[:2]
        for i, det in enumerate(det_array.detections):
            x1, y1 = int(det.bbox_x1), int(det.bbox_y1)
            x2, y2 = int(det.bbox_x2), int(det.bbox_y2)
            color = ((0, 165, 255) if det.class_id == GATE_FRONT_CLASS_ID
                     else (0, 255, 0))
            if (det.class_id == GATE_FRONT_CLASS_ID and polygons
                    and i < len(polygons) and polygons[i] is not None):
                poly = polygons[i].astype(np.int32)
                cv2.fillPoly(overlay, [poly], color)
                cv2.polylines(annotated, [poly], True, color, 2)
            else:
                cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            feature_type = int(getattr(det, 'feature_type', 0))
            feature_x = float(getattr(det, 'feature_pixel_x', 0.0))
            feature_y = float(getattr(det, 'feature_pixel_y', 0.0))
            if (feature_type > FEATURE_BBOX_CENTER
                    and np.isfinite(feature_x) and np.isfinite(feature_y)):
                cv2.circle(annotated, (int(round(feature_x)), int(round(feature_y))),
                           6, (255, 0, 255), -1)
                cv2.putText(annotated, 'anchor',
                            (int(round(feature_x)) + 8,
                             int(round(feature_y)) - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                            (255, 0, 255), 1, cv2.LINE_AA)
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

    def _save_frame(self, img, channel, header, stereo_pair_id):
        if self._dataset_recorder is not None:
            return self._dataset_recorder.submit(
                img, channel, header=header, stereo_pair_id=stereo_pair_id)
        return False

    def fail_dataset_recording(self, reason):
        """Fail the active dataset session from a sensor or node error."""
        if self._dataset_recorder is not None:
            self._dataset_recorder.fail(str(reason))

    def shutdown(self):
        self._aruco_stop.set()
        if self._aruco_thread is not None:
            self._aruco_thread.join(timeout=1.0)
        if self._dataset_recorder is not None:
            try:
                self._dataset_recorder.close()
            except Exception as error:
                self.node.get_logger().error(
                    f'Failed to close dataset recorder: {error}')
