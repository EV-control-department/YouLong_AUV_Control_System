"""九宫格静态目标建图任务。

YOLO 只负责给出分割掩膜和类别；距离由左右相机图像的 SGBM 视差得到。
掩膜内的深度采用直方图主峰，随后用静态目标卡尔曼滤波器融合多次测量。
"""

from collections import deque
import json
import math
import threading
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from uv_camera.object_localizer import StereoCalibration
from uv_msgs.action import BasicMotion
from uv_msgs.msg import DetectionArray, PoseInfo


def _rpy_matrix(roll, pitch, yaw):
    roll, pitch, yaw = (math.radians(float(value))
                        for value in (roll, pitch, yaw))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=float)


def _stamp(message):
    stamp = message.header.stamp if hasattr(message, 'header') else message.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class StaticPositionFilter:
    """静态物体模型：位置不变，过程噪声只吸收艇位和模型误差。"""

    def __init__(self, position, covariance, timestamp):
        self.position = np.asarray(position, dtype=float)
        self.covariance = np.asarray(covariance, dtype=float)
        self.timestamp = float(timestamp)
        self.observations = 1
        self.accepted = 1

    def update(self, position, covariance, timestamp, process_noise, gate):
        timestamp = float(timestamp)
        if timestamp <= self.timestamp:
            return False, 'out_of_order'
        dt = min(timestamp - self.timestamp, 2.0)
        prediction_covariance = self.covariance + np.eye(3) * process_noise * dt
        residual = np.asarray(position, dtype=float) - self.position
        innovation = prediction_covariance + np.asarray(covariance, dtype=float)
        try:
            mahalanobis = float(residual @ np.linalg.solve(innovation, residual))
        except np.linalg.LinAlgError:
            return False, 'singular_covariance'
        if not math.isfinite(mahalanobis) or mahalanobis > gate:
            return False, f'outlier:{mahalanobis:.3f}'
        gain = prediction_covariance @ np.linalg.inv(innovation)
        self.position += gain @ residual
        remainder = np.eye(3) - gain
        self.covariance = (remainder @ prediction_covariance @ remainder.T
                           + gain @ covariance @ gain.T)
        self.covariance = (self.covariance + self.covariance.T) / 2.0
        self.timestamp = timestamp
        self.observations += 1
        self.accepted += 1
        return True, 'accepted'


class MappingTask:
    """访问已知九宫格并输出方形/圆形锥桶的地图。"""

    def __init__(self, node, params):
        self.node = node
        self.params = params
        self.lock = threading.RLock()
        self.bridge = CvBridge()
        self.subscriptions = []
        self.pose_history = deque(maxlen=300)
        self.image_history = deque(maxlen=12)
        self.left_detections = deque(maxlen=24)
        self.right_detections = deque(maxlen=24)
        self.camera_info = {}
        self.calibration = None
        self.rectify_maps = None
        self.filters = {}
        self.class_votes = {}
        self.visit_order = []
        self.observation_failures = []
        self.cell_observations = {}
        self.current_cell = None
        self.state = 'initializing'
        self.last_detection_stamp = -1.0
        self.perception_stop = threading.Event()
        self.perception_thread = None
        self._next_perception_error_log = 0.0
        self.perception_stats = {
            'detection_pairs': 0,
            'image_unavailable': 0,
            'pose_unavailable': 0,
            'processed_pairs': 0,
        }
        self.tag_scan_stats = {'frames': 0, 'markers': 0, 'wrong_id': 0,
                               'no_depth': 0}
        self.last_tag_reason = 'not_scanned'
        self.deadline = time.monotonic() + float(params['timeout'])

        self.grid_center = np.array([
            float(params['grid_center_x']), float(params['grid_center_y']),
            float(params['floor_z']),
        ])
        self.grid_rotation = _rpy_matrix(0, 0, params['grid_yaw_deg'])
        self.grid_centers = self._make_grid_centers()
        # Keep raw world-space measurements for DDS visualization and offline
        # parameter tuning. The filtered track remains task-authoritative.
        self.measurement_points = {
            index: deque(maxlen=120) for index in self.grid_centers
        }
        self.camera_translation = np.asarray(params['left_translation'], dtype=float)
        self.right_translation = np.asarray(params['right_translation'], dtype=float)
        self.camera_rotation = np.asarray(
            params['camera_rotation'], dtype=float).reshape(3, 3)

        qos_map = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = node.create_publisher(String, '/task/mapping/map', qos_map)
        self.event_pub = node.create_publisher(String, '/task/mapping/events', 100)
        self._subscribe(PoseInfo, '/basic_motion/pose_info', self._pose_cb)
        self._subscribe(Image, params['image_topic'], self._image_cb)
        self._subscribe(DetectionArray, params['left_detection_topic'],
                        self._left_detection_cb)
        self._subscribe(DetectionArray, params['right_detection_topic'],
                        self._right_detection_cb)
        self._subscribe(CameraInfo, params['left_info_topic'],
                        lambda message: self._info_cb('left', message))
        self._subscribe(CameraInfo, params['right_info_topic'],
                        lambda message: self._info_cb('right', message))
        self.publish_timer = node.create_timer(1.0, self.publish_map)

        if not hasattr(cv2, 'aruco'):
            raise RuntimeError('OpenCV ArUco module is required for the tag trigger')
        dictionary_id = getattr(cv2.aruco, params['tag_dictionary'], None)
        if dictionary_id is None:
            raise ValueError(f'unknown ArUco dictionary: {params["tag_dictionary"]}')
        self.tag_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        # OpenCV 4.7+ moved marker detection to ArucoDetector.  Ubuntu/ROS
        # images with the contrib package expose either API, so keep the task
        # compatible with both without changing its public interface.
        self.tag_detector = None
        detector_type = getattr(cv2.aruco, 'ArucoDetector', None)
        if detector_type is not None:
            parameters_type = getattr(cv2.aruco, 'DetectorParameters', None)
            parameters = parameters_type() if parameters_type is not None else None
            self.tag_detector = (detector_type(self.tag_dictionary, parameters)
                                 if parameters is not None else
                                 detector_type(self.tag_dictionary))
        self.tag_detectors = [(params['tag_dictionary'], self.tag_detector)]

    def _subscribe(self, message_type, topic, callback):
        self.subscriptions.append(self.node.create_subscription(
            message_type, topic, callback, qos_profile_sensor_data))

    def _make_grid_centers(self):
        spacing = float(self.params['grid_side_m']) / 3.0
        return {
            row * 3 + column: self.grid_center + self.grid_rotation @ np.array([
                (1 - row) * spacing, (column - 1) * spacing, 0.0])
            for row in range(3) for column in range(3)
        }

    def _pose_cb(self, message):
        with self.lock:
            self.pose_history.append(message)

    def _image_cb(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            width = image.shape[1] // 2
            if width < 2:
                return
            with self.lock:
                self.image_history.append((_stamp(message),
                                           image[:, :width].copy(),
                                           image[:, width:].copy()))
        except (cv2.error, ValueError) as error:
            self.node.get_logger().warning(f'mapping image rejected: {error}')

    def _left_detection_cb(self, message):
        with self.lock:
            self.left_detections.append(message)

    def _right_detection_cb(self, message):
        with self.lock:
            self.right_detections.append(message)

    def _info_cb(self, side, message):
        with self.lock:
            self.camera_info[side] = message

    def _emit(self, event, **values):
        payload = {
            'schema_version': 1,
            'event': event,
            'state': self.state,
            'cell': self.current_cell,
            'stamp': self.node.get_clock().now().nanoseconds * 1e-9,
            **values,
        }
        self.event_pub.publish(String(data=json.dumps(payload, allow_nan=False)))
        if event in ('frame_rejected', 'measurement_rejected'):
            now = time.monotonic()
            if now >= getattr(self, '_next_rejection_log', 0.0):
                self.node.get_logger().warning(
                    f'建图数据被拒绝：格点={self.current_cell}，原因={values.get("reason")}')
                self._next_rejection_log = now + 3.0

    def publish_map(self):
        with self.lock:
            cells = []
            for index, center in self.grid_centers.items():
                track = self.filters.get(index)
                votes = self.class_votes.get(index, [0, 0])
                total = sum(votes)
                label = None
                if total and max(votes) / total >= float(self.params['class_vote_ratio']):
                    label = 'square_cone' if votes[0] >= votes[1] else 'round_cone'
                cells.append({
                    'id': index,
                    'row': index // 3,
                    'column': index % 3,
                    'center': center.tolist(),
                    'visited': index in self.visit_order,
                    'label': label,
                    'class_id': int(np.argmax(votes)) if label else None,
                    'position': track.position.tolist() if track else None,
                    'covariance': track.covariance.tolist() if track else None,
                    'observations': track.observations if track else 0,
                    'accepted_observations': track.accepted if track else 0,
                    'votes': votes,
                    'observation': self.cell_observations.get(index),
                    'residual': (track.position - center).tolist() if track else None,
                    'measurements': list(self.measurement_points.get(index, ())),
                })
            payload = {
                'schema_version': 2,
                'frame_id': 'mapping_odom',
                'state': self.state,
                'stamp': self.node.get_clock().now().nanoseconds * 1e-9,
                'grid': {
                    'center': self.grid_center.tolist(),
                    'side_m': float(self.params['grid_side_m']),
                    'yaw_deg': float(self.params['grid_yaw_deg']),
                    'floor_z': float(self.params['floor_z']),
                },
                'tag': self._tag_json(),
                'cells': cells,
                'visit_order': list(self.visit_order),
                'observation_failures': list(self.observation_failures),
                'all_cells_visited': len(self.visit_order) == len(self.grid_centers),
                'measurement_count': sum(
                    len(points) for points in self.measurement_points.values()),
            }
        self.map_pub.publish(String(data=json.dumps(payload, allow_nan=False)))

    def _tag_json(self):
        if not hasattr(self, 'tag_filter') or self.tag_filter is None:
            return None
        return {
            'id': self.tag_id,
            'position': self.tag_filter.position.tolist(),
            'covariance': self.tag_filter.covariance.tolist(),
            'observations': self.tag_filter.observations,
        }

    def _ready(self):
        return rclpy.ok() and not self.node.stopped and time.monotonic() < self.deadline

    def _pose_for(self, timestamp):
        with self.lock:
            poses = list(self.pose_history)
        if not poses:
            raise ValueError('pose is unavailable')
        pose = min(poses, key=lambda item: abs(_stamp(item) - timestamp))
        delta = abs(_stamp(pose) - timestamp)
        strict_slop = float(self.params['pose_slop_s'])
        fallback_slop = max(
            strict_slop,
            5.0 if self.state in ('reading_tag', 'observe_cell') else 1.0)
        if delta > fallback_slop:
            raise ValueError('no pose close enough to image timestamp')
        if (delta > strict_slop
                and self.state in ('reading_tag', 'observe_cell')):
            now = time.monotonic()
            if now >= getattr(self, '_next_pose_fallback_log', 0.0):
                self.node.get_logger().warning(
                    f'位姿时间戳未严格同步，使用最近位姿：时间差={delta:.3f}s，'
                    f'严格阈值={strict_slop:.3f}s')
                self._next_pose_fallback_log = now + 3.0
        return pose

    def _prepare_calibration(self):
        with self.lock:
            infos = dict(self.camera_info)
        if set(infos) != {'left', 'right'}:
            return False
        self.calibration = StereoCalibration.from_camera_info(
            'down_mapping', infos['left'], infos['right'],
            self.camera_translation, self.camera_rotation,
            self.right_translation, self.camera_rotation)
        size = (int(infos['left'].width), int(infos['left'].height))
        left_map = cv2.initUndistortRectifyMap(
            self.calibration.camera_matrix_left, self.calibration.dist_left,
            self.calibration.rectification_left,
            self.calibration.projection_left[:, :3], size, cv2.CV_32FC1)
        right_map = cv2.initUndistortRectifyMap(
            self.calibration.camera_matrix_right, self.calibration.dist_right,
            self.calibration.rectification_right,
            self.calibration.projection_right[:, :3], size, cv2.CV_32FC1)
        self.rectify_maps = (left_map, right_map)
        self.sgbm = cv2.StereoSGBM_create(
            minDisparity=int(self.params['sgbm_min_disparity']),
            numDisparities=int(self.params['sgbm_num_disparities']),
            blockSize=int(self.params['sgbm_block_size']),
            P1=8 * 1 * int(self.params['sgbm_block_size']) ** 2,
            P2=32 * 1 * int(self.params['sgbm_block_size']) ** 2,
            disp12MaxDiff=1, uniquenessRatio=8,
            speckleWindowSize=80, speckleRange=2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        return True

    def _image_for(self, timestamp):
        with self.lock:
            if not self.image_history:
                return None
            candidate = min(self.image_history,
                            key=lambda item: abs(item[0] - timestamp))
        delta = abs(candidate[0] - timestamp)
        strict_slop = float(self.params['image_slop_s'])
        # Detection and stitched-image messages are produced by different
        # callbacks.  At a low simulation/render rate, the nearest image can
        # arrive later than the strict synchronization window.  The vehicle
        # is stationary during cell observation, so a bounded fallback is
        # valid and prevents a detection from being discarded unnecessarily.
        fallback_slop = max(
            strict_slop,
            5.0 if self.state in ('reading_tag', 'observe_cell') else 1.0)
        if delta > fallback_slop:
            return None
        if delta > strict_slop:
            now = time.monotonic()
            if now >= getattr(self, '_next_image_fallback_log', 0.0):
                self.node.get_logger().warning(
                    f'图像时间戳未严格同步，使用最近帧：时间差={delta:.3f}s，'
                    f'严格阈值={strict_slop:.3f}s')
                self._next_image_fallback_log = now + 3.0
        left, right = candidate[1], candidate[2]
        left = cv2.remap(left, *self.rectify_maps[0], cv2.INTER_LINEAR)
        right = cv2.remap(right, *self.rectify_maps[1], cv2.INTER_LINEAR)
        return left, right

    def _detection_pair(self):
        with self.lock:
            left_messages = list(self.left_detections)
            right_messages = list(self.right_detections)
        for left_message in reversed(left_messages):
            timestamp = _stamp(left_message)
            if timestamp <= self.last_detection_stamp:
                continue
            candidates = []
            for right_message in right_messages:
                left_id = int(getattr(left_message, 'stereo_pair_id', 0) or 0)
                right_id = int(getattr(right_message, 'stereo_pair_id', 0) or 0)
                # The simulator carries the same pair ID even when the two
                # camera headers differ by roughly 100 ms.  Pair ID is the
                # authoritative association; timestamp is the fallback for
                # legacy publishers that do not provide it.
                if left_id and right_id:
                    if left_id != right_id:
                        continue
                elif (abs(_stamp(right_message) - timestamp)
                      > float(self.params['detection_slop_s'])):
                    continue
                candidates.append(right_message)
            if candidates:
                right_message = min(candidates,
                                    key=lambda item: abs(_stamp(item) - timestamp))
                return left_message, right_message, timestamp
        return None

    def _record_observation_frame(self):
        """记录当前格点收到了一帧可用于定位的同步感知数据。"""
        if self.state != 'observe_cell' or self.current_cell is None:
            return
        with self.lock:
            observation = self.cell_observations.setdefault(
                self.current_cell,
                {'synchronized_frames': 0, 'valid_measurements': 0})
            observation['synchronized_frames'] += 1

    def _record_observation_measurement(self, cell):
        """把本次有效深度测量记入其理论格点。"""
        with self.lock:
            observation = self.cell_observations.setdefault(
                int(cell),
                {'synchronized_frames': 0, 'valid_measurements': 0})
            observation['valid_measurements'] += 1

    def _depth_map(self, left, right):
        left_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
        disparity = self.sgbm.compute(left_gray, right_gray).astype(np.float32) / 16.0
        depth = np.full(disparity.shape, np.nan, dtype=np.float32)
        valid = disparity > float(self.params['min_disparity'])
        depth[valid] = (self.calibration.projection_left[0, 0]
                        * self.calibration.baseline_m / disparity[valid])
        return disparity, depth

    def _mask_depth_mode(self, detection, depth):
        xs = np.asarray(getattr(detection, 'mask_x', []), dtype=float)
        ys = np.asarray(getattr(detection, 'mask_y', []), dtype=float)
        if len(xs) < 3 or len(xs) != len(ys):
            return None
        polygon = np.column_stack((xs, ys)).astype(np.int32)
        mask = np.zeros(depth.shape, dtype=np.uint8)
        cv2.fillPoly(mask, [polygon], 1)
        values = depth[mask.astype(bool)]
        values = values[np.isfinite(values)]
        values = values[(values >= float(self.params['min_depth_m']))
                        & (values <= float(self.params['max_depth_m']))]
        if len(values) < int(self.params['min_depth_points']):
            return None
        bin_size = float(self.params['depth_bin_m'])
        bins = np.arange(float(self.params['min_depth_m']),
                         float(self.params['max_depth_m']) + bin_size,
                         bin_size)
        counts, edges = np.histogram(values, bins=bins)
        peak = int(np.argmax(counts))
        selected = values[(values >= edges[peak]) & (values < edges[peak + 1])]
        if len(selected) < max(int(self.params['min_depth_points']),
                               int(len(values) * self.params['depth_peak_ratio'])):
            return None
        center = np.median(np.column_stack(np.where(mask > 0)), axis=0)
        return float(np.median(selected)), (float(center[1]), float(center[0])), int(len(selected))

    def _world_measurement(self, pixel, depth, pose, rectified=False):
        rectified_pixel = (np.asarray(pixel, dtype=float) if rectified else
                           self.calibration.rectified_pixel(
                               'left', np.asarray(pixel, dtype=float)))
        fx = self.calibration.projection_left[0, 0]
        fy = self.calibration.projection_left[1, 1]
        cx = self.calibration.projection_left[0, 2]
        cy = self.calibration.projection_left[1, 2]
        point_rectified = np.array([
            (rectified_pixel[0] - cx) * depth / fx,
            (rectified_pixel[1] - cy) * depth / fy,
            depth,
        ])
        point_optical = self.calibration.rectification_left.T @ point_rectified
        body_rotation = _rpy_matrix(pose.robot_roll, pose.robot_pitch, pose.robot_yaw)
        body_position = np.array([pose.robot_x, pose.robot_y, pose.robot_z])
        return body_position + body_rotation @ (
            self.camera_translation + self.camera_rotation @ point_optical)

    def _process_cone_pair(self, left_message, right_message, timestamp,
                           image_pair, pose):
        """处理一对已经完成时间配对的左右检测。"""
        _, depth = self._depth_map(*image_pair)
        self._record_observation_frame()
        right_detections = [d for d in right_message.detections
                            if int(d.class_id) in (0, 1)]
        measurements = 0
        for left_detection in left_message.detections:
            class_id = int(left_detection.class_id)
            if (class_id not in (0, 1)
                    or float(left_detection.confidence)
                    < float(self.params['min_confidence'])):
                continue
            if not any(int(item.class_id) == class_id for item in right_detections):
                continue
            result = self._mask_depth_mode(left_detection, depth)
            if result is None:
                self._emit('measurement_rejected', reason='no_depth_mode', class_id=class_id,
                           measurement_stamp=timestamp)
                continue
            distance, pixel, sample_count = result
            point = self._world_measurement(pixel, distance, pose)
            cell = min(self.grid_centers,
                       key=lambda index: np.linalg.norm(
                           point[:2] - self.grid_centers[index][:2]))
            residual = float(np.linalg.norm(point[:2] - self.grid_centers[cell][:2]))
            point_record = {
                'position': point.tolist(),
                'class_id': class_id,
                'confidence': float(left_detection.confidence),
                'depth_m': distance,
                'residual_m': residual,
                'accepted': False,
                'timestamp': float(timestamp),
            }
            with self.lock:
                self.measurement_points[cell].append(point_record)
            if residual > float(self.params['cell_gate_m']):
                self._emit('measurement_rejected', reason='outside_cell_gate', class_id=class_id,
                           position=point.tolist(), residual_m=residual,
                           measurement_stamp=timestamp)
                continue
            covariance = np.eye(3) * float(self.params['measurement_sigma_m']) ** 2
            covariance[2, 2] *= 2.0
            with self.lock:
                track = self.filters.get(cell)
                if track is None:
                    self.filters[cell] = StaticPositionFilter(point, covariance, timestamp)
                    self.class_votes[cell] = [0, 0]
                    accepted, reason = True, 'initial'
                else:
                    accepted, reason = track.update(
                        point, covariance, timestamp,
                        float(self.params['process_noise']),
                        float(self.params['mahalanobis_gate']))
                if accepted:
                    self.class_votes[cell][class_id] += 1
                    point_record['accepted'] = True
            self._record_observation_measurement(cell)
            self._emit('cone_measurement', cell=cell, class_id=class_id,
                       confidence=float(left_detection.confidence), depth_m=distance,
                       depth_samples=sample_count, position=point.tolist(),
                       residual_m=residual, accepted=accepted, reason=reason,
                       measurement_stamp=timestamp)
            measurements += 1
        return measurements

    def _observe_cones(self):
        """等待后台感知线程累计当前格点的观测，不再主动取帧。"""
        cell = self.current_cell
        with self.lock:
            start = dict(self.cell_observations.get(
                cell, {'synchronized_frames': 0, 'valid_measurements': 0}))
        end = min(self.deadline, time.monotonic() + float(self.params['observe_seconds']))
        while self._ready() and time.monotonic() < end:
            time.sleep(0.05)
        with self.lock:
            current = dict(self.cell_observations.get(cell, start))
        observed_frames = current['synchronized_frames'] - start['synchronized_frames']
        measurements = current['valid_measurements'] - start['valid_measurements']
        self.node.get_logger().info(
            f'格点{cell}观察完成：后台同步帧={observed_frames}，'
            f'新增有效目标测量={measurements}，累计有效目标测量={current["valid_measurements"]}；'
            f'感知统计={self.perception_stats}')
        return observed_frames >= int(self.params['min_observations'])

    def _tag_measurement(self, image_pair, pose):
        left, _ = image_pair
        gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        self.tag_scan_stats['frames'] += 1
        self.last_tag_reason = '未发现可解码标记'
        # Rendering, water contrast and the stitched image can make the
        # native detector sensitive to scale.  Keep the original image and a
        # contrast-normalized variant; do not use annotated frames here.
        variants = (gray, cv2.createCLAHE(clipLimit=2.0,
                                          tileGridSize=(8, 8)).apply(gray))
        for family, detector in self.tag_detectors:
            for variant in variants:
                corners, ids, _ = (detector.detectMarkers(variant) if detector is not None
                                   else cv2.aruco.detectMarkers(variant, self.tag_dictionary))
                if ids is None:
                    continue
                self.tag_scan_stats['markers'] += len(ids)
                for polygon, tag_id in zip(corners, ids.flatten()):
                    tag_id = int(tag_id)
                    if int(self.params['tag_id']) >= 0 and tag_id != int(self.params['tag_id']):
                        self.tag_scan_stats['wrong_id'] += 1
                        self.last_tag_reason = f'{family}:wrong_id:{tag_id}'
                        continue
                    detection = type('TagMask', (), {
                        'mask_x': polygon.reshape(-1, 2)[:, 0].tolist(),
                        'mask_y': polygon.reshape(-1, 2)[:, 1].tolist(),
                    })()
                    result = self._mask_depth_mode(
                        detection, self._depth_map(*image_pair)[1])
                    if result is None:
                        self.tag_scan_stats['no_depth'] += 1
                        self.last_tag_reason = f'{family}:no_depth_mode'
                        continue
                    distance, pixel, sample_count = result
                    self.last_tag_reason = f'{family}:accepted'
                    return (tag_id, self._world_measurement(pixel, distance, pose, rectified=True),
                            sample_count)
        if self.tag_scan_stats['markers'] == 0:
            self.last_tag_reason = 'no_marker'
        return None

    def _accept_tag_result(self, result, timestamp):
        """将后台线程得到的 AprilTag 测量写入静态滤波器。"""
        if result is None:
            return False
        tag_id, point, sample_count = result
        with self.lock:
            if hasattr(self, 'tag_filter') and tag_id != self.tag_id:
                self.last_tag_reason = '标记ID改变，拒绝合并不同目标'
                return False
            covariance = np.eye(3) * float(self.params['measurement_sigma_m']) ** 2
            if not hasattr(self, 'tag_filter'):
                self.tag_id = tag_id
                self.tag_filter = StaticPositionFilter(point, covariance, timestamp)
                accepted, reason = True, 'initial'
            else:
                accepted, reason = self.tag_filter.update(
                    point, covariance, timestamp,
                    float(self.params['process_noise']),
                    float(self.params['mahalanobis_gate']))
            observations = self.tag_filter.observations
        self._emit('tag_measurement', tag_id=tag_id, position=point.tolist(),
                   depth_samples=sample_count, accepted=accepted, reason=reason,
                   measurement_stamp=timestamp)
        if observations >= int(self.params['min_observations']):
            self.node.get_logger().info(
                f'标记确认成功：ID={tag_id}，有效观测={observations}')
        return accepted

    def _perception_loop(self):
        """后台持续处理最新图像、AprilTag 和左右目分割检测。"""
        last_image_processed = -1.0
        self.node.get_logger().info('建图后台感知线程已启动：视觉与WTRAVEL并行运行')
        while self._ready() and not self.perception_stop.is_set():
            did_work = False
            with self.lock:
                images = list(self.image_history)
            for timestamp, _, _ in sorted(images, key=lambda item: item[0]):
                if timestamp <= last_image_processed:
                    continue
                try:
                    pose = self._pose_for(timestamp)
                    image_pair = self._image_for(timestamp)
                except (ValueError, cv2.error) as error:
                    self.last_tag_reason = f'图像或位姿无效：{error}'
                    continue
                if image_pair is None:
                    # 图像时间配对可能在下一次回调才成立，暂不消费该帧。
                    continue
                last_image_processed = timestamp
                try:
                    self._accept_tag_result(
                        self._tag_measurement(image_pair, pose), timestamp)
                except (ValueError, cv2.error) as error:
                    self.last_tag_reason = f'标记处理失败：{error}'
                did_work = True

            pair = self._detection_pair()
            if pair is not None:
                self.perception_stats['detection_pairs'] += 1
                left_message, right_message, timestamp = pair
                image_pair = self._image_for(timestamp)
                if image_pair is None:
                    self.perception_stats['image_unavailable'] += 1
                    self._emit('frame_rejected', reason='image timestamp unavailable',
                               measurement_stamp=timestamp)
                else:
                    try:
                        pose = self._pose_for(timestamp)
                        self.last_detection_stamp = timestamp
                        self._process_cone_pair(
                            left_message, right_message, timestamp, image_pair, pose)
                        self.perception_stats['processed_pairs'] += 1
                    except (ValueError, cv2.error) as error:
                        if 'pose' in str(error):
                            self.perception_stats['pose_unavailable'] += 1
                        self.last_detection_stamp = timestamp
                        self._emit('frame_rejected', reason=str(error),
                                   measurement_stamp=timestamp)
                    did_work = True
            if not did_work:
                self.perception_stop.wait(0.03)
        self.node.get_logger().info('建图后台感知线程已停止')

    def _start_perception_worker(self):
        if self.perception_thread is not None and self.perception_thread.is_alive():
            return
        self.perception_stop.clear()
        self.perception_thread = threading.Thread(
            target=self._perception_loop, name='mapping-perception', daemon=True)
        self.perception_thread.start()

    def _stop_perception_worker(self):
        self.perception_stop.set()
        if self.perception_thread is not None:
            self.perception_thread.join(timeout=2.0)
            self.perception_thread = None

    def _read_tag(self):
        self.state = 'reading_tag'
        self.node.get_logger().info(
            f'开始识别池底标记：字典={self.params["tag_dictionary"]}，'
            f'期望ID={self.params["tag_id"]}，图像={self.params["image_topic"]}；'
            f'代码路径={__file__}')
        next_log = time.monotonic()
        end = min(self.deadline, time.monotonic() + float(self.params['tag_timeout']))
        while self._ready() and time.monotonic() < end:
            with self.lock:
                tag_filter = getattr(self, 'tag_filter', None)
                observations = tag_filter.observations if tag_filter else 0
            if observations >= int(self.params['min_observations']):
                return True
            if time.monotonic() >= next_log:
                self.node.get_logger().info(
                    f'标记识别进度：{self.tag_scan_stats}，最近原因={self.last_tag_reason}')
                next_log = time.monotonic() + 3.0
            time.sleep(0.05)
        self.node.get_logger().error(
            'AprilTag 识别超时: frames=%d markers=%d wrong_id=%d '
            'no_depth=%d last=%s expected_id=%s families=%s' % (
                self.tag_scan_stats['frames'], self.tag_scan_stats['markers'],
                self.tag_scan_stats['wrong_id'], self.tag_scan_stats['no_depth'],
                self.last_tag_reason, self.params['tag_id'],
                ','.join(name for name, _ in self.tag_detectors)))
        return False

    def _travel_to(self, position, label):
        if not self._ready():
            return False
        timeout = min(float(self.params['move_timeout']),
                      max(1.0, self.deadline - time.monotonic()))
        success, message = self.node._send_action_goal(
            BasicMotion.Goal.WTRAVEL,
            [float(position[0]), float(position[1]), float(self.params['survey_z']),
             float(self.params['survey_yaw_deg'])],
            'xyzrz', timeout=timeout,
            task_context=self.node._format_motion_context(label))
        self._emit('travel_result', label=label, success=success,
                   message=message, target=list(map(float, position)))
        return success

    def execute(self):
        self._emit('started', model='best.pt', class_map={0: 'square_cone', 1: 'round_cone'},
                   depth_method='sgbm_mask_mode')
        try:
            ready_end = min(self.deadline, time.monotonic() + 20.0)
            while self._ready() and time.monotonic() < ready_end:
                with self.lock:
                    ready = len(self.camera_info) == 2 and bool(self.pose_history)
                if ready and self._prepare_calibration():
                    break
                time.sleep(0.05)
            if self.calibration is None:
                raise RuntimeError('camera calibration or pose unavailable')
            self._emit('calibration_ready', baseline_m=self.calibration.baseline_m)
            self._start_perception_worker()
            self.state = 'travel_to_tag'
            if not self._travel_to(
                    [self.params['tag_x'], self.params['tag_y']], 'mapping:travel_to_april_tag'):
                raise RuntimeError('WTRAVEL to tag failed')
            if not self._read_tag():
                raise RuntimeError('AprilTag/ArUco trigger was not confirmed')
            for cell in tuple(self.params['visit_order']):
                self.current_cell = int(cell)
                self.state = 'travel_to_cell'
                center = self.grid_centers[self.current_cell]
                if not self._travel_to(center[:2], f'mapping:travel_to_cell_{cell}'):
                    raise RuntimeError(f'WTRAVEL to cell {cell} failed')
                self.state = 'observe_cell'
                observation_ok = self._observe_cones()
                self.visit_order.append(self.current_cell)
                observation = self.cell_observations.get(self.current_cell, {})
                if not observation_ok:
                    self.observation_failures.append(self.current_cell)
                    self.node.get_logger().warning(
                        f'格点{cell}未获得足够有效观测，记录为空格并继续检索九宫格：'
                        f'同步帧={observation.get("synchronized_frames", 0)}，'
                        f'有效目标测量={observation.get("valid_measurements", 0)}')
                    self._emit(
                        'cell_completed', cell=self.current_cell,
                        observation_ok=False, **observation)
                else:
                    self._emit(
                        'cell_completed', cell=self.current_cell,
                        observation_ok=True, **observation)
                self.publish_map()
            confirmed = [index for index, votes in self.class_votes.items()
                         if sum(votes) >= int(self.params['min_observations'])
                         and max(votes) / sum(votes) >= float(self.params['class_vote_ratio'])]
            if len(confirmed) != int(self.params['expected_cones']):
                raise RuntimeError(
                    f'expected {self.params["expected_cones"]} cones, confirmed {len(confirmed)}')
            self.state = 'complete'
            self._emit('completed', confirmed_cells=confirmed)
            return True
        except Exception as error:
            self.state = 'stopped' if self.node.stopped else 'failed'
            self._emit('failed', reason=str(error))
            self.node.get_logger().error(f'mapping task failed: {error}')
            self.node.stopped = True
            return False
        finally:
            self._stop_perception_worker()
            self.publish_map()

    def destroy(self):
        self._stop_perception_worker()
        self.node.destroy_timer(self.publish_timer)
        for subscription in self.subscriptions:
            self.node.destroy_subscription(subscription)
