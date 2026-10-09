"""前视转盘视觉：YOLO 整盘掩膜、可选 HSV 黄标及双目 SGBM。"""

from collections import deque
import json
import math
from pathlib import Path
import threading
import time

import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from uv_msgs.msg import PoseInfo

from .object_localizer import StereoCalibration

EXPECTED_DISK_DIAMETER_M = 0.230


def _installed_extrinsics(left, right, rotation, upside_down):
    """在原始标定目序中计算倒装外参；前向光学z不反转。"""
    left, right = np.asarray(left, float), np.asarray(right, float)
    rotation = np.asarray(rotation, float).reshape(3, 3)
    if not upside_down:
        return left, right, rotation
    midpoint = (left + right) / 2
    return (2 * midpoint - left, 2 * midpoint - right,
            rotation @ np.diag([-1., -1., 1.]))


def _polygon(detection):
    xs = list(getattr(detection, 'mask_x', ()))
    ys = list(getattr(detection, 'mask_y', ()))
    if len(xs) < 3 or len(xs) != len(ys):
        return None
    points = np.column_stack((xs, ys)).astype(np.float32)
    return points if np.all(np.isfinite(points)) else None


def _centroid(points):
    moment = cv2.moments(points)
    if abs(moment['m00']) < 1e-6:
        return None
    return moment['m10'] / moment['m00'], moment['m01'] / moment['m00']


def _best(message, class_id, confidence):
    candidates = [d for d in getattr(message, 'detections', ())
                  if int(d.class_id) == class_id
                  and float(d.confidence) >= confidence
                  and _polygon(d) is not None]
    return max(candidates, key=lambda d: float(d.confidence), default=None)


def estimate(message, disk_class_id, label_class_id=-1, min_confidence=0.5,
             min_radial_fraction=0.15, min_axis_ratio=0.7, frame=None):
    """盘体与相位分别给出有效性；黄标未检出不等于盘体不存在。"""
    result = {'valid': False, 'phase_valid': False, 'reason': 'missing_disk_mask'}
    disk = _best(message, disk_class_id, min_confidence)
    if disk is None:
        return result
    disk_poly = _polygon(disk)
    if len(disk_poly) < 5:
        result['reason'] = 'disk_contour_too_short'
        return result
    try:
        (cx, cy), (diameter_a, diameter_b), _ = cv2.fitEllipse(disk_poly)
    except cv2.error:
        result['reason'] = 'disk_ellipse_fit_failed'
        return result
    major, minor = max(diameter_a, diameter_b), min(diameter_a, diameter_b)
    if major <= 0 or minor / major < min_axis_ratio:
        result['reason'] = 'disk_not_front_facing'
        return result
    rx = max(1.0, (float(np.max(disk_poly[:, 0])) - float(np.min(disk_poly[:, 0]))) / 2)
    ry = max(1.0, (float(np.max(disk_poly[:, 1])) - float(np.min(disk_poly[:, 1]))) / 2)
    result.update(valid=True, reason='ok', disk_center_px=[float(cx), float(cy)],
                  disk_radius_px=float((rx+ry)/2), axis_ratio=float(minor/major),
                  disk_major_px=float(major), confidence=float(disk.confidence))
    label = _best(message, label_class_id, min_confidence) if label_class_id >= 0 else None
    label_center = _centroid(_polygon(label)) if label is not None else None
    source = 'yolo' if label_center is not None else 'hsv'
    if label_center is None and frame is not None:
        mask = np.zeros(frame.shape[:2], np.uint8)
        cv2.fillPoly(mask, [disk_poly.astype(np.int32)], 255)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        yellow = cv2.bitwise_and(cv2.inRange(hsv, (16, 80, 65), (40, 255, 255)), mask)
        yellow = cv2.morphologyEx(yellow, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        minimum = max(8.0, cv2.contourArea(disk_poly)*0.002)
        candidates = [(cv2.contourArea(contour), _centroid(contour)) for contour in contours
                      if cv2.contourArea(contour) >= minimum]
        candidates = [(area, point) for area, point in candidates if point is not None
                      and 0.15 <= math.hypot((point[0]-cx)/rx, (point[1]-cy)/ry) <= 1.1]
        label_center = max(candidates, default=(0, None))[1]
    if label_center is None:
        result['phase_reason'] = 'yellow_marker_not_found'
        return result
    lu, lv = label_center
    dx, dy = (lu - cx) / rx, -(lv - cy) / ry
    radial = math.hypot(dx, dy)
    if not min_radial_fraction <= radial <= 1.15:
        result['phase_reason'] = 'label_not_on_radial_track'
        return result
    result.update(phase_valid=True, phase_source=source,
                  angle_deg=math.degrees(math.atan2(dy, dx)) % 360.0,
                  label_center_px=[float(lu), float(lv)], radial_fraction=float(radial))
    return result


def _sgbm_disk(left, right, polygon, calibration, matcher, black_limit, min_points,
               maps=None):
    """仅在盘体黑色内区拟合三维盘面；不拿黑色背景当深度。"""
    height, width = left.shape[:2]
    if maps is None:
        maps = [cv2.initUndistortRectifyMap(k, d, r, p[:, :3], (width, height),
                                            cv2.CV_32FC1)
                for k, d, r, p in (
                    (calibration.camera_matrix_left, calibration.dist_left,
                     calibration.rectification_left, calibration.projection_left),
                    (calibration.camera_matrix_right, calibration.dist_right,
                     calibration.rectification_right, calibration.projection_right))]
    gray_l = cv2.remap(cv2.cvtColor(left, cv2.COLOR_BGR2GRAY), *maps[0], cv2.INTER_LINEAR)
    gray_r = cv2.remap(cv2.cvtColor(right, cv2.COLOR_BGR2GRAY), *maps[1], cv2.INTER_LINEAR)
    disparity = matcher.compute(gray_l, gray_r).astype(np.float32)/16.0
    mask = np.zeros((height, width), np.uint8)
    cv2.fillPoly(mask, [polygon.astype(np.int32)], 255)
    mask = cv2.erode(cv2.remap(mask, *maps[0], cv2.INTER_NEAREST),
                     np.ones((7, 7), np.uint8))
    # 排除中心透孔（可看到后方黑背景）及外圈边缘。只保留盘面上的环带。
    raw_center = np.array(_centroid(polygon), dtype=np.float64).reshape(1, 1, 2)
    rect_center = cv2.undistortPoints(
        raw_center, calibration.camera_matrix_left, calibration.dist_left,
        R=calibration.rectification_left, P=calibration.projection_left[:, :3]).reshape(2)
    radius = max(1.0, (np.ptp(polygon[:, 0])+np.ptp(polygon[:, 1]))/4)
    yy, xx = np.ogrid[:height, :width]
    radial = ((xx-rect_center[0])**2+(yy-rect_center[1])**2)/(radius*radius)
    ys, xs = np.where((mask > 0) & (radial >= 0.35**2) &
                      (radial <= 0.85**2) & (gray_l < black_limit) &
                      (disparity > 1.0))
    if len(xs) < min_points:
        raise ValueError(f'黑色盘面有效视差仅 {len(xs)} 点')
    stride = max(1, len(xs)//3000)
    xs, ys = xs[::stride], ys[::stride]
    p = calibration.projection_left
    depth = p[0, 0]*calibration.baseline_m/disparity[ys, xs]
    good = np.isfinite(depth) & (depth > 0.25) & (depth < 3.0)
    xs, ys, depth = xs[good], ys[good], depth[good]
    if len(depth) < min_points:
        raise ValueError('黑色盘面深度不在 0.25–3m 范围')
    points = np.column_stack(((xs-p[0, 2])*depth/p[0, 0],
                              (ys-p[1, 2])*depth/p[1, 1], depth))
    median_depth = np.median(depth)
    points = points[np.abs(points[:, 2]-median_depth) < 0.05]
    if len(points) < min_points:
        raise ValueError('盘面深度离群点过多')
    mean = points.mean(axis=0)
    _, singular, vh = np.linalg.svd(points-mean, full_matrices=False)
    if singular[1]/math.sqrt(len(points)) < 0.015:
        raise ValueError('黑色盘面有效点分布过窄，无法估计法向')
    normal = vh[-1]
    if normal[2] < 0:
        normal = -normal
    if normal[2] < 0.70:
        raise ValueError('盘面法向过于倾斜，无法安全正视')
    residual = float(np.median(np.abs((points-mean)@normal)))
    if residual > 0.02:
        raise ValueError(f'盘面平面残差 {residual:.3f}m')
    center = _centroid(polygon)
    pixel = calibration.rectified_pixel('left', np.array(center))
    ray = np.array([(pixel[0]-p[0, 2])/p[0, 0],
                    (pixel[1]-p[1, 2])/p[1, 1], 1.0])
    denominator = float(ray@normal)
    if abs(denominator) < 0.3:
        raise ValueError('盘面与中心射线近乎平行')
    scale = float((mean@normal)/denominator)
    if not 0.25 < scale < 3.0:
        raise ValueError('盘心射线交点异常')
    return (calibration.rectification_left.T@(ray*scale),
            calibration.rectification_left.T@normal, len(points), residual)


class TurntableVision:
    """前视原图不出 camera；发布采集戳、世界盘心/盘轴及质量。"""

    def __init__(self, node, disk_class_id, label_class_id=-1, min_confidence=0.5):
        if int(disk_class_id) < 0:
            raise ValueError('转盘整体 YOLO class ID 未配置')
        self.node = node
        self.publisher = node.create_publisher(String, '/perception/turntable/observation', 10)
        self.disk_class_id = int(disk_class_id)
        self.label_class_id = int(label_class_id)
        self.min_confidence = float(min_confidence)
        self.sim_mode = bool(node.get_parameter('sim_mode').value)
        self.left_translation = np.asarray(node.get_parameter('turntable_left_translation').value, float)
        self.right_translation = np.asarray(node.get_parameter('turntable_right_translation').value, float)
        self.body_rotation = np.asarray(node.get_parameter('turntable_camera_rotation').value, float).reshape(3, 3)
        self.upside_down = (not bool(node.get_parameter('sim_mode').value)
                            and bool(node.get_parameter('turntable_camera_upside_down').value))
        # 外参输入保持正装名义值，禁止同时手工翻转矩阵造成双重补偿。
        self.left_translation, self.right_translation, self.body_rotation = \
            _installed_extrinsics(self.left_translation, self.right_translation,
                                  self.body_rotation, self.upside_down)
        self.black_limit = int(node.get_parameter('turntable_black_threshold').value)
        self.min_points = int(node.get_parameter('turntable_min_depth_points').value)
        self.expected_size = tuple(int(v) for v in node.get_parameter('turntable_image_size').value)
        self.lock = threading.Lock()
        self.poses = deque(maxlen=100)
        self.infos = {}
        self.calibration = None
        self._rectify_maps = None
        self._rectify_size = None
        self._last_warning = 0.0
        self.pose_sub = node.create_subscription(PoseInfo, '/basic_motion/pose_info',
                                                 self._pose_cb, qos_profile_sensor_data)
        if self.sim_mode:
            self.info_subs = [node.create_subscription(
                CameraInfo, f'/sim/front_cam/{side}/camera_info',
                lambda msg, side=side: self._info_cb(side, msg), qos_profile_sensor_data)
                for side in ('left', 'right')]
        else:
            path = str(node.get_parameter('turntable_calibration_file').value).strip()
            if not path:
                source_path = Path(__file__).resolve().parents[1]/'config'/'front.npz'
                if source_path.is_file():
                    path = str(source_path)
                else:
                    from ament_index_python.packages import get_package_share_directory
                    path = str(Path(get_package_share_directory('uv_camera')) /
                               'config' / 'front.npz')
            native_size = tuple(int(v) for v in node.get_parameter(
                'turntable_calibration_native_size').value)
            if (len(native_size) != 2 or len(self.expected_size) != 2
                    or min(*native_size, *self.expected_size) <= 0):
                raise ValueError('转盘前视图像/原始标定尺寸无效')
            self.calibration = StereoCalibration.load('turntable_front', path).scaled(
                self.expected_size[0] / native_size[0],
                self.expected_size[1] / native_size[1])
            node.get_logger().info(
                f'转盘前视双目标定：{path}，单目 {native_size} → {self.expected_size}')
        block = 5
        self.matcher = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=128, blockSize=block,
            P1=8*block*block, P2=32*block*block, uniquenessRatio=8,
            speckleWindowSize=60, speckleRange=2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)

    def _pose_cb(self, msg):
        with self.lock:
            self.poses.append(msg)

    def _info_cb(self, side, msg):
        with self.lock:
            self.infos[side] = msg

    def _pose_at(self, stamp_ns):
        with self.lock:
            poses = tuple(self.poses)
        if not poses:
            return None
        pose = min(poses, key=lambda p: abs(int(p.stamp.sec)*10**9+
                       int(p.stamp.nanosec)-stamp_ns))
        age = abs(int(pose.stamp.sec)*10**9+int(pose.stamp.nanosec)-stamp_ns)/1e9
        return pose if age <= 0.12 else None

    def process(self, frame, left, right):
        stamp = left.header.stamp
        stamp_ns = int(stamp.sec)*10**9+int(stamp.nanosec)
        mid = frame.shape[1]//2
        observation = estimate(left, self.disk_class_id, self.label_class_id,
                               self.min_confidence, frame=frame[:, :mid])
        observation['expected_disk_class_id'] = self.disk_class_id
        observation['detected_classes'] = [
            {'id': int(d.class_id), 'confidence': round(float(d.confidence), 3),
             'mask_points': len(getattr(d, 'mask_x', ()))}
            for d in getattr(left, 'detections', ())]
        if self.upside_down and observation.get('phase_valid'):
            observation['angle_deg'] = (observation['angle_deg'] + 180.) % 360.
        observation['capture_stamp_ns'] = stamp_ns
        observation['camera'] = 'front_left'
        if observation['valid']:
            try:
                if not self.sim_mode and (mid, frame.shape[0]) != self.expected_size:
                    raise ValueError(f'前视图像尺寸 {(mid, frame.shape[0])} 与标定尺寸 {self.expected_size} 不符')
                if self.calibration is None:
                    with self.lock:
                        infos = dict(self.infos)
                    if set(infos) != {'left', 'right'}:
                        raise ValueError('等待前视双目 CameraInfo')
                    self.calibration = StereoCalibration.from_camera_info(
                        'turntable_front', infos['left'], infos['right'],
                        self.left_translation, self.body_rotation,
                        self.right_translation, self.body_rotation)
                pose = self._pose_at(stamp_ns)
                if pose is None:
                    raise ValueError('采集时间附近无新鲜位姿')
                if max(abs(float(pose.robot_roll)), abs(float(pose.robot_pitch))) > 3.0:
                    raise ValueError('AUV 横滚/俯仰超过 3°，当前任务仅适用近水平机体')
                disk = _best(left, self.disk_class_id, self.min_confidence)
                size = (mid, frame.shape[0])
                if self._rectify_size != size:
                    calibration = self.calibration
                    self._rectify_maps = [cv2.initUndistortRectifyMap(
                        k, d, r, p[:, :3], size, cv2.CV_32FC1)
                        for k, d, r, p in (
                            (calibration.camera_matrix_left, calibration.dist_left,
                             calibration.rectification_left, calibration.projection_left),
                            (calibration.camera_matrix_right, calibration.dist_right,
                             calibration.rectification_right, calibration.projection_right))]
                    self._rectify_size = size
                center, normal, count, residual = _sgbm_disk(
                    frame[:, :mid], frame[:, mid:], _polygon(disk),
                    self.calibration, self.matcher, self.black_limit, self.min_points,
                    self._rectify_maps)
                diameter = (observation['disk_major_px']*center[2]/
                            float(self.calibration.projection_left[0, 0]))
                if not (0.7 * EXPECTED_DISK_DIAMETER_M <= diameter
                        <= 1.3 * EXPECTED_DISK_DIAMETER_M):
                    raise ValueError(f'视觉直径 {diameter:.3f}m 不符合 23cm 转盘')
                body = self.left_translation+self.body_rotation@center
                axis = self.body_rotation@normal
                yaw = math.radians(float(pose.robot_yaw))
                c, s = math.cos(yaw), math.sin(yaw)
                observation['disk_center_world'] = [
                    float(pose.robot_x+c*body[0]-s*body[1]),
                    float(pose.robot_y+s*body[0]+c*body[1]),
                    float(pose.robot_z+body[2])]
                observation['disk_axis_yaw_deg'] = math.degrees(math.atan2(
                    s*axis[0]+c*axis[1], c*axis[0]-s*axis[1]))
                observation['depth_points'] = count
                observation['plane_residual_m'] = residual
                observation['estimated_diameter_m'] = diameter
                observation['center_optical_m'] = [float(v) for v in center]
            except (ValueError, cv2.error, np.linalg.LinAlgError) as exc:
                observation['valid'] = False
                observation['reason'] = str(exc)
        if not observation['valid'] and time.monotonic()-self._last_warning > 5.0:
            self.node.get_logger().warn(
                f'转盘视觉无效：{observation["reason"]}；期望类别={self.disk_class_id}，'
                f'置信度阈值={self.min_confidence}，左目实际检测={observation["detected_classes"]}，'
                f'图像={frame.shape[1]}x{frame.shape[0]}，倒装补偿={self.upside_down}')
            self._last_warning = time.monotonic()
        self.publisher.publish(String(data=json.dumps(observation, allow_nan=False)))
