"""从前视 YOLO-Seg 多边形生成小型转盘角度观测；不传输图像。

角度在图像平面定义：圆心向右为 0°，向上为 +90°。这里不猜测
相机到世界系的旋转或目标机械零位；任务配置负责角度零位和方向标定。
黄色标签必须有偏离圆心的掩膜质心。若标签横跨圆心、质心近于零，
单帧角度不可辨识，明确返回无效观测，而非随机选择 0/180°。
"""

import json
import math

import cv2
import numpy as np
from std_msgs.msg import String


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


def estimate(message, disk_class_id, label_class_id, min_confidence=0.5,
             min_radial_fraction=0.15, min_axis_ratio=0.7):
    """返回一个可 JSON 序列化的单目观测；异常图形一律不产生角度。"""
    result = {'valid': False, 'reason': 'missing_segment_masks'}
    disk = _best(message, disk_class_id, min_confidence)
    label = _best(message, label_class_id, min_confidence)
    if disk is None or label is None:
        return result
    disk_poly, label_poly = _polygon(disk), _polygon(label)
    if len(disk_poly) < 5:
        result['reason'] = 'disk_contour_too_short'
        return result
    (cx, cy), (diameter_a, diameter_b), _ = cv2.fitEllipse(disk_poly)
    major, minor = max(diameter_a, diameter_b), min(diameter_a, diameter_b)
    if major <= 0 or minor / major < min_axis_ratio:
        result['reason'] = 'disk_not_front_facing'
        return result
    label_center = _centroid(label_poly)
    if label_center is None:
        result['reason'] = 'label_centroid_undefined'
        return result
    lu, lv = label_center
    # 接近正视时用各图像轴的半径归一化；倾斜过大已在上方拒绝。
    rx = max(1.0, (float(np.max(disk_poly[:, 0])) - float(np.min(disk_poly[:, 0]))) / 2)
    ry = max(1.0, (float(np.max(disk_poly[:, 1])) - float(np.min(disk_poly[:, 1]))) / 2)
    dx, dy = (lu - cx) / rx, -(lv - cy) / ry
    radial = math.hypot(dx, dy)
    if not min_radial_fraction <= radial <= 1.15:
        result['reason'] = 'label_not_on_radial_track'
        return result
    return {
        'valid': True, 'reason': 'ok',
        'angle_deg': math.degrees(math.atan2(dy, dx)) % 360.0,
        'disk_center_px': [float(cx), float(cy)],
        'label_center_px': [float(lu), float(lv)],
        'disk_radius_px': float((rx + ry) / 2),
        'axis_ratio': float(minor / major),
        'radial_fraction': float(radial),
        'confidence': float(min(disk.confidence, label.confidence)),
    }


class TurntableVision:
    """在 camera 进程内消费前视检测，只发布角度/质量元数据。"""

    def __init__(self, node, disk_class_id, label_class_id, min_confidence=0.5):
        self.publisher = node.create_publisher(
            String, '/perception/turntable/observation', 10)
        self.disk_class_id = int(disk_class_id)
        self.label_class_id = int(label_class_id)
        self.min_confidence = float(min_confidence)
        if (self.disk_class_id < 0 or self.label_class_id < 0
                or self.disk_class_id == self.label_class_id):
            raise ValueError('转盘整体与黄色标签必须配置不同的非负 YOLO class ID')

    def process(self, left, right):
        observation = estimate(left, self.disk_class_id,
                               self.label_class_id, self.min_confidence)
        stamp = left.header.stamp
        observation['capture_stamp_ns'] = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        observation['camera'] = 'front_left'
        if observation['valid']:
            other = estimate(right, self.disk_class_id,
                             self.label_class_id, self.min_confidence)
            if other['valid']:
                difference = ((observation['angle_deg'] - other['angle_deg'] + 180) % 360) - 180
                if abs(difference) > 15.0:
                    observation['valid'] = False
                    observation['reason'] = 'stereo_angle_disagreement'
                else:
                    observation['stereo_angle_difference_deg'] = abs(difference)
        self.publisher.publish(String(data=json.dumps(observation, allow_nan=False)))
