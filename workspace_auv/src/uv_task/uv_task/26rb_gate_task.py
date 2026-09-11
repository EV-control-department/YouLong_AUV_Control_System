"""仅使用相机完成过门的任务。

本模块刻意不订阅任何目标定位器输出，而是直接使用前视相机图像和
uv_camera 发布的前视门框 AI 检测结果。搜索阶段允许用左目快速发现目标；
一旦锁定目标，后续对准必须获得左右目配对观测，并使用双目极线一致性、
图像中心误差和偏航 PID 闭合控制回路。
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import threading
import time

import cv2
import numpy as np
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from uv_msgs.action import BasicMotion
from uv_msgs.msg import DetectionArray
from uv_camera.model_classes import model_class_id


# 前视相机默认值是任务在应用可选场景覆盖参数之前使用的值。它们保留为
# 任务参数，因此其他场景无需修改控制器即可替换这些参数。
_IMAGE_WIDTH = 1280
_IMAGE_HEIGHT = 960
_FRONT_HFOV_DEG = 57.19
_DEFAULT_FX = _IMAGE_WIDTH / (
    2.0 * math.tan(math.radians(_FRONT_HFOV_DEG) / 2.0))
_DEFAULT_K = np.array([
    [_DEFAULT_FX, 0.0, _IMAGE_WIDTH / 2.0],
    [0.0, _DEFAULT_FX, _IMAGE_HEIGHT / 2.0],
    [0.0, 0.0, 1.0],
], dtype=np.float64)

_FRONT_OFFSET_LEFT = np.array([0.19, -0.05, 0.176], dtype=np.float64)
_FRONT_OFFSET_RIGHT = np.array([0.19, 0.05, 0.176], dtype=np.float64)

# Stonefish 前视 ColorCamera 光轴 -> XUNYUN 机体/NED 坐标轴。仿真器相机
# 定义规定局部 +X 指向图像右侧、+Y 指向图像下方、+Z 指向前方；下面的
# 矩阵在机体坐标系（前、右、下）中保持这些方向不变。
_OPTICAL_TO_BODY = np.array([
    [0.0, 0.0, 1.0],
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
], dtype=np.float64)

# 锁定后允许同一门框在连续图像中的中心变化范围。超过这个范围时视为
# 原目标丢失，不切换到另一个门框。
_LOCK_MAX_CENTER_DELTA_FRACTION = 0.35

_GATE_FRONT_CLASS_ID = model_class_id('gate_front')


@dataclass(frozen=True)
class GateCandidate:
    """单幅相机图像中的一个门框候选。"""

    center: np.ndarray
    corners: np.ndarray
    width_px: float
    height_px: float
    extent_fraction: float
    frame_score: float
    # 将检测器的边界框中心与可选的中心线/分割锚点分开保存。前者是控制
    # 观测量：门管道部分遮挡时不会发生跳变。
    bbox_center: np.ndarray | None = None
    # 被裁剪的边界框/最小外接矩形无法提供门框的四个真实角点，因此不能
    # 用于估计平面法向量。
    clipped: bool = False


@dataclass(frozen=True)
class GateObservation:
    """以 AUV 机体坐标系表示的门几何/视觉测量结果。"""

    center_body: np.ndarray
    normal_body: np.ndarray
    distance_m: float
    extent_fraction: float
    center_px: np.ndarray
    left: GateCandidate
    right: GateCandidate
    normal_reliable: bool = True
    # 搜索阶段可以暂时使用单目；锁定后的对准观测必须是双目。
    monocular: bool = False


@dataclass(frozen=True)
class FilteredGate:
    """供伺服控制使用的低通滤波边界框和相机坐标系高度。"""

    center_u: float
    center_v: float
    width_px: float
    height_px: float
    center_z_m: float
    extent_fraction: float


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


def _wrap_degrees(value: float) -> float:
    return (float(value) + 180.0) % 360.0 - 180.0


def _decode_image(message: Image) -> np.ndarray | None:
    """解码 Stonefish 和相机桥接使用的 8 位图像编码。"""

    try:
        height = int(message.height)
        width = int(message.width)
        step = int(message.step)
        encoding = str(message.encoding).strip().lower()
        raw = np.frombuffer(message.data, dtype=np.uint8)
    except (AttributeError, TypeError, ValueError):
        return None
    channels = {
        'bgr8': 3,
        'rgb8': 3,
        '8uc3': 3,
        'bgra8': 4,
        'rgba8': 4,
        '8uc4': 4,
        'mono8': 1,
        '8uc1': 1,
    }.get(encoding)
    if height <= 0 or width <= 0 or channels is None:
        return None
    minimum_step = width * channels
    step = max(step, minimum_step)
    required = height * step
    if raw.size < required:
        return None
    rows = raw[:required].reshape(height, step)
    image = rows[:, :minimum_step].reshape(height, width, channels).copy()
    if encoding in ('rgb8', 'rgba8'):
        code = cv2.COLOR_RGB2BGR if channels == 3 else cv2.COLOR_RGBA2BGR
        image = cv2.cvtColor(image, code)
    elif channels == 4:
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    elif channels == 1:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    return image


def _triangulate_rays(
    origin_left: np.ndarray,
    ray_left: np.ndarray,
    origin_right: np.ndarray,
    ray_right: np.ndarray,
) -> np.ndarray | None:
    """返回两条前向射线最近点的中点。"""

    d1 = ray_left / max(np.linalg.norm(ray_left), 1e-12)
    d2 = ray_right / max(np.linalg.norm(ray_right), 1e-12)
    w = origin_left - origin_right
    a = float(np.dot(d1, d1))
    b = float(np.dot(d1, d2))
    c = float(np.dot(d2, d2))
    d = float(np.dot(d1, w))
    e = float(np.dot(d2, w))
    denominator = a * c - b * b
    if abs(denominator) < 1e-10:
        return None
    t1 = (b * e - c * d) / denominator
    t2 = (a * e - b * d) / denominator
    if t1 <= 0.0 or t2 <= 0.0:
        return None
    point_left = origin_left + d1 * t1
    point_right = origin_right + d2 * t2
    point = (point_left + point_right) * 0.5
    if not np.all(np.isfinite(point)):
        return None
    return point


class RB26GateTask:
    """仅使用前视图像搜索、对准并通过四个门。"""

    def __init__(self, node, params: dict):
        self._node = node
        self._params = params
        self._logger = node.get_logger()
        self._lock = threading.RLock()
        self._latest_left: tuple[float, np.ndarray] | None = None
        self._latest_right: tuple[float, np.ndarray] | None = None
        self._latest_stitched: tuple[float, np.ndarray] | None = None
        self._latest_left_detections: tuple[float, DetectionArray] | None = None
        self._latest_right_detections: tuple[float, DetectionArray] | None = None
        # 左右目检测回调并非严格交错到达。只保存各自最新一帧会把“左目
        # 新帧 + 右目旧帧”误判成配对 ID 不匹配，从而丢掉一组本来有效的
        # 双目结果。短缓存用于按 stereo_pair_id 找回同一组检测。
        self._detection_history = {
            'left': deque(maxlen=16),
            'right': deque(maxlen=16),
        }
        self._locked_observation: GateObservation | None = None
        self._last_observation_reason = '尚未评估'
        self._last_observation_reason_log = 0.0
        self._camera_k = {'left': _DEFAULT_K.copy(), 'right': _DEFAULT_K.copy()}
        self._subs = []

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._input_mode = str(params.get('image_input', 'stitched')).strip().lower()
        if self._input_mode not in {'stitched', 'separate'}:
            self._input_mode = 'stitched'
        if self._input_mode == 'separate':
            left_topic = str(params.get(
                'left_image_topic', '/sim/front_cam/left/image_color'))
            self._subs.append(node.create_subscription(
                Image, left_topic, self._left_image_cb, qos))
            right_topic = str(params.get(
                'right_image_topic', '/sim/front_cam/right/image_color'))
            self._subs.append(node.create_subscription(
                Image, right_topic, self._right_image_cb, qos))
        else:
            stitched_topic = str(params.get(
                'stitched_image_topic', '/auv/front_cam/stitched'))
            self._subs.append(node.create_subscription(
                Image, stitched_topic, self._stitched_image_cb, qos))

        # 搜索阶段首先使用左目相机链路的检测结果作为视觉线索；锁定后
        # 还会订阅并校验右目结果。这仍然是纯相机方案：不消费
        # /perception/objects 或 object_localizer 输出。
        self._subs.append(node.create_subscription(
            DetectionArray,
            str(params.get(
                'left_detection_topic',
                '/perception/detection/front_left')),
            self._left_detection_cb, qos))
        self._subs.append(node.create_subscription(
            DetectionArray,
            str(params.get(
                'right_detection_topic',
                '/perception/detection/front_right')),
            self._right_detection_cb, qos))

        # CameraInfo 是标定数据，不是目标定位器输出。因此任务可以使用
        # 仿真器相机的实际内参，同时完全独立于 object_localizer.py。
        for side in ('left', 'right'):
            topic = str(params.get(
                f'{side}_camera_info_topic',
                f'/sim/front_cam/{side}/camera_info'))
            self._subs.append(node.create_subscription(
                CameraInfo, topic,
                lambda msg, s=side: self._camera_info_cb(s, msg), qos))

        self._front_hfov_deg = float(params.get('front_hfov_deg', _FRONT_HFOV_DEG))
        default_fx = _IMAGE_WIDTH / (
            2.0 * math.tan(math.radians(self._front_hfov_deg) / 2.0))
        default_k = np.array([
            [default_fx, 0.0, _IMAGE_WIDTH / 2.0],
            [0.0, default_fx, _IMAGE_HEIGHT / 2.0],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)
        self._camera_k = {'left': default_k.copy(), 'right': default_k.copy()}
        self._min_extent = _clamp(
            float(params.get('min_gate_extent_fraction', 0.25)), 0.05, 0.95)
        self._target_extent = _clamp(
            float(params.get('target_gate_extent_fraction', 0.75)), 0.20, 0.95)
        self._target_height_fraction = _clamp(
            float(params.get('target_gate_height_fraction', 2.0 / 3.0)),
            0.20, 0.95)
        self._extent_tolerance = max(
            0.01, float(params.get('extent_tolerance', 0.15)))
        self._heading_check_seconds = max(
            0.1, float(params.get('heading_check_seconds', 1.0)))
        self._post_turn_settle_seconds = max(
            0.0, float(params.get('post_turn_settle_seconds', 1.0)))
        self._post_turn_observation_timeout = max(
            3.0, float(params.get('post_turn_observation_timeout', 3.0)))
        self._bbox_filter_alpha = _clamp(
            float(params.get('bbox_filter_alpha', 0.25)), 0.01, 1.0)
        self._bbox_log_period = max(
            0.1, float(params.get('bbox_log_period', 0.5)))
        self._bbox_ratio_target = max(
            1.0, float(params.get('bbox_ratio_target', 1.2)))
        self._bbox_ratio_hold_seconds = max(
            0.1, float(params.get('bbox_ratio_hold_seconds', 1.0)))
        self._minimum_attitude_correction_seconds = max(
            0.0, float(params.get('minimum_attitude_correction_seconds', 10.0)))
        self._frame_timeout = max(
            0.2, float(params.get('image_timeout', 8.0)))
        self._pair_slop = max(
            0.01, float(params.get('stereo_pair_slop', 0.25)))
        self._detection_timeout = max(
            0.2, float(params.get('detection_timeout', 1.5)))
        self._detection_min_confidence = max(
            0.0, float(params.get('min_gate_detection_confidence', 0.02)))
        self._image_center_tolerance = _clamp(
            float(params.get('image_center_tolerance_fraction', 0.04)),
            0.01, 0.20)
        self._stereo_vertical_tolerance = _clamp(
            float(params.get('stereo_vertical_tolerance_fraction', 0.04)),
            0.01, 0.20)
        self._distance_control_min_m = max(
            0.2, float(params.get('distance_control_min_m', 0.5)))
        self._distance_control_max_m = max(
            self._distance_control_min_m,
            float(params.get('distance_control_max_m', 5.0)))
        self._monocular_reference_distance = max(
            self._distance_control_min_m,
            float(params.get('monocular_reference_distance_m', 1.5)))

        self._front_left_offset = self._vector_param(
            'front_left_offset', _FRONT_OFFSET_LEFT)
        self._front_right_offset = self._vector_param(
            'front_right_offset', _FRONT_OFFSET_RIGHT)
        self._optical_to_body = np.asarray(
            params.get('optical_to_body', _OPTICAL_TO_BODY), dtype=np.float64)
        if self._optical_to_body.shape != (3, 3):
            self._optical_to_body = _OPTICAL_TO_BODY.copy()

        self._gates_to_pass = max(1, int(params.get('gate_count', 4)))
        # 四个门可能分别消耗配置的搜索、对准和通过时间预算。除非调用者
        # 明确提供更小的超时时间，否则让总看门狗时间长于最坏情况。
        self._task_timeout = max(10.0, float(params.get('timeout', 720.0)))
        self._scan_settle = max(0.0, float(params.get('search_settle_time', 0.4)))
        self._scan_timeout = max(1.0, float(params.get('search_timeout', 60.0)))
        self._rotate_timeout = max(1.0, float(params.get('rotate_timeout', 12.0)))
        self._search_stop_height_fraction = _clamp(
            float(params.get('search_stop_height_fraction', 0.4)),
            0.05, 0.95)
        self._search_stopped_on_height = False
        self._scan_yaw_rate = max(
            1.0, float(params.get('search_yaw_rate_deg_s', 36.0)))
        self._scan_publish_period = max(
            0.02, float(params.get(
                'search_yaw_publish_period', 0.05)))
        self._right_yaw_sign = float(params.get('right_yaw_sign', 1.0))
        self._search_start_offset_deg = float(
            params.get('search_start_offset_deg', -30.0))
        self._search_sweep_deg = abs(float(
            params.get('search_sweep_deg', 60.0)))

        self._alignment_timeout = max(
            5.0, float(params.get('alignment_timeout', 45.0)))
        self._control_period = max(
            0.05, float(params.get('control_period', 0.25)))
        self._command_timeout = max(
            0.25, float(params.get('command_timeout', 8.0)))
        self._max_forward_step = max(
            0.01, float(params.get('max_forward_step', 0.12)))
        self._max_back_step = max(
            0.01, float(params.get('max_back_step', 0.08)))
        self._max_lateral_step = max(
            0.01, float(params.get('max_lateral_step', 0.10)))
        self._max_vertical_step = max(
            0.01, float(params.get('max_vertical_step', 0.08)))
        self._max_yaw_step = max(
            1.0, float(params.get('max_yaw_step_deg', 3.0)))
        # 对准采用连续的机体速度控制回路。这里有意不使用位置动作控制器：
        # 一次 BMOVE 可能耗时一秒或更久，下一次修正发出前相机观察到的姿态
        # 就可能已经不同。速度回路会保持小幅前进/垂向维持指令，并使用
        # 机体 y 方向速度闭合带符号的水平边界框中心误差。
        self._velocity_period = max(
            0.02, float(params.get('velocity_period', 0.05)))
        self._max_forward_speed = max(
            0.01, float(params.get('max_forward_speed_mps', 0.18)))
        self._max_reverse_speed = max(
            0.01, float(params.get('max_reverse_speed_mps', 0.12)))
        self._max_lateral_speed = max(
            0.01, float(params.get('max_lateral_speed_mps', 0.18)))
        self._max_vertical_speed = max(
            0.01, float(params.get('max_vertical_speed_mps', 0.12)))
        self._max_yaw_rate = max(
            0.5, float(params.get('max_yaw_rate_deg_s', 36.0)))
        self._distance_velocity_gain = max(
            0.01, float(params.get('distance_velocity_gain', 0.45)))
        self._vertical_velocity_gain = max(
            0.01, float(params.get('vertical_velocity_gain', 0.55)))
        self._yaw_velocity_gain = max(
            0.01, float(params.get('yaw_velocity_gain_deg_s', 0.8)))
        self._distance_gain = max(
            0.05, float(params.get('distance_gain', 0.8)))
        self._lateral_gain = max(
            0.05, float(params.get('lateral_gain', 0.8)))
        self._vertical_gain = max(
            0.05, float(params.get('vertical_gain', 0.8)))
        self._yaw_gain = max(
            0.05, float(params.get('yaw_gain', 0.8)))
        self._vertical_center_offset = float(
            params.get('vertical_center_offset_m', 0.0))
        self._height_pid_kp = float(params.get('height_pid_kp', 0.8))
        self._height_pid_ki = float(params.get('height_pid_ki', 0.02))
        self._height_pid_kd = float(params.get('height_pid_kd', 0.05))
        self._height_pid_integral_limit = max(
            0.01, float(params.get('height_pid_integral_limit', 0.5)))
        self._height_pid_stable_error_m = max(
            0.001, float(params.get('height_pid_stable_error_m', 0.03)))
        self._height_pid_stable_derivative_mps = max(
            0.001, float(params.get('height_pid_stable_derivative_mps', 0.02)))
        self._height_pid_stable_seconds = max(
            0.1, float(params.get('height_pid_stable_seconds', 0.5)))
        # 姿态对准和横向极值搜索统一使用 4-DOF 视觉伺服模型：横向
        # 速度 vy 是唯一的宽度极值搜索激励，偏航使用图像中心 PID 和
        # vy/d 的运动学前馈补偿。
        self._arc_lateral_speed = max(
            0.0, float(params.get('arc_lateral_speed_mps', 0.08)))
        self._yaw_center_kp = max(
            0.01, float(params.get('yaw_center_kp', 3.6)))
        # 偏航 PID 的误差单位是角度（°），输出单位是角速度（°/秒）。
        # 默认 Kp 与原有 yaw_center_kp 等效，Ki/Kd 默认为零，确保迁移后
        # 的默认运动行为连续；实际稳定判定由显式阈值负责。
        self._yaw_pid_kp = float(params.get('yaw_pid_kp', self._yaw_center_kp))
        self._yaw_pid_ki = float(params.get('yaw_pid_ki', 0.0))
        self._yaw_pid_kd = float(params.get('yaw_pid_kd', 0.0))
        self._yaw_pid_integral_limit_deg = max(
            0.01, float(params.get('yaw_pid_integral_limit_deg', 10.0)))
        self._arc_probe_speed = max(
            0.01, float(params.get('arc_probe_speed_mps', 0.10)))
        self._arc_probe_window_seconds = max(
            0.2, float(params.get('arc_probe_seconds', 2.0)))
        self._arc_window_seconds = max(
            0.2, float(params.get(
                'arc_gradient_window_seconds',
                5.0)))
        self._arc_objective_deadband = max(
            0.0005, float(params.get('arc_objective_deadband', 0.01)))
        self._arc_reverse_cooldown = max(
            0.1, float(params.get('arc_reverse_cooldown_seconds', 0.5)))
        self._arc_near_extremum_speed_scale = _clamp(
            float(params.get('arc_near_extremum_speed_scale', 0.5)),
            0.1, 1.0)
        self._lost_timeout = max(
            0.2, float(params.get('lost_observation_timeout', 10.0)))
        self._lost_command_hold_seconds = max(
            0.0, float(params.get('lost_command_hold_seconds', 1.0)))
        self._pass_distance = max(
            0.2, float(params.get('pass_distance_m', 1.15)))
        self._pass_timeout = max(
            2.0, float(params.get('pass_timeout', 45.0)))
        self._post_pass_pause = max(
            0.0, float(params.get('post_pass_pause', 0.5)))

        self._logger.info(
            '26rb_gate_task 已创建：搜索使用左目，锁定后使用双目视觉伺服，'
            f'输入={self._input_mode}，门数={self._gates_to_pass}，'
            f'占比范围 {self._min_extent:.2f}->{self._target_extent:.2f}，'
            f'前后距离按 bbox 高度={self._target_height_fraction:.3f}，'
            '跟踪当前视野中的最大门框，'
            f'图像中心误差<={self._image_center_tolerance:.2f}，'
            f'速度回路周期={self._velocity_period:.2f} 秒，'
            f'最大速度=({self._max_forward_speed:.2f}，'
            f'{self._max_lateral_speed:.2f}，'
            f'{self._max_vertical_speed:.2f}) 米/秒，'
            f'搜索偏航角速度={self._scan_yaw_rate:.1f}°/秒，'
            f'左目外参偏移={self._front_left_offset.tolist()}，'
            f'光轴到机体坐标变换={self._optical_to_body.tolist()}，'
            f'横向极值搜索={self._arc_lateral_speed:.2f}米/秒，'
            f'首段窗口={self._arc_probe_window_seconds:.1f}秒，'
            f'首段窗口速度={self._arc_probe_speed:.2f}米/秒，'
            f'固定分段窗口={self._arc_window_seconds:.1f}秒，'
            f'丢失指令保持={self._lost_command_hold_seconds:.1f} 秒，'
            f'搜索高度阈值={self._search_stop_height_fraction:.3f}，'
            f'偏航 PID=({self._yaw_pid_kp:.2f}，'
            f'{self._yaw_pid_ki:.2f}，{self._yaw_pid_kd:.2f})')

    def _vector_param(self, name: str, default: np.ndarray) -> np.ndarray:
        value = self._params.get(name, default)
        try:
            vector = np.asarray(value, dtype=np.float64).reshape(3)
            if np.all(np.isfinite(vector)):
                return vector
        except (TypeError, ValueError):
            pass
        return default.copy()

    def destroy(self):
        for subscription in self._subs:
            try:
                self._node.destroy_subscription(subscription)
            except (AttributeError, RuntimeError):
                pass
        self._subs.clear()

    # ── 相机回调 ─────────────────────────────────────────────────────

    def _left_image_cb(self, message: Image):
        image = _decode_image(message)
        if image is not None:
            with self._lock:
                self._latest_left = (time.monotonic(), image)

    def _right_image_cb(self, message: Image):
        image = _decode_image(message)
        if image is not None:
            with self._lock:
                self._latest_right = (time.monotonic(), image)

    def _stitched_image_cb(self, message: Image):
        image = _decode_image(message)
        if image is not None:
            with self._lock:
                self._latest_stitched = (time.monotonic(), image)

    def _left_detection_cb(self, message: DetectionArray):
        received = time.monotonic()
        with self._lock:
            self._latest_left_detections = (received, message)
            self._detection_history['left'].append((received, message))

    def _right_detection_cb(self, message: DetectionArray):
        received = time.monotonic()
        with self._lock:
            self._latest_right_detections = (received, message)
            self._detection_history['right'].append((received, message))

    def _camera_info_cb(self, side: str, message: CameraInfo):
        try:
            matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
            if np.all(np.isfinite(matrix)) and matrix[0, 0] > 1.0:
                with self._lock:
                    self._camera_k[side] = matrix
        except (AttributeError, TypeError, ValueError):
            pass

    # ── 相机几何 ─────────────────────────────────────────────────────

    def _left_camera(self):
        """返回当前左目图像及其标定内参。

        拼接图像在搜索阶段这里只取左半部分；锁定后的对准会通过
        ``_camera_pair`` 同时检查左右目图像。
        """

        now = time.monotonic()
        with self._lock:
            if self._input_mode == 'stitched':
                if self._latest_stitched is None:
                    self._last_observation_reason = '未收到拼接图像'
                    return None
                received, stitched = self._latest_stitched
                if now - received > self._frame_timeout:
                    self._last_observation_reason = (
                        f'拼接图像已过期（{now - received:.2f} 秒）')
                    return None
                if stitched.ndim < 2 or stitched.shape[1] < 2 * stitched.shape[0]:
                    self._last_observation_reason = (
                        f'拼接图像尺寸无效：{stitched.shape[1]}x{stitched.shape[0]}')
                    return None
                split = stitched.shape[1] // 2
                return stitched[:, :split].copy(), self._camera_k['left'].copy()

            if self._latest_left is None:
                self._last_observation_reason = '缺少左目图像'
                return None
            received, left = self._latest_left
            if now - received > self._frame_timeout:
                self._last_observation_reason = (
                    f'左目图像已过期（{now - received:.2f} 秒）')
                return None
            return left.copy(), self._camera_k['left'].copy()

    def _camera_pair(self):
        """返回一组满足时效/同步要求的双目图像。"""
        now = time.monotonic()
        with self._lock:
            if self._input_mode == 'stitched':
                if self._latest_stitched is None:
                    self._last_observation_reason = '未收到拼接图像'
                    return None
                received, stitched = self._latest_stitched
                if now - received > self._frame_timeout:
                    self._last_observation_reason = (
                        f'拼接图像已过期（{now - received:.2f} 秒）')
                    return None
                if stitched.shape[1] < 2 * stitched.shape[0]:
                    self._last_observation_reason = (
                        f'拼接图像尺寸无效：{stitched.shape[1]}x{stitched.shape[0]}')
                    return None
                split = stitched.shape[1] // 2
                left = stitched[:, :split].copy()
                right = stitched[:, split:].copy()
                return left, right, self._camera_k['left'].copy(), self._camera_k['right'].copy()

            if self._latest_left is None or self._latest_right is None:
                self._last_observation_reason = '缺少分离的双目图像'
                return None
            left_time, left = self._latest_left
            right_time, right = self._latest_right
            if (now - left_time > self._frame_timeout
                    or now - right_time > self._frame_timeout
                    or abs(left_time - right_time) > self._pair_slop):
                self._last_observation_reason = '分离的双目图像未同步'
                return None
            return left.copy(), right.copy(), self._camera_k['left'].copy(), self._camera_k['right'].copy()

    def _detection_pair(self):
        """按 stereo_pair_id 返回最新的一组相机检测结果。

        左右目检测发布通常来自两个独立的 ROS 回调，回调顺序不保证与
        相机采集顺序一致。因此不能只比较两条 latest 消息；先在短缓存
        中找同一 ``stereo_pair_id``，只有缓存中确实没有对应帧时才报告
        双目配对失败。
        """

        now = time.monotonic()
        with self._lock:
            left_history = list(self._detection_history['left'])
            right_history = list(self._detection_history['right'])
        if not left_history or not right_history:
            return None

        left_history = [
            entry for entry in left_history
            if now - entry[0] <= self._detection_timeout
        ]
        right_history = [
            entry for entry in right_history
            if now - entry[0] <= self._detection_timeout
        ]
        if not left_history or not right_history:
            self._last_observation_reason = '相机检测结果已过期'
            return None

        def pair_id(message):
            return int(getattr(message, 'stereo_pair_id', 0) or 0)

        # 优先按明确的采集帧 ID 配对。按接收时间倒序找最近的一组，避免
        # 因单个回调先到而错过刚刚完成的另一目消息。
        by_right_id = {}
        for received, message in reversed(right_history):
            message_id = pair_id(message)
            if message_id:
                by_right_id.setdefault(message_id, (received, message))
        id_matches = []
        for left_received, left in reversed(left_history):
            left_id = pair_id(left)
            if not left_id or left_id not in by_right_id:
                continue
            right_received, right = by_right_id[left_id]
            id_matches.append((
                max(left_received, right_received),
                abs(left_received - right_received),
                left,
                right,
            ))
        if id_matches:
            _newest, _arrival_delta, left, right = max(
                id_matches, key=lambda item: (item[0], -item[1]))
            return left, right

        # 没有 ID 时兼容旧消息，使用消息头时间戳；若时间戳不可用则使用
        # ROS 回调到达时间。仍然要求在 pair_slop 内，禁止随意拼接两帧。
        def stamp(received, message):
            try:
                header_stamp = message.header.stamp
                return (float(header_stamp.sec)
                        + float(header_stamp.nanosec) * 1e-9)
            except (AttributeError, TypeError, ValueError):
                return float(received)

        fallback_matches = []
        for left_received, left in left_history:
            left_id = pair_id(left)
            for right_received, right in right_history:
                right_id = pair_id(right)
                if left_id and right_id:
                    continue
                delta = abs(stamp(left_received, left)
                            - stamp(right_received, right))
                if delta <= self._pair_slop:
                    fallback_matches.append((
                        max(left_received, right_received), delta,
                        left, right,
                    ))
        if fallback_matches:
            _newest, _stamp_delta, left, right = max(
                fallback_matches, key=lambda item: (item[0], -item[1]))
            return left, right

        self._last_observation_reason = '相机检测双目配对 ID 不匹配'
        return None

    @staticmethod
    def _detection_feature(det):
        """优先使用 uv_camera 的门框锚点，否则使用边界框中心。"""

        try:
            feature_type = int(getattr(det, 'feature_type', 0) or 0)
            feature = np.array([
                float(getattr(det, 'feature_pixel_x')),
                float(getattr(det, 'feature_pixel_y')),
            ], dtype=np.float64)
            if feature_type > 0 and np.all(np.isfinite(feature)):
                return feature
        except (AttributeError, TypeError, ValueError):
            pass
        return np.array([
            (float(det.bbox_x1) + float(det.bbox_x2)) * 0.5,
            (float(det.bbox_y1) + float(det.bbox_y2)) * 0.5,
        ], dtype=np.float64)

    def _detection_candidates(self, message: DetectionArray,
                              width: int, height: int) -> list[GateCandidate]:
        candidates = []
        for det in getattr(message, 'detections', []):
            try:
                if (int(det.class_id) != _GATE_FRONT_CLASS_ID
                        or float(det.confidence) < self._detection_min_confidence):
                    continue
                x1, y1 = float(det.bbox_x1), float(det.bbox_y1)
                x2, y2 = float(det.bbox_x2), float(det.bbox_y2)
                values = np.array([x1, y1, x2, y2], dtype=np.float64)
                if not np.all(np.isfinite(values)):
                    continue
                x1, x2 = sorted((x1, x2))
                y1, y2 = sorted((y1, y2))
                box_width, box_height = x2 - x1, y2 - y1
                if min(box_width, box_height) < max(12.0, height * 0.025):
                    continue
                extent = max(box_width / max(width, 1),
                             box_height / max(height, 1))
                if extent < self._min_extent:
                    continue
                corners = np.array([
                    [x1, y1], [x2, y1], [x2, y2], [x1, y2],
                ], dtype=np.float64)
                candidates.append(GateCandidate(
                    center=self._detection_feature(det),
                    corners=corners,
                    width_px=box_width,
                    height_px=box_height,
                    extent_fraction=float(extent),
                    frame_score=float(det.confidence),
                    bbox_center=np.array([
                        (x1 + x2) * 0.5, (y1 + y2) * 0.5,
                    ], dtype=np.float64),
                    clipped=bool(
                        x1 <= 1.0 or y1 <= 1.0
                        or x2 >= width - 1.0 or y2 >= height - 1.0),
                ))
            except (AttributeError, TypeError, ValueError):
                continue
        candidates.sort(
            key=lambda item: (item.extent_fraction, item.frame_score),
            reverse=True,
        )
        return candidates

    def _ray_in_body(self, pixel: np.ndarray, matrix: np.ndarray,
                     offset: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        fx = max(1.0, float(matrix[0, 0]))
        fy = max(1.0, float(matrix[1, 1]))
        cx = float(matrix[0, 2])
        cy = float(matrix[1, 2])
        camera_ray = np.array([
            (float(pixel[0]) - cx) / fx,
            (float(pixel[1]) - cy) / fy,
            1.0,
        ], dtype=np.float64)
        body_ray = self._optical_to_body @ camera_ray
        body_ray /= max(np.linalg.norm(body_ray), 1e-12)
        return offset.copy(), body_ray

    def _triangulate_pixel(self, left_pixel: np.ndarray,
                           right_pixel: np.ndarray,
                           left_k: np.ndarray,
                           right_k: np.ndarray) -> np.ndarray | None:
        left_origin, left_ray = self._ray_in_body(
            left_pixel, left_k, self._front_left_offset)
        right_origin, right_ray = self._ray_in_body(
            right_pixel, right_k, self._front_right_offset)
        return _triangulate_rays(
            left_origin, left_ray, right_origin, right_ray)

    def _make_observation(self, left: GateCandidate, right: GateCandidate,
                          left_k: np.ndarray, right_k: np.ndarray):
        center = self._triangulate_pixel(
            left.center, right.center, left_k, right_k)
        if center is None or center[0] <= 0.0:
            return None

        left_corners = left.corners
        right_corners = right.corners
        points = []
        for left_pixel, right_pixel in zip(left_corners, right_corners):
            point = self._triangulate_pixel(
                left_pixel, right_pixel, left_k, right_k)
            if point is not None and point[0] > 0.0:
                points.append(point)

        normal_reliable = (
            len(points) >= 3 and not left.clipped and not right.clipped)
        normal = None
        if normal_reliable:
            cloud = np.asarray(points, dtype=np.float64)
            centered = cloud - cloud.mean(axis=0)
            _, _, vh = np.linalg.svd(centered, full_matrices=False)
            normal = vh[-1]
            normal /= max(np.linalg.norm(normal), 1e-12)
        if normal is None:
            # 被裁剪的矩形无法进行有效的四角点平面估计。使用中心方位
            # 作为后备几何估计；在获得完整且未被裁剪的观测之前，控制器
            # 不会发出基于法向量的偏航修正。
            normal = center / max(np.linalg.norm(center), 1e-12)
        if float(np.dot(normal, center)) < 0.0:
            normal = -normal
        extent = max(left.extent_fraction, right.extent_fraction)
        return GateObservation(
            center_body=center,
            normal_body=normal,
            distance_m=float(np.linalg.norm(center)),
            extent_fraction=float(extent),
            center_px=(left.center + right.center) * 0.5,
            left=left,
            right=right,
            normal_reliable=normal_reliable,
        )

    def _make_left_observation(self, candidate: GateCandidate,
                               left_k: np.ndarray) -> GateObservation:
        """在保留相机外参的同时构建左目观测。

        单个边界框不包含米制深度信息。因此，边界框占比为速度回路提供
        有界的距离代理值；然后使用标定后的左目相机射线将边界框中心转换
        为机体坐标系中的 X/Y/Z。这样既能保持 AUV 坐标系中横向/垂向修正
        的符号和尺度，又不会把单目边界框误当成双目深度。
        """

        pixel = (candidate.bbox_center
                 if candidate.bbox_center is not None else candidate.center)
        extent = max(float(candidate.extent_fraction), self._min_extent)
        distance = self._monocular_reference_distance * (
            self._target_extent / max(extent, 1e-6))
        distance = _clamp(
            distance, self._distance_control_min_m,
            self._distance_control_max_m)
        origin, ray = self._ray_in_body(
            np.asarray(pixel, dtype=np.float64), left_k,
            self._front_left_offset)
        center = origin + ray * distance
        normal = center / max(float(np.linalg.norm(center)), 1e-12)
        # 将左目候选复用到右目槽位，使现有数据对象保持不可变，并兼容
        # 旧版几何辅助函数。
        return GateObservation(
            center_body=center,
            normal_body=normal,
            distance_m=float(np.linalg.norm(center)),
            extent_fraction=float(candidate.extent_fraction),
            center_px=np.asarray(pixel, dtype=np.float64),
            left=candidate,
            right=candidate,
            normal_reliable=False,
            monocular=True,
        )

    def _left_detection_candidates(self):
        """返回当前左目相机内参矩阵和有效门框候选。"""

        camera = self._left_camera()
        if camera is None:
            return None
        left_image, left_k = camera
        height, width = left_image.shape[:2]

        # 只使用 uv_camera 对左目进行语义分类得到的门框；这里不涉及
        # 右目同步或 object_localizer 话题。
        with self._lock:
            left_entry = self._latest_left_detections
        if left_entry is None:
            self._last_observation_reason = '尚未收到左目门框检测结果'
            return None
        received, left_message = left_entry
        if time.monotonic() - received > self._detection_timeout:
            self._last_observation_reason = '左目门框检测结果已过期'
            return None
        left_detections = self._detection_candidates(
            left_message, width, height)
        if not left_detections:
            self._last_observation_reason = '左目未检测到有效门框'
            return None
        return left_k, left_detections

    @staticmethod
    def _candidate_pixel_center(candidate: GateCandidate) -> np.ndarray:
        if candidate.bbox_center is not None:
            return np.asarray(candidate.bbox_center, dtype=np.float64)
        return np.asarray(candidate.center, dtype=np.float64)

    def _select_locked_candidate(
            self, candidates: list[GateCandidate],
            reference: GateCandidate | None) -> GateCandidate | None:
        """在候选框中寻找同一门框，不因另一门更大而切换目标。"""

        if not candidates:
            return None
        if reference is None:
            return candidates[0]

        reference_center = self._candidate_pixel_center(reference)
        reference_extent = max(float(reference.extent_fraction), 1e-6)
        scored = []
        for candidate in candidates:
            center = self._candidate_pixel_center(candidate)
            center_delta = float(np.linalg.norm(
                (center - reference_center)
                / np.array([_IMAGE_WIDTH, _IMAGE_HEIGHT], dtype=np.float64)))
            if center_delta > _LOCK_MAX_CENTER_DELTA_FRACTION:
                continue
            extent_delta = abs(math.log(
                max(float(candidate.extent_fraction), 1e-6)
                / reference_extent))
            scored.append((center_delta + 0.05 * extent_delta, candidate))
        if not scored:
            self._last_observation_reason = '锁定门框超出允许跟踪范围'
            return None
        scored.sort(key=lambda item: item[0])
        return scored[0][1]

    def _observe(self) -> GateObservation | None:
        """搜索阶段的快速左目观测。

        搜索只负责尽快发现并锁定一个候选，避免右目检测的短暂延迟拖慢
        扫描。进入对准阶段后必须改用 ``_observe_stereo``，不能把这个
        单目观测当作最终的双目几何结果。
        """
        context = self._left_detection_candidates()
        if context is None:
            return None
        left_k, left_detections = context
        return self._make_left_observation(left_detections[0], left_k)

    def _observe_locked(self, reference: GateObservation) -> GateObservation | None:
        """只更新锁定门框，并要求左右目形成有效配对。"""

        return self._observe_stereo(reference)

    def _pair_candidates(self, left_candidates: list[GateCandidate],
                         right_candidates: list[GateCandidate],
                         left_k: np.ndarray, right_k: np.ndarray,
                         height: int) -> GateObservation | None:
        """配对候选框，并返回当前视野中最大的有效门框。"""

        pairs = []
        for left in left_candidates:
            for right in right_candidates:
                # 按照仿真器前视相机约定，左目看到的前方目标应比右目中
                # 更靠右。这个简单的视差符号检查可以在三角测量前排除
                # 很多左门/右门交叉配对。
                left_pixel = self._candidate_pixel_center(left)
                right_pixel = self._candidate_pixel_center(right)
                disparity = float(left_pixel[0] - right_pixel[0])
                if disparity <= 1.0:
                    continue
                centre_y_error = abs(float(left_pixel[1] - right_pixel[1]))
                if centre_y_error > max(50.0, height * 0.12):
                    continue
                size_ratio = max(
                    left.extent_fraction / max(right.extent_fraction, 1e-6),
                    right.extent_fraction / max(left.extent_fraction, 1e-6),
                )
                if size_ratio > 1.8:
                    continue
                pair_score = (
                    min(left.extent_fraction, right.extent_fraction)
                    + 0.15 * min(left.frame_score, right.frame_score)
                    - centre_y_error / max(height, 1) * 0.2
                )
                pairs.append((pair_score, left, right))
        pairs.sort(key=lambda item: item[0], reverse=True)
        observations = []
        for pair_score, left, right in pairs:
            observation = self._make_observation(left, right, left_k, right_k)
            if observation is not None:
                # 不假设目标身份。选择当前左目视野中最大的有效门框；当
                # 占比相同时，使用检测置信度打破平局。
                observations.append((
                    observation.extent_fraction,
                    pair_score,
                    observation,
                ))
        if observations:
            observations.sort(key=lambda item: (item[0], item[1]),
                              reverse=True)
            return observations[0][2]
        return None

    def _select_locked_right_candidate(
            self, left: GateCandidate,
            right_candidates: list[GateCandidate],
            reference: GateObservation,
            height: int) -> GateCandidate | None:
        """为已锁定的左目候选选择同一目标的右目候选。

        右目不能单纯选择最大框。先用双目视差符号和极线垂直误差过滤，
        再优先匹配上一帧右目位置；这样另一扇更大的门进入视野时不会抢锁。
        """

        left_pixel = self._candidate_pixel_center(left)
        right_reference = None
        if not reference.monocular:
            right_reference = self._candidate_pixel_center(reference.right)
        matches = []
        for candidate in right_candidates:
            right_pixel = self._candidate_pixel_center(candidate)
            disparity = float(left_pixel[0] - right_pixel[0])
            if disparity <= 1.0:
                continue
            vertical_error = abs(float(left_pixel[1] - right_pixel[1]))
            if vertical_error > max(50.0, height * 0.12):
                continue
            size_ratio = max(
                left.extent_fraction / max(candidate.extent_fraction, 1e-6),
                candidate.extent_fraction / max(left.extent_fraction, 1e-6),
            )
            if size_ratio > 1.8:
                continue
            if right_reference is None:
                right_distance = 0.0
            else:
                right_distance = float(np.linalg.norm(
                    (right_pixel - right_reference)
                    / np.array([_IMAGE_WIDTH, _IMAGE_HEIGHT],
                               dtype=np.float64)))
                if right_distance > _LOCK_MAX_CENTER_DELTA_FRACTION:
                    continue
            extent_delta = abs(math.log(
                max(candidate.extent_fraction, 1e-6)
                / max(left.extent_fraction, 1e-6)))
            score = (
                vertical_error / max(height, 1)
                + right_distance
                + 0.05 * extent_delta
            )
            matches.append((score, candidate))
        if not matches:
            self._last_observation_reason = '锁定门框没有匹配的右目极线候选'
            return None
        matches.sort(key=lambda item: item[0])
        return matches[0][1]

    def _observe_stereo(self, reference: GateObservation | None = None):
        """获取双目配对观测；有参考值时只跟踪已锁定门框。"""

        camera_context = self._camera_pair()
        if camera_context is None:
            return None
        left_image, right_image, left_k, right_k = camera_context
        detection_context = self._detection_pair()
        if detection_context is None:
            return None
        left_message, right_message = detection_context
        left_height, left_width = left_image.shape[:2]
        right_height, right_width = right_image.shape[:2]
        left_candidates = self._detection_candidates(
            left_message, left_width, left_height)
        right_candidates = self._detection_candidates(
            right_message, right_width, right_height)
        if not left_candidates or not right_candidates:
            self._last_observation_reason = '双目中至少一目未检测到有效门框'
            return None

        if reference is None:
            observation = self._pair_candidates(
                left_candidates, right_candidates, left_k, right_k,
                min(left_height, right_height))
            if observation is None:
                self._last_observation_reason = '左右目门框无法通过极线配对'
            return observation

        left = self._select_locked_candidate(
            left_candidates, reference.left)
        if left is None:
            return None
        right = self._select_locked_right_candidate(
            left, right_candidates, reference,
            min(left_height, right_height))
        if right is None:
            return None
        observation = self._make_observation(left, right, left_k, right_k)
        if observation is None:
            self._last_observation_reason = '锁定门框双目三角测量失败'
        return observation

    def _wait_observation(self, timeout: float) -> GateObservation | None:
        deadline = time.monotonic() + max(0.0, timeout)
        while (time.monotonic() < deadline and rclpy_ok()
               and not self._node.stopped):
            observation = self._observe()
            if observation is not None:
                return observation
            time.sleep(0.04)
        return None

    def _wait_stereo_observation(
            self, timeout: float,
            reference: GateObservation | None = None) -> GateObservation | None:
        """在不重新扫描的情况下等待一组双目锁定观测。"""

        deadline = time.monotonic() + max(0.0, timeout)
        while (time.monotonic() < deadline and rclpy_ok()
               and not self._node.stopped):
            observation = self._observe_stereo(reference)
            if observation is not None:
                return observation
            time.sleep(0.04)
        return None

    # ── 运动控制 ─────────────────────────────────────────────────────

    def _rotate_to(self, yaw: float, label: str) -> bool:
        yaw = _wrap_degrees(yaw)
        success, message = self._node._send_action_goal(
            BasicMotion.Goal.SET,
            [self._node._cmd_x, self._node._cmd_y,
             self._node._cmd_z, yaw],
            'rz', timeout=self._rotate_timeout, quiet=True,
            task_context=self._node._format_motion_context(label),
        )
        if success:
            self._node._cmd_yaw = yaw
            return True
        self._logger.warning(f'过门任务：{label}失败：{message}')
        return False

    def _rotate_velocity_to(self, yaw: float, label: str) -> bool:
        """使用恒定机体偏航角速度转到指定航向。

        过去的搜索会为每个 30 度采样点发送一次位置 SET，导致每次较长
        动作期间相机都在观察移动目标，并在动作结束时留下过期的选定观测。
        现在搜索与视觉伺服使用相同的机体速度接口：在采样航向之间以固定
        偏航角速度移动，到达指定航向后发送中性指令。
        """

        target = _wrap_degrees(yaw)
        try:
            current = float(self._node._latest_robot_pose()[5])
            if not math.isfinite(current):
                raise ValueError
        except (AttributeError, TypeError, IndexError, ValueError):
            current = float(self._node._cmd_yaw)
        initial_error = _wrap_degrees(target - current)
        if abs(initial_error) <= 1.0:
            self._node._cmd_yaw = target
            return True

        # 允许测量姿态滞后于速度指令，但仍保留配置的超时时间作为安全
        # 上限。
        timeout = max(
            self._rotate_timeout,
            abs(initial_error) / self._scan_yaw_rate * 2.5 + 1.0)
        deadline = time.monotonic() + timeout
        last_publish = float('-inf')
        velocity_started = False
        reached = False
        self._logger.info(
            f'过门任务：{label}以恒定偏航角速度 '
            f'{current:.1f}°->{target:.1f}°，速度为 '
            f'{self._scan_yaw_rate:.1f}°/秒')
        try:
            while (time.monotonic() < deadline and rclpy_ok()
                   and not self._node.stopped):
                try:
                    current = float(self._node._latest_robot_pose()[5])
                    if not math.isfinite(current):
                        raise ValueError
                except (AttributeError, TypeError, IndexError, ValueError):
                    current = float(self._node._cmd_yaw)
                error = _wrap_degrees(target - current)
                if abs(error) <= 1.5:
                    reached = True
                    break

                now = time.monotonic()
                if now - last_publish >= self._scan_publish_period:
                    yaw_rate = math.copysign(self._scan_yaw_rate, error)
                    self._node._publish_body_velocity(
                        yaw_rate_deg_s=yaw_rate)
                    velocity_started = True
                    last_publish = now
                time.sleep(0.02)
        finally:
            if velocity_started:
                self._node._publish_body_velocity()

        try:
            measured_yaw = float(self._node._latest_robot_pose()[5])
            if math.isfinite(measured_yaw):
                self._node._cmd_yaw = _wrap_degrees(measured_yaw)
            elif reached:
                self._node._cmd_yaw = target
        except (AttributeError, TypeError, IndexError, ValueError):
            if reached:
                self._node._cmd_yaw = target

        if not reached:
            self._logger.warning(
                f'过门任务：{label}恒速旋转超时 '
                f'（目标={target:.1f}°，当前={self._node._cmd_yaw:.1f}°）')
        return reached

    def _body_step(self, dx: float, dy: float, dz: float, dyaw: float,
                   label: str) -> bool:
        dx, dy, dz, dyaw = float(dx), float(dy), float(dz), float(dyaw)
        if max(abs(dx), abs(dy), abs(dz), abs(dyaw)) < 1e-5:
            return True
        success, message = self._node._send_action_goal(
            BasicMotion.Goal.BMOVE,
            [dx, dy, dz, dyaw],
            'xyzrz', timeout=self._command_timeout, quiet=True,
            task_context=self._node._format_motion_context(label),
        )
        if not success:
            self._logger.warning(f'过门任务：{label}失败：{message}')
            return False
        yaw = math.radians(self._node._cmd_yaw)
        self._node._cmd_x += math.cos(yaw) * dx - math.sin(yaw) * dy
        self._node._cmd_y += math.sin(yaw) * dx + math.cos(yaw) * dy
        self._node._cmd_z += dz
        self._node._cmd_yaw = _wrap_degrees(self._node._cmd_yaw + dyaw)
        return True

    def _sweep_for_largest(self, start_yaw: float):
        """扫描航向区间，或在门框足够高时立即停止并锁定。"""

        sweep = self._search_sweep_deg
        direction = 1.0 if sweep >= 0.0 else -1.0
        target_yaw = _wrap_degrees(start_yaw + sweep)
        deadline = time.monotonic() + self._scan_timeout
        best = None
        best_heading = start_yaw
        self._search_stopped_on_height = False
        last_publish = float('-inf')
        velocity_started = False

        def measured_yaw():
            try:
                value = float(self._node._latest_robot_pose()[5])
                if math.isfinite(value):
                    return _wrap_degrees(value)
            except (AttributeError, TypeError, IndexError, ValueError):
                pass
            return _wrap_degrees(float(self._node._cmd_yaw))

        self._logger.info(
            f'过门任务：扫描 {_wrap_degrees(start_yaw):.1f}° -> '
            f'{target_yaw:.1f}°，速度为 {self._scan_yaw_rate:.1f}°/秒')
        try:
            while (time.monotonic() < deadline and rclpy_ok()
                   and not self._node.stopped):
                current_yaw = measured_yaw()
                observation = self._observe()
                if observation is not None:
                    height_fraction = max(
                        float(observation.left.height_px),
                        float(observation.right.height_px),
                    ) / max(_IMAGE_HEIGHT, 1)
                    if height_fraction > self._search_stop_height_fraction:
                        best = observation
                        best_heading = current_yaw
                        self._search_stopped_on_height = True
                        self._logger.info(
                            '过门任务：门框高度占图像高度为 '
                            f'{height_fraction:.3f}，超过阈值 '
                            f'{self._search_stop_height_fraction:.3f}；'
                            '停止扫描并锁定当前门框')
                        break
                if (observation is not None
                        and observation.extent_fraction >= self._min_extent
                        and (best is None
                             or observation.extent_fraction
                             > best.extent_fraction)):
                    best = observation
                    best_heading = current_yaw
                    self._logger.info(
                        f'过门任务：当前最大边界框已更新，航向为 '
                        f'{best_heading:.1f}°，占比={best.extent_fraction:.3f}')

                error = _wrap_degrees(target_yaw - current_yaw)
                if direction * error <= 1.5:
                    break
                now = time.monotonic()
                if now - last_publish >= self._scan_publish_period:
                    self._node._publish_body_velocity(
                        yaw_rate_deg_s=direction * self._scan_yaw_rate)
                    velocity_started = True
                    last_publish = now
                time.sleep(0.02)
        finally:
            if velocity_started:
                self._node._publish_body_velocity()

        return best, best_heading

    def _scan_headings(self, initial: bool) -> GateObservation | None:
        """首次面向东方，然后转到 -30° 并扫描 +60°。"""

        start_yaw = float(self._node._cmd_yaw)
        if initial:
            # 在 NED 坐标系中，从上方俯视时正偏航为顺时针方向。先建立
            # 面向东方的参考方向，再进行第一次视觉扫描。
            start_yaw += 90.0
            if not self._rotate_to(start_yaw, '开始扫描前位置环顺时针旋转90度'):
                return None
            if self._scan_settle > 0.0:
                time.sleep(self._scan_settle)
            start_yaw = float(self._node._cmd_yaw)

        # -30° 的扫描起点相对于当前参考方向。因而第一次过门的默认扫描
        # 范围是以东方为中心的 -30° 到 +30°。
        start_yaw += self._search_start_offset_deg
        if not self._rotate_to(start_yaw, '位置环转到搜索起点-30度'):
            return None
        if self._scan_settle > 0.0:
            time.sleep(self._scan_settle)
        start_yaw = float(self._node._cmd_yaw)

        best, best_heading = self._sweep_for_largest(start_yaw)
        if best is None:
            self._logger.error(
                '过门任务：扫描未找到占比不小于 '
                f'{self._min_extent:.2f} 的左目门框')
            return None

        if self._search_stopped_on_height:
            # 已经在当前航向停止，保留触发阈值的观测，不能重新搜索。
            # 这里仅等待同一门框的右目配对，不允许换成另一个更大的框。
            self._logger.info(
                '过门任务：保留高度阈值触发时的门框，'
                f'航向={best_heading:.1f}°，不再重新搜索；等待双目锁定')
        else:
            if not self._rotate_to(best_heading, '位置环转到最大门视线'):
                return None
            if self._scan_settle > 0.0:
                time.sleep(self._scan_settle)
        # 进入对准前必须完成一次双目锁定。位置环转向可能会短暂影响
        # 观测，因此给感知一段时间重新配对；等待期间不再改变目标。
        fresh_timeout = max(
            self._post_turn_observation_timeout,
            self._scan_settle * 3.0)
        best = self._wait_stereo_observation(fresh_timeout, best)
        if best is None:
            self._logger.error(
                '过门任务：未获得锁定门框的双目极线配对观测')
            return None
        if self._search_stopped_on_height:
            self._logger.info(
                f'过门任务：已锁定高度阈值门框，'
                f'占比={best.extent_fraction:.3f}')
        else:
            self._logger.info(
                f'过门任务：已选择当前视野中的最大门框，'
                f'占比={best.extent_fraction:.3f}')
        return best

    def _image_center_errors(self, observation: GateObservation):
        """返回所选门框的归一化图像误差。

        对准阶段使用左右目 bbox 中心的平均水平误差作为 yaw PID 输入，
        并额外保留左右目垂直误差，用于双目极线一致性判断。搜索阶段的
        单目观测仍会将左目数值镜像到兼容字段中。
        """

        def normalized_error(point, camera_matrix):
            point = np.asarray(point, dtype=np.float64).reshape(2)
            fx = max(float(camera_matrix[0, 0]), 1.0)
            fy = max(float(camera_matrix[1, 1]), 1.0)
            return (
                (float(point[0]) - float(camera_matrix[0, 2])) / fx,
                (float(point[1]) - float(camera_matrix[1, 2])) / fy,
            )

        # 使用检测器的边界框中心作为伺服观测量。可选的中心线/分割点仍
        # 可用于双目几何，但当管道被遮挡或在图像边缘被裁剪时，它不一定
        # 是光学中心。
        left_point = (observation.left.bbox_center
                      if observation.left.bbox_center is not None
                      else observation.left.center)
        right_point = (observation.right.bbox_center
                       if observation.right.bbox_center is not None
                       else observation.right.center)
        left_u, left_v = normalized_error(
            left_point, self._camera_k['left'])
        if observation.monocular:
            return {
                'left_u': left_u,
                'right_u': left_u,
                'left_v': left_v,
                'right_v': left_v,
                'u': left_u,
                'v': left_v,
                'vertical_disagreement': 0.0,
            }

        right_u, right_v = normalized_error(
            right_point, self._camera_k['right'])
        return {
            'left_u': left_u,
            'right_u': right_u,
            'left_v': left_v,
            'right_v': right_v,
            'u': 0.5 * (left_u + right_u),
            'v': 0.5 * (left_v + right_v),
            'vertical_disagreement': abs(left_v - right_v),
        }

    def _image_is_centered(self, observation: GateObservation) -> bool:
        """判断当前相机视野是否以门框为中心。"""

        errors = self._image_center_errors(observation)
        return (
            abs(errors['u']) <= self._image_center_tolerance
            and abs(errors['left_v']) <= self._image_center_tolerance
            and abs(errors['right_v']) <= self._image_center_tolerance
            and errors['vertical_disagreement']
            <= self._stereo_vertical_tolerance
        )

    def _height_distance_is_ready(self, observation: GateObservation) -> bool:
        """判断门框高度和距离是否已准备好进行横向搜索。"""

        errors = self._image_center_errors(observation)
        bbox_height_fraction = max(
            float(observation.left.height_px),
            float(observation.right.height_px)) / max(_IMAGE_HEIGHT, 1)
        return (
            max(abs(errors['left_v']), abs(errors['right_v']))
            <= self._image_center_tolerance
            and errors['vertical_disagreement']
            <= self._stereo_vertical_tolerance
            and abs(self._target_height_fraction - bbox_height_fraction)
            <= self._extent_tolerance
        )

    def _height_distance_delta(self, observation: GateObservation):
        """只修正 z 方向和距离；横向/偏航在第二阶段保持不变。"""

        errors = self._image_center_errors(observation)
        distance = _clamp(
            observation.distance_m,
            self._distance_control_min_m,
            self._distance_control_max_m)
        bbox_height_fraction = max(
            float(observation.left.height_px),
            float(observation.right.height_px)) / max(_IMAGE_HEIGHT, 1)
        size_error = self._target_height_fraction - bbox_height_fraction
        vertical_error = errors['v'] * distance

        dx = self._distance_gain * size_error * distance
        if dx >= 0.0:
            dx = _clamp(dx, 0.0, self._max_forward_step)
        else:
            dx = _clamp(dx, -self._max_back_step, 0.0)
        dz = _clamp(
            self._vertical_gain * vertical_error,
            -self._max_vertical_step, self._max_vertical_step)
        return dx, 0.0, dz, 0.0, vertical_error, size_error

    def _reset_servo_state(self):
        """重置滤波、偏航/高度 PID 和横向弧线搜索状态。"""

        self._filtered_gate = None
        self._yaw_pid_integral = 0.0
        self._yaw_pid_previous_error = None
        self._yaw_pid_previous_time = None
        self._height_pid_integral = 0.0
        self._height_pid_previous_error = None
        self._height_pid_previous_time = None
        self._arc_direction = 1.0
        self._last_arc_lateral_speed = 0.0
        self._arc_near_extremum = False
        self._arc_last_reversal_time = float('-inf')
        self._arc_initial_window_pending = True
        self._arc_window_start_objective = None
        self._arc_window_start_observation_angle = None
        self._arc_window_start_time = None
        self._arc_window_last_delta = 0.0
        self._arc_window_last_observation_angle_delta = 0.0
        # 首个窗口只负责积攒完整的首段探测样本；后续方向由窗口端点
        # 的 J/观察角变化决定，窗口中途不改方向。

    def _filter_gate(self, observation: GateObservation) -> FilteredGate:
        """对边界框测量值和标定后的高度代理值进行低通滤波。"""

        errors = self._image_center_errors(observation)
        if observation.monocular:
            width_px = float(observation.left.width_px)
            height_px = float(observation.left.height_px)
        else:
            width_px = 0.5 * (float(observation.left.width_px)
                              + float(observation.right.width_px))
            height_px = 0.5 * (float(observation.left.height_px)
                               + float(observation.right.height_px))
        raw = FilteredGate(
            center_u=float(errors['u']),
            center_v=float(errors['v']),
            width_px=max(width_px, 1.0),
            height_px=max(height_px, 1.0),
            center_z_m=float(observation.center_body[2]),
            extent_fraction=max(
                width_px / max(_IMAGE_WIDTH, 1),
                height_px / max(_IMAGE_HEIGHT, 1)),
        )
        previous = getattr(self, '_filtered_gate', None)
        if previous is None:
            filtered = raw
        else:
            alpha = self._bbox_filter_alpha
            filtered = FilteredGate(
                center_u=previous.center_u + alpha * (raw.center_u - previous.center_u),
                center_v=previous.center_v + alpha * (raw.center_v - previous.center_v),
                width_px=previous.width_px + alpha * (raw.width_px - previous.width_px),
                height_px=previous.height_px + alpha * (raw.height_px - previous.height_px),
                center_z_m=previous.center_z_m + alpha * (raw.center_z_m - previous.center_z_m),
                extent_fraction=previous.extent_fraction + alpha * (
                    raw.extent_fraction - previous.extent_fraction),
            )
        self._filtered_gate = filtered
        return filtered

    @staticmethod
    def _arc_objective(filtered: FilteredGate) -> float:
        """返回横向搜索的目标函数 J=width/height。"""

        return filtered.width_px / max(filtered.height_px, 1.0)

    @staticmethod
    def _observation_angle_deg(filtered: FilteredGate) -> float:
        """返回门框相对相机光轴的水平观察角，而不是世界系 yaw。"""
        return math.degrees(math.atan(float(filtered.center_u)))

    def _update_arc_window(self, filtered: FilteredGate, now: float,
                           allow_direction_update: bool = True):
        """Evaluate the lateral direction once per fixed, non-overlapping window.

        The lateral command remains unchanged during one window.  At the
        boundary, the filtered objective and camera observation angle at the
        two ends are recorded, then the resulting direction is used for the next
        window.  This prevents frame-rate noise from changing the arc
        direction halfway through a segment.
        """
        objective = self._arc_objective(filtered)
        observation_angle = self._observation_angle_deg(filtered)
        start_objective = self._arc_window_start_objective
        start_observation_angle = self._arc_window_start_observation_angle
        start_time = self._arc_window_start_time
        initial_window = getattr(self, '_arc_initial_window_pending', True)
        window_seconds = getattr(
            self, '_arc_probe_window_seconds', self._arc_window_seconds)
        if not initial_window:
            window_seconds = self._arc_window_seconds

        if (start_objective is None or start_observation_angle is None
                or start_time is None):
            self._arc_window_start_objective = objective
            self._arc_window_start_observation_angle = observation_angle
            self._arc_window_start_time = now
            self._logger.info(
                '过门任务：开始累计弧线窗口；'
                f'方向={self._arc_direction:+.0f}，'
                f'至少等待 {window_seconds:.1f} 秒后判断 ΔJ')
            return objective, self._arc_near_extremum, 0.0

        if now - start_time < window_seconds:
            return objective, self._arc_near_extremum, 0.0

        objective_delta = objective - start_objective
        observation_angle_delta = _wrap_degrees(
            observation_angle - start_observation_angle)
        if (allow_direction_update
                and objective_delta < -self._arc_objective_deadband):
            if now - self._arc_last_reversal_time >= self._arc_reverse_cooldown:
                self._arc_direction *= -1.0
                self._arc_last_reversal_time = now
                self._arc_near_extremum = False
                decision = '反向'
            else:
                decision = '保持（反向冷却中）'
        elif (allow_direction_update
              and abs(objective_delta) <= self._arc_objective_deadband):
            self._arc_near_extremum = True
            decision = '保持并降低速度（接近极值）'
        else:
            self._arc_near_extremum = False
            decision = '保持'

        self._arc_window_last_delta = objective_delta
        self._arc_window_last_observation_angle_delta = (
            observation_angle_delta)
        was_initial_window = getattr(
            self, '_arc_initial_window_pending', True)
        self._arc_initial_window_pending = False
        self._logger.info(
            '过门任务：弧线窗口结束；'
            f'J={start_objective:.3f}->{objective:.3f}，'
            f'ΔJ={objective_delta:+.3f}，'
            f'观察角={start_observation_angle:.1f}->'
            f'{observation_angle:.1f}°，'
            f'Δ观察角={observation_angle_delta:+.1f}°，'
            f'{"首段完成；" if was_initial_window else ""}'
            f'下一窗口方向={self._arc_direction:+.0f}（{decision}）')

        # The current endpoint is the reference for the next non-overlapping
        # fixed segment. It is deliberately not updated on intermediate
        # frames, so this is not a rolling window.
        self._arc_window_start_objective = objective
        self._arc_window_start_observation_angle = observation_angle
        self._arc_window_start_time = now
        return objective, self._arc_near_extremum, objective_delta

    def _height_pid(self, filtered: FilteredGate, now: float):
        """返回垂向 PID 输出、误差和滤波后的误差导数。"""

        error = filtered.center_z_m - self._vertical_center_offset
        previous_time = self._height_pid_previous_time
        if previous_time is None:
            dt = self._velocity_period
            derivative = 0.0
        else:
            dt = _clamp(now - previous_time, 0.01, 0.5)
            previous_error = (self._height_pid_previous_error
                              if self._height_pid_previous_error is not None
                              else error)
            derivative = (error - previous_error) / dt
        self._height_pid_integral = _clamp(
            self._height_pid_integral + error * dt,
            -self._height_pid_integral_limit,
            self._height_pid_integral_limit)
        output = (
            self._height_pid_kp * error
            + self._height_pid_ki * self._height_pid_integral
            + self._height_pid_kd * derivative
        )
        self._height_pid_previous_error = error
        self._height_pid_previous_time = now
        return _clamp(output, -self._max_vertical_speed,
                      self._max_vertical_speed), error, derivative

    def _yaw_pid_update(self, center_u: float, now: float):
        """根据双目图像中心的水平角误差更新偏航 PID。

        ``center_u`` 是左右目 bbox 中心的归一化水平中点。将它换算成
        角度后再做 PID，可以让 YAML 中的增益直接对应 °/秒 和 °，也能
        用同一误差的导数判断目标是否真的停止摆动。
        """

        error_deg = math.degrees(math.atan(float(center_u)))
        previous_time = getattr(self, '_yaw_pid_previous_time', None)
        if previous_time is None:
            dt = max(0.01, float(getattr(self, '_velocity_period', 0.05)))
            derivative = 0.0
        else:
            dt = _clamp(now - previous_time, 0.01, 0.5)
            previous_error = getattr(self, '_yaw_pid_previous_error', None)
            if previous_error is None:
                previous_error = error_deg
            derivative = (error_deg - previous_error) / dt

        integral_limit = max(
            0.01, float(getattr(
                self, '_yaw_pid_integral_limit_deg', 10.0)))
        integral = getattr(self, '_yaw_pid_integral', 0.0) + error_deg * dt
        self._yaw_pid_integral = _clamp(
            integral, -integral_limit, integral_limit)
        self._yaw_pid_previous_error = error_deg
        self._yaw_pid_previous_time = now

        kp = float(getattr(
            self, '_yaw_pid_kp', getattr(self, '_yaw_center_kp', 3.6)))
        ki = float(getattr(self, '_yaw_pid_ki', 0.0))
        kd = float(getattr(self, '_yaw_pid_kd', 0.0))
        output = kp * error_deg + ki * self._yaw_pid_integral + kd * derivative
        output = _clamp(
            output, -float(getattr(self, '_max_yaw_rate', 36.0)),
            float(getattr(self, '_max_yaw_rate', 36.0)))
        return output, error_deg, derivative

    def _servo_velocity(self, observation: GateObservation,
                        filtered: FilteredGate, height_output: float,
                        height_active: bool, attitude_active: bool,
                        arc_probe: bool = False, now: float | None = None,
                        yaw_pid_output: float | None = None):
        """生成统一 4-DOF 视觉伺服指令。

        ``vx`` 根据门框尺寸调距离，``vz`` 使用高度 PID，``vy`` 沿固定
        弧线方向搜索 ``J=width/height``。偏航由 ``-vy/d`` 运动学前馈
        和双目图像中心的 yaw PID 组成。
        """

        distance = _clamp(
            observation.distance_m,
            self._distance_control_min_m,
            self._distance_control_max_m)
        # 前后距离只看 bbox 高度，不再使用 max(width_fraction,
        # height_fraction) 的 extent 代理。这样门的横向姿态变化不会把
        # 宽度变化误当成距离误差。
        height_fraction = filtered.height_px / max(_IMAGE_HEIGHT, 1)
        size_error = self._target_height_fraction - height_fraction
        forward = self._distance_velocity_gain * size_error * distance
        if forward >= 0.0:
            forward = _clamp(forward, 0.0, self._max_forward_speed)
        else:
            forward = _clamp(forward, -self._max_reverse_speed, 0.0)
        vertical = height_output if height_active else 0.0

        lateral = 0.0
        yaw_rate = 0.0
        if attitude_active:
            arc_speed = (self._arc_probe_speed if arc_probe
                         else self._arc_lateral_speed)
            if (not arc_probe and getattr(self, '_arc_near_extremum', False)):
                arc_speed *= self._arc_near_extremum_speed_scale
            lateral = _clamp(
                self._arc_direction * arc_speed,
                -self._max_lateral_speed, self._max_lateral_speed)

            # 相对门框模型：x_dot=-vx+r*y, y_dot=-vy-r*x，因此保持门框
            # 视线方向不变所需的偏航前馈为 -vy/d。反馈项使用双目 bbox
            # 中心角的 PID，最终与前馈相加后输出 deg/s。
            feedforward_deg_s = math.degrees(
                -lateral / max(distance, self._distance_control_min_m))
            if yaw_pid_output is None:
                if now is None:
                    now = time.monotonic()
                yaw_pid_output, _, _ = self._yaw_pid_update(
                    filtered.center_u, now)
            yaw_rate = feedforward_deg_s + float(yaw_pid_output)
            yaw_rate = _clamp(
                yaw_rate, -self._max_yaw_rate, self._max_yaw_rate)
        self._last_arc_lateral_speed = lateral
        return forward, lateral, vertical, yaw_rate

    def _servo_delta(self, observation: GateObservation,
                     adjust_distance: bool = True):
        """计算分阶段视觉伺服指令。

        在双目图像中点居中之前，只允许进行图像居中修正；尤其是 dx
        必须严格为零。居中后启用距离误差，同时继续使用相同的图像误差
        闭合回路，因此前进不会在不知不觉中使门框偏离双目中心。
        """

        normal = observation.normal_body
        yaw_error = 0.0
        if observation.normal_reliable:
            yaw_error = math.degrees(
                math.atan2(float(normal[1]), float(normal[0])))
        image_errors = self._image_center_errors(observation)
        centered = self._image_is_centered(observation)

        # 观测距离只用于缩放角度图像误差。有界距离可以避免双目配对异常
        # 时，在重新获取锁定过程中产生较大的横向/垂向跳变。
        distance = _clamp(
            observation.distance_m,
            self._distance_control_min_m,
            self._distance_control_max_m)
        lateral_error = image_errors['u'] * distance
        vertical_error = image_errors['v'] * distance
        height_fraction = max(
            float(observation.left.height_px),
            float(observation.right.height_px)) / max(_IMAGE_HEIGHT, 1)
        size_error = self._target_height_fraction - height_fraction

        dx = 0.0
        if adjust_distance and centered:
            dx = self._distance_gain * size_error * distance
            if dx >= 0.0:
                dx = _clamp(dx, 0.0, self._max_forward_step)
            else:
                dx = _clamp(dx, -self._max_back_step, 0.0)
        dy = _clamp(
            self._lateral_gain * lateral_error,
            -self._max_lateral_step, self._max_lateral_step)
        dz = _clamp(
            self._vertical_gain * vertical_error,
            -self._max_vertical_step, self._max_vertical_step)
        # 不要让法向量估计干扰第一阶段的图像居中。图像居中后，同时修正
        # 偏航和距离，并由图像回路保持门框居中。
        dyaw = 0.0 if not centered else _clamp(
            self._yaw_gain * yaw_error,
            -self._max_yaw_step, self._max_yaw_step)
        if centered and abs(dyaw) > 1e-6:
            # 正机体偏航会将居中的目标转向负图像 u。同步施加负机体 y
            # 方向平移，可以在 AUV 朝门框法向转动时保持目标位于双目中点。
            # 这是之前控制器缺少的关键耦合；之前的控制器会先转向，几秒
            # 后才重新居中。
            forward_range = _clamp(
                float(observation.center_body[0]),
                self._distance_control_min_m,
                self._distance_control_max_m)
            dy += -math.tan(math.radians(dyaw)) * forward_range
            dy = _clamp(
                dy, -self._max_lateral_step, self._max_lateral_step)
        return dx, dy, dz, dyaw, lateral_error, vertical_error, yaw_error

    def _stop_observe_and_turn(self,
                               observation_hint: GateObservation | None) -> bool:
        """停止，在短时间窗口内估计航向，然后发送一次偏航 SET。

        边界框尺寸保持条件是距离触发器。在航向检查期间不会发送速度
        指令，因此门框不会因同步运动修正而被推出视野。圆均值使单次
        位置环偏航指令不易受某个噪声边界框中心的影响。
        """

        self._node._publish_body_velocity()
        center_yaw_samples = []
        deadline = time.monotonic() + self._heading_check_seconds
        locked_reference = getattr(self, '_locked_observation', None)
        while (time.monotonic() < deadline and rclpy_ok()
               and not self._node.stopped):
            if locked_reference is None:
                observation = self._observe_stereo()
            else:
                observation = self._observe_locked(locked_reference)
                if observation is not None:
                    locked_reference = observation
                    self._locked_observation = observation
            if observation is not None:
                # 使用水平边界框中心，而不是重建得到的门框平面法向量。
                # 归一化图像 u 误差等价于边界框中心相对于相机的方位角。
                filtered = self._filter_gate(observation)
                center_yaw_samples.append(math.atan2(
                    float(filtered.center_u), 1.0))
            time.sleep(0.02)

        if center_yaw_samples:
            sine = sum(math.sin(angle) for angle in center_yaw_samples)
            cosine = sum(math.cos(angle) for angle in center_yaw_samples)
            yaw_error = math.degrees(math.atan2(sine, cosine))
        elif getattr(self, '_filtered_gate', None) is not None:
            yaw_error = math.degrees(math.atan2(
                float(self._filtered_gate.center_u), 1.0))
        elif observation_hint is not None:
            image_errors = self._image_center_errors(observation_hint)
            yaw_error = math.degrees(math.atan2(
                float(image_errors['u']), 1.0))
        else:
            self._logger.error(
                '过门任务：停止后没有门框边界框')
            return False

        try:
            pose = self._node._latest_robot_pose()
            self._node._cmd_x = pose[0]
            self._node._cmd_y = pose[1]
            self._node._cmd_z = pose[2]
            current_yaw = float(pose[5])
        except (AttributeError, TypeError, IndexError, ValueError):
            current_yaw = float(self._node._cmd_yaw)

        target_yaw = _wrap_degrees(current_yaw + yaw_error)
        self._logger.info(
            f'过门任务：已停止；边界框中心已检查 '
            f'{self._heading_check_seconds:.1f} 秒，发送一次位置环偏航 '
            f'SET（{current_yaw:.1f}° -> {target_yaw:.1f}°）')
        if not self._rotate_to(target_yaw, '停止后位置环朝向对准'):
            return False
        if self._post_turn_settle_seconds > 0.0:
            time.sleep(self._post_turn_settle_seconds)
        return True

    def _align_and_hold(self, observation_hint: GateObservation | None = None) -> bool:
        """滤波边界框，运行高度/弧线伺服，然后通过门框。"""

        self._reset_servo_state()
        # 进入对准后沿用扫描阶段选中的门框。后续检测只允许在这个
        # 门框附近更新，不能因为另一门的 bbox 更大而切换目标。
        locked_observation = observation_hint
        self._locked_observation = locked_observation
        alignment_started = time.monotonic()
        deadline = alignment_started + self._alignment_timeout
        lost_since = None
        servo_stage = None
        ratio_hold_since = None
        height_stable_since = None
        attitude_correction_started = None
        attitude_minimum_reported = False
        attitude_done = False
        height_done = False
        velocity_started = False
        last_velocity_publish = float('-inf')
        last_bbox_log = float('-inf')
        last_velocity_command = None
        lost_zero_sent = False

        def publish_velocity(forward=0.0, lateral=0.0,
                             vertical=0.0, yaw_rate=0.0):
            nonlocal velocity_started, last_velocity_publish
            nonlocal last_velocity_command
            last_velocity_command = (
                float(forward), float(lateral),
                float(vertical), float(yaw_rate))
            self._node._publish_body_velocity(
                forward_mps=forward,
                lateral_mps=lateral,
                vertical_mps=vertical,
                yaw_rate_deg_s=yaw_rate,
            )
            velocity_started = True
            last_velocity_publish = time.monotonic()

        try:
            while (time.monotonic() < deadline and rclpy_ok()
                   and not self._node.stopped):
                if locked_observation is None:
                    observation = self._observe_stereo()
                else:
                    observation = self._observe_locked(locked_observation)
                    if observation is not None:
                        locked_observation = observation
                        self._locked_observation = observation
                now = time.monotonic()
                if observation is None:
                    # 所有完成条件都要求连续满足。当前视野缺少双目门框
                    # 时，会中断姿态和高度的保持计时器。
                    if not attitude_done:
                        ratio_hold_since = None
                    # 不跨越丢帧间隔计算 ΔJ，避免恢复时的跳变误判为
                    # 横向极值方向。弧线方向保留，重新获得连续观测后
                    # 再继续估计趋势。
                    # 窗口端点必须来自同一段连续观测；丢帧后重新开窗，
                    # 但不改变已经选定的弧线方向。
                    self._arc_window_start_objective = None
                    self._arc_window_start_observation_angle = None
                    self._arc_window_start_time = None
                    self._arc_near_extremum = False
                    if not height_done:
                        height_stable_since = None
                    if lost_since is None:
                        lost_since = now
                        self._logger.warning(
                            '过门任务：双目门框观测丢失；保持上一条速度指令 '
                            f'{self._lost_command_hold_seconds:.1f} 秒 '
                            f'（{self._last_observation_reason}）')
                        lost_zero_sent = False
                    if velocity_started:
                        lost_duration = now - lost_since
                        if (lost_duration < self._lost_command_hold_seconds
                                and last_velocity_command is not None
                                and now - last_velocity_publish
                                >= self._velocity_period):
                            publish_velocity(*last_velocity_command)
                        elif (lost_duration >= self._lost_command_hold_seconds
                              and not lost_zero_sent):
                            self._logger.warning(
                                '过门任务：双目门框观测在 '
                                f'{self._lost_command_hold_seconds:.1f} 秒后仍未恢复；'
                                '速度已置零')
                            publish_velocity()
                            lost_zero_sent = True
                        elif (lost_zero_sent
                              and now - last_velocity_publish
                              >= self._velocity_period):
                            publish_velocity()
                    if now - lost_since > self._lost_timeout:
                        self._logger.error('过门任务：伺服过程中门框丢失')
                        return False
                    time.sleep(0.02)
                    continue

                lost_since = None
                lost_zero_sent = False
                filtered = self._filter_gate(observation)
                image_errors = self._image_center_errors(observation)
                yaw_pid_output = None
                yaw_error_deg = 0.0
                yaw_error_derivative = 0.0
                if not attitude_done:
                    # 只要弧线阶段尚未完成，每个双目观测都更新一次偏航
                    # PID 状态。该输出只负责保持正确的运动方向；弧线
                    # 是否停止由 J 的目标和保持时间决定，不能被偏航角
                    # 是否已经小于某个阈值阻塞。
                    yaw_pid_output, yaw_error_deg, yaw_error_derivative = (
                        self._yaw_pid_update(image_errors['u'], now))
                if attitude_correction_started is None:
                    attitude_correction_started = now
                    self._logger.info(
                        '过门任务：开始计算姿态最短修正时间，至少 '
                        f'{self._minimum_attitude_correction_seconds:.1f} 秒')
                if attitude_done:
                    bbox_ratio = self._arc_objective(filtered)
                    objective_near_extremum = self._arc_near_extremum
                    objective_delta = 0.0
                else:
                    # A direction is held for one complete window, then the
                    # endpoint J/yaw pair chooses the next window segment.
                    bbox_ratio, objective_near_extremum, objective_delta = (
                        self._update_arc_window(filtered, now))
                initial_window_active = (
                    not attitude_done
                    and getattr(self, '_arc_initial_window_pending', True))
                attitude_minimum_met = (
                    attitude_correction_started is not None
                    and now - attitude_correction_started
                    >= self._minimum_attitude_correction_seconds)
                if (attitude_minimum_met and not attitude_minimum_reported):
                    attitude_minimum_reported = True
                    self._logger.info(
                        '过门任务：姿态最短修正时间已满足，开始评估 J')
                # 初始 bbox 可能已经很大，但在首个完整窗口结束、J 连续
                # 达到目标保持一段时间前，不允许姿态阶段瞬间完成。阶段
                # 切换不再额外约束 bbox 水平中心或 |ΔJ|；横移方向判断
                # 仍在 _update_arc_window() 中独立运行。J 达标后立即停止
                # 弧线横移，最终偏航由后续位置环修正。
                if (not attitude_done and not initial_window_active
                        and attitude_minimum_met):
                    if bbox_ratio >= self._bbox_ratio_target:
                        if ratio_hold_since is None:
                            ratio_hold_since = now
                            self._logger.info(
                                '过门任务：横向目标函数 J 达到目标 '
                                f'{self._bbox_ratio_target:.2f}（当前 '
                                f'J={bbox_ratio:.2f}）；保持 '
                                f'{self._bbox_ratio_hold_seconds:.1f} 秒')
                        elif (now - ratio_hold_since
                              >= self._bbox_ratio_hold_seconds):
                            attitude_done = True
                            self._logger.info(
                                '过门任务：J 已达标并保持，'
                                '横向弧线搜索已停止；'
                                f'J={bbox_ratio:.2f} 已保持 '
                                f'{self._bbox_ratio_hold_seconds:.1f} 秒；'
                                '当前偏航由后续位置环修正')
                    else:
                        ratio_hold_since = None
                elif not attitude_done:
                    ratio_hold_since = None

                height_output = 0.0
                height_error = 0.0
                height_derivative = 0.0
                if not height_done:
                    height_output, height_error, height_derivative = (
                        self._height_pid(filtered, now))
                    height_stable = (
                        abs(height_error) <= self._height_pid_stable_error_m
                        and abs(height_derivative)
                        <= self._height_pid_stable_derivative_mps)
                    if height_stable:
                        if height_stable_since is None:
                            height_stable_since = now
                            self._logger.info(
                                '过门任务：高度 PID 进入稳定窗口')
                        elif now - height_stable_since >= self._height_pid_stable_seconds:
                            height_done = True
                            height_output = 0.0
                            self._logger.info(
                                '过门任务：高度 PID 已稳定；停止垂向调整')
                    else:
                        height_stable_since = None

                stage = (
                    f'姿态={"完成" if attitude_done else "弧线搜索"}，'
                    f'高度={"完成" if height_done else "PID"}')
                if stage != servo_stage:
                    self._logger.info(f'过门任务：{stage}')
                    servo_stage = stage

                if attitude_done and height_done:
                    self._logger.info(
                        '过门任务：姿态和高度均已完成；停止并根据边界框中心'
                        '进行位置环转向')
                    self._locked_observation = locked_observation
                    return self._stop_observe_and_turn(observation)

                if (not velocity_started
                        or now - last_velocity_publish >= self._velocity_period):
                    forward, lateral, vertical, yaw_rate = self._servo_velocity(
                        observation, filtered, height_output,
                        height_active=not height_done,
                        attitude_active=not attitude_done,
                        arc_probe=initial_window_active,
                        now=now,
                        yaw_pid_output=yaw_pid_output)
                    publish_velocity(forward, lateral, vertical, yaw_rate)
                    self._logger.debug(
                        f'过门任务：速度 vx={forward:.3f} '
                        f'vy={lateral:.3f} vz={vertical:.3f} '
                        f'偏航角速度={yaw_rate:.2f}°/秒，'
                        f'J={bbox_ratio:.2f}，ΔJ={objective_delta:+.3f}，'
                        f'弧线方向={self._arc_direction:+.0f}，'
                        f'高度误差={height_error:.3f}，'
                        f'偏航误差={yaw_error_deg:.2f}°，'
                        f'极线垂直误差={image_errors["vertical_disagreement"]:.4f}')
                    if now - last_bbox_log >= self._bbox_log_period:
                        self._logger.info(
                            '过门任务：当前滤波 bbox：'
                            f'宽={filtered.width_px:.1f}px，'
                            f'高={filtered.height_px:.1f}px，'
                            f'J=宽/高={bbox_ratio:.3f}，'
                            f'ΔJ={objective_delta:+.3f}，'
                            f'vy={lateral:+.3f}米/秒，'
                            f'弧线方向={self._arc_direction:+.0f}')
                        last_bbox_log = now
                time.sleep(0.02)
        finally:
            # 任务失败、取消或将控制权交给 BTRAVEL 时，绝不能让 ZIT6
            # 控制器停留在非零速度模式。
            if velocity_started:
                publish_velocity()
            # 速度控制会改变测量姿态，但不会改变 TaskRunner 的位置指令
            # 缓存。同步该缓存，使下一次搜索的绝对偏航 SET 基于 AUV 的
            # 真实位置发送，而不是基于通过当前门框之前的位置发送。
            try:
                pose = self._node._latest_robot_pose()
                (self._node._cmd_x, self._node._cmd_y,
                 self._node._cmd_z, self._node._cmd_yaw) = (
                    pose[0], pose[1], pose[2], pose[5])
            except (AttributeError, TypeError, IndexError):
                pass
            self._locked_observation = None

        self._logger.error('过门任务：对准超时')
        return False

    def _pass_current_gate(self, index: int) -> bool:
        self._logger.info(
            f'过门任务：[{index}/{self._gates_to_pass}] '
            f'向前移动 {self._pass_distance:.2f} 米通过门框')
        success, message = self._node._send_action_goal(
            BasicMotion.Goal.BTRAVEL,
            [self._pass_distance, 0.0, 0.0, 0.0],
            'x', timeout=self._pass_timeout, quiet=False,
            task_context=self._node._format_motion_context(
                f'第{index}个门前进通过'),
        )
        if not success:
            self._logger.error(f'过门任务：第 {index} 个门通过失败：{message}')
            return False
        # BTRAVEL 由 BasicMotion 根据测量姿态闭环执行。重新读取该姿态，
        # 不要基于过期的速度控制前位置指令缓存进行积分。
        try:
            pose = self._node._latest_robot_pose()
            (self._node._cmd_x, self._node._cmd_y,
             self._node._cmd_z, self._node._cmd_yaw) = (
                pose[0], pose[1], pose[2], pose[5])
        except (AttributeError, TypeError, IndexError):
            yaw = math.radians(self._node._cmd_yaw)
            self._node._cmd_x += math.cos(yaw) * self._pass_distance
            self._node._cmd_y += math.sin(yaw) * self._pass_distance
        if self._post_pass_pause > 0.0:
            time.sleep(self._post_pass_pause)
        return True

    def execute(self) -> bool:
        """执行顺时针搜索，并通过配置数量的门框。"""

        deadline = time.monotonic() + self._task_timeout
        initial = True
        for index in range(1, self._gates_to_pass + 1):
            if self._node.stopped or not rclpy_ok() or time.monotonic() >= deadline:
                return False
            self._logger.info(
                f'过门任务：正在搜索第 {index}/{self._gates_to_pass} 个门')
            selected = self._scan_headings(initial=initial)
            initial = False
            if selected is None:
                return False
            if time.monotonic() >= deadline:
                return False

            self._logger.info(
                f'过门任务：已选择第 {index} 个门；已完成左右目配对，'
                f'机体坐标代理中心={selected.center_body.tolist()}，'
                f'边界框占比={selected.extent_fraction:.3f}')
            if not self._align_and_hold(selected):
                return False
            if not self._pass_current_gate(index):
                return False
        self._logger.info(f'过门任务：已完成 {self._gates_to_pass} 个门框')
        return True


def rclpy_ok() -> bool:
    """保持测试导入轻量，同时保留正常的 ROS 状态检查。"""

    try:
        import rclpy
        return bool(rclpy.ok())
    except (ImportError, RuntimeError):
        return True
