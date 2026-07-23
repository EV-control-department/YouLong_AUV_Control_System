"""管道巡线子任务 — 自包含感知订阅、PID 和搜索逻辑。

仅在 ``follow_line`` 任务执行期间存活。构造函数创建下视
LineState / DetectionArray 订阅，``destroy()`` 清理所有资源，
因此不会对其他任务产生副作用。
"""

import math
import threading
import time

import numpy as np

from uv_msgs.action import BasicMotion
from uv_msgs.msg import Detection, DetectionArray, LineState, PoseInfo


# ==========================================================================
# 下视双目相机参数（与 position.py CAM_PARAMS 保持一致）
# ==========================================================================

_DOWN_HFOV = 87.19
_DOWN_WIDTH = 1280
_DOWN_HEIGHT = 960
_DOWN_FX = _DOWN_WIDTH / (2.0 * math.tan(math.radians(_DOWN_HFOV) / 2.0))
_DOWN_FY = _DOWN_FX
_DOWN_CX = _DOWN_WIDTH / 2.0
_DOWN_CY = _DOWN_HEIGHT / 2.0

# 下视相机在机体 NED 系中的安装偏移
_DOWN_OFFSET_LEFT = np.array([-0.13, -0.05, 0.0645])
_DOWN_OFFSET_RIGHT = np.array([-0.13, 0.05, 0.0645])

# 下视相机 optical → body NED 旋转矩阵
_DOWN_OPTICAL_TO_BODY = np.array([[0, -1, 0],
                                   [1, 0, 0],
                                   [0, 0, 1]])


# ==========================================================================
# 模块级辅助函数
# ==========================================================================


def _clamp(value: float, low: float, high: float) -> float:
    """将控制量限制在指定上下限内。"""
    return max(low, min(high, value))


def _wrap_degrees(angle: float) -> float:
    """将角度归一化到 [-180, 180]。"""
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def _euler_to_rotation_matrix(rx_deg: float, ry_deg: float,
                               rz_deg: float) -> 'np.ndarray':
    """ZYX 欧拉角 (度) → 旋转矩阵（与 position.py 一致）。"""
    rx, ry, rz = math.radians(rx_deg), math.radians(ry_deg), math.radians(rz_deg)
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    return np.array([
        [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
        [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
        [-sy,     cy * sx,                cy * cx],
    ])


def _ray_intersection_midpoint(o1: 'np.ndarray', d1: 'np.ndarray',
                                o2: 'np.ndarray', d2: 'np.ndarray'):
    """两射线公垂线中点。返回 (x,y,z) 或 None（平行/交点在身后）。"""
    w = o1 - o2
    a = float(np.dot(d1, d1))
    b = float(np.dot(d1, d2))
    c_val = float(np.dot(d2, d2))
    denom = a * c_val - b * b
    if abs(denom) < 1e-10:
        return None
    e = float(np.dot(d1, w))
    f = float(np.dot(d2, w))
    t1 = (b * f - c_val * e) / denom
    t2 = (a * f - b * e) / denom
    if t1 <= 0 or t2 <= 0:
        return None
    return (o1 + d1 * t1 + o2 + d2 * t2) / 2.0


def _apply_body_delta(cmd_x, cmd_y, cmd_z, cmd_yaw, dx, dy, dz, dyaw):
    """BMOVE 成功后更新命令跟踪值（FRD 增量 → NED 世界系）。"""
    yaw_rad = math.radians(cmd_yaw)
    return (
        cmd_x + math.cos(yaw_rad) * dx - math.sin(yaw_rad) * dy,
        cmd_y + math.sin(yaw_rad) * dx + math.cos(yaw_rad) * dy,
        cmd_z + dz,
        _wrap_degrees(cmd_yaw + dyaw),
    )


# ==========================================================================
# LineFollower
# ==========================================================================


class LineFollower:
    """管道巡线子任务 — 自包含感知订阅、PID 和搜索逻辑。"""

    def __init__(self, node: 'TaskRunnerNode', params: dict):
        self._node = node
        self._params = params
        self._logger = node.get_logger()
        self._stopped = False

        # ── 感知订阅（仅下视相机） ──
        self._lock = threading.RLock()
        self._line_states = {}       # camera_name → (monotonic, LineState)
        self._down_detections = {}   # camera_name → (monotonic, DetectionArray)
        self._robot_pose = None      # (x, y, z, roll_deg, pitch_deg, yaw_deg)
        self._subs = []
        for cam in ('down_left', 'down_right'):
            self._subs.append(node.create_subscription(
                LineState, f'/perception/line/{cam}',
                lambda msg, c=cam: self._line_cb(c, msg), 10))
            self._subs.append(node.create_subscription(
                DetectionArray, f'/perception/detection/{cam}',
                lambda msg, c=cam: self._det_cb(c, msg), 10))
        self._subs.append(node.create_subscription(
            PoseInfo, '/basic_motion/pose_info', self._pose_cb, 10))

        # ── PID 状态（横向和偏航独立控制）──
        # 横向 PID: center_error → dy (把 AUV 推回管道中心)
        self._lat_prev_err = None
        self._lat_integral = 0.0
        self._lat_last_t = None
        # 偏航 PID: heading_error_deg → dyaw (对齐管道方向)
        self._head_prev_err = None
        self._head_integral = 0.0
        self._head_last_t = None

        # ── YOLO 丢帧恢复 ──
        self._lost_count = 0           # 连续丢帧计数
        self._lost_max = int(params.get('lost_tolerance', 100))
        self._last_valid_line = None   # 最后有效的 LineState (用于盲跟)

        # ── 标记 class_id（可通过 tasks.json 参数覆盖）──
        self._triangle_cid = int(params.get('triangle_class_id', 5))
        self._square_cid = int(params.get('square_class_id', 6))

        # ── 搜索状态 ──
        self._step_count = 0           # 巡线步数计数
        self._search_spiral_dir = 0    # 螺旋方向计数 (0…7)
        self._search_spiral_size = float(params.get('search_spiral_start', 0.02))

        # ── 三角形/正方形标记抑制 + 任务计数 ──
        self._triangle_approached = False  # True=刚处理过三角形，跳过检测
        self._square_approached = False    # True=刚处理过正方形，跳过检测
        self._triangle_count = 0           # 已完成三角形任务数（下潜）
        self._square_count = 0             # 已完成正方形任务数（旋转）

        self._logger.info(
            f'LineFollower created: timeout={params.get("timeout", 120)}s')

    # ── 感知回调 ──────────────────────────────────────────────────────

    def _line_cb(self, camera_name: str, msg: LineState):
        with self._lock:
            self._line_states[camera_name] = (time.monotonic(), msg)

    def _det_cb(self, camera_name: str, msg: DetectionArray):
        with self._lock:
            self._down_detections[camera_name] = (time.monotonic(), msg)

    def _pose_cb(self, msg: PoseInfo):
        self._robot_pose = (msg.robot_x, msg.robot_y, msg.robot_z,
                            msg.robot_roll, msg.robot_pitch, msg.robot_yaw)

    # ── 感知融合 ──────────────────────────────────────────────────────

    def _latest_line(self):
        """融合左右下视 LineState。

        center_error 使用普通平均；heading_error_deg 是 180° 周期量，
        使用双角度圆周平均，避免 +89° 和 -89° 被错误平均为 0°。
        返回 None 表示当前没有有效的管道检测。
        """
        max_age = float(self._params.get('perception_max_age', 0.60))
        now = time.monotonic()
        with self._lock:
            states = [
                self._line_states[name][1]
                for name in ('down_left', 'down_right')
                if name in self._line_states
                and now - self._line_states[name][0] <= max_age
                and self._line_states[name][1].detected
            ]

        if not states:
            return None
        if len(states) == 1:
            return states[0]

        state = LineState()
        state.detected = True
        state.center_error = sum(
            item.center_error for item in states) / len(states)
        doubled = [math.radians(2.0 * item.heading_error_deg)
                   for item in states]
        mean_x = sum(math.cos(a) for a in doubled)
        mean_y = sum(math.sin(a) for a in doubled)
        state.heading_error_deg = math.degrees(
            math.atan2(mean_y, mean_x) / 2.0)
        state.area_ratio = sum(
            item.area_ratio for item in states) / len(states)
        return state

    def _best_detection(self, class_ids=None):
        """返回置信度最高的 ``(Detection, camera_name)``。

        用于 ``stop_when_marker``：当检测到指定类别时以该点为终点。
        """
        wanted = None if class_ids is None else set(class_ids)
        candidates = []
        max_age = float(self._params.get('perception_max_age', 0.60))
        now = time.monotonic()
        with self._lock:
            arrays = list(self._down_detections.items())
        for camera_name, (received_at, array) in arrays:
            if now - received_at > max_age:
                continue
            for det in array.detections:
                if wanted is None or det.class_id in wanted:
                    candidates.append((det, camera_name))
        if not candidates:
            return None
        return max(candidates, key=lambda item: item[0].confidence)

    # ── 主循环 ────────────────────────────────────────────────────────

    def execute(self) -> bool:
        """搜索 → 跟踪主循环。返回 True=成功，False=超时或停止。

        YOLO 丢帧恢复：连续丢帧 ≤ lost_tolerance 时，用最后有效
        LineState 继续盲跟一小段；超过阈值后才进入搜索模式。
        """
        timeout = float(self._params.get('timeout', 120.0))
        deadline = time.monotonic() + timeout
        self._logger.info(
            f'LineFollower start: timeout={timeout:.0f}s, '
            f'stop_when_marker={self._params.get("stop_when_marker", False)}, '
            f'lost_tolerance={self._lost_max} frames')

        while time.monotonic() < deadline and not self._stopped:
            line = self._latest_line()
            if line is not None:
                # 有管道 → 正常跟踪
                self._lost_count = 0
                self._last_valid_line = line
                ok = self._follow_step(line)
                if not ok:
                    return False
                # ── 三角形标记处理 ──
                # 抑制恢复：三角形 bbox 中心在图像上 1/3 区域 → 新三角形出现
                det = self._best_detection([self._triangle_cid])
                if det is not None:
                    bbox_center_y = (det[0].bbox_y1 + det[0].bbox_y2) / 2.0
                    height = float(self._params.get(
                        'image_height',
                        self._node.get_parameter('down_image_height').value))
                    if self._triangle_approached:
                        if bbox_center_y < height / 3.0:
                            self._triangle_approached = False
                            self._logger.info(
                                'LineFollower: triangle in upper 1/3, '
                                'flag reset to 0')

                    if (not self._triangle_approached
                            and bbox_center_y > height / 5.0):
                        det = self._best_detection([self._triangle_cid])
                        if det is not None:
                            self._logger.info(
                                f'LineFollower: triangle (class={self._triangle_cid}) detected, '
                                'approaching')
                            # 1. 对准三角形
                            self._node._align_to_class(
                                self._triangle_cid, 'triangle')
                            # 亮红灯 — 识别到三角形
                            self._node.set_light(
                                self._node.LIGHT_RED, 'triangle marker detected')
                            self._logger.info(
                                '🚨 LineFollower: RED LIGHT — triangle marker!')
                            time.sleep(1)
                            # 关灯
                            self._node.light_off()
                            # 2. 下沉 0.5m → 上升 0.5m
                            self._node._send_action_goal(
                                BasicMotion.Goal.BMOVE,
                                [0.0, 0.0, 0.5, 0.0], 'z',
                                timeout=15.0, quiet=True)
                            self._node._cmd_z += 0.5
                            self._node._send_action_goal(
                                BasicMotion.Goal.BMOVE,
                                [0.0, 0.0, -0.5, 0.0], 'z',
                                timeout=15.0, quiet=True)
                            self._node._cmd_z -= 0.5
                            # 3. 标记已处理
                            self._triangle_approached = True
                            self._triangle_count += 1
                            total = self._triangle_count + self._square_count
                            self._logger.info(
                                f'LineFollower: triangle handled, '
                                f'task #{self._triangle_count} done, '
                                f'total={total}/4')

                # ── 正方形标记处理 ──
                # 抑制恢复：正方形 bbox 中心在图像上 1/3 区域 → 新正方形出现
                det = self._best_detection([self._square_cid])
                if det is not None:
                    bbox_center_y = (det[0].bbox_y1 + det[0].bbox_y2) / 2.0
                    height = float(self._params.get(
                        'image_height',
                        self._node.get_parameter('down_image_height').value))
                    if self._square_approached:
                        if bbox_center_y < height / 3.0:
                            self._square_approached = False
                            self._logger.info(
                                'LineFollower: square in upper 1/3, '
                                'flag reset to 0')

                    if (not self._square_approached
                            and bbox_center_y > height / 5.0):
                        det = self._best_detection([self._square_cid])
                        if det is not None:
                            self._logger.info(
                                f'LineFollower: square (class={self._square_cid}) detected, '
                                'approaching')
                            # 1. 对准正方形
                            self._node._align_to_class(
                                self._square_cid, 'square')
                            # 闪两次绿灯 — 识别到正方形
                            for flash in range(2):
                                self._node.set_light(
                                    self._node.LIGHT_GREEN,
                                    f'square marker — flash #{flash+1}')
                                self._logger.info(
                                    f'🟢 LineFollower: GREEN LIGHT #{flash+1} '
                                    f'— square marker!')
                                time.sleep(0.5)
                                # 关灯
                                self._node.light_off()
                                time.sleep(0.5)
                            # 2. 自转 360°（3 × 120° BMOVE rz）
                            self._logger.info(
                                'LineFollower: rotating 360° (3×120°)')
                            for _ in range(3):
                                if self._stopped:
                                    return False
                                self._node._send_action_goal(
                                    BasicMotion.Goal.BMOVE,
                                    [0.0, 0.0, 0.0, 120.0], 'rz',
                                    timeout=15.0, quiet=True)
                                self._node._cmd_yaw += 120.0
                            # 3. 标记已处理
                            self._square_approached = True
                            self._square_count += 1
                            total = self._triangle_count + self._square_count
                            self._logger.info(
                                f'LineFollower: square handled, '
                                f'task #{self._square_count} done, '
                                f'total={total}/4')
            elif self._lost_count < self._lost_max and self._last_valid_line is not None:
                # 短暂丢帧 → 用最后的有效方向盲跟一小步
                self._lost_count += 1
                self._logger.info(
                    f'LineFollower: lost frame #{self._lost_count}/'
                    f'{self._lost_max}, coasting with last heading')
                ok = self._follow_step(self._last_valid_line, coast_mode=True)
                if not ok:
                    return False
            else:
                # 持续丢帧 → 已完成 4 个任务则巡线成功
                total = self._triangle_count + self._square_count
                if total >= 4:
                    self._logger.info(
                        f'LineFollower: {total} tasks completed, '
                        f'line lost → mission complete')
                    return True
                # 否则进入搜索模式
                self._lost_count = 0
                self._last_valid_line = None
                if not self._search_for_line():
                    time.sleep(0.05)

        expired = time.monotonic() >= deadline
        self._logger.info(
            f'LineFollower: {"timeout" if expired else "stopped"}')
        return False

    # ── 搜索阶段 ──────────────────────────────────────────────────────

    def _search_for_line(self) -> bool:
        """方框螺旋搜索 — 仅平移，不旋转。

        按 8 方向循环走方框：前→右前→右→右后→后→左后→左→左前，
        每完成一圈 (8 步) 后框大小增加 ``search_spiral_step``，
        直到达到 ``search_max_spiral``。
        返回 True=执行了移动动作，False=搜索范围已耗尽。
        """
        size = self._search_spiral_size
        max_spiral = float(self._params.get('search_max_spiral', 2.0))
        if size > max_spiral:
            return False

        max_step = float(self._params.get('search_max_step', 0.50))
        step = min(size, max_step)

        dirs = [
            (step, 0.0), (step, step), (0.0, step), (-step, step),
            (-step, 0.0), (-step, -step), (0.0, -step), (step, -step),
        ]
        dx, dy = dirs[self._search_spiral_dir]

        self._logger.info(
            f'Line search: dir={self._search_spiral_dir} '
            f'size={size:.2f}m step=({dx:.2f}, {dy:.2f})')

        success, _ = self._node._send_action_goal(
            BasicMotion.Goal.BMOVE,
            [float(dx), float(dy), 0.0, 0.0], 'xy', timeout=8.0,
            quiet=True)
        if success:
            self._node._cmd_x, self._node._cmd_y, \
            self._node._cmd_z, self._node._cmd_yaw = _apply_body_delta(
                self._node._cmd_x, self._node._cmd_y,
                self._node._cmd_z, self._node._cmd_yaw,
                dx, dy, 0.0, 0.0)

        self._search_spiral_dir = (self._search_spiral_dir + 1) % 8
        if self._search_spiral_dir == 0:
            spiral_step = float(
                self._params.get('search_spiral_step', 0.30))
            self._search_spiral_size += spiral_step
            self._logger.info(
                f'Line search: size increased to '
                f'{self._search_spiral_size:.2f}m')

        return success

    # ── PID 巡线步 ────────────────────────────────────────────────────

    def _follow_step(self, line: LineState, coast_mode: bool = False) -> bool:
        """基于 LineState 执行一次 PID 闭环巡线。

        控制映射（双通道独立 PID）：
        - center_error → 横向 PID → dy（机体系横移，把 AUV 推回管道中心）
        - heading_error_deg → 偏航 PID → dyaw（旋转对齐管道方向）
        转向较大时自动降低前进步长。

        coast_mode: 丢帧盲跟模式 — 用最后有效数据前进，不更新 I/D 状态，
        前进量减半，dyaw 清零（不做无视觉反馈的旋转）。
        """
        if coast_mode:
            # 盲跟：只用最后有效 center_error 做比例修正，不更新 PID 状态
            lateral_sign = float(self._params.get('line_lateral_sign', -1.0))
            max_lateral = abs(float(self._params.get('max_lateral_step', 0.03)))
            dy = _clamp(lateral_sign * line.center_error * 0.15, -max_lateral, max_lateral)
            dyaw = 0.0
            forward = float(self._params.get('coast_forward_step', 0.05))

            self._step_count += 1
            self._logger.info(
                f'Line follow #{self._step_count} [coast]: '
                f'fwd={forward:.3f} dy={dy:.3f} dyaw=0 '
                f'center={line.center_error:.3f}')

            success, _ = self._node._send_action_goal(
                BasicMotion.Goal.BMOVE,
                [forward, dy, 0.0, 0.0], 'xy', timeout=8.0,
                quiet=True)
            if success:
                self._node._cmd_x, self._node._cmd_y, \
                self._node._cmd_z, self._node._cmd_yaw = _apply_body_delta(
                    self._node._cmd_x, self._node._cmd_y,
                    self._node._cmd_z, self._node._cmd_yaw,
                    forward, dy, 0.0, 0.0)
            return success

        image_width = float(self._params.get(
            'image_width',
            self._node.get_parameter('down_image_width').value))

        # ── 横向 PID：center_error → dy ──────────────────────────────
        pixel_err = line.center_error * (image_width / 2.0)  # 归一化 → 像素
        now = time.monotonic()

        dy, lat_pid_out = self._pid_step(
            pixel_err, now,
            prev_err_attr='_lat_prev_err',
            integral_attr='_lat_integral',
            last_t_attr='_lat_last_t',
            pid_p=self._params.get('line_pid_p', 0.009),
            pid_i=self._params.get('line_pid_i', 0.0),
            pid_d=self._params.get('line_pid_d', 0.0),
            output_limit=self._params.get('line_pid_output_limit', 0.30),
            reset_dt=self._params.get('line_pid_reset_dt', 1.0),
            integral_limit_px=self._params.get('line_pid_integral_limit_px', 1000.0),
        )
        lateral_sign = float(self._params.get('line_lateral_sign', -1.0))
        max_lateral = abs(float(self._params.get('max_lateral_step', 0.03)))
        dy = -_clamp(lateral_sign * dy, -max_lateral, max_lateral)

        # ── 偏航 PID：heading_error_deg → dyaw ────────────────────────
        heading_err = line.heading_error_deg  # 管道长轴与竖直方向的夹角

        dyaw, head_pid_out = self._pid_step(
            heading_err, now,
            prev_err_attr='_head_prev_err',
            integral_attr='_head_integral',
            last_t_attr='_head_last_t',
            pid_p=self._params.get('heading_pid_p', 0.15),
            pid_i=self._params.get('heading_pid_i', 0.005),
            pid_d=self._params.get('heading_pid_d', 0.02),
            output_limit=self._params.get('heading_pid_output_limit', 6.0),
            reset_dt=self._params.get('heading_pid_reset_dt', 1.5),
            integral_limit_px=self._params.get('heading_integral_limit', 60.0),
        )
        yaw_sign = float(self._params.get('line_yaw_sign', 1.0))
        max_yaw = abs(float(self._params.get('max_yaw_step', 3.0)))
        dyaw = _clamp(yaw_sign * dyaw, -max_yaw, max_yaw)

        # ── 前进步长：修正量越大越慢 ──────────────────────────────────
        forward_base = float(self._params.get('forward_step', 0.10))
        min_forward = max(0.0, float(self._params.get('min_forward_step', 0.02)))
        total_correction = abs(dy / max(max_lateral, 0.001)) + abs(dyaw / max(max_yaw, 0.001))
        speed_ratio = max(0.25, 1.0 - 0.5 * total_correction)
        forward = max(min_forward, forward_base * speed_ratio)

        self._step_count += 1
        self._logger.debug(
            f'Line follow #{self._step_count}: '
            f'fwd={forward:.3f} dy={dy:.3f}(pid={lat_pid_out:.2f}) '
            f'dyaw={dyaw:.2f}°(pid={head_pid_out:.2f}) '
            f'center={line.center_error:.3f} head={line.heading_error_deg:.1f}°')
        if self._step_count % 20 == 1:
            self._logger.info(
                f'Line follow #{self._step_count}: '
                f'fwd={forward:.3f} dy={dy:.3f} dyaw={dyaw:.2f}° '
                f'center={line.center_error:.3f} head={line.heading_error_deg:.1f}°')

        # 发送 BMOVE xyrz（同步前向 + 横向 + 偏航）
        success, _ = self._node._send_action_goal(
            BasicMotion.Goal.BMOVE,
            [forward, dy, 0.0, dyaw], 'xyrz', timeout=8.0,
            quiet=True)
        if success:
            self._node._cmd_x, self._node._cmd_y, \
            self._node._cmd_z, self._node._cmd_yaw = _apply_body_delta(
                self._node._cmd_x, self._node._cmd_y,
                self._node._cmd_z, self._node._cmd_yaw,
                forward, dy, 0.0, dyaw)
        return success

    def _pid_step(self, error: float, now: float,
                  prev_err_attr: str, integral_attr: str, last_t_attr: str,
                  pid_p, pid_i, pid_d, output_limit, reset_dt,
                  integral_limit_px):
        """通用 PID 步进，通过属性名读写状态以支持多通道。

        Returns:
            (raw_output, clamped_pid_output) — 调用方自行限幅应用。
        """
        prev_err = getattr(self, prev_err_attr)
        integral = getattr(self, integral_attr)
        last_t = getattr(self, last_t_attr)

        dt = now - last_t if last_t is not None else 0.0
        reset_dt = float(reset_dt)

        if last_t is None or dt <= 0.0 or dt > reset_dt:
            integral = 0.0
            derivative = 0.0
        else:
            integral += error * dt
            limit = abs(float(integral_limit_px))
            integral = _clamp(integral, -limit, limit)
            if prev_err is not None:
                derivative = (error - prev_err) / dt
            else:
                derivative = 0.0

        setattr(self, prev_err_attr, error)
        setattr(self, integral_attr, integral)
        setattr(self, last_t_attr, now)

        output = (float(pid_p) * error
                  + float(pid_i) * integral
                  + float(pid_d) * derivative)
        pid_max = abs(float(output_limit))
        output = _clamp(output, -pid_max, pid_max)
        return output, output

    # ── 双目三角测量 ──────────────────────────────────────────────────

    def _stereo_pair(self, class_id: int):
        """返回左右目对 class_id 的最佳匹配检测 ``(left_det, right_det)``。

        任一眼缺失或过期则返回 None。
        """
        max_age = float(self._params.get('perception_max_age', 0.60))
        now = time.monotonic()
        with self._lock:
            left_entry = self._down_detections.get('down_left')
            right_entry = self._down_detections.get('down_right')

        if left_entry is None or right_entry is None:
            return None
        left_t, left_msg = left_entry
        right_t, right_msg = right_entry
        if now - left_t > max_age or now - right_t > max_age:
            return None

        best_left = max(
            (d for d in left_msg.detections if d.class_id == class_id),
            key=lambda d: d.confidence, default=None)
        best_right = max(
            (d for d in right_msg.detections if d.class_id == class_id),
            key=lambda d: d.confidence, default=None)
        if best_left is None or best_right is None:
            return None
        return (best_left, best_right)

    def _pixel_to_world_ray(self, px: float, py: float,
                             offset: 'np.ndarray', R_robot: 'np.ndarray',
                             robot_pos: 'np.ndarray'):
        """像素坐标 → 世界系射线 (origin, direction)。"""
        v_cam = np.array([(px - _DOWN_CX) / _DOWN_FX,
                          (py - _DOWN_CY) / _DOWN_FY, 1.0])
        v_cam = v_cam / np.linalg.norm(v_cam)
        v_body = _DOWN_OPTICAL_TO_BODY @ v_cam
        v_world = R_robot @ v_body
        v_world = v_world / np.linalg.norm(v_world)
        origin = robot_pos + R_robot @ offset
        return origin, v_world

    def _triangulate_object(self, class_id: int):
        """对 class_id 进行双目三角测量，返回世界坐标 ``(x, y, z)`` 或 None。"""
        pair = self._stereo_pair(class_id)
        if pair is None or self._robot_pose is None:
            return None

        left_det, right_det = pair
        rx, ry, rz, roll, pitch, yaw = self._robot_pose

        R_robot = _euler_to_rotation_matrix(roll, pitch, yaw)
        robot_pos = np.array([rx, ry, rz])

        l_origin, l_dir = self._pixel_to_world_ray(
            left_det.pixel_x, left_det.pixel_y,
            _DOWN_OFFSET_LEFT, R_robot, robot_pos)
        r_origin, r_dir = self._pixel_to_world_ray(
            right_det.pixel_x, right_det.pixel_y,
            _DOWN_OFFSET_RIGHT, R_robot, robot_pos)

        pos = _ray_intersection_midpoint(l_origin, l_dir, r_origin, r_dir)
        if pos is None:
            return None
        return (float(pos[0]), float(pos[1]), float(pos[2]))

    # ── 资源清理 ──────────────────────────────────────────────────────

    def destroy(self):
        """销毁所有感知订阅，释放资源。"""
        for sub in self._subs:
            self._node.destroy_subscription(sub)
        self._subs.clear()
        self._logger.info('LineFollower destroyed')
