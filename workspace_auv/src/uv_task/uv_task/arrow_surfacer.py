"""
箭头对准 + ArUco 读数 + 短暂出水子任务。

仅在 ``arrow_surface`` 任务执行期间存活。构造函数创建下视检测 + 前视
ArUco ID 订阅，``destroy()`` 清理所有资源，不会对其他任务产生副作用。
"""

import math
import threading
import time

import numpy as np

from uv_msgs.action import BasicMotion
from uv_msgs.msg import Detection, DetectionArray, PoseInfo
from std_msgs.msg import Int32MultiArray
from uv_camera.model_classes import configured_class_id


# ==========================================================================
# 下视双目相机参数（与 position.py / line_follower.py 保持一致）
# ==========================================================================

_DOWN_HFOV = 87.19
_DOWN_WIDTH = 1280
_DOWN_HEIGHT = 960
_DOWN_FX = _DOWN_WIDTH / (2.0 * math.tan(math.radians(_DOWN_HFOV) / 2.0))
_DOWN_FY = _DOWN_FX
_DOWN_CX = _DOWN_WIDTH / 2.0
_DOWN_CY = _DOWN_HEIGHT / 2.0

_DOWN_OFFSET_LEFT = np.array([-0.13, -0.05, 0.0645])
_DOWN_OFFSET_RIGHT = np.array([-0.13, 0.05, 0.0645])

_DOWN_OPTICAL_TO_BODY = np.array([[0, -1, 0],
                                   [1, 0, 0],
                                   [0, 0, 1]])


# ==========================================================================
# 模块级辅助函数
# ==========================================================================

def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _euler_to_rotation_matrix(rx_deg: float, ry_deg: float,
                               rz_deg: float) -> np.ndarray:
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


def _ray_intersection_midpoint(o1: np.ndarray, d1: np.ndarray,
                                o2: np.ndarray, d2: np.ndarray):
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


# ==========================================================================
# ArrowSurfacer
# ==========================================================================


class ArrowSurfacer:
    """箭头对准子任务 — 搜索箭头，双目接近，读取 ArUco，出水。"""

    # 8 方向搜索序列：前→右前→右→右后→后→左后→左→左前
    _SEARCH_DIRS = [
        (1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (-1.0, 1.0),
        (-1.0, 0.0), (-1.0, -1.0), (0.0, -1.0), (1.0, -1.0),
    ]

    def __init__(self, node, params: dict):
        self._node = node
        self._params = params
        self._logger = node.get_logger()
        self._stopped = False

        # 默认从统一模型元数据读取。当前 robotcup 模型没有箭头/彩色扇区
        # 类别，因此不会再误用旧模型的 0/1/2/3；旧场景可显式传入覆盖值。
        self._arrow_cid = configured_class_id(
            params, 'arrow_class_id', 'arrow', required=False)

        # ── 扇区参数 ──
        self._sector_x = float(params.get('sector_x', 0.0))
        self._sector_y = float(params.get('sector_y', 0.0))
        self._yellow_cid = configured_class_id(
            params, 'yellow_class_id', 'yellow', required=False)
        self._red_cid = configured_class_id(
            params, 'red_class_id', 'red', required=False)
        self._green_cid = configured_class_id(
            params, 'green_class_id', 'green', required=False)
        self._view_yaw = float(params.get('view_yaw', 90.0))
        # 指定投掷扇区：'yellow' / 'green' / 'red'，空=ArUco 读取映射
        self._target_sector = str(params.get('target_sector', '')).strip().lower()
        # 对准后 BMOVE 位移（机体系 x/y）：让信号弹撒放器对准扇区中心
        self._drop_offset_x = float(params.get('drop_offset_x', 0.25))
        self._drop_offset_y = float(params.get('drop_offset_y', 0.15))
        # 完整流程参数
        self._surface_depth = float(params.get('surface_depth', -1.0))    # 出水深度偏移
        self._recover_depth = float(params.get('recover_depth', 0.4))     # 出水后恢复深度
        self._sector_travel_z = float(params.get('sector_travel_z', 0.4)) # 前往扇区深度
        self._servo_reset_delay = float(params.get('servo_reset_delay', 4.0))  # 投球后复位延时
        self._servo_reset_angle = float(params.get('servo_reset_angle',
                                                    getattr(node, 'ANGLE_INIT', 0.0)))

        # ── 感知订阅 ──
        self._lock = threading.RLock()
        self._down_detections = {}   # camera_name → (monotonic, DetectionArray)
        self._robot_pose = None      # (x, y, z, roll_deg, pitch_deg, yaw_deg)
        self._aruco_ids = set()      # 累积收集的 ArUco marker ID
        self._subs = []

        for cam in ('down_left', 'down_right'):
            self._subs.append(node.create_subscription(
                DetectionArray, f'/perception/detection/{cam}',
                lambda msg, c=cam: self._det_cb(c, msg), 10))
        self._subs.append(node.create_subscription(
            PoseInfo, '/basic_motion/pose_info', self._pose_cb, 10))
        self._subs.append(node.create_subscription(
            Int32MultiArray, '/perception/aruco/ids', self._aruco_cb, 10))

        self._logger.info('ArrowSurfacer：已创建')

    # ── 感知回调 ──────────────────────────────────────────────────────

    def _det_cb(self, camera_name: str, msg: DetectionArray):
        with self._lock:
            self._down_detections[camera_name] = (time.monotonic(), msg)

    def _pose_cb(self, msg: PoseInfo):
        self._robot_pose = (msg.robot_x, msg.robot_y, msg.robot_z,
                            msg.robot_roll, msg.robot_pitch, msg.robot_yaw)

    def _aruco_cb(self, msg: Int32MultiArray):
        for mid in msg.data:
            if 1 <= mid <= 6:
                if mid not in self._aruco_ids:
                    self._aruco_ids.add(int(mid))
                    self._logger.info(
                        f'ArrowSurfacer：检测到 ArUco 标记 #{mid}，'
                        f'当前总数={sorted(self._aruco_ids)}')

    # ── 感知融合 ──────────────────────────────────────────────────────

    def _best_detection(self, class_ids=None):
        """返回指定 class_id 中置信度最高的 Detection，或 None。"""
        wanted = None if class_ids is None else set(class_ids)
        max_age = 0.60
        now = time.monotonic()
        candidates = []
        with self._lock:
            arrays = list(self._down_detections.items())
        for _camera_name, (received_at, array) in arrays:
            if now - received_at > max_age:
                continue
            for det in array.detections:
                if wanted is not None and det.class_id not in wanted:
                    continue
                candidates.append(det)
        if not candidates:
            return None
        return max(candidates, key=lambda d: d.confidence)

    # ── 双目三角测量 ──────────────────────────────────────────────────

    def _stereo_pair(self, class_id: int):
        """返回左右目对 class_id 的最佳匹配 (left_det, right_det) 或 None。"""
        max_age = 0.60
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
                             offset: np.ndarray, R_robot: np.ndarray,
                             robot_pos: np.ndarray):
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
        """对 class_id 进行双目三角测量，返回世界坐标 (x,y,z) 或 None。"""
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

    # ── 搜索（通用） ──────────────────────────────────────────────────

    def _search_for_class(self, class_id: int, label: str) -> bool:
        """非阻塞微步螺旋搜索指定 class_id。

        8 方向方框螺旋，每步 0.01m SET + 0.01s timeout（非阻塞），
        步间检查检测结果，一旦发现目标立即返回 True。
        搜索范围耗尽返回 False。
        """
        search_dir = 0
        search_size = 0.02
        search_max = 2.0
        spiral_step = 0.30
        micro_step = 0.01

        while search_size <= search_max:
            if self._stopped:
                return False

            if self._best_detection([class_id]) is not None:
                self._logger.info(f'ArrowSurfacer：搜索过程中发现 {label}！')
                return True

            dx_norm, dy_norm = self._SEARCH_DIRS[search_dir]
            target_dx = search_size * dx_norm
            target_dy = search_size * dy_norm

            total_dist = math.sqrt(target_dx ** 2 + target_dy ** 2)
            n_steps = max(1, int(total_dist / micro_step))
            step_dx = target_dx / n_steps
            step_dy = target_dy / n_steps

            self._logger.info(
                f'ArrowSurfacer 搜索 [{label}]：方向={search_dir}，'
                f'范围={search_size:.2f}m，'
                f'步数={n_steps}（每步 {micro_step:.2f}m）')

            for _ in range(n_steps):
                if self._stopped:
                    return False

                tx = self._node._cmd_x + step_dx
                ty = self._node._cmd_y + step_dy

                self._node._send_action_goal(
                    BasicMotion.Goal.SET,
                    [tx, ty, self._node._cmd_z, self._node._cmd_yaw],
                    'xy', timeout=0.01, quiet=True)
                self._node._cmd_x = tx
                self._node._cmd_y = ty

                if self._best_detection([class_id]) is not None:
                    self._logger.info(
                        f'ArrowSurfacer：微步移动过程中检测到 {label}！')
                    return True

            search_dir = (search_dir + 1) % 8
            if search_dir == 0:
                search_size += spiral_step
                self._logger.info(
                    f'ArrowSurfacer：[{label}] 搜索范围扩大到 '
                    f'{search_size:.2f}m')

        self._logger.warn(
            f'ArrowSurfacer：[{label}] 搜索范围耗尽，未找到目标')
        return False

    def _search_for_arrow(self) -> bool:
        return self._search_for_class(self._arrow_cid, 'arrow')

    # ── ArUco → 扇区映射 ───────────────────────────────────────────

    _ARUCO_SECTOR = {
        1: 'yellow', 2: 'yellow',
        3: 'green',  4: 'green',
        5: 'red',    6: 'red',
    }

    def _aruco_to_sector(self):
        """Return ``(color_name, class_id)`` or None if no ArUco ID detected."""
        if not self._aruco_ids:
            return None
        mid = min(self._aruco_ids)
        color = self._ARUCO_SECTOR.get(mid)
        if color is None:
            return None
        cid = {'yellow': self._yellow_cid,
               'red':    self._red_cid,
               'green':  self._green_cid}[color]
        return (color, cid)

    # ── 前视 ArUco 搜索 ────────────────────────────────────────────

    def _search_aruco_frontal(self):
        """前视搜索 ArUco tag：左右扫 ±15° + 下潜上升 0.5m，最多 2 轮。"""
        if self._aruco_ids:
            return  # 已有数据，无需搜索

        self._logger.info(
            'ArrowSurfacer：尚未获得 ArUco ID，开始前视搜索')

        for rnd in range(2):
            if self._stopped or self._aruco_ids:
                return

            self._logger.info(f'ArrowSurfacer：ArUco 搜索第 {rnd + 1}/2 轮')

            # 右转 15°
            self._node._send_action_goal(
                BasicMotion.Goal.SET,
                [self._node._cmd_x, self._node._cmd_y,
                 self._node._cmd_z, self._view_yaw + 15.0],
                'rz', timeout=10.0, quiet=True)
            self._node._cmd_yaw = self._view_yaw + 15.0
            time.sleep(0.5)
            if self._aruco_ids:
                self._logger.info(
                    f'ArrowSurfacer：在 +15° 处发现 ArUco：'
                    f'{sorted(self._aruco_ids)}')
                return

            # 左转 15°
            self._node._send_action_goal(
                BasicMotion.Goal.SET,
                [self._node._cmd_x, self._node._cmd_y,
                 self._node._cmd_z, self._view_yaw - 15.0],
                'rz', timeout=10.0, quiet=True)
            self._node._cmd_yaw = self._view_yaw - 15.0
            time.sleep(0.5)
            if self._aruco_ids:
                self._logger.info(
                    f'ArrowSurfacer：在 -15° 处发现 ArUco：'
                    f'{sorted(self._aruco_ids)}')
                return

            # 恢复朝向
            self._node._send_action_goal(
                BasicMotion.Goal.SET,
                [self._node._cmd_x, self._node._cmd_y,
                 self._node._cmd_z, self._view_yaw],
                'rz', timeout=10.0, quiet=True)
            self._node._cmd_yaw = self._view_yaw

            # 下潜 0.5m
            self._node._send_action_goal(
                BasicMotion.Goal.BMOVE,
                [0.0, 0.0, 0.5, 0.0], 'z', timeout=10.0, quiet=True)
            self._node._cmd_z += 0.5
            time.sleep(0.5)
            if self._aruco_ids:
                self._logger.info(
                    f'ArrowSurfacer：下潜后发现 ArUco：'
                    f'{sorted(self._aruco_ids)}')
                return

            # 上升 0.5m
            self._node._send_action_goal(
                BasicMotion.Goal.BMOVE,
                [0.0, 0.0, -0.5, 0.0], 'z', timeout=10.0, quiet=True)
            self._node._cmd_z -= 0.5
            time.sleep(0.5)
            if self._aruco_ids:
                self._logger.info(
                    f'ArrowSurfacer：上升后发现 ArUco：'
                    f'{sorted(self._aruco_ids)}')
                return

        self._logger.warn('ArrowSurfacer：ArUco 前视搜索范围耗尽')

    # ── 主执行 ────────────────────────────────────────────────────────

    def execute(self) -> bool:
        """完整箭头对准 + ArUco 读数 + 出水 + 扇区投信标任务。"""

        if self._arrow_cid is None:
            self._logger.error(
                'ArrowSurfacer：当前模型元数据没有箭头类别；'
                '旧模型请提供 arrow_class_id，或更新模型元数据')
            return False
        if (self._yellow_cid is None and self._red_cid is None
                and self._green_cid is None):
            self._logger.error(
                'ArrowSurfacer：当前模型元数据没有扇区类别；'
                '旧模型请提供 yellow/red/green 类别 ID')
            return False

        # 1. 转向 view_yaw
        self._logger.info(f'ArrowSurfacer：旋转到偏航角={self._view_yaw:.1f}°')
        success, msg = self._node._send_action_goal(
            BasicMotion.Goal.SET,
            [self._node._cmd_x, self._node._cmd_y,
             self._node._cmd_z, self._view_yaw],
            'rz', timeout=15.0)
        if not success:
            self._logger.error(f'ArrowSurfacer：setrz 失败：{msg}')
            return False
        self._node._cmd_yaw = self._view_yaw
        self._logger.info(
            f'ArrowSurfacer：旋转完成，偏航角={self._view_yaw:.1f}°')

        # 2. 搜索并对准箭头
        self._logger.info(
            f'ArrowSurfacer：对准箭头（class={self._arrow_cid}）')
        if not self._node._align_to_class(self._arrow_cid, '箭头'):
            self._logger.warn('ArrowSurfacer：箭头对准失败')
            return False

        # 3. 短暂出水：读取水面 ArUco
        self._logger.info(
            f'ArrowSurfacer：上浮（setz={self._surface_depth}）……'
            f'已收集 ArUco ID：{sorted(self._aruco_ids)}')
        self._node._send_action_goal(
            BasicMotion.Goal.WMOVE,
            [self._node._cmd_x, self._node._cmd_y, self._surface_depth,
             self._node._cmd_yaw],
            'z', timeout=10.0, quiet=True)

        # 4. 恢复深度
        self._logger.info(
            f'ArrowSurfacer：恢复到深度 {self._recover_depth}')
        success, msg = self._node._send_action_goal(
            BasicMotion.Goal.WMOVE,
            [self._node._cmd_x, self._node._cmd_y, self._recover_depth,
             self._node._cmd_yaw],
            'z', timeout=15.0)
        if success:
            self._node._cmd_z = self._recover_depth

        # 5. 若未指定 target_sector 且无 ArUco ID，前视扫描找 tag
        if not self._target_sector:
            self._search_aruco_frontal()

        # 6. 扇区选择：target_sector 优先，否则 ArUco → 映射，默认绿色
        if self._target_sector:
            color_name = self._target_sector
            sector_cid = {'yellow': self._yellow_cid,
                          'green': self._green_cid,
                          'red': self._red_cid}.get(color_name)
            if sector_cid is None:
                self._logger.warn(
                    f'ArrowSurfacer：未知目标扇区 {self._target_sector!r}，'
                    f'默认使用绿色')
                color_name, sector_cid = 'green', self._green_cid
        else:
            sector = self._aruco_to_sector()
            if sector is None:
                self._logger.warn(
                    'ArrowSurfacer：未检测到 ArUco ID，默认使用绿色')
                sector = ('green', self._green_cid)
            color_name, sector_cid = sector
        if sector_cid is None:
            self._logger.error(
                f'ArrowSurfacer：扇区 {color_name!r} 未配置类别 ID；'
                f'请提供对应的任务参数')
            return False
        self._logger.info(
            f'ArrowSurfacer：已选择 {color_name} 扇区 '
            f'（class_id={sector_cid}）')

        # 7. 亮对应颜色灯
        color_light = {
            'yellow': self._node.LIGHT_YELLOW,
            'red':    self._node.LIGHT_RED,
            'green':  self._node.LIGHT_GREEN,
        }.get(color_name, 0)
        if color_light:
            self._node.set_light(
                color_light, f'{color_name} 扇区')
            self._logger.info(
                f'🏮 ArrowSurfacer：{color_name.upper()} 扇区指示灯已打开！')

        # 8. WTRAVEL to sector position
        self._logger.info(
            f'ArrowSurfacer：移动到扇区 '
            f'({self._sector_x:.2f}, {self._sector_y:.2f}, '
            f'z={self._sector_travel_z:.2f})')
        success, msg = self._node._send_action_goal(
            BasicMotion.Goal.WTRAVEL,
            [self._sector_x, self._sector_y, self._sector_travel_z,
             self._view_yaw],
            timeout=60.0)
        if not success:
            self._logger.error(f'ArrowSurfacer：移动到扇区失败：{msg}')
            return False
        self._node._cmd_x = self._sector_x
        self._node._cmd_y = self._sector_y
        self._node._cmd_yaw = self._view_yaw

        # 9. 搜索并对准扇区
        self._logger.info(
            f'ArrowSurfacer：对准 {color_name} 扇区 '
            f'（class={sector_cid}）')
        self._node._align_to_class(sector_cid, f'{color_name} 扇区')

        # 9.5 对准后 BMOVE（机体系 x/y）：把信号弹撒放器挪到扇区中心正上方
        if self._drop_offset_x or self._drop_offset_y:
            self._logger.info(
                f'ArrowSurfacer：使用 BMOVE 将投放器移动到扇区中心上方 '
                f'（dx={self._drop_offset_x:.2f}，dy={self._drop_offset_y:.2f}）')
            success, msg = self._node._send_action_goal(
                BasicMotion.Goal.BMOVE,
                [self._drop_offset_x, self._drop_offset_y, 0.0, 0.0],
                'xy', timeout=10.0, quiet=True)
            if success:
                self._node._cmd_x += self._drop_offset_x
                self._node._cmd_y += self._drop_offset_y
            else:
                self._logger.warn(
                    f'ArrowSurfacer：投放器 BMOVE 失败：{msg}')

        # 10. 投信标 + 关灯
        self._node.set_servo(
            self._node.ANGLE_DROP_BEACON,
            f'向 {color_name} 扇区投放信标')
        self._logger.info(
            f'🔫 ArrowSurfacer：信标已投放到 {color_name.upper()} 扇区！')
        self._node.light_off()

        # 11. [保留用户修改] 延时后复位舵机
        time.sleep(self._servo_reset_delay)
        self._node.set_servo(self._servo_reset_angle, '舵机复位')

        self._logger.info(
            f'ArrowSurfacer：任务完成。'
            f'ArUco ID：{sorted(self._aruco_ids)}')
        return True

    # ── 资源清理 ──────────────────────────────────────────────────────

    def destroy(self):
        """销毁所有感知订阅，释放资源。"""
        for sub in self._subs:
            self._node.destroy_subscription(sub)
        self._subs.clear()
        self._logger.info('ArrowSurfacer：已销毁')
