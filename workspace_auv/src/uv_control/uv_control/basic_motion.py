"""
basic_motion.py — 运动控制节点（合并 ZIT6 底层 + 高级运动 API）

分层架构（从底到顶）：

  Layer 5: 对外高级 API (SET / WMOVE / BMOVE / TRAVEL)
  Layer 4: 步进坐标系 (运动方向 along/lateral 分解 + 动态步长)
  Layer 3: 机器人坐标系 (body frame — set_body / set_step)
  Layer 2: 世界坐标系 (odom — start / set_world / get_state)
  Layer 1: ZIT6 底层 (map 坐标系 — _send_setpoint)

坐标系说明：
  map 系 — MCU/仿真器上报的原始位置，原点未知
  odom 系 — 以 AUV 启动位置为原点的世界坐标系（start() 时初始化）
  body 系 — 以 AUV 当前位置为原点的机体坐标系

指令路径（保证 single source of truth）：
  高级 API → set_world → set_map → _send_setpoint → /zit6/cmd/setpoint
  set_body/set_step → Coordinate 变换 → set_world → …
  set_map 是唯一的协议出口，迁移协议只需改此函数

================================================================================
系统架构
================================================================================

  ┌─────────────────────────────────────────────────────┐
  │  uv_task (任务层)                                    │
  │  JSON 任务加载 → 顺序执行 → 调用导航/运动              │
  ├─────────────────────────────────────────────────────┤
  │  uv_nav (导航层)                                     │
  │  A* 路径规划 → 避障 → 调用 basic_motion 执行           │
  ├─────────────────────────────────────────────────────┤
  │  uv_control (控制层)  ← 本文件所在层                   │
  │  basic_motion (本节点)                                │
  │    → set_world / set_body / set_step                 │
  │    → _send_setpoint → /zit6/cmd/setpoint             │
  ├─────────────────────────────────────────────────────┤
  │  uv_hm (硬件管理层)                                   │
  │  sim_bridge (仿真) / hw_manager (实车)                │
  │  级联PID + 推力混合 → 6推进器                          │
  ├─────────────────────────────────────────────────────┤
  │  Stonefish / MCU                                     │
  │  物理仿真引擎 / STM32 微控制器                         │
  └─────────────────────────────────────────────────────┘

================================================================================
三类运动函数
================================================================================

1. SET 系列 (绝对定位)
   - 直接发送 odom 系绝对坐标 + 等到达
   - 适用于已知精确目标位置的场景

2. WMOVE / BMOVE 系列 (步进移动)
   - WMOVE: 世界系步进，参数为世界系偏移量
   - BMOVE: 机体系步进，参数为机体系偏移量，内部转世界系
   - 使用动态步进算法：把长距离拆成小段逐段发送
   - 步长根据当前误差动态调整

3. TRAVEL 系列 (直线移动)
   - WTRAVEL: 世界系直线移动
   - BTRAVEL: 机体系直线移动
   - 先转向目标方向，再沿 body-X 轴前进
   - 适用于需要直线轨迹的任务（过门、巡线等）
"""

import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from zit6_interfaces.msg import ZitSetpoint, ZitStatus
from uv_msgs.action import BasicMotion
from uv_control.coordinate import Coordinate, wrap_deg, wrap_rad

# ═════════════════════════════════════════════════════════════════════════════
# ZIT6 control_key 常量
# ═════════════════════════════════════════════════════════════════════════════
# 只走 0x00 位置模式，body/增量转换在上层完成
CK_POS = 0          # position mode (唯一的 control_key)

# ═════════════════════════════════════════════════════════════════════════════
# 轴掩码
# ═════════════════════════════════════════════════════════════════════════════
AX_X = 0x01
AX_Y = 0x02
AX_Z = 0x04
AX_RZ = 0x08
AX_XY = AX_X | AX_Y
AX_XYZ = AX_X | AX_Y | AX_Z
AX_ALL = AX_X | AX_Y | AX_Z | AX_RZ

# ═════════════════════════════════════════════════════════════════════════════
# 默认容差
# ═════════════════════════════════════════════════════════════════════════════
TOL_X = 0.1     # X 轴容差 (米)
TOL_Y = 0.1     # Y 轴容差 (米)
TOL_Z = 0.1     # Z 轴容差 (米)
TOL_RZ = 5.0    # Yaw 轴容差 (度)

# ═════════════════════════════════════════════════════════════════════════════
# 步进控制参数
# ═════════════════════════════════════════════════════════════════════════════
STEP_X = 0.6             # X 方向步长 (椭圆半轴，米)
STEP_Y = 0.4             # Y 方向步长 (椭圆半轴，米)
STEP_PERIOD = 0.3        # 目标步进间隔/收敛时间阈值 (秒)
LATERAL_LAMBDA = 2.0     # 横向误差指数衰减系数


class BasicMotionNode(Node):
    """运动控制节点：ZIT6 协议接口 + 高级运动 API，single source of truth。"""

    def __init__(self):
        super().__init__('basic_motion')

        # ── 状态 ───────────────────────────────────────────────
        self.status = ZitStatus()
        self.pose = Coordinate()        # 当前位置 (odom 系)
        self._target = Coordinate()     # 当前目标 (odom 系)
        self._origin = None             # odom 原点 (map 系 Coordinate)，start() 时设置
        self._origin_warned = False     # 防止 _pos_cb 重复打印警告
        self._state_lock = threading.Lock()
        self.vel_body = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rz': 0.0}   # 机体速度                     # 世界速度

        # ─────────────────────────────────────────────────────────
        # Layer 1: ZIT6 协议发布/订阅
        # ─────────────────────────────────────────────────────────
        self.pub_setpoint = self.create_publisher(
            ZitSetpoint, '/zit6/cmd/setpoint', 10)
        self.create_subscription(
            ZitStatus, '/zit6/state/status', self._status_cb, 10)
        self.create_subscription(
            Float32MultiArray, '/zit6/state/pos', self._pos_cb, 10)
        self.create_subscription(
            Float32MultiArray, '/zit6/state/vel', self._vel_cb, 10)

        # ── Action Server ────────────────────────────────────────
        self._action_server = ActionServer(
            self, BasicMotion, 'basic_motion',
            goal_callback=self._action_goal_cb,
            cancel_callback=self._action_cancel_cb,
            execute_callback=self._action_execute_cb,
        )
        self._action_goal_handle = None
        self._action_target = None       # 绝对目标 {x, y, z, yaw}，用于反馈
        self._action_axes = None         # 当前 action 的生效轴，用于反馈
        self.create_timer(0.5, self._action_feedback_cb)

        self.get_logger().info('BasicMotion node started')

    # ═════════════════════════════════════════════════════════════════════════
    # Layer 1: ZIT6 底层 (map 坐标系)
    # ═════════════════════════════════════════════════════════════════════════

    def _send_setpoint(self, control_key: int, type_mask: int,
                       x: float, y: float, z: float, yaw_rad: float):
        """发送 ZitSetpoint。坐标是 map 系，yaw 是弧度，只走 CK_POS 位置模式。"""
        msg = ZitSetpoint()
        msg.control_key = control_key
        msg.type_mask = type_mask
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        msg.yaw = float(yaw_rad)
        msg.seq = 0
        self._target = self._map_to_odom(Coordinate(x=x, y=y, z=z, rz=math.degrees(yaw_rad)))
        self.pub_setpoint.publish(msg)

    def _status_cb(self, msg: ZitStatus):
        with self._state_lock:
            self.status = msg

    def _vel_cb(self, msg: Float32MultiArray):
        """ZIT6 速度回调。机体速度 [vx, vy, vz, vyaw_rad_s]，存机体+世界两套。"""
        with self._state_lock:
            if len(msg.data) < 4:
                return
            self.vel_body = {
                'x': msg.data[0],
                'y': msg.data[1],
                'z': msg.data[2],
                'rz': msg.data[3],   # rad/s
            }

    def _pos_cb(self, msg: Float32MultiArray):
        """ZIT6 位置回调。原始数据是 map 系，内部转 odom 系后存为 self.pose。"""
        with self._state_lock:
            if len(msg.data) < 4:
                return
            map_pos = Coordinate(
                x=msg.data[0],
                y=msg.data[1],
                z=msg.data[2],
                rz=math.degrees(msg.data[3]),   # 弧度→度
            )
            if self._origin is not None:
                self.pose = self._map_to_odom(map_pos)
            else:
                if not self._origin_warned:
                    self._origin_warned = True
                    self.get_logger().warning('里程计原点未设置，使用 map 坐标作为 odom 坐标')
                self.pose = map_pos


    def set_map(self, x: float, y: float, z: float, yaw_deg: float):
        """直接发送 map 系绝对位置。yaw 单位为度。

        所有 ZIT6 协议细节集中于此（CK_POS, type_mask=0）。
        迁移到其他协议时只需修改此函数。
        """
        self._send_setpoint(CK_POS, 0, x, y, z, math.radians(yaw_deg))


    # ═════════════════════════════════════════════════════════════════════════
    # Layer 2: 世界坐标系 (odom)
    # ═════════════════════════════════════════════════════════════════════════

    def start(self):
        """初始化 odom 原点。AUV 开始作业前第一个调用。

        记录当前 map 位置为 odom 原点，之后所有坐标都是相对于此原点。
        odom 0° = AUV 初始朝向（yaw 也做偏移）。
        例：AUV 朝东 (map 90°) start → odom 0° = 东，set_world(x=5) 向东走 5m。
        """
        with self._state_lock:
            if self._origin is not None:
                self.get_logger().info('odom origin already set, skipping start()')
                return
            self._origin = Coordinate(
                x=self.pose.x, y=self.pose.y, z=self.pose.z, rz=self.pose.rz)
            self.get_logger().info(
                f'DEBUG pose before zero: x={self.pose.x:.4f}, y={self.pose.y:.4f}, '
                f'z={self.pose.z:.4f}, rz={self.pose.rz:.4f}')
            # 将当前 pose 全部置零成为 odom 原点
            self.pose.x = 0.0
            self.pose.y = 0.0
            self.pose.z = 0.0
            self.pose.rz = 0.0
        self.get_logger().info(
            f'odom origin set: map({self._origin.x:.2f}, '
            f'{self._origin.y:.2f}, {self._origin.z:.2f}), '
            f'yaw={self._origin.rz:.1f}°')

    def _odom_to_map(self, pos: Coordinate) -> Coordinate:
        """odom 坐标 → map 坐标（x/y/z/rz 全做偏移）。

        Returns: (map_x, map_y, map_z, map_yaw_deg)
        """
        if self._origin is None:
            return pos
        return self._origin.to_world_frame(pos)
    def _map_to_odom(self, pos: Coordinate) -> Coordinate:
        """map 坐标字典 → odom 坐标字典（x/y/z/rz 全做偏移）。"""
        if self._origin is None:
            return pos
        return self._origin.to_local_frame(pos)
    
    def _map_to_body(self, pos: Coordinate) -> Coordinate:
        """map 坐标 → body 坐标（以当前 pose 为原点）。"""
        return self._odom_to_body(self._map_to_odom(pos))
    
    def _body_to_map(self, pos: Coordinate) -> Coordinate:
        """body 坐标 → map 坐标（以当前 pose 为原点）。"""
        return self._odom_to_map(self._body_to_odom(pos))
    
    def _body_to_odom(self, pos: Coordinate) -> Coordinate:
        """body 坐标 → odom 坐标（2D 旋转 + 平移）。"""
        p, _, _ = self.get_state()
        return p.body_to_world(pos.x, pos.y, pos.z)

    def _odom_to_body(self, pos: Coordinate) -> Coordinate:
        """odom 绝对坐标 → body 相对坐标（先算位移差, 再 2D 旋转）。"""
        p, _, _ = self.get_state()
        dx = pos.x - p.x
        dy = pos.y - p.y
        dz = pos.z - p.z
        r = p.world_to_body(dx, dy, dz)
        r.rz = wrap_deg(pos.rz - p.rz) if pos.rz is not None else 0.0
        return r


    def set_world(self, x: float, y: float, z: float, yaw_deg: float):
        """设置 odom 系绝对位置。yaw 单位为度。"""
        odom = Coordinate(x=x, y=y, z=z, rz=yaw_deg)
        m = self._odom_to_map(odom)
        self.set_map(m.x, m.y, m.z, m.rz)

    def get_state(self):
        """获取 AUV 当前状态（odom 系，线程安全）。

        Returns:
            (pose: Coordinate, target: Coordinate, status: ZitStatus)
        """
        with self._state_lock:
            return self.pose, self._target, self.status

    # ═════════════════════════════════════════════════════════════════════════
    # Layer 3: 机器人坐标系 (body frame)
    # ═════════════════════════════════════════════════════════════════════════

    def set_body(self, x: float, y: float, z: float, yaw_deg: float):
        """设置机体系绝对位置。yaw 单位为度。

        以当前 target 为参考坐标系，转换 body 目标 → set_world。
        """
        _, t, _ = self.get_state()
        body_target = Coordinate(x=x, y=y, z=z, rz=yaw_deg)
        new_target = t.to_world_frame(body_target)
        self.set_world(new_target.x, new_target.y, new_target.z, new_target.rz)

    def set_step(self, dx: float, dy: float, dz: float, dyaw_deg: float):
        """设置机体系增量步进。dyaw 单位为度。

        body 增量 → Coordinate → to_world_frame → 叠加到target的map 位姿 → set_world。
        """
        _, t, _ = self.get_state()
        target_step = Coordinate(x=dx, y=dy, z=dz, rz=dyaw_deg)
        map_target = t.to_world_frame(target_step)

        self.set_world(map_target.x, map_target.y, map_target.z, map_target.rz)

    # ═════════════════════════════════════════════════════════════════════════
    # 内部工具： 等待到达
    # ═════════════════════════════════════════════════════════════════════════

    def _is_cancelled(self) -> bool:
        """检查当前 action goal 是否被取消（线程安全）。"""
        gh = self._action_goal_handle
        return gh is not None and gh.is_cancel_requested

    def _cmd_and_wait(self, target_x, target_y, target_z, target_yaw_degree, timeout):
        """完成步进移动的最后一步，等待到达目标位置。"""
        self.set_world(target_x, target_y, target_z, target_yaw_degree)
        return self._wait_reached(timeout=timeout)
    
    
    def _wait_reached(self,
                      tol_x: float = TOL_X, tol_y: float = TOL_Y,
                      tol_z: float = TOL_Z, tol_rz: float = TOL_RZ,
                      timeout: float = 60.0) -> bool:
        """阻塞等待 AUV 到达目标位置。

        以 10Hz 检查各轴误差是否在容差范围内。
        """
        start = time.monotonic()

        while rclpy.ok():
            if self._is_cancelled():
                self.get_logger().info('_wait_reached: action cancelled')
                return False
            elapsed = time.monotonic() - start
            if elapsed > timeout:
                self.get_logger().warning('_wait_reached超时')
                return False

            body_target = self._odom_to_body(self._target)

            reached = True

            if abs(body_target.x) > tol_x:
                reached = False
            if abs(body_target.y) > tol_y:
                reached = False
            if abs(body_target.z) > tol_z:
                reached = False
            yaw_err = abs(wrap_deg(body_target.rz))
            if yaw_err > tol_rz:
                reached = False

            if reached:
                return True
            time.sleep(0.1)

        return False

    # ═════════════════════════════════════════════════════════════════════════
    # Layer 4: 步进坐标系 (运动方向 along/lateral 分解 + 动态步长)
    # ═════════════════════════════════════════════════════════════════════════

    def _calc_step_size(self, move_angle_rad: float) -> float:
        """根据运动方向计算基础步进距离（椭圆模型）。

        r(θ) = 1 / sqrt((cosθ/a)² + (sinθ/b)²)
        纯 X (θ=0°): a = STEP_X, 纯 Y (θ=90°): b = STEP_Y。
        """
        ca = math.cos(move_angle_rad)
        sa = math.sin(move_angle_rad)
        return 1.0 / math.sqrt((ca / STEP_X) ** 2 + (sa / STEP_Y) ** 2)

    def _wait_step_convergence(self, move_angle: float,
                               timeout: float = 10.0) -> bool:
        """等待步进收敛（在运动方向坐标系中判断）。

        将误差投影到运动方向坐标系（along/lateral），
        估算以当前速度消除前向误差的时间，≤ STEP_PERIOD 则判为收敛。

        Args:
            move_angle: 运动方向角 (弧度)
            timeout: 超时秒数

        Returns:
            True = 收敛, False = 超时
        """
        start = time.monotonic()
        min_wait = STEP_PERIOD * 0.5
        ca = math.cos(move_angle)
        sa = math.sin(move_angle)

        while rclpy.ok():
            if self._is_cancelled():
                self.get_logger().info('_wait_step_convergence: action cancelled')
                return False
            elapsed = time.monotonic() - start
            if elapsed > timeout:
                return False
            if elapsed < min_wait:
                time.sleep(0.05)
                continue


            body_target = self._odom_to_body(self._target)


            # 机体坐标系误差 → 运动方向坐标系
            e_along = ca * body_target.x + sa * body_target.y      # 前向剩余距离
            e_lateral = -sa * body_target.x + ca * body_target.y   # 横向偏差

            # 机体坐标系速度投影到运动方向坐标系
            with self._state_lock:
                vx_body = self.vel_body['x']
                vy_body = self.vel_body['y']
            v_along = ca * vx_body + sa * vy_body

            # 方向一致性：朝目标前进（速度 > 0）且还有路要走，或已非常近
            converging = (e_along > 0 and v_along > 0) or e_along < 0.01

            # 指数衰减：横向偏差越大，有效前向速度越小
            v_eff = v_along * math.exp(-LATERAL_LAMBDA * abs(e_lateral))

            if v_eff > 0.01:
                t_remaining = e_along / v_eff
            else:
                t_remaining = float('inf')

            step_thresh = self._calc_step_size(move_angle) * 0.3
            if converging and (t_remaining <= STEP_PERIOD / 2 or e_along < step_thresh):
                return True

            time.sleep(0.1)

        return False

    def _step_move_world(self, target_x: float, target_y: float,
                         target_z: float, target_yaw_degree: float,
                         timeout: float = 60.0) -> bool:
        """步进移动：odom 系目标，拆成小段逐段发送。

        每步根据当前横向误差计算动态步长，收敛后再发下一步。
        最后发送精确目标并等待到达。

        Args:
            target_x, target_y, target_z: 目标位置 (odom 系)
            target_yaw_degree: 目标偏航 (角度)
            timeout: 超时秒数

        Returns:
            True = 全部到达, False = 超时
        """
        start = time.monotonic()
        while rclpy.ok():
            # 总体超时检查
            remaining = timeout - (time.monotonic() - start)
            if remaining <= 0:
                self.get_logger().warning('_step_move_world: overall timeout')
                return False
            if self._is_cancelled():
                self.get_logger().info('_step_move_world: action cancelled')
                return False

            target_world = Coordinate(x=target_x, y=target_y, z=target_z)
            target_body = self._odom_to_body(target_world)

            _, target_odom, _ = self.get_state()
            target_target = target_odom.to_local_frame(target_world)


            # 机体坐标系误差 → 运动方向坐标系
            ex_body = target_body.x
            ey_body = target_body.y
            dist_xy = math.sqrt(target_target.x**2 + target_target.y**2)


            move_angle = math.atan2(ey_body, ex_body)
            base_step = self._calc_step_size(move_angle)

            ca = math.cos(move_angle)
            sa = math.sin(move_angle)
            e_along = ca * target_body.x + sa * target_body.y
            e_lateral = -sa * target_body.x + ca * target_body.y

            if e_along < base_step:
                break

            # 动态步长：横向误差越大步长越小
            step_along = base_step * math.exp(-LATERAL_LAMBDA * abs(e_lateral))



            dz_total = target_z - self.pose.z
            drz_total = target_yaw_degree - self.pose.rz
            steps_remaining = max(1, math.ceil(dist_xy / base_step))

            step_z = dz_total / steps_remaining
            step_rz = drz_total / steps_remaining

            vector_body = Coordinate(x = ca * step_along, y = sa * step_along, z = step_z, rz = step_rz)

            # 步进目标 = 当前 target + 步进增量
            self.set_step(dx = vector_body.x, dy = vector_body.y, dz = vector_body.z, dyaw_deg = vector_body.rz)

            # 每一步用剩余时间（至少 1 秒）
            step_timeout = max(1.0, remaining)
            if not self._wait_step_convergence(move_angle, timeout=step_timeout):
                self.get_logger().warning('_step_move_world: step convergence failed')

        # 最终目标（用剩余时间）
        remaining = timeout - (time.monotonic() - start)
        if remaining <= 0:
            return False
        return self._cmd_and_wait(
            target_x, target_y, target_z, target_yaw_degree,
            timeout=remaining)

    # ═════════════════════════════════════════════════════════════════════════
    # Layer 5: 对外高级 API
    # ═════════════════════════════════════════════════════════════════════════

    # --- SET 系列 (绝对定位) ------------------------------------------------

    def setxyzrz(self, x: float, y: float, z: float, rz: float,
                 timeout: float = 60.0) -> bool:
        """odom 系绝对位置 + 偏航。rz 单位为度。"""
        return self._cmd_and_wait(x, y, z, rz,
                                  timeout=timeout)

    def setxyz(self, x: float, y: float, z: float,
               timeout: float = 60.0) -> bool:
        """odom 系绝对位置（不改变偏航）。"""
        p, _, _ = self.get_state()
        return self._cmd_and_wait(x, y, z, p.rz,
                                  timeout=timeout)

    def setxy(self, x: float, y: float, timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        return self._cmd_and_wait(x, y, p.z, p.rz,
                                  timeout=timeout)

    def setxyrz(self, x: float, y: float, rz: float,
                timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        return self._cmd_and_wait(x, y, p.z, rz,
                                  timeout=timeout)

    def setz(self, z: float, timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        return self._cmd_and_wait(p.x, p.y, z, p.rz,
                                  timeout=timeout)

    def setrz(self, rz: float, timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        return self._cmd_and_wait(p.x, p.y, p.z, rz,
                                  timeout=timeout)

    def setx(self, x: float, timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        return self._cmd_and_wait(x, p.y, p.z, p.rz,
                                  timeout=timeout)

    def sety(self, y: float, timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        return self._cmd_and_wait(p.x, y, p.z, p.rz,
                                  AX_Y, timeout=timeout)

    # --- WMOVE 系列 (世界系步进) --------------------------------------------

    def wmovexyzrz(self, dx: float, dy: float, dz: float, drz: float,
                   timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        return self._step_move_world(
            p.x + dx, p.y + dy, p.z + dz,
            p.rz + drz, timeout)

    def wmovexyz(self, dx: float, dy: float, dz: float,
                 timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        return self._step_move_world(
            p.x + dx, p.y + dy, p.z + dz,
            p.rz, timeout)

    def wmovexy(self, dx: float, dy: float, timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        return self._step_move_world(
            p.x + dx, p.y + dy, p.z,
            p.rz, timeout)

    def wmovexyrz(self, dx: float, dy: float, drz: float,
                  timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        return self._step_move_world(
            p.x + dx, p.y + dy, p.z,
            p.rz + drz, timeout)

    def wmovez(self, dz: float, timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        return self._step_move_world(
            p.x, p.y, p.z + dz,
            p.rz, timeout)

    def wmoverz(self, drz: float, timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        return self._step_move_world(
            p.x, p.y, p.z,
            p.rz + drz, timeout)

    def wmovex(self, dx: float, timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        return self._step_move_world(
            p.x + dx, p.y, p.z,
            p.rz, timeout)

    def wmovey(self, dy: float, timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        return self._step_move_world(
            p.x, p.y + dy, p.z,
            p.rz, timeout)

    # --- BMOVE 系列 (机体系步进) --------------------------------------------

    def bmovexyzrz(self, dx: float, dy: float, dz: float, drz: float,
                   timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        off = p.body_to_world(dx, dy)
        return self._step_move_world(
            p.x + off.x, p.y + off.y, p.z + dz,
            p.rz + drz, timeout)

    def bmovexyz(self, dx: float, dy: float, dz: float,
                 timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        off = p.body_to_world(dx, dy)
        return self._step_move_world(
            p.x + off.x, p.y + off.y, p.z + dz,
            p.rz, timeout)

    def bmovexy(self, dx: float, dy: float, timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        off = p.body_to_world(dx, dy)
        return self._step_move_world(
            p.x + off.x, p.y + off.y, p.z,
            p.rz, timeout)

    def bmovexyrz(self, dx: float, dy: float, drz: float,
                  timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        off = p.body_to_world(dx, dy)
        return self._step_move_world(
            p.x + off.x, p.y + off.y, p.z,
            p.rz + drz, timeout)

    def bmovez(self, dz: float, timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        return self._step_move_world(
            p.x, p.y, p.z + dz,
            p.rz, timeout)

    def bmoverz(self, drz: float, timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        return self._step_move_world(
            p.x, p.y, p.z,
            p.rz + drz, timeout)

    def bmovex(self, dx: float, timeout: float = 60.0) -> bool:
        return self.bmovexy(dx, 0.0, timeout=timeout)

    def bmovey(self, dy: float, timeout: float = 60.0) -> bool:
        return self.bmovexy(0.0, dy, timeout=timeout)

    # --- TRAVEL 系列 (直线移动) ---------------------------------------------

    def _travel_world(self, dx_w: float, dy_w: float, dz: float = 0.0,
                      timeout: float = 60.0) -> bool:
        """直线移动基础函数：先转向目标方向，再沿 body-X 步进前进。

        Args:
            dx_w: 世界系 X 偏移 (北)
            dy_w: 世界系 Y 偏移 (东)
            dz: Z 轴偏移 (深度)
            timeout: 超时秒数
        """
        p, _, _ = self.get_state()
        target_x = p.x + dx_w
        target_y = p.y + dy_w
        target_z = p.z + dz

        dist_xy = math.sqrt(dx_w**2 + dy_w**2)
        if dist_xy > 0.03:
            target_yaw = math.degrees(math.atan2(dy_w, dx_w))
            # 第一步：转向目标方向
            self._step_move_world(p.x, p.y, p.z, target_yaw,
                                  timeout=timeout)
            # 第二步：沿 body-X 前进
            return self._step_move_world(
                target_x, target_y, target_z, target_yaw,
                timeout=timeout)
        else:
            return self._step_move_world(
                p.x, p.y, target_z, p.rz,
                timeout=timeout)

    def wtravelxy(self, dx: float, dy: float, timeout: float = 60.0) -> bool:
        return self._travel_world(dx, dy, 0.0, timeout=timeout)

    def wtravelxyz(self, dx: float, dy: float, dz: float,
                   timeout: float = 60.0) -> bool:
        return self._travel_world(dx, dy, dz, timeout=timeout)

    def wtravelx(self, dx: float, timeout: float = 60.0) -> bool:
        return self._travel_world(dx, 0.0, 0.0, timeout=timeout)

    def wtravely(self, dy: float, timeout: float = 60.0) -> bool:
        return self._travel_world(0.0, dy, 0.0, timeout=timeout)

    def wtravelz(self, dz: float, timeout: float = 60.0) -> bool:
        return self._travel_world(0.0, 0.0, dz, timeout=timeout)

    def btravelxy(self, dx: float, dy: float, timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        off = p.body_to_world(dx, dy)
        return self._travel_world(off.x, off.y, 0.0, timeout=timeout)

    def btravelxyz(self, dx: float, dy: float, dz: float,
                   timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        off = p.body_to_world(dx, dy)
        return self._travel_world(off.x, off.y, dz, timeout=timeout)

    def btravelx(self, dx: float, timeout: float = 60.0) -> bool:
        return self.bmovex(dx, timeout=timeout)

    def btravely(self, dy: float, timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        off = p.body_to_world(0.0, dy)
        return self._travel_world(off.x, off.y, 0.0, timeout=timeout)

    def btravelz(self, dz: float, timeout: float = 60.0) -> bool:
        return self._travel_world(0.0, 0.0, dz, timeout=timeout)

    # ═════════════════════════════════════════════════════════════════════════
    # Action Server 回调
    # ═════════════════════════════════════════════════════════════════════════

    def _action_goal_cb(self, goal_request):
        self.get_logger().info(
            f'Action goal received: cmd_type={goal_request.cmd_type}, '
            f'axes="{goal_request.axes}", target={list(goal_request.target)}, '
            f'timeout={goal_request.timeout:.1f}s')
        return GoalResponse.ACCEPT

    def _action_cancel_cb(self, _goal_handle):
        self.get_logger().info('Action cancel requested, accepting')
        return CancelResponse.ACCEPT

    def _action_execute_cb(self, goal_handle):
        req = goal_handle.request
        self._action_goal_handle = goal_handle

        # START: 初始化 odom 原点，不需要任何前置校验
        if req.cmd_type == BasicMotion.Goal.START:
            self.get_logger().info('Action START: initializing odom origin')
            self.start()
            result = BasicMotion.Result()
            result.success = True
            result.message = "odom origin set"
            goal_handle.succeed()
            self._action_goal_handle = None
            self.get_logger().info('Action START: done')
            return result

        # ── 参数校验 ──────────────────────────────────────────
        if len(req.target) < 4:
            self.get_logger().error(
                f'Action rejected: target needs 4 values [x,y,z,yaw], '
                f'got {len(req.target)}')
            result = BasicMotion.Result()
            result.success = False
            result.message = f"target needs 4 values [x, y, z, yaw], got {len(req.target)}"
            goal_handle.abort()
            self._action_goal_handle = None
            return result

        if self._origin is None:
            self.get_logger().error(
                'Action rejected: odom origin not set, send START first')
            result = BasicMotion.Result()
            result.success = False
            result.message = "odom origin not set, call start() first"
            goal_handle.abort()
            self._action_goal_handle = None
            return result

        x, y, z, yaw = req.target
        timeout = req.timeout if req.timeout > 0 else 60.0
        p, _, _ = self.get_state()

        # ── 派发运动类型 ──────────────────────────────────────
        type_names = {1: 'WMOVE', 2: 'BMOVE', 3: 'SET', 4: 'WTRAVEL', 5: 'BTRAVEL'}
        type_name = type_names.get(req.cmd_type, f'UNKNOWN({req.cmd_type})')
        self.get_logger().info(
            f'Action {type_name}: axes="{req.axes}", '
            f'target=[{x:.2f}, {y:.2f}, {z:.2f}, {yaw:.1f}], '
            f'pose=[{p.x:.2f}, {p.y:.2f}, {p.z:.2f}, {p.rz:.1f}], '
            f'timeout={timeout:.1f}s')

        if req.cmd_type == BasicMotion.Goal.SET:
            self._action_target = {'x': x, 'y': y, 'z': z, 'yaw': yaw}
            success = self.setxyzrz(x, y, z, yaw, timeout=timeout)
        elif req.cmd_type == BasicMotion.Goal.WMOVE:
            self._action_target = {'x': p.x + x, 'y': p.y + y,
                                   'z': p.z + z, 'yaw': p.rz + yaw}
            success = self.wmovexyzrz(x, y, z, yaw, timeout=timeout)
        elif req.cmd_type == BasicMotion.Goal.BMOVE:
            off = p.body_to_world(x, y)
            self._action_target = {'x': p.x + off.x, 'y': p.y + off.y,
                                   'z': p.z + z, 'yaw': p.rz + yaw}
            success = self.bmovexyzrz(x, y, z, yaw, timeout=timeout)
        elif req.cmd_type == BasicMotion.Goal.WTRAVEL:
            self._action_target = {'x': p.x + x, 'y': p.y + y, 'z': p.z + z, 'yaw': p.rz}
            success = self._travel_world(x, y, z, timeout=timeout)
        elif req.cmd_type == BasicMotion.Goal.BTRAVEL:
            off = p.body_to_world(x, y)
            self._action_target = {'x': p.x + off.x, 'y': p.y + off.y,
                                   'z': p.z + z, 'yaw': p.rz}
            success = self._travel_world(off.x, off.y, z, timeout=timeout)
        else:
            self.get_logger().error(f'Action rejected: unknown cmd_type={req.cmd_type}')
            result = BasicMotion.Result()
            result.success = False
            result.message = f"unknown cmd_type: {req.cmd_type}"
            goal_handle.abort()
            self._action_goal_handle = None
            return result

        result = BasicMotion.Result()
        result.success = success
        if success:
            result.message = ""
            self.get_logger().info(
                f'Action {type_name}: SUCCESS, '
                f'final_target=({self._action_target["x"]:.2f}, '
                f'{self._action_target["y"]:.2f}, '
                f'{self._action_target["z"]:.2f}, '
                f'{self._action_target["yaw"]:.1f})')
            goal_handle.succeed()
        elif goal_handle.is_cancel_requested:
            result.message = "cancelled"
            self.get_logger().info(f'Action {type_name}: CANCELLED')
            goal_handle.canceled()
        else:
            result.message = "motion timeout"
            self.get_logger().error(f'Action {type_name}: TIMEOUT')
            goal_handle.abort()
        self._action_goal_handle = None
        self._action_target = None
        return result

    def _action_feedback_cb(self):
        if self._action_goal_handle is None or self._action_target is None:
            return
        t = self._action_target
        with self._state_lock:
            dx = t['x'] - self.pose.x
            dy = t['y'] - self.pose.y
            dz = t['z'] - self.pose.z
        feedback = BasicMotion.Feedback()
        feedback.distance_remaining = float(math.sqrt(dx**2 + dy**2 + dz**2))
        self._action_goal_handle.publish_feedback(feedback)


def main(args=None):
    rclpy.init(args=args)
    node = BasicMotionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
