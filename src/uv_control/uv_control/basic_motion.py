"""
basic_motion.py — 高级运动控制节点

================================================================================
系统架构说明
================================================================================

YouLong AUV 控制系统采用分层架构：

  ┌─────────────────────────────────────────────────────┐
  │  uv_task (任务层)                                    │
  │  JSON 任务加载 → 顺序执行 → 调用导航/运动              │
  ├─────────────────────────────────────────────────────┤
  │  uv_nav (导航层)                                     │
  │  A* 路径规划 → 避障 → 调用 basic_motion 执行           │
  ├─────────────────────────────────────────────────────┤
  │  uv_control (控制层)  ← 本文件所在层                   │
  │  basic_motion (高级API) → minimal_control (硬件抽象)   │
  ├─────────────────────────────────────────────────────┤
  │  uv_hm (硬件管理层)                                   │
  │  sim_bridge (仿真) / hw_manager (实车)                │
  │  级联PID + 推力混合 → 6推进器                          │
  ├─────────────────────────────────────────────────────┤
  │  Stonefish / MCU                                     │
  │  物理仿真引擎 / STM32 微控制器                         │
  └─────────────────────────────────────────────────────┘

本节点 (basic_motion) 的定位：
  - 不直接与硬件/sim_bridge 通信
  - 通过发布 MotionCommand 消息到 /auv/cmd/motion 来下达运动指令
  - 订阅 /auv/state 获取 AUV 当前状态（位置/速度/目标）
  - 提供三类高级运动函数：SET / WMOVE·BMOVE / TRAVEL

数据流：
  basic_motion ──MotionCommand──→ sim_bridge ──thrusters──→ Stonefish
       ↑                            │
       └────── AuvState ────────────┘
               (位置/速度/目标)

================================================================================
坐标系约定 (NED)
================================================================================

世界系 (North-East-Down)：
  X = 北 (North)    — 正方向朝北
  Y = 东 (East)     — 正方向朝东
  Z = 下 (Down)     — 正方向朝下（深度增加）
  Yaw = 0° 朝北，顺时针为正

机体系 (Body Frame)：
  X = 前进 (Surge)  — 正方向为 AUV 前进方向
  Y = 右移 (Sway)   — 正方向为 AUV 右侧
  Z = 下潜 (Heave)  — 正方向为 AUV 下方
  Yaw = 机体系偏航角

坐标变换：
  body_to_world(dx_body, dy_body, yaw):
    dx_world =  cos(yaw) * dx_body - sin(yaw) * dy_body
    dy_world =  sin(yaw) * dx_body + cos(yaw) * dy_body

  world_to_body(dx_world, dy_world, yaw):
    dx_body =  cos(yaw) * dx_world + sin(yaw) * dy_world
    dy_body = -sin(yaw) * dx_world + cos(yaw) * dy_world

================================================================================
三类运动函数
================================================================================

1. SET 系列 (绝对定位)
   - 直接发送世界系绝对坐标 + 等到达
   - 不走步进，一次到位
   - 适用于已知精确目标位置的场景

2. WMOVE / BMOVE 系列 (步进移动)
   - WMOVE: 世界系步进，参数为世界系偏移量
   - BMOVE: 机体系步进，参数为机体系偏移量，内部转世界系
   - 使用动态步进算法：把长距离拆成小段逐段发送
   - 步长根据当前误差动态调整（见下方"步进算法"）

3. TRAVEL 系列 (直线移动)
   - WTRAVEL: 世界系直线移动
   - BTRAVEL: 机体系直线移动
   - 核心区别：先转向目标方向，再沿 body-X 轴前进
   - 保证 AUV 始终面朝运动方向，走直线路径
   - 适用于需要直线轨迹的场景（过门、巡线等）

函数调用关系：
  setxyzrz = setxy + setz + setrz (分解调用)
  bmovexy  = body→world 变换 + wmovexy
  wmovexy  = _step_move_world (核心步进函数)
  wtravelxy = _travel_world (先转向 + 步进)
  btravelxy = body→world 变换 + _travel_world

================================================================================
步进算法详解
================================================================================

传统方式的问题：
  发送一个远距离目标 → PID 控制器产生曲线轨迹 → AUV 走曲线到达
  在水下环境中，水流干扰会导致 AUV 偏离预期路径

步进方式：
  把长距离移动拆成很多小段，逐段发送，每段等收敛后再发下一段
  AUV 的运动轨迹更接近分段直线，更可控

运动方向坐标系 (Movement Direction Coordinate System)：
  不使用世界系 XY 判断误差，而是将误差投影到"当前→目标"的方向上：
  - 前向轴 (along): 当前位置→目标位置的方向
  - 横向轴 (lateral): 垂直于前向轴

  投影公式：
    move_angle = atan2(ey_world, ex_world)  — 运动方向角
    e_along   =  cos(θ) * ex + sin(θ) * ey  — 前向剩余距离
    e_lateral = -sin(θ) * ex + cos(θ) * ey  — 横向偏差

动态步长：
  step_size = base_step * exp(-λ * |e_lateral|)

  其中：
  - base_step = |cos(θ)| * TOL_X + |sin(θ)| * TOL_Y  — 基础步长
  - λ = LATERAL_LAMBDA  — 横向误差敏感度
  - e_lateral — 当前横向偏差

  效果：
  - 横向偏差大 → 步长小 → AUV 有更多时间修正航向
  - 横向偏差小 → 步长大 → 快速前进
  - 即使横向误差很大，AUV 仍保持微小前进力（不会完全停死）
  - 过渡平滑，速度导数连续，对系统冲击最小

收敛检测：
  每一步发送后，等待 AUV 收敛到当前步目标附近再发下一步
  收敛条件：估算"以当前有效速度消除前向误差所需时间" ≤ STEP_PERIOD

  具体步骤：
  1. 将世界系误差投影到运动方向坐标系
  2. 将世界系速度投影到运动方向
  3. 指数衰减：v_eff = v_along * exp(-λ * |e_lateral|)
     → 横向偏差越大，有效前向速度越小
  4. 估算剩余时间：t_remaining = e_along / v_eff
  5. 方向一致性检查：前向误差 > 0 且前向速度 > 0（正在靠近目标）
  6. 收敛条件：converging 且 (t_remaining ≤ STEP_PERIOD 或 e_along < 阈值)

================================================================================
"""

import math
import threading

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from zit6_interfaces.msg import ZitSetpoint, ZitStatus

from uv_control.coordinate import Coordinate

# control_key constants
CK_POS = 0       # position mode
CK_VEL = 1       # velocity mode
CK_FORCE = 2     # force mode
CK_BODY = 0x10   # body frame
CK_INC = 0x20    # incremental


# ============================================================================
# 轴掩码：用于指定哪些轴参与控制和到达判断
# ============================================================================
AX_X = 0x01      # X 轴 (北/前进)
AX_Y = 0x02      # Y 轴 (东/右移)
AX_Z = 0x04      # Z 轴 (深度)
AX_RZ = 0x08     # Yaw 轴 (偏航)
AX_XY = AX_X | AX_Y                         # XY 平面
AX_XYZ = AX_X | AX_Y | AX_Z                 # XYZ 三维
AX_ALL = AX_X | AX_Y | AX_Z | AX_RZ         # 全部 4 轴

# ============================================================================
# 默认容差：到达目标的判定阈值
# ============================================================================
TOL_X = 0.1      # X 轴容差 (米)
TOL_Y = 0.1      # Y 轴容差 (米)
TOL_Z = 0.1      # Z 轴容差 (米)
TOL_RZ = 5.0     # Yaw 轴容差 (度)

# ============================================================================
# 步进控制参数
# ============================================================================
STEP_PERIOD = 0.3      # 目标步进间隔 (秒)，也是收敛检测的时间阈值
                        # 含义：当"以当前速度消除误差的时间 ≤ STEP_PERIOD"时，发下一步
LATERAL_LAMBDA = 2.0   # 横向误差敏感度（指数衰减参数）
                        # λ 越大，横向偏差对前进速度的抑制越强
                        # λ=2.0 时，横向误差 0.3m → 前进速度衰减到 e^(-0.6)≈0.55 倍
                        #           横向误差 1.0m → 前进速度衰减到 e^(-2.0)≈0.14 倍


class BasicMotionNode(Node):
    """高级运动控制节点。通过 ZIT6 协议控制 AUV。"""

    def __init__(self):
        super().__init__('basic_motion')

        # 当前 AUV 状态
        self.status = ZitStatus()
        self.pose = Coordinate()              # 当前位置
        self._target = Coordinate()           # 当前目标
        self._state_lock = threading.Lock()

        # 发布 ZIT6 setpoint 到 sim_bridge
        self.pub_setpoint = self.create_publisher(ZitSetpoint, '/zit6/cmd/setpoint', 10)
        # 订阅 ZIT6 状态
        self.create_subscription(ZitStatus, '/zit6/state/status', self._status_cb, 10)
        self.create_subscription(Float32MultiArray, '/zit6/state/pos', self._pos_cb, 10)

        self.get_logger().info('BasicMotion node started (ZIT6 protocol)')

    def _status_cb(self, msg: ZitStatus):
        with self._state_lock:
            self.status = msg

    def _pos_cb(self, msg: Float32MultiArray):
        with self._state_lock:
            self.pose = Coordinate.from_zit6_pos(msg.data)

    def get_state(self):
        """获取 AUV 当前状态（线程安全）。

        Returns: (pose: Coordinate, target: Coordinate, status: ZitStatus)
        """
        with self._state_lock:
            return self.pose.copy(), self._target.copy(), self.status

    # ========================================================================
    # 内部工具：发送指令 + 等待到达
    # ========================================================================

    def _cmd_and_wait(self, mode: int, x: float, y: float, z: float, yaw: float,
                      axes_mask: int = AX_ALL,
                      tol_x: float = TOL_X, tol_y: float = TOL_Y,
                      tol_z: float = TOL_Z, tol_rz: float = TOL_RZ,
                      timeout: float = 60.0) -> bool:
        """发送运动指令并阻塞等待到达目标位置。

        Args:
            mode: MotionCommand 模式（世界系/机体系/速度/力）
            x, y, z, yaw: 目标位置/速度/力（根据 mode 解释）
            axes_mask: 需要检查的轴掩码
            tol_x/y/z/rz: 各轴到达容差
            timeout: 超时时间 (秒)

        Returns:
            True = 到达目标, False = 超时
        """
        msg = ZitSetpoint()
        msg.control_key = mode
        msg.type_mask = 0
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        msg.yaw = float(yaw)
        msg.seq = 0
        self.pub_setpoint.publish(msg)

        # Track target for _wait_reached
        self._target = Coordinate(x=float(x), y=float(y), z=float(z),
                                   rz=math.degrees(yaw))

        return self._wait_reached(axes_mask, tol_x, tol_y, tol_z, tol_rz, timeout)

    def _wait_reached(self, axes_mask: int = AX_ALL,
                      tol_x: float = TOL_X, tol_y: float = TOL_Y,
                      tol_z: float = TOL_Z, tol_rz: float = TOL_RZ,
                      timeout: float = 60.0) -> bool:
        """阻塞等待 AUV 到达目标位置。

        以 10Hz 频率检查各轴误差是否在容差范围内。
        AuvState 中的 target_x/y/z/yaw 是 sim_bridge 的目标位置，
        pos_x/y/z/yaw 是 AUV 当前实际位置。
        """
        rate = self.create_rate(10)
        start = self.get_clock().now()

        while rclpy.ok():
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > timeout:
                self.get_logger().warn('move_wait timeout')
                return False

            p, t, _ = self.get_state()
            reached = True

            # 检查各轴误差是否在容差内
            if axes_mask & AX_X:
                if abs(t.x - p.x) > tol_x:
                    reached = False
            if axes_mask & AX_Y:
                if abs(t.y - p.y) > tol_y:
                    reached = False
            if axes_mask & AX_Z:
                if abs(t.z - p.z) > tol_z:
                    reached = False
            if axes_mask & AX_RZ:
                yaw_err = abs(math.degrees(t.yaw_rad - p.yaw_rad))
                if yaw_err > 180:
                    yaw_err = 360 - yaw_err
                if yaw_err > tol_rz:
                    reached = False

            if reached:
                return True

            rate.sleep()

        return False

    # ========================================================================
    # 步进控制辅助函数
    # ========================================================================

    def _calc_step_size(self, move_angle_rad: float) -> float:
        """根据运动方向计算基础步进距离。

        步长 = |cos(θ)| * TOL_X + |sin(θ)| * TOL_Y

        这是运动方向坐标系下的"允许误差"投影：
        - 纯 X 移动 (θ=0°): step = TOL_X = 0.1m
        - 纯 Y 移动 (θ=90°): step = TOL_Y = 0.1m
        - 斜向 45°: step = 0.707 * 0.1 + 0.707 * 0.1 ≈ 0.14m
        """
        ca = abs(math.cos(move_angle_rad))
        sa = abs(math.sin(move_angle_rad))
        return ca * TOL_X + sa * TOL_Y

    def _wait_step_convergence(self, move_angle: float,
                               timeout: float = 10.0) -> bool:
        """等待步进收敛（在运动方向坐标系中判断）。

        核心思想：估算"以当前有效速度消除前向误差所需时间"，
        当这个时间 ≤ STEP_PERIOD 时，认为可以发下一步。

        使用指数衰减来限制前进速度：
        v_eff = v_along * exp(-λ * |e_lateral|)
        → 横向偏差越大，有效前向速度越小，AUV 有更多时间修正航向

        Args:
            move_angle: 运动方向角 (弧度)，即 atan2(dy, dx)
            timeout: 超时时间 (秒)

        Returns:
            True = 收敛, False = 超时
        """
        rate = self.create_rate(10)
        start = self.get_clock().now()
        min_wait = STEP_PERIOD * 0.5  # 最少等待时间，防止过快下发
        ca = math.cos(move_angle)
        sa = math.sin(move_angle)

        while rclpy.ok():
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > timeout:
                return False
            if elapsed < min_wait:
                rate.sleep()
                continue

            p, t, _ = self.get_state()

            # 世界系误差
            ex_world = t.x - p.x
            ey_world = t.y - p.y

            # 投影到运动方向坐标系
            e_along = ca * ex_world + sa * ey_world      # 前向误差（剩余距离）
            e_lateral = -sa * ex_world + ca * ey_world   # 横向偏差

            # 世界系速度投影到运动方向
            v_along = ca * s.vel_world_x + sa * s.vel_world_y

            # 方向一致性：前向误差 > 0 且前向速度 > 0（正在朝目标前进）
            # 或前向误差已经很小（接近到达）
            converging = (e_along > 0 and v_along > 0) or e_along < 0.01

            # 指数衰减：横向偏差越大，有效前向速度越小
            v_eff = v_along * math.exp(-LATERAL_LAMBDA * abs(e_lateral))

            # 估算以当前有效速度消除前向误差所需时间
            if v_eff > 0.01:
                t_remaining = e_along / v_eff
            else:
                t_remaining = float('inf')

            # 收敛条件：正在靠近 且 (剩余时间 ≤ 阈值 或 误差已很小)
            step_thresh = self._calc_step_size(move_angle) * 0.1
            if converging and (t_remaining <= STEP_PERIOD or e_along < step_thresh):
                return True

            rate.sleep()

        return False

    # ========================================================================
    # 核心步进函数（动态步长）
    # ========================================================================

    def _step_move_world(self, target_x: float, target_y: float,
                         target_z: float, target_yaw_rad: float,
                         axes_mask: int = AX_ALL,
                         timeout: float = 60.0) -> bool:
        """步进移动：将长距离移动拆成小段逐段发送，每步根据当前误差动态计算步长。

        算法流程：
        1. 计算当前误差（世界系），投影到运动方向坐标系
        2. 根据横向误差计算动态步长：step = base * exp(-λ * |e_lateral|)
        3. 发送这一步的目标位置
        4. 等待收敛（误差+速度判断）
        5. 重复 1-4 直到达标
        6. 最后发送最终目标并等待精确到达

        动态步长的效果：
        - 横向偏差大 → 步长小 → AUV 有更多时间修正航向
        - 横向偏差小 → 步长大 → 快速前进
        - 每一步都重新计算，自适应当前状态
        """
        while rclpy.ok():
            p, t, _ = self.get_state()
            ex_world = target_x - p.x  # 世界系 X 误差
            ey_world = target_y - p.y  # 世界系 Y 误差
            dist_xy = math.sqrt(ex_world**2 + ey_world**2)

            # 计算运动方向角和基础步长
            if dist_xy > 0.03:
                move_angle = math.atan2(ey_world, ex_world)
                base_step = self._calc_step_size(move_angle)
            else:
                move_angle = 0.0
                base_step = TOL_X

            # 到达判定：XY 距离 < 基础步长的 10%
            if dist_xy < base_step * 0.1:
                break

            ca = math.cos(move_angle)
            sa = math.sin(move_angle)

            # 投影到运动方向坐标系
            e_along = ca * ex_world + sa * ey_world      # 前向剩余距离
            e_lateral = -sa * ex_world + ca * ey_world   # 横向偏差

            # 剩余距离不到一步 → 直接发最终目标，不再细分
            if e_along < base_step:
                break

            # 动态步长 = 基础步长 * 指数衰减(横向偏差)
            # 横向偏差越大，步长越小（优先修正横向）
            step_along = base_step * math.exp(-LATERAL_LAMBDA * abs(e_lateral))

            # 计算下一步的世界系目标坐标
            frac = step_along / dist_xy if dist_xy > 0.03 else 1.0
            step_x = p.x + ex_world * frac
            step_y = p.y + ey_world * frac
            # Z 轴：线性插值到最终目标（按剩余步数均分）
            dz_total = target_z - p.z
            steps_remaining = max(1, math.ceil(dist_xy / base_step))
            step_z = p.z + dz_total / steps_remaining

            # 发送这一步的目标位置
            msg = ZitSetpoint()
            msg.control_key = CK_POS
            msg.x = float(step_x)
            msg.y = float(step_y)
            msg.z = float(step_z)
            msg.yaw = float(target_yaw_rad)
            msg.seq = 0
            self.pub_setpoint.publish(msg)
            self._target = {'x': step_x, 'y': step_y, 'z': step_z, 'rz': math.degrees(target_yaw_rad)}
            self.pub_setpoint.publish(msg)

            # 等待收敛（在运动方向坐标系中判断）
            self._wait_step_convergence(move_angle, timeout=timeout)

        # 最终：发送精确目标并等待到达
        return self._cmd_and_wait(CK_POS,
                                  target_x, target_y, target_z, target_yaw_rad,
                                  axes_mask, timeout=timeout)

    # ========================================================================
    # SET 系列 — 绝对定位（世界系，直接发目标 + 等到达，不走步进）
    # ========================================================================
    # 适用于已知精确目标位置的场景。直接发送世界系绝对坐标，
    # sim_bridge 的级联 PID 控制器负责将 AUV 移动到目标位置。

    def setxyzrz(self, x: float, y: float, z: float, rz: float,
                 timeout: float = 60.0) -> bool:
        """设置绝对世界系位置 + 偏航角。rz 单位为度。"""
        rz_rad = math.radians(rz)
        self._cmd_and_wait(CK_POS, x, y, z, rz_rad, AX_ALL, timeout=timeout)
        return True

    def setxyz(self, x: float, y: float, z: float, timeout: float = 60.0) -> bool:
        """设置绝对世界系位置（不改变偏航角）。"""
        p, t, _ = self.get_state()
        self._cmd_and_wait(CK_POS, x, y, z, p.yaw_rad, AX_XYZ, timeout=timeout)
        return True

    def setxy(self, x: float, y: float, timeout: float = 60.0) -> bool:
        """设置绝对世界系 XY 位置（不改变深度和偏航）。"""
        p, t, _ = self.get_state()
        self._cmd_and_wait(CK_POS, x, y, p.z, p.yaw_rad, AX_XY, timeout=timeout)
        return True

    def setxyrz(self, x: float, y: float, rz: float, timeout: float = 60.0) -> bool:
        """设置绝对世界系 XY 位置 + 偏航角。"""
        rz_rad = math.radians(rz)
        p, t, _ = self.get_state()
        self._cmd_and_wait(CK_POS, x, y, p.z, rz_rad,
                           AX_X | AX_Y | AX_RZ, timeout=timeout)
        return True

    def setz(self, z: float, timeout: float = 60.0) -> bool:
        """设置绝对深度。"""
        p, t, _ = self.get_state()
        self._cmd_and_wait(CK_POS, p.x, p.y, z, p.yaw_rad,
                           AX_Z, timeout=timeout)
        return True

    def setrz(self, rz: float, timeout: float = 60.0) -> bool:
        """设置绝对偏航角（单位：度）。"""
        rz_rad = math.radians(rz)
        p, t, _ = self.get_state()
        self._cmd_and_wait(CK_POS, p.x, p.y, p.z, rz_rad,
                           AX_RZ, timeout=timeout)
        return True

    def setx(self, x: float, timeout: float = 60.0) -> bool:
        """设置绝对 X (北) 位置。"""
        p, t, _ = self.get_state()
        self._cmd_and_wait(CK_POS, x, p.y, p.z, p.yaw_rad,
                           AX_X, timeout=timeout)
        return True

    def sety(self, y: float, timeout: float = 60.0) -> bool:
        """设置绝对 Y (东) 位置。"""
        p, t, _ = self.get_state()
        self._cmd_and_wait(CK_POS, p.x, y, p.z, p.yaw_rad,
                           AX_Y, timeout=timeout)
        return True

    # ========================================================================
    # WMOVE 系列 — 世界系步进移动
    # ========================================================================
    # 参数为世界系偏移量 (dx, dy, dz, drz)，相对于当前位置。
    # 内部调用 _step_move_world，将长距离拆成小段逐段发送。
    # 与 SET 的区别：SET 一次到位，WMOVE 分段步进。

    def wmovexyzrz(self, dx: float, dy: float, dz: float, drz: float,
                   timeout: float = 60.0) -> bool:
        """世界系步进：移动 (dx, dy, dz) + 旋转 drz 度。"""
        p, t, _ = self.get_state()
        target_x = p.x + dx
        target_y = p.y + dy
        target_z = p.z + dz
        target_rz = p.yaw_rad + math.radians(drz)
        return self._step_move_world(target_x, target_y, target_z, target_rz,
                                     AX_ALL, timeout=timeout)

    def wmovexyz(self, dx: float, dy: float, dz: float, timeout: float = 60.0) -> bool:
        """世界系步进：移动 (dx, dy, dz)。"""
        p, t, _ = self.get_state()
        target_x = p.x + dx
        target_y = p.y + dy
        target_z = p.z + dz
        return self._step_move_world(target_x, target_y, target_z, p.yaw_rad,
                                     AX_XYZ, timeout=timeout)

    def wmovexy(self, dx: float, dy: float, timeout: float = 60.0) -> bool:
        """世界系步进：XY 平面移动。"""
        p, t, _ = self.get_state()
        target_x = p.x + dx
        target_y = p.y + dy
        return self._step_move_world(target_x, target_y, p.z, p.yaw_rad,
                                     AX_XY, timeout=timeout)

    def wmovexyrz(self, dx: float, dy: float, drz: float, timeout: float = 60.0) -> bool:
        """世界系步进：XY 移动 + 偏航旋转。"""
        p, t, _ = self.get_state()
        target_x = p.x + dx
        target_y = p.y + dy
        target_rz = p.yaw_rad + math.radians(drz)
        return self._step_move_world(target_x, target_y, p.z, target_rz,
                                     AX_X | AX_Y | AX_RZ, timeout=timeout)

    def wmovez(self, dz: float, timeout: float = 60.0) -> bool:
        """世界系步进：改变深度。"""
        p, t, _ = self.get_state()
        target_z = p.z + dz
        return self._step_move_world(p.x, p.y, target_z, p.yaw_rad,
                                     AX_Z, timeout=timeout)

    def wmoverz(self, drz: float, timeout: float = 60.0) -> bool:
        """世界系步进：偏航旋转（单位：度）。"""
        p, t, _ = self.get_state()
        target_rz = p.yaw_rad + math.radians(drz)
        return self._step_move_world(p.x, p.y, p.z, target_rz,
                                     AX_RZ, timeout=timeout)

    def wmovex(self, dx: float, timeout: float = 60.0) -> bool:
        """世界系步进：沿北方向移动。"""
        p, t, _ = self.get_state()
        target_x = p.x + dx
        return self._step_move_world(target_x, p.y, p.z, p.yaw_rad,
                                     AX_X, timeout=timeout)

    def wmovey(self, dy: float, timeout: float = 60.0) -> bool:
        """世界系步进：沿东方向移动。"""
        p, t, _ = self.get_state()
        target_y = p.y + dy
        return self._step_move_world(p.x, target_y, p.z, p.yaw_rad,
                                     AX_Y, timeout=timeout)

    # ========================================================================
    # BMOVE 系列 — 机体系步进移动
    # ========================================================================
    # 参数为机体系偏移量 (dx=前进, dy=右移, dz=下潜, drz=偏航)。
    # 内部先做 body→world 坐标变换，再调用 _step_move_world。
    # 与 WMOVE 的区别：WMOVE 参数是世界系，BMOVE 参数是机体系。

    def bmovexyzrz(self, dx: float, dy: float, dz: float, drz: float,
                   timeout: float = 60.0) -> bool:
        """机体系步进：移动 (dx, dy, dz) + 旋转 drz 度。"""
        p, t, _ = self.get_state()
        off = p.body_to_world(dx, dy); dx_w, dy_w = off.x, off.y
        target_x = p.x + dx_w
        target_y = p.y + dy_w
        target_z = p.z + dz
        target_rz = p.yaw_rad + math.radians(drz)
        return self._step_move_world(target_x, target_y, target_z, target_rz,
                                     AX_ALL, timeout=timeout)

    def bmovexyz(self, dx: float, dy: float, dz: float, timeout: float = 60.0) -> bool:
        """机体系步进：移动 (dx, dy, dz)。"""
        p, t, _ = self.get_state()
        off = p.body_to_world(dx, dy); dx_w, dy_w = off.x, off.y
        target_x = p.x + dx_w
        target_y = p.y + dy_w
        target_z = p.z + dz
        return self._step_move_world(target_x, target_y, target_z, p.yaw_rad,
                                     AX_XYZ, timeout=timeout)

    def bmovexy(self, dx: float, dy: float, timeout: float = 60.0) -> bool:
        """机体系步进：XY 平面移动（dx=前进, dy=右移）。"""
        p, t, _ = self.get_state()
        off = p.body_to_world(dx, dy); dx_w, dy_w = off.x, off.y
        target_x = p.x + dx_w
        target_y = p.y + dy_w
        return self._step_move_world(target_x, target_y, p.z, p.yaw_rad,
                                     AX_XY, timeout=timeout)

    def bmovexyrz(self, dx: float, dy: float, drz: float, timeout: float = 60.0) -> bool:
        """机体系步进：XY 移动 + 偏航旋转。"""
        p, t, _ = self.get_state()
        off = p.body_to_world(dx, dy); dx_w, dy_w = off.x, off.y
        target_x = p.x + dx_w
        target_y = p.y + dy_w
        target_rz = p.yaw_rad + math.radians(drz)
        return self._step_move_world(target_x, target_y, p.z, target_rz,
                                     AX_X | AX_Y | AX_RZ, timeout=timeout)

    def bmovez(self, dz: float, timeout: float = 60.0) -> bool:
        """机体系步进：改变深度（正=下潜, 负=上浮）。"""
        p, t, _ = self.get_state()
        target_z = p.z + dz
        return self._step_move_world(p.x, p.y, target_z, p.yaw_rad,
                                     AX_Z, timeout=timeout)

    def bmoverz(self, drz: float, timeout: float = 60.0) -> bool:
        """机体系步进：偏航旋转（单位：度）。"""
        p, t, _ = self.get_state()
        target_rz = p.yaw_rad + math.radians(drz)
        return self._step_move_world(p.x, p.y, p.z, target_rz,
                                     AX_RZ, timeout=timeout)

    def bmovex(self, dx: float, timeout: float = 60.0) -> bool:
        """机体系步进：前进 dx 米。"""
        return self.bmovexy(dx, 0.0, timeout=timeout)

    def bmovey(self, dy: float, timeout: float = 60.0) -> bool:
        """机体系步进：右移 dy 米。"""
        return self.bmovexy(0.0, dy, timeout=timeout)

    # ========================================================================
    # TRAVEL 系列 — 直线移动
    # ========================================================================
    # 与 WMOVE/BMOVE 的区别：
    #   WMOVE/BMOVE: 直接朝目标步进，AUV 可能斜着走（不朝向运动方向）
    #   TRAVEL: 先转向目标方向，再沿 body-X 轴前进，保证走直线
    #
    # 适用场景：需要直线路径的任务，如过门、巡线、精确对接等
    #
    # 流程：
    #   1. 计算目标方向角 θ = atan2(dy, dx)
    #   2. 旋转 AUV 使 body-X 朝向目标（先转 yaw）
    #   3. 沿 body-X 步进前进（同时可调整 Z）
    #
    # WTRAVEL: 世界系偏移量参数
    # BTRAVEL: 机体系偏移量参数（内部转世界系后调用 _travel_world）

    def _travel_world(self, dx_w: float, dy_w: float, dz: float = 0.0,
                      timeout: float = 60.0) -> bool:
        """直线移动基础函数：先转向目标方向，再沿 body-X 步进前进。

        Args:
            dx_w: 世界系 X 偏移 (北)
            dy_w: 世界系 Y 偏移 (东)
            dz: Z 轴偏移 (深度)
            timeout: 超时时间 (秒)
        """
        p, t, _ = self.get_state()
        target_x = p.x + dx_w
        target_y = p.y + dy_w
        target_z = p.z + dz

        dist_xy = math.sqrt(dx_w**2 + dy_w**2)
        if dist_xy > 0.03:
            # 计算目标方向角（NED: atan2(东, 北)）
            target_yaw = math.atan2(dy_w, dx_w)
            # 第一步：旋转 AUV 朝向目标
            self._step_move_world(p.x, p.y, p.z, target_yaw,
                                  AX_RZ, timeout=timeout)
            # 第二步：沿 body-X 步进前进（同时调整深度）
            return self._step_move_world(target_x, target_y, target_z, target_yaw,
                                         AX_XY | AX_Z, timeout=timeout)
        else:
            # 纯深度移动（无 XY 偏移）
            return self._step_move_world(p.x, p.y, target_z, p.yaw_rad,
                                         AX_Z, timeout=timeout)

    # --- WTRAVEL: 世界系直线移动 ---

    def wtravelxy(self, dx: float, dy: float, timeout: float = 60.0) -> bool:
        """世界系直线移动：先转向目标，再沿 body-X 前进。"""
        return self._travel_world(dx, dy, 0.0, timeout=timeout)

    def wtravelxyz(self, dx: float, dy: float, dz: float,
                   timeout: float = 60.0) -> bool:
        """世界系直线移动：先转向目标，沿 body-X 前进 + 调整深度。"""
        return self._travel_world(dx, dy, dz, timeout=timeout)

    def wtravelx(self, dx: float, timeout: float = 60.0) -> bool:
        """世界系直线移动：沿北方向前进。"""
        return self._travel_world(dx, 0.0, 0.0, timeout=timeout)

    def wtravely(self, dy: float, timeout: float = 60.0) -> bool:
        """世界系直线移动：先转向东/西，再沿 body-X 前进。"""
        return self._travel_world(0.0, dy, 0.0, timeout=timeout)

    def wtravelz(self, dz: float, timeout: float = 60.0) -> bool:
        """世界系直线移动：纯深度变化。"""
        return self._travel_world(0.0, 0.0, dz, timeout=timeout)

    # --- BTRAVEL: 机体系直线移动 ---

    def btravelxy(self, dx: float, dy: float, timeout: float = 60.0) -> bool:
        """机体系直线移动：先转向目标，再沿 body-X 前进。dx=前进, dy=右移。"""
        p, t, _ = self.get_state()
        off = p.body_to_world(dx, dy); dx_w, dy_w = off.x, off.y
        return self._travel_world(dx_w, dy_w, 0.0, timeout=timeout)

    def btravelxyz(self, dx: float, dy: float, dz: float,
                   timeout: float = 60.0) -> bool:
        """机体系直线移动：先转向目标，沿 body-X 前进 + 调整深度。"""
        p, t, _ = self.get_state()
        off = p.body_to_world(dx, dy); dx_w, dy_w = off.x, off.y
        return self._travel_world(dx_w, dy_w, dz, timeout=timeout)

    def btravelx(self, dx: float, timeout: float = 60.0) -> bool:
        """机体系直线移动：纯前进。等同于 bmovex。"""
        return self.bmovex(dx, timeout=timeout)

    def btravely(self, dy: float, timeout: float = 60.0) -> bool:
        """机体系直线移动：先转向右侧方向，再沿 body-X 前进。"""
        p, t, _ = self.get_state()
        off = p.body_to_world(0.0, dy); dx_w, dy_w = off.x, off.y
        return self._travel_world(dx_w, dy_w, 0.0, timeout=timeout)

    def btravelz(self, dz: float, timeout: float = 60.0) -> bool:
        """机体系直线移动：纯深度变化。"""
        return self._travel_world(0.0, 0.0, dz, timeout=timeout)


def main(args=None):
    rclpy.init(args=args)
    node = BasicMotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
