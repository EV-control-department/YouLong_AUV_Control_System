"""
basic_motion.py — 运动控制节点（合并 ZIT6 底层 + 高级运动 API）

分层架构（从底到顶）：

  Layer 5: 对外高级 API (SET / WMOVE / BMOVE / TRAVEL / BODY_VELOCITY)
  Layer 4: 步进坐标系 (运动方向 along/lateral 分解 + 动态步长)
  Layer 3: 机器人坐标系 (body frame — set_body / set_step)
  Layer 2: 世界坐标系 (odom — start / set_world / get_state)
  Layer 1: ZIT6 底层 (map 坐标系 — _send_setpoint)

坐标系说明：
  map 系 — ZIT6 协议使用的绝对坐标；原点由 uv_localization 提供
  odom 系 — 以 AUV 启动位置为原点的世界坐标系（start() 时初始化）
  body 系 — 以 AUV 当前位置为原点的机体坐标系

指令路径（保证 single source of truth）：
  高级 API → set_world → set_map → _send_setpoint → /auv/hardware/zit6/cmd/setpoint
  set_body/set_step → Coordinate 变换 → set_world → …
  set_map 是唯一的协议出口，迁移协议只需改此函数

================================================================================
系统架构
================================================================================

  ┌─────────────────────────────────────────────────────┐
  │  uv_task (任务层)                                    │
  │  YAML mission 加载 → 顺序执行 → 调用导航/运动            │
  ├─────────────────────────────────────────────────────┤
  │  uv_nav (导航层)                                     │
  │  A* 路径规划 → 避障 → 调用 basic_motion 执行           │
  ├─────────────────────────────────────────────────────┤
  │  uv_control (控制层)  ← 本文件所在层                   │
  │  basic_motion (本节点)                                │
  │    → set_world / set_body / set_step                 │
  │    → _send_setpoint → /auv/hardware/zit6/cmd/setpoint│
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

4. BODY_VELOCITY
   - 机体系瞬时速度指令，通过短租约保持有效
   - 租约未续期时自动发送零速度
   - 视觉伺服等高频控制只通过 BasicMotion Action 进入本层
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import Empty
from auv_protocol.topics import (
    BASIC_MOTION, LEGACY_BASIC_MOTION, LEGACY_POSE_INFO,
    LEGACY_ZIT6_SETPOINT, STATE_ODOM, STATE_RESET, STATE_TWIST,
    ZIT6_SETPOINT,
    ZIT6_STATUS,
)

from zit6_interfaces.msg import ZitSetpoint, ZitStatus
from uv_msgs.action import BasicMotion
from uv_msgs.msg import PoseInfo
from uv_control.coordinate import Coordinate, wrap_deg, wrap_rad

# ═════════════════════════════════════════════════════════════════════════════
# ZIT6 control_key 常量
# ═════════════════════════════════════════════════════════════════════════════
# Position and body-velocity modes are the only ZIT6 modes emitted here;
# body/增量转换在上层完成。
CK_POS = 0          # position mode (唯一的 control_key)
CK_VEL_BODY = 0x11  # body-frame velocity mode (VEL | BODY)
DEFAULT_VELOCITY_LEASE = 0.25
VELOCITY_WATCHDOG_PERIOD = 0.05

# ═════════════════════════════════════════════════════════════════════════════
# 轴掩码
# ═════════════════════════════════════════════════════════════════════════════
AX_X = 0x01
AX_Y = 0x02
AX_Z = 0x04
# ZitSetpoint.type_mask follows the native six-axis order
# [x, y, z, roll, pitch, yaw].  A set bit means "leave this axis
# unchanged".  Keep the old AX_RZ spelling as a compatibility alias, but
# point it at the yaw bit (bit 5), not roll (bit 3).
AX_ROLL = 0x08
AX_PITCH = 0x10
AX_YAW = 0x20
AX_RZ = AX_YAW
AX_XY = AX_X | AX_Y
AX_XYZ = AX_X | AX_Y | AX_Z
# The controller is intentionally used as a four-DOF vehicle (x/y/z/yaw).
AX_ALL = AX_X | AX_Y | AX_Z | AX_YAW
AX_ALL_6DOF = AX_ALL | AX_ROLL | AX_PITCH

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
STEP_PERIOD = 0.2        # 目标步进间隔/收敛时间阈值 (秒)
LATERAL_LAMBDA = 2.0     # 横向误差指数衰减系数


def _as_bool(value) -> bool:
    """Parse launch/YAML booleans without treating ``'false'`` as true."""
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


class BasicMotionNode(Node):
    """运动控制节点：ZIT6 协议接口 + 高级运动 API，single source of truth。"""

    def __init__(self):
        super().__init__('basic_motion')

        # ── 状态 ───────────────────────────────────────────────
        self.status = ZitStatus()
        self.pose = Coordinate()        # 当前位置 (odom 系)
        self._target = Coordinate()     # 当前目标 (odom 系)
        self._state_origin = Coordinate()  # estimator-provided map origin
        self._origin = None             # active odom origin (map Coordinate)
        self._state_lock = threading.Lock()
        self._velocity_lock = threading.Lock()
        self._shutdown_requested = False
        self._timers = []
        self._velocity_active = False
        self._velocity_deadline = 0.0
        self.vel_body = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rz': 0.0}   # 机体速度
        self._pose_stamp = self.get_clock().now().to_msg()
        self.declare_parameter('sim_mode', False)
        self._sim_mode = _as_bool(self.get_parameter('sim_mode').value)

        # ─────────────────────────────────────────────────────────
        # Layer 1: ZIT6 协议发布/订阅
        # ─────────────────────────────────────────────────────────
        self.pub_setpoint = self.create_publisher(
            ZitSetpoint, ZIT6_SETPOINT, 10)
        # Compatibility output for the pre-V1 hardware adapter.  New nodes
        # must use ZIT6_SETPOINT above.
        self.pub_setpoint_legacy = self.create_publisher(
            ZitSetpoint, LEGACY_ZIT6_SETPOINT, 10)
        self.create_subscription(
            ZitStatus, ZIT6_STATUS, self._status_cb, 10)
        self.create_subscription(
            PoseInfo, STATE_ODOM, self._state_odom_cb, 10)
        self.create_subscription(
            TwistWithCovarianceStamped, STATE_TWIST,
            self._state_twist_cb, 10)

        # ── Action Server ────────────────────────────────────────
        self._action_server = ActionServer(
            self, BasicMotion, BASIC_MOTION,
            goal_callback=self._action_goal_cb,
            cancel_callback=self._action_cancel_cb,
            execute_callback=self._action_execute_cb,
        )
        self._legacy_action_server = ActionServer(
            self, BasicMotion, LEGACY_BASIC_MOTION,
            goal_callback=self._action_goal_cb,
            cancel_callback=self._action_cancel_cb,
            execute_callback=self._action_execute_cb,
        )
        self._action_goal_handle = None
        self._action_target = None       # 绝对目标 {x, y, z, yaw}，用于反馈
        self._action_axes = None         # 当前 action 的生效轴，用于反馈
        self._timers.append(self.create_timer(0.5, self._action_feedback_cb))
        self._timers.append(self.create_timer(
            VELOCITY_WATCHDOG_PERIOD, self._velocity_watchdog_cb))

        # ``/auv/state/odom`` and ``/auv/tf`` belong exclusively to
        # uv_localization.  BasicMotion keeps the old PoseInfo stream only as
        # a compatibility output for tools that still consume it.
        self.pub_pose_legacy = self.create_publisher(
            PoseInfo, LEGACY_POSE_INFO, 10)
        self.pub_state_reset = self.create_publisher(Empty, STATE_RESET, 10)
        self._timers.append(self.create_timer(1.0 / 30.0, self._publish_pose_info))
        self.context.on_shutdown(self._on_context_shutdown)

        self.get_logger().info('BasicMotion node started')

    def _on_context_shutdown(self):
        """Stop callbacks before the ROS context starts destroying handles.

        ``MultiThreadedExecutor`` may still dispatch a timer callback during
        SIGINT handling.  Marking the node as quiescing and cancelling the
        timers prevents those callbacks from publishing through an already
        destroyed DDS handle.
        """
        if not self._shutdown_requested:
            try:
                self._publish_body_velocity()
            except Exception:
                pass
        self._shutdown_requested = True
        for timer in self._timers:
            try:
                timer.cancel()
            except Exception:
                # A late shutdown callback can observe a timer whose handle
                # has already been destroyed by rclpy.
                pass

    def _publish_while_running(self, publisher, message):
        """Publish unless shutdown has started, tolerating its final race."""
        if self._shutdown_requested:
            return
        try:
            publisher.publish(message)
        except Exception:
            # A timer can pass the flag check immediately before rclpy tears
            # down its publisher handle.  Do not turn that expected shutdown
            # race into an un-retrieved task exception; surface all other
            # publish failures normally.
            if not self._shutdown_requested:
                raise

    def destroy_node(self):
        """Destroy action servers and timers in a deterministic order."""
        self._on_context_shutdown()
        self._action_server.destroy()
        self._legacy_action_server.destroy()
        return super().destroy_node()

    # ═════════════════════════════════════════════════════════════════════════
    # Layer 1: ZIT6 底层 (map 坐标系)
    # ═════════════════════════════════════════════════════════════════════════

    def _send_setpoint(self, control_key: int, type_mask: int,
                       x: float, y: float, z: float, yaw_rad: float):
        """发送 ZitSetpoint。坐标是 map 系，yaw 是弧度，只走 CK_POS 位置模式。"""
        # A position command supersedes any leased body-velocity command.
        with self._velocity_lock:
            self._velocity_active = False
            self._velocity_deadline = 0.0
        msg = ZitSetpoint()
        msg.control_key = control_key
        msg.type_mask = type_mask
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        msg.yaw = float(yaw_rad)
        msg.roll = 0.0         # roll/pitch 未使用，控制栈保持 4-DOF
        msg.pitch = 0.0
        msg.seq = 0
        odom_t = self._map_to_odom(
            Coordinate(x=x, y=y, z=z, rz=math.degrees(yaw_rad)))
        with self._state_lock:
            self._target = odom_t
        self.pub_setpoint.publish(msg)
        self.pub_setpoint_legacy.publish(msg)
        self.get_logger().info(
            f'发往ZIT6: map=({x:.2f}, {y:.2f}, {z:.2f}, {math.degrees(yaw_rad):.1f}°), '
            f'对应odom=({odom_t.x:.2f}, {odom_t.y:.2f}, {odom_t.z:.2f}, {odom_t.rz:.1f}°)')

    def _publish_body_velocity(self, forward_mps: float = 0.0,
                               lateral_mps: float = 0.0,
                               vertical_mps: float = 0.0,
                               yaw_rate_deg_s: float = 0.0,
                               lease_s: float = DEFAULT_VELOCITY_LEASE):
        """Publish a body velocity command and arm its expiry watchdog.

        Higher-level tasks must use the ``BODY_VELOCITY`` BasicMotion action;
        this method is the only place in the motion node that knows the ZIT6
        velocity wire format.
        """
        values = (
            float(forward_mps), float(lateral_mps), float(vertical_mps),
            float(yaw_rate_deg_s),
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError('body velocity command must be finite')

        is_zero = max(abs(value) for value in values) <= 1e-6
        with self._velocity_lock:
            if is_zero:
                self._velocity_active = False
                self._velocity_deadline = 0.0
            else:
                lease = (float(lease_s) if float(lease_s) > 0.0
                         else DEFAULT_VELOCITY_LEASE)
                self._velocity_active = True
                self._velocity_deadline = time.monotonic() + lease

        msg = ZitSetpoint()
        msg.control_key = CK_VEL_BODY
        msg.type_mask = 0
        msg.x = values[0]
        msg.y = values[1]
        msg.z = values[2]
        msg.roll = 0.0
        msg.pitch = 0.0
        msg.yaw = math.radians(values[3])
        msg.seq = 0
        self._publish_while_running(self.pub_setpoint, msg)
        self._publish_while_running(self.pub_setpoint_legacy, msg)

    def _velocity_watchdog_cb(self):
        """Stop a velocity command when its action lease is not renewed."""
        with self._velocity_lock:
            expired = (
                self._velocity_active
                and time.monotonic() >= self._velocity_deadline
            )
        if expired:
            self.get_logger().warning(
                'BODY_VELOCITY command lease expired; sending zero velocity')
            self._publish_body_velocity()

    def _status_cb(self, msg: ZitStatus):
        with self._state_lock:
            self.status = msg

    def _vel_cb(self, msg: Float32MultiArray):
        """Deprecated raw velocity callback; control uses ``STATE_TWIST``."""
        del msg

    def _state_odom_cb(self, msg: PoseInfo):
        """Consume the estimator state used by all higher-level control APIs."""
        values = (msg.robot_x, msg.robot_y, msg.robot_z,
                  msg.robot_roll, msg.robot_pitch, msg.robot_yaw)
        if not all(math.isfinite(float(value)) for value in values):
            return
        with self._state_lock:
            # In SIL/HIL the estimator starts at a zero odom origin and there
            # is no raw ZIT6 position stream.  Allow actions after START to use
            # the formal state without manufacturing a second state publisher.
            if self._sim_mode and self._origin is None:
                self._origin = Coordinate()
            self._state_origin = Coordinate(
                x=float(msg.origin_x), y=float(msg.origin_y),
                z=float(msg.origin_z), rz=float(msg.origin_yaw))
            self.pose = Coordinate(
                x=float(msg.robot_x), y=float(msg.robot_y),
                z=float(msg.robot_z), rx=float(msg.robot_roll),
                ry=float(msg.robot_pitch), rz=float(msg.robot_yaw))
            self._pose_stamp = msg.stamp

    def _state_twist_cb(self, msg: TwistWithCovarianceStamped):
        """Consume estimator body velocity for feedback and step control."""
        with self._state_lock:
            twist = msg.twist.twist
            values = (twist.linear.x, twist.linear.y, twist.linear.z,
                      twist.angular.x, twist.angular.y, twist.angular.z)
            if all(math.isfinite(float(value)) for value in values):
                self.vel_body = {
                    'x': float(twist.linear.x), 'y': float(twist.linear.y),
                    'z': float(twist.linear.z), 'rx': float(twist.angular.x),
                    'ry': float(twist.angular.y), 'rz': float(twist.angular.z),
                }


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
                self.get_logger().info(
                    'odom origin already set, updating to current map pose')
            self._origin = Coordinate(
                x=self._state_origin.x, y=self._state_origin.y,
                z=self._state_origin.z, rz=self._state_origin.rz)
            self.get_logger().info(
                f'DEBUG estimator origin: x={self._origin.x:.4f}, '
                f'y={self._origin.y:.4f}, z={self._origin.z:.4f}, '
                f'rz={self._origin.rz:.4f}')
            self.pose.x = 0.0
            self.pose.y = 0.0
            self.pose.z = 0.0
            self.pose.rz = 0.0
            # START 重新定义坐标系时，旧动作目标也必须一起清零。
            self._target = Coordinate()
        self._publish_while_running(self.pub_state_reset, Empty())
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
        # Action callbacks and estimator callbacks run concurrently.  Return
        # snapshots instead of the mutable objects stored by the callbacks.
        with self._state_lock:
            pose = Coordinate(
                x=self.pose.x, y=self.pose.y, z=self.pose.z,
                rx=self.pose.rx, ry=self.pose.ry, rz=self.pose.rz)
            target = Coordinate(
                x=self._target.x, y=self._target.y, z=self._target.z,
                rx=self._target.rx, ry=self._target.ry, rz=self._target.rz)
            status = self.status
        return pose, target, status

    def _target_error_snapshot(self):
        """Return one consistent measured-pose/target/body-error snapshot."""
        with self._state_lock:
            pose = Coordinate(
                x=self.pose.x, y=self.pose.y, z=self.pose.z,
                rx=self.pose.rx, ry=self.pose.ry, rz=self.pose.rz)
            target = Coordinate(
                x=self._target.x, y=self._target.y, z=self._target.z,
                rx=self._target.rx, ry=self._target.ry, rz=self._target.rz)
        error = pose.world_to_body(
            target.x - pose.x, target.y - pose.y, target.z - pose.z)
        error.rz = wrap_deg(target.rz - pose.rz)
        return pose, target, error

    # ═════════════════════════════════════════════════════════════════════════
    # Layer 3: 机器人坐标系 (body frame)
    # ═════════════════════════════════════════════════════════════════════════

    def set_body(self, x: float, y: float, z: float, yaw_deg: float):
        """设置机体系绝对位置。yaw 单位为度。"""
        _, t, _ = self.get_state()
        body_target = Coordinate(x=x, y=y, z=z, rz=yaw_deg)
        new_target = t.to_world_frame(body_target)
        self.get_logger().info(
            f'set_body: body目标=({x:.2f}, {y:.2f}, {z:.2f}, {yaw_deg:.1f}°) '
            f'→ world目标=({new_target.x:.2f}, {new_target.y:.2f}, {new_target.z:.2f}, {new_target.rz:.1f}°)')
        self.set_world(new_target.x, new_target.y, new_target.z, new_target.rz)

    def set_step(self, dx: float, dy: float, dz: float, dyaw_deg: float):
        """设置机体系增量步进。dyaw 单位为度。"""
        # 增量必须从当前实测位姿开始；使用上一次目标会在上一段未完全
        # 收敛时把下一段继续向前推，造成 BMOVE 的目标和误差看起来漂移。
        p, _, _ = self.get_state()
        target_step = Coordinate(x=dx, y=dy, z=dz, rz=dyaw_deg)
        map_target = p.to_world_frame(target_step)

        self.get_logger().info(
            f'set_step: 增量=({dx:.3f}, {dy:.3f}, {dz:.3f}, {dyaw_deg:.2f}°) '
            f'→ world目标=({map_target.x:.2f}, {map_target.y:.2f}, {map_target.z:.2f}, {map_target.rz:.1f}°)')
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
        p, _, _ = self.get_state()
        dist = math.sqrt((target_x-p.x)**2 + (target_y-p.y)**2 + (target_z-p.z)**2)
        self.get_logger().info(
            f'最终定位: 目标=({target_x:.2f}, {target_y:.2f}, {target_z:.2f}, {target_yaw_degree:.1f}°), '
            f'当前位置=({p.x:.2f}, {p.y:.2f}, {p.z:.2f}, {p.rz:.1f}°), '
            f'距离={dist:.2f}m')
        self.set_world(target_x, target_y, target_z, target_yaw_degree)
        return self._wait_reached(timeout=timeout)
    
    
    def _wait_reached(self,
                      tol_x: float = TOL_X, tol_y: float = TOL_Y,
                      tol_z: float = TOL_Z, tol_rz: float = TOL_RZ,
                      timeout: float = 60.0) -> bool:
        """阻塞等待 AUV 到达目标位置。"""
        start = time.monotonic()
        self._wait_count = 0

        while rclpy.ok():
            if self._is_cancelled():
                self.get_logger().info('等待到达: 被取消')
                return False
            elapsed = time.monotonic() - start
            if elapsed > timeout:
                self.get_logger().warning(f'等待到达超时 ({timeout:.0f}s)')
                return False

            pose, target, body_target = self._target_error_snapshot()

            err_x = abs(body_target.x)
            err_y = abs(body_target.y)
            err_z = abs(body_target.z)
            err_yaw = abs(wrap_deg(body_target.rz))

            reached = True
            if err_x > tol_x:
                reached = False
            if err_y > tol_y:
                reached = False
            if err_z > tol_z:
                reached = False
            if err_yaw > tol_rz:
                reached = False

            # 每秒打印一次误差
            self._wait_count += 1
            if self._wait_count % 10 == 0:
                self.get_logger().info(
                    f'等待到达: pose=({pose.x:.3f}, {pose.y:.3f}, '
                    f'{pose.z:.3f}, {pose.rz:.2f}°), '
                    f'target=({target.x:.3f}, {target.y:.3f}, '
                    f'{target.z:.3f}, {target.rz:.2f}°), '
                    f'err_body_signed=({body_target.x:.3f}, '
                    f'{body_target.y:.3f}, {body_target.z:.3f}, '
                    f'{body_target.rz:.2f}°), '
                    f'容差=({tol_x}, {tol_y}, {tol_z}, {tol_rz}), '
                    f'已用{elapsed:.0f}s/{timeout:.0f}s')

            if reached:
                self.get_logger().info(f'到达目标, 共耗时{elapsed:.1f}s')
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

    def _wait_step_convergence(self, move_angle: float = 0.0,
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


            _, _, body_target = self._target_error_snapshot()


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
            if converging and (t_remaining <= STEP_PERIOD * 2  or e_along < step_thresh):
                return True

            time.sleep(0.1)

        return False

    def _step_move_world(self, target_x: float, target_y: float,
                         target_z: float, target_yaw_degree: float,
                         timeout: float = 60.0) -> bool:
        """步进移动：odom 系目标，拆成小段逐段发送。"""
        step_no = 0
        start = time.monotonic()
        self.get_logger().info(
            f'步进开始: 目标=({target_x:.2f}, {target_y:.2f}, {target_z:.2f}, {target_yaw_degree:.1f}°), '
            f'总超时={timeout:.0f}s')
        while rclpy.ok():
            remaining = timeout - (time.monotonic() - start)
            if remaining <= 0:
                self.get_logger().warning(f'步进超时: 已用{time.monotonic()-start:.0f}s')
                return False
            if self._is_cancelled():
                self.get_logger().info('步进被取消')
                return False

            target_world = Coordinate(x=target_x, y=target_y, z=target_z)
            p, _, _ = self.get_state()
            target_body = p.world_to_body(
                target_world.x - p.x, target_world.y - p.y,
                target_world.z - p.z)

            # 机体坐标系误差 → 运动方向坐标系
            ex_body = target_body.x
            ey_body = target_body.y
            dist_xy = math.hypot(target_world.x - p.x,
                                 target_world.y - p.y)
            dist_3d = math.sqrt(dist_xy**2 + (target_z - p.z)**2)

            move_angle = math.atan2(ey_body, ex_body)
            base_step = self._calc_step_size(move_angle)

            ca = math.cos(move_angle)
            sa = math.sin(move_angle)
            e_along = ca * target_body.x + sa * target_body.y
            e_lateral = -sa * target_body.x + ca * target_body.y

            if e_along < base_step:
                self.get_logger().info(
                    f'步进结束: 前向误差{e_along:.3f}m < 基步长{base_step:.3f}m, '
                    f'剩余距离={dist_3d:.2f}m')
                break

            # 动态步长：横向误差越大步长越小
            step_along = base_step * math.exp(-LATERAL_LAMBDA * abs(e_lateral))

            dz_total = target_z - p.z
            drz_total = target_yaw_degree - p.rz
            steps_remaining = max(1, math.ceil(dist_xy / base_step))

            step_z = dz_total / steps_remaining
            step_rz = drz_total / steps_remaining

            vector_body = Coordinate(x=ca * step_along, y=sa * step_along, z=step_z, rz=step_rz)

            step_no += 1
            self.get_logger().info(
                f'步进第{step_no}步: 步长={step_along:.3f}m, '
                f'前向误差={e_along:.2f}m, 横向误差={e_lateral:.2f}m, '
                f'剩余距离={dist_3d:.2f}m, '
                f'步进向量=({vector_body.x:.3f}, {vector_body.y:.3f}, {vector_body.z:.3f}, {vector_body.rz:.2f}°)')

            # 步进目标 = 当前 target + 步进增量
            self.set_step(dx=vector_body.x, dy=vector_body.y, dz=vector_body.z, dyaw_deg=vector_body.rz)

            # 每一步只能使用当前动作剩余的时间，不能把已经超出的时间再借回来。
            # 保留一个很小的下限，避免剩余时间过小时进入无意义的等待。
            step_timeout = max(0.1, remaining)
            if not self._wait_step_convergence(move_angle, timeout=step_timeout):
                self.get_logger().warning(f'步进第{step_no}步收敛超时')

        # 最终目标
        remaining = timeout - (time.monotonic() - start)
        if remaining <= 0:
            self.get_logger().warning('步进最终段超时')
            return False
        self.get_logger().info(f'步进完成, 共{step_no}步, 发送最终目标等待到达')
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
        return self._cmd_and_wait(p.x, y, p.z, p.rz, timeout=timeout)

    # --- WMOVE 系列 (世界系步进) --------------------------------------------

    def wmovexyzrz(self, x: float, y: float, z: float, rz: float,
                   timeout: float = 60.0) -> bool:
        return self._step_move_world(x, y, z, rz, timeout)

    def wmovexyz(self, x: float, y: float, z: float,
                 timeout: float = 60.0) -> bool:
        _, t, _ = self.get_state()
        return self._step_move_world(x, y, z, t.rz, timeout)

    def wmovexy(self, x: float, y: float, timeout: float = 60.0) -> bool:
        _, t, _ = self.get_state()
        return self._step_move_world(x, y, t.z, t.rz, timeout)

    def wmovexyrz(self, x: float, y: float, rz: float,
                  timeout: float = 60.0) -> bool:
        _, t, _ = self.get_state()
        return self._step_move_world(x, y, t.z, rz, timeout)

    def wmovez(self, z: float, timeout: float = 60.0) -> bool:
        _, t, _ = self.get_state()
        return self._step_move_world(t.x, t.y, z, t.rz, timeout)

    def wmoverz(self, rz: float, timeout: float = 60.0) -> bool:
        _, t, _ = self.get_state()
        return self._step_move_world(t.x, t.y, t.z, rz, timeout)

    def wmovex(self, x: float, timeout: float = 60.0) -> bool:
        _, t, _ = self.get_state()
        return self._step_move_world(x, t.y, t.z, t.rz, timeout)

    def wmovey(self, y: float, timeout: float = 60.0) -> bool:
        _, t, _ = self.get_state()
        return self._step_move_world(t.x, y, t.z, t.rz, timeout)

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

    def _travel_world(self, x_w: float, y_w: float, z: float = 0.0, rz: float = 0.0,
                      timeout: float = 60.0) -> bool:
        """直线移动基础函数：先转向目标方向，再沿 body-X 步进前进。"""
        # t 是上一次发布的目标，不能拿它当当前位置。
        # 任务刚启动时，目标通常仍是 (0, 0, 0, 0)，而实测深度可能已经
        # 与 0 有偏差；使用 t 会让 BTRAVEL 的“转向阶段”错误地等待深度归零。
        p, _, _ = self.get_state()
        target_x = x_w
        target_y = y_w
        target_z = z
        dx_w = target_x - p.x
        dy_w = target_y - p.y

        dist_xy = math.sqrt(dx_w**2 + dy_w**2)
        if dist_xy > 0.01:
            target_yaw = math.degrees(math.atan2(dy_w, dx_w))
            deadline = time.monotonic() + max(0.0, timeout)
            self.get_logger().info(
                f'直线移动 第一阶段(转向): 目标朝向{target_yaw:.1f}°, '
                f'当前朝向{p.rz:.1f}°')
            rotate_timeout = deadline - time.monotonic()
            if rotate_timeout <= 0 or not self._step_move_world(
                    p.x, p.y, p.z, target_yaw, timeout=rotate_timeout):
                self.get_logger().warning('直线移动第一阶段失败，不进入前进阶段')
                return False
            self.get_logger().info(
                f'直线移动 第二阶段(前进): 目标=({target_x:.2f}, {target_y:.2f}), '
                f'距离={dist_xy:.2f}m')
            move_timeout = deadline - time.monotonic()
            if move_timeout <= 0:
                self.get_logger().warning('直线移动第二阶段没有剩余超时时间')
                return False
            return self._step_move_world(
                target_x, target_y, target_z, target_yaw,
                timeout=move_timeout)
        else:
            self.get_logger().info(
                f'直线移动: 距离过短({dist_xy:.3f}m), 直发深度/偏航')
        self.get_logger().info(
            f'开始最终定位: 目标=({target_x:.2f}, {target_y:.2f}, {target_z:.2f}, {rz:.1f}°)')
        return self._step_move_world(
            target_x, target_y, target_z, rz,
            timeout=timeout)


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
        # Cancelling a position goal must also leave the vehicle out of any
        # previously leased velocity mode.
        try:
            self._publish_body_velocity()
        except Exception:
            if not self._shutdown_requested:
                raise
        return CancelResponse.ACCEPT

    def _action_execute_cb(self, goal_handle):
        req = goal_handle.request
        self._action_goal_handle = goal_handle

        # START: 初始化 odom 原点，不需要任何前置校验
        if req.cmd_type == BasicMotion.Goal.START:
            self.get_logger().info('Action START: initializing odom origin')
            task_context = str(getattr(req, 'task_context', '')).strip()
            if task_context:
                self.get_logger().info(
                    f'Action START: task_context="{task_context}"')
            self.start()
            result = BasicMotion.Result()
            result.success = True
            result.message = "odom origin set"
            goal_handle.succeed()
            self._action_goal_handle = None
            self.get_logger().info('Action START: done')
            return result

        # BODY_VELOCITY is intentionally a short, immediately-completed
        # action.  The task layer renews the lease at its control period; if
        # it stops publishing, the watchdog above sends a neutral command.
        # This keeps the ZIT6 wire format exclusively inside BasicMotion while
        # retaining a responsive visual-servo loop.
        if req.cmd_type == BasicMotion.Goal.BODY_VELOCITY:
            if len(req.target) < 4:
                result = BasicMotion.Result()
                result.success = False
                result.message = (
                    'BODY_VELOCITY target needs 4 values '
                    '[vx, vy, vz, yaw_rate_deg_s]')
                goal_handle.abort()
                self._action_goal_handle = None
                return result
            vx, vy, vz, yaw_rate_deg_s = req.target[:4]
            self._publish_body_velocity(
                vx, vy, vz, yaw_rate_deg_s,
                lease_s=(req.velocity_lease
                         if req.velocity_lease > 0.0
                         else DEFAULT_VELOCITY_LEASE),
            )
            result = BasicMotion.Result()
            result.success = True
            result.message = ''
            goal_handle.succeed()
            self._action_goal_handle = None
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
        task_context = str(getattr(req, 'task_context', '')).strip()
        p, t, _ = self.get_state()

        # ── 派发运动类型 ──────────────────────────────────────
        type_names = {
            1: 'WMOVE', 2: 'BMOVE', 3: 'SET', 4: 'WTRAVEL',
            5: 'BTRAVEL', 7: 'BODY_VELOCITY',
        }
        type_name = type_names.get(req.cmd_type, f'UNKNOWN({req.cmd_type})')
        context_text = (
            f' task_context="{task_context}"' if task_context else '')
        self.get_logger().info(
            "============================="
            f'动作 {type_name}: axes="{req.axes}"{context_text}开始'
            "=============================")
        self.get_logger().info(
            f'Action {type_name}: axes="{req.axes}", '
            f'target=[{x:.2f}, {y:.2f}, {z:.2f}, {yaw:.1f}], '
            f'pose=[{p.x:.2f}, {p.y:.2f}, {p.z:.2f}, {p.rz:.1f}], '
            f'timeout={timeout:.1f}s')

        if req.cmd_type == BasicMotion.Goal.SET:
            axes = req.axes or 'xyzrz'
            # 未选中的轴保持“当前实测值”，而不是上一次动作的旧目标。
            # 例如 xyrz 回原点时必须保持当前深度。
            tx, ty, tz, tyaw = p.x, p.y, p.z, p.rz
            if 'x' in axes: tx = x
            if 'y' in axes: ty = y
            if 'z' in axes.replace('rz', ''): tz = z
            if 'rz' in axes: tyaw = yaw
            self._action_target = {'x': tx, 'y': ty, 'z': tz, 'yaw': tyaw}
            success = self.setxyzrz(tx, ty, tz, tyaw, timeout=timeout)

        elif req.cmd_type == BasicMotion.Goal.WMOVE:
            axes = req.axes or 'xyzrz'
            tx, ty, tz, trz = p.x, p.y, p.z, p.rz
            if 'x' in axes: tx = x
            if 'y' in axes: ty = y
            if 'z' in axes.replace('rz', ''): tz = z
            if 'rz' in axes: trz = yaw
            self._action_target = {'x': tx, 'y': ty, 'z': tz, 'yaw': trz}
            success = self.wmovexyzrz(tx, ty, tz, trz, timeout=timeout)
        elif req.cmd_type == BasicMotion.Goal.BMOVE:
            off = p.body_to_world(x, y)
            self._action_target = {'x': p.x + off.x, 'y': p.y + off.y,
                                   'z': p.z + z, 'yaw': p.rz + yaw}
            self.get_logger().info(
                f'BMOVE坐标变换: body偏移=({x:.2f}, {y:.2f}) '
                f'→ world偏移=({off.x:.2f}, {off.y:.2f}) '
                f'当前yaw={p.rz:.1f}°')
            success = self.bmovexyzrz(x, y, z, yaw, timeout=timeout)
        elif req.cmd_type == BasicMotion.Goal.WTRAVEL:
            axes = req.axes or 'xyzrz'
            tx, ty, tz, trz = p.x, p.y, p.z, p.rz
            if 'x' in axes: tx = x
            if 'y' in axes: ty = y
            if 'z' in axes.replace('rz', ''): tz = z
            if 'rz' in axes: trz = yaw
            self._action_target = {'x': tx, 'y': ty,
                                   'z': tz, 'yaw': trz}
            self.get_logger().info(
                f'WTRAVEL: 目标=({tx:.2f}, {ty:.2f}, {tz:.2f}), '
                f'方向角={math.degrees(math.atan2(ty - t.y, tx - t.x)):.1f}°')
            success = self._travel_world(tx, ty, tz, trz, timeout=timeout)
        elif req.cmd_type == BasicMotion.Goal.BTRAVEL:
            off = p.body_to_world(x, y)
            tx_w = p.x + off.x
            ty_w = p.y + off.y
            tz_w = p.z + z
            self._action_target = {'x': tx_w, 'y': ty_w,
                                   'z': tz_w, 'yaw': p.rz}
            self.get_logger().info(
                f'BTRAVEL: body偏移=({x:.2f}, {y:.2f}) '
                f'→ world偏移=({off.x:.2f}, {off.y:.2f})')
            success = self._travel_world(tx_w, ty_w, tz_w, p.rz, timeout=timeout)
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
        if (self._shutdown_requested or self._action_goal_handle is None
                or self._action_target is None):
            return
        t = self._action_target
        with self._state_lock:
            dx = t['x'] - self.pose.x
            dy = t['y'] - self.pose.y
            dz = t['z'] - self.pose.z
        feedback = BasicMotion.Feedback()
        feedback.distance_remaining = float(math.sqrt(dx**2 + dy**2 + dz**2))
        try:
            self._action_goal_handle.publish_feedback(feedback)
        except Exception:
            if not self._shutdown_requested:
                raise


    def _publish_pose_info(self):
        """Publish origin, robot pose, and target pose at 30Hz."""
        if self._shutdown_requested:
            return
        msg = PoseInfo()
        msg.stamp = self._pose_stamp
        with self._state_lock:
            if self._origin is not None:
                msg.origin_x = float(self._origin.x)
                msg.origin_y = float(self._origin.y)
                msg.origin_z = float(self._origin.z)
                msg.origin_yaw = float(self._origin.rz)
            msg.robot_x = float(self.pose.x)
            msg.robot_y = float(self.pose.y)
            msg.robot_z = float(self.pose.z)
            msg.robot_roll = float(self.pose.rx)
            msg.robot_pitch = float(self.pose.ry)
            msg.robot_yaw = float(self.pose.rz)
            msg.target_x = float(self._target.x)
            msg.target_y = float(self._target.y)
            msg.target_z = float(self._target.z)
            msg.target_yaw = float(self._target.rz)
        self._publish_while_running(self.pub_pose_legacy, msg)


def main(args=None):
    rclpy.init(args=args)
    node = BasicMotionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        # launch may already have shut down the default context while
        # delivering SIGINT.
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
