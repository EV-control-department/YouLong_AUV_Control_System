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
  │  YAML mission 加载 → 顺序执行 → 调用导航/运动           │
  ├─────────────────────────────────────────────────────┤
  │  uv_nav (导航层)                                     │
  │  A* 路径规划 → 避障 → 调用 basic_motion 执行           │
  ├─────────────────────────────────────────────────────┤
  │  uv_control (控制层)  ← 本文件所在层                   │
  │  basic_motion (本节点)                               │
  │    → set_world / set_body / set_step                │
  │    → _send_setpoint → /zit6/cmd/setpoint            │
  ├─────────────────────────────────────────────────────┤
  │  uv_hm (硬件管理层)                                   │
  │  sim_bridge (仿真) / hw_manager (实车)                │
  │  级联PID + 推力混合 → 6推进器                          │
  ├─────────────────────────────────────────────────────┤
  │  Stonefish / MCU                                    │
  │  物理仿真引擎 / STM32 微控制器                          │
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

from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, UInt32

from zit6_interfaces.msg import ZitSetpoint, ZitStatus, ZitOdom
from zit6_interfaces.srv import SetOrigin
from uv_msgs.action import BasicMotion
from uv_msgs.msg import PoseInfo
from uv_msgs.srv import CorrectOdomXY
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
STEP_PERIOD = 0.2        # 目标步进间隔/收敛时间阈值 (秒)
LATERAL_LAMBDA = 2.0     # 横向误差指数衰减系数


class BasicMotionNode(Node):
    """运动控制节点：ZIT6 协议接口 + 高级运动 API，single source of truth。"""

    def __init__(self):
        super().__init__('basic_motion')

        # ── 状态 ───────────────────────────────────────────────
        self.status = ZitStatus()
        self._last_status_received = 0.0
        self.pose = Coordinate()        # 当前位置 (odom 系)
        self._target = Coordinate()     # 当前目标 (odom 系)
        self._map_pose = Coordinate()   # 原始 map 系位置（ZIT6 上报，未经 odom 转换）
        self._origin = None             # odom 原点 (map 系 Coordinate)，start() 时设置
        self._origin_warned = False     # 防止 _pos_cb 重复打印警告
        self._state_lock = threading.Lock()
        self.vel_body = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rz': 0.0}   # 机体速度                     # 世界速度
        self._pose_stamp = self.get_clock().now().to_msg()            # 位姿测量时间（_pos_cb 接收时刻）
        self.declare_parameter('reset_mcu_origin_on_start', True)
        self.declare_parameter('origin_service_timeout', 5.0)
        self._use_mcu_odom = bool(self.get_parameter('reset_mcu_origin_on_start').value)
        self.declare_parameter('heartbeat_rate', 15.0)
        self.declare_parameter('arm_mode', 1)
        hb_rate = float(self.get_parameter('heartbeat_rate').value)
        if not math.isfinite(hb_rate) or not 10 <= hb_rate <= 100:
            raise ValueError('heartbeat_rate 必须为10~100 Hz')
        if self.get_parameter('arm_mode').value not in (1, 3):
            raise ValueError('arm_mode 必须为1或3')
        self._heartbeat_enabled = False  # 启动节点不解锁，只有START握手完成后才开启。
        self._heartbeat_sent = 0
        self._odom_received_count = 0
        self._diagnostic_last_log = float('-inf')
        self._odom_invalid_last_log = float('-inf')
        self._heartbeat_group = MutuallyExclusiveCallbackGroup()
        self._heartbeat_pub = None
        if self._use_mcu_odom:
            self._heartbeat_pub = self.create_publisher(UInt32, '/zit6/cmd/agxhbt', 10)
            self.create_timer(1.0/hb_rate, self._heartbeat_cb,
                              callback_group=self._heartbeat_group)
        self._mcu_odom = None
        self._mcu_odom_received = 0.0
        # START执行时会等待服务/新代数，必须让响应和遥测在另一回调组运行。
        self._hardware_group = MutuallyExclusiveCallbackGroup()
        self._origin_client = self.create_client(
            SetOrigin, '/zit6/cmd/setorigin', callback_group=self._hardware_group)
        if self._use_mcu_odom:
            self.create_subscription(ZitOdom, '/zit6/state/odom', self._odom_cb, 10,
                                     callback_group=self._hardware_group)

        # ─────────────────────────────────────────────────────────
        # Layer 1: ZIT6 协议发布/订阅
        # ─────────────────────────────────────────────────────────
        self.pub_setpoint = self.create_publisher(
            ZitSetpoint, '/zit6/cmd/setpoint', 10)
        self.create_subscription(
            ZitStatus, '/zit6/state/status', self._status_cb, 10,
            callback_group=self._hardware_group)
        self.create_subscription(
            Float32MultiArray, '/zit6/state/pos', self._pos_cb, 10)
        self.create_subscription(
            Float32MultiArray, '/zit6/state/vel', self._vel_cb, 10)

        # ── Action Server ────────────────────────────────────────
        self.declare_parameter('basic_motion_action', '/basic_motion')
        action_name = self.get_parameter('basic_motion_action').value
        self._action_server = ActionServer(
            self, BasicMotion, action_name,
            goal_callback=self._action_goal_cb,
            cancel_callback=self._action_cancel_cb,
            execute_callback=self._action_execute_cb,
        )
        self._action_goal_handle = None
        self._action_target = None       # 绝对目标 {x, y, z, yaw}，用于反馈
        self._action_axes = None         # 当前 action 的生效轴，用于反馈
        self.create_service(CorrectOdomXY, '/basic_motion/correct_odom_xy',
                            self._correct_odom_xy_cb)
        self.create_timer(0.5, self._action_feedback_cb)

        # ── PoseInfo Publisher ─────────────────────────────────
        self.pub_pose = self.create_publisher(PoseInfo, '/basic_motion/pose_info', 10)
        self.create_timer(1.0 / 30.0, self._publish_pose_info)
        self.create_timer(1.0, self._diagnostic_cb, callback_group=self._heartbeat_group)

        self.get_logger().info('BasicMotion node started')
        self.get_logger().info(
            f'启动配置：MCU原点重置={self._use_mcu_odom}，'
            f'心跳发布器已创建={self._heartbeat_pub is not None}，频率={hb_rate:g}Hz，'
            f'arm_mode={self.get_parameter("arm_mode").value}；等待START成功后才发送心跳')
        self.get_logger().info(
            '接口：调用MCU服务 /zit6/cmd/setorigin；接收 /zit6/state/odom 和 '
            '/zit6/state/status；发布 /basic_motion/pose_info 和 /zit6/cmd/agxhbt。'
            'BasicMotion不提供setorigin服务，也不发布MCU的state/odom话题。')

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
        msg.roll = 0.0         # roll/pitch 未使用，控制栈保持 4-DOF
        msg.pitch = 0.0
        msg.seq = 0
        self._target = self._map_to_odom(Coordinate(x=x, y=y, z=z, rz=math.degrees(yaw_rad)))
        self.pub_setpoint.publish(msg)
        odom_t = self._target
        self.get_logger().info(
            f'发往ZIT6: map=({x:.2f}, {y:.2f}, {z:.2f}, {math.degrees(yaw_rad):.1f}°), '
            f'对应odom=({odom_t.x:.2f}, {odom_t.y:.2f}, {odom_t.z:.2f}, {odom_t.rz:.1f}°)')

    def _status_cb(self, msg: ZitStatus):
        with self._state_lock:
            self.status = msg
            self._last_status_received = time.monotonic()

    def _vel_cb(self, msg: Float32MultiArray):
        """ZIT6 速度回调。机体速度 6-DOF [vx, vy, vz, vroll_rad, vpitch_rad, vyaw_rad_s]。"""
        with self._state_lock:
            if len(msg.data) >= 6:
                self.vel_body = {
                    'x': msg.data[0], 'y': msg.data[1], 'z': msg.data[2],
                    'rx': msg.data[3], 'ry': msg.data[4], 'rz': msg.data[5],
                }
            elif len(msg.data) >= 4:
                self.vel_body = {
                    'x': msg.data[0], 'y': msg.data[1],
                    'z': msg.data[2], 'rz': msg.data[3],
                }

    def _pos_cb(self, msg: Float32MultiArray):
        if self._use_mcu_odom:
            return  # 新固件使用带原点代数的odom，避免重置前排队的旧pos污染位姿。
        self._update_position(msg)

    def _correct_odom_xy_cb(self, request, response):
        """地标校正：同时改变反馈及目标转换，保持已下发MCU目标不跳变。"""
        values = (request.expected_x, request.expected_y,
                  request.corrected_x, request.corrected_y)
        with self._state_lock:
            if (not all(math.isfinite(v) for v in values)
                    or self._origin is None or self._action_goal_handle is not None):
                response.message = '坐标无效、未START或运动动作仍在执行'
                return response
            age = (self.get_clock().now().nanoseconds -
                   (self._pose_stamp.sec * 1000000000 + self._pose_stamp.nanosec)) / 1e9
            if not 0 <= age <= 0.5 or math.hypot(
                    self.pose.x - request.expected_x,
                    self.pose.y - request.expected_y) > 0.05:
                response.message = '地标校正位姿过期或校正期间机器人已移动'
                return response
            old_target = self._odom_to_map(self._target)
            dx = request.corrected_x - request.expected_x
            dy = request.corrected_y - request.expected_y
            yaw = math.radians(self._origin.rz)
            self._origin.x -= math.cos(yaw) * dx - math.sin(yaw) * dy
            self._origin.y -= math.sin(yaw) * dx + math.cos(yaw) * dy
            self.pose = self._map_to_odom(self._map_pose)
            self._target = self._map_to_odom(old_target)
        response.success = True
        response.message = f'任务odom平移修正=({dx:+.3f},{dy:+.3f})m；MCU原始DVL未改写'
        self.get_logger().warning(response.message)
        self._publish_pose_info()
        return response

    def _odom_cb(self, msg: ZitOdom):
        values = list(msg.pose_odom)
        if len(values) != 6 or not all(math.isfinite(v) for v in values):
            now = time.monotonic()
            if now - self._odom_invalid_last_log >= 3.0:
                self._odom_invalid_last_log = now
                self.get_logger().error(f'odom接收后被丢弃：pose_odom长度应为6且全部有限，实际={values}')
            return
        self._update_position(Float32MultiArray(data=values))
        with self._state_lock:
            self._mcu_odom = msg
            self._mcu_odom_received = time.monotonic()
            self._odom_received_count += 1

    def _diagnostic_cb(self):
        """独立于长动作的诊断；每3秒汇报一次，即使尚未发送START也能定位。"""
        now = time.monotonic()
        if now - self._diagnostic_last_log < 3.0:
            return
        self._diagnostic_last_log = now
        with self._state_lock:
            odom = self._mcu_odom
            detail = ('未收到有效格式odom' if odom is None else
                      f'接收年龄={now-self._mcu_odom_received:.3f}s，'
                      f'nav_valid={odom.nav_valid}，origin_initialized={odom.origin_initialized}，'
                      f'generation={odom.origin_generation}')
            self.get_logger().info(
                f'运行诊断：status接收年龄='
                f'{"未收到" if self._last_status_received == 0 else format(now-self._last_status_received, ".3f") + "s"}，'
                f'is_armed={self.status.is_armed}，navigation_ready={self.status.navigation_ready}；'
                f'odom累计={self._odom_received_count}，{detail}；'
                f'上位机原点已设置={self._origin is not None}；'
                f'心跳启用={self._heartbeat_enabled}，已发送={self._heartbeat_sent}包')

    def _update_position(self, msg: Float32MultiArray):
        """ZIT6 位置回调。原始数据是 map 系，内部转 odom 系后存为 self.pose。

        支持 6 元素 [x, y, z, roll_rad, pitch_rad, yaw_rad] 和
        旧 4 元素 [x, y, z, yaw_rad] 两种格式。
        """
        with self._state_lock:
            if len(msg.data) < 4:
                return
            self._pose_stamp = self.get_clock().now().to_msg()  # 记录接收时刻作为测量时间
            if len(msg.data) >= 6:
                map_pos = Coordinate(
                    x=msg.data[0], y=msg.data[1], z=msg.data[2],
                    rx=math.degrees(msg.data[3]),   # roll rad
                    ry=math.degrees(msg.data[4]),   # pitch rad
                    rz=math.degrees(msg.data[5]),   # yaw rad
                )
            else:
                map_pos = Coordinate(
                    x=msg.data[0], y=msg.data[1], z=msg.data[2],
                    rz=math.degrees(msg.data[3]),   # 弧度→度
                )
            self._map_pose = map_pos   # 始终保存原始 map 坐标，供 start() 使用
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
                self.get_logger().info(
                    'odom origin already set, updating to current map pose')
            self._origin = Coordinate(
                x=self._map_pose.x, y=self._map_pose.y,
                z=0.0, rz=self._map_pose.rz)
            self.get_logger().info(
                f'DEBUG map pose: x={self._map_pose.x:.4f}, '
                f'y={self._map_pose.y:.4f}, z={self._map_pose.z:.4f}, '
                f'rz={self._map_pose.rz:.4f}')
            self.pose.x = 0.0
            self.pose.y = 0.0
            self.pose.z = 0.0
            self.pose.rz = 0.0
            # START 重新定义坐标系时，旧动作目标也必须一起清零。
            self._target = Coordinate()
        self.get_logger().info(
            f'odom origin set: map({self._origin.x:.2f}, '
            f'{self._origin.y:.2f}, {self._origin.z:.2f}), '
            f'yaw={self._origin.rz:.1f}°')

    def _reset_mcu_origin(self, goal_handle):
        """单次显式调用；拒绝/超时不重试，不绕过导航或未解锁保护。"""
        timeout = float(self.get_parameter('origin_service_timeout').value)
        if not math.isfinite(timeout) or timeout <= 0:
            return False, 'origin_service_timeout 必须为有限正数'
        deadline = time.monotonic() + timeout
        self.get_logger().info(f'START[2/6]：查找MCU setorigin服务，握手总预算={timeout:g}s')
        if not self._origin_client.wait_for_service(timeout_sec=timeout):
            return False, 'MCU setorigin服务不可用；检查agent/固件/SetOrigin接口'
        if goal_handle.is_cancel_requested:
            return False, 'START已取消，未发送MCU重置'
        with self._state_lock:
            self._origin = None  # 结果不确定时不能继续使用旧任务原点。
        self.get_logger().info('START：请求MCU设置零点（必须未解锁且导航新鲜）')
        future = self._origin_client.call_async(SetOrigin.Request())
        self.get_logger().info('START[3/6]：setorigin请求已发送，等待服务响应')
        last_log = time.monotonic()
        while not future.done() and time.monotonic() < deadline and rclpy.ok():
            if time.monotonic() - last_log >= 1.0:
                last_log = time.monotonic()
                self.get_logger().info(f'START等待服务响应：剩余预算={max(0., deadline-last_log):.2f}s')
            time.sleep(0.01)
        if not future.done():
            future.cancel()
            return False, 'MCU setorigin响应超时；请求可能已执行，需人工核对原点代数，禁止盲目重试'
        try:
            response = future.result()
        except Exception as error:
            return False, f'MCU setorigin调用异常：{error}'
        if response is None or not response.success:
            reason = response.message if response is not None else '没有返回响应'
            self.get_logger().info(f'START服务响应：success=False，原因={reason}')
            return False, f'MCU零点设置失败：{reason}'
        generation = response.origin_generation
        self.get_logger().info(f'MCU零点已设置，等待新坐标反馈：generation={generation}')
        last_log = float('-inf')
        # 等到同一代的新鲜、有效遥测，不能把重置前的缓存当成新坐标。
        while time.monotonic() < deadline and rclpy.ok():
            if goal_handle.is_cancel_requested:
                return False, 'START已取消；MCU零点已改变，任务原点未初始化'
            with self._state_lock:
                odom = self._mcu_odom
                age = time.monotonic() - self._mcu_odom_received
                if (odom is not None and odom.origin_initialized and odom.nav_valid
                        and odom.origin_generation == generation and age <= .2):
                    self.get_logger().info(f'START[4/6]：有效同代odom已确认，generation={generation}，年龄={age:.3f}s')
                    return True, f'MCU原点已确认，generation={generation}'
            if time.monotonic() - last_log >= 1.0:
                last_log = time.monotonic()
                detail = ('尚未收到odom' if odom is None else
                          f'nav_valid={odom.nav_valid}，origin_initialized={odom.origin_initialized}，'
                          f'实际generation={odom.origin_generation}，年龄={age:.3f}s')
                self.get_logger().info(f'START等待odom：期望generation={generation}，{detail}')
            time.sleep(0.01)
        return False, f'MCU已重置，但未收到有效同代odom（generation={generation}），START未完成'

    def _heartbeat_cb(self):
        # 由独立回调组调度，长动作等待不能阻塞心跳。发布与停用共用锁，
        # 防止START暂停心跳后仍有旧回调补发一个包。
        with self._state_lock:
            if self._heartbeat_enabled and self._heartbeat_pub is not None:
                self._heartbeat_pub.publish(UInt32(data=int(self.get_parameter('arm_mode').value)))
                self._heartbeat_sent = getattr(self, '_heartbeat_sent', 0) + 1
                if self._heartbeat_sent == 1:
                    self.get_logger().info('心跳：首包agxhbt已提交发布，后续由独立定时器持续发送；不代表MCU已解锁')

    def _prepare_start(self, goal_handle):
        """暂停自己的心跳，等待固件租约上锁；不发送强制解锁或跳过导航指令。"""
        with self._state_lock:
            self._heartbeat_enabled = False
            self._origin = None
        self.get_logger().info('START[1/6]：暂停BasicMotion心跳，等待MCU上锁，再设置原点')
        deadline = time.monotonic() + 3.0
        last_log = float('-inf')
        while rclpy.ok() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return False, 'START已取消，心跳保持暂停'
            with self._state_lock:
                fresh = time.monotonic()-self._last_status_received <= .5
                disarmed = not self.status.is_armed
            if fresh and disarmed:
                self.get_logger().info('START[1/6]：收到新鲜状态，MCU已上锁')
                return True, 'MCU已上锁'
            if time.monotonic() - last_log >= 1.0:
                last_log = time.monotonic()
                self.get_logger().info(
                    f'START等待上锁：状态新鲜={fresh}，is_armed={not disarmed}，'
                    f'status接收年龄={last_log-self._last_status_received:.3f}s')
            time.sleep(.02)
        return False, 'MCU未确认上锁；检查其他心跳源、MCU状态和1秒心跳超时保护'

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
        _, t, _ = self.get_state()
        target_step = Coordinate(x=dx, y=dy, z=dz, rz=dyaw_deg)
        map_target = t.to_world_frame(target_step)

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

            body_target = self._odom_to_body(self._target)

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
                    f'等待到达: err=({err_x:.3f}, {err_y:.3f}, {err_z:.3f}, {err_yaw:.2f}°), '
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
            target_body = self._odom_to_body(target_world)

            p, target_odom, _ = self.get_state()
            target_target = target_odom.to_local_frame(target_world)

            # 机体坐标系误差 → 运动方向坐标系
            ex_body = target_body.x
            ey_body = target_body.y
            dist_xy = math.sqrt(target_target.x**2 + target_target.y**2)
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
        _, t, _ = self.get_state()
        off = t.body_to_world(dx, dy)
        return self._step_move_world(
            t.x + off.x, t.y + off.y, t.z + dz,
            t.rz + drz, timeout)

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
        return CancelResponse.ACCEPT

    def _action_execute_cb(self, goal_handle):
        req = goal_handle.request
        self._action_goal_handle = goal_handle

        # 真机START先重置MCU原点并确认遥测；仿真显式关闭该步骤。
        if req.cmd_type == BasicMotion.Goal.START:
            self.get_logger().info('Action START: initializing odom origin')
            task_context = str(getattr(req, 'task_context', '')).strip()
            if task_context:
                self.get_logger().info(
                    f'Action START: task_context="{task_context}"')
            result = BasicMotion.Result()
            if self._use_mcu_odom:
                success, message = self._prepare_start(goal_handle)
                if success:
                    success, message = self._reset_mcu_origin(goal_handle)
                if not success:
                    result.success = False
                    result.message = message
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                    else:
                        goal_handle.abort()
                    self._action_goal_handle = None
                    self.get_logger().error(f'Action START失败：{message}')
                    return result
            self.start()
            self.get_logger().info('START[5/6]：上位机原点已初始化，准备发布PoseInfo')
            # 必须先发布新任务原点，再发送第一个解锁心跳。
            self._publish_pose_info()
            if self._use_mcu_odom:
                with self._state_lock:
                    self._heartbeat_enabled = True
                self._heartbeat_cb()
                self.get_logger().info('START[6/6]：PoseInfo已发布，agxhbt心跳已启用；START返回成功，不等待MCU解锁')
            result.success = True
            result.message = "odom origin set"
            goal_handle.succeed()
            self._action_goal_handle = None
            self.get_logger().info('Action START: done')
            return result

        if self._use_mcu_odom:
            # 重置原点后固件重新积累心跳；不能在尚未解锁时发送第一段位移。
            arm_deadline = time.monotonic() + 5.0
            armed = False
            last_log = float('-inf')
            while rclpy.ok() and time.monotonic() < arm_deadline:
                with self._state_lock:
                    armed = (self.status.is_armed and self.status.navigation_ready
                             and time.monotonic()-self._last_status_received <= .5)
                if armed or goal_handle.is_cancel_requested:
                    break
                if time.monotonic() - last_log >= 1.0:
                    last_log = time.monotonic()
                    self.get_logger().info(
                        f'移动前等待解锁：is_armed={self.status.is_armed}，'
                        f'navigation_ready={self.status.navigation_ready}，'
                        f'status接收年龄={last_log-self._last_status_received:.3f}s')
                time.sleep(.02)
            if not armed or goal_handle.is_cancel_requested:
                result = BasicMotion.Result()
                result.success = False
                result.message = 'MCU未解锁/导航未就绪或动作取消；检查BasicMotion心跳、原点和导航'
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
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
        type_names = {1: 'WMOVE', 2: 'BMOVE', 3: 'SET', 4: 'WTRAVEL', 5: 'BTRAVEL'}
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


    def _publish_pose_info(self):
        """Publish origin, robot pose, and target pose at 30Hz."""
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
        self.pub_pose.publish(msg)


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
        with node._state_lock:
            node._heartbeat_enabled = False
        executor.shutdown()
        node.destroy_node()
        # launch may already have shut down the default context while
        # delivering SIGINT.
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
