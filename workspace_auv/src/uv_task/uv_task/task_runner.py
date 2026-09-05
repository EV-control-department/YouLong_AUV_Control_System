"""Task runner node: loads task list from JSON, executes tasks sequentially.

Each task calls basic_motion via the BasicMotion action server.
The task runner is the single source of truth for commanded position,
tracked locally (not from external topics).
"""

import glob
import json
import math
import os
import threading
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float32, UInt8
from std_srvs.srv import Trigger

from zit6_interfaces.msg import ZitSetpoint, ZitStatus

from uv_msgs.action import BasicMotion
from uv_msgs.msg import (
    DetectionArray,
    ObjectPositionArray,
    PoseInfo,
    TargetPosition,
    TargetPositionArray,
    TaskStatus,
)
from uv_msgs.srv import ExecTask, RunTask

from uv_task.arrow_surfacer import (
    _DOWN_CX, _DOWN_CY, _DOWN_FX, _DOWN_FY,
    _DOWN_OFFSET_LEFT, _DOWN_OFFSET_RIGHT, _DOWN_OPTICAL_TO_BODY,
    _euler_to_rotation_matrix, _ray_intersection_midpoint,
)
from uv_task.arrow_surfacer import ArrowSurfacer
from uv_task.line_follower import LineFollower


_IMPACT_BALL_CLASS_IDS = {
    'impact_ball_blue': 5,
    'impact_ball_red': 6,
}
_IMPACT_BALL_ALIASES = {
    'blue': 'impact_ball_blue',
    'blue_ball': 'impact_ball_blue',
    'impact_blue': 'impact_ball_blue',
    'impact_ball_blue': 'impact_ball_blue',
    'red': 'impact_ball_red',
    'red_ball': 'impact_ball_red',
    'impact_red': 'impact_ball_red',
    'impact_ball_red': 'impact_ball_red',
}


class TaskRunnerNode(Node):
    """Task runner: JSON task loader and sequential executor."""

    # ── 灯光常量 (/zit6/cmd/light) ─────────────────────────────────
    LIGHT_OFF = 0
    LIGHT_YELLOW = 1
    LIGHT_GREEN = 2
    LIGHT_RED = 3

    # ── 舵机角度 (/zit6/cmd/servo, rad) ────────────────────────────
    ANGLE_DROP_BEACON = 90       #   投信标
    ANGLE_SAMPLE_WATER = 0.0               # 采水样
    ANGLE_RELEASE_SAMPLER = 0   #   释放取水器
    ANGLE_INIT = 0.0

    def __init__(self):
        super().__init__('task_runner')

        # Commanded position tracker (updated after every motion command).
        # Starts at (0,0,0,0) after START, which matches basic_motion's odom origin.
        # Used by single-axis SET tasks to fill non-targeted axes.
        self._cmd_x = 0.0
        self._cmd_y = 0.0
        self._cmd_z = 0.0
        self._cmd_yaw = 0.0
        self.objects = ObjectPositionArray()
        self.target_positions = TargetPositionArray()

        # ── 下视感知（release_sampler 对齐用）──
        self._perception_lock = threading.RLock()
        self._down_detections = {}   # camera_name → (monotonic, DetectionArray)
        self._robot_pose = None      # (x, y, z, roll_deg, pitch_deg, yaw_deg)

        self.tasks = []
        self.current_index = 0
        self._current_task_name = ''
        self._current_task_step = 0
        self.running = False
        self.stopped = False
        self._active_goal_handle = None

        # Debug mode
        self.declare_parameter('debug_mode', False)
        self._debug_mode = self.get_parameter('debug_mode').get_parameter_value().bool_value
        self._debug_task_name = None
        self._debug_executing = False
        self._debug_timeout = -1.0

        # Competition metadata only.  The current cruise environment keeps all
        # targets visible; this value tells task logic which one is correct and
        # intentionally does not create scoring or grasping behaviour.
        self.declare_parameter('target_id', 'yellow_golf')
        self.target_id = self.get_parameter('target_id').get_parameter_value().string_value
        valid_target_ids = {'yellow_golf', 'pink_golf', 'red_ring'}
        if self.target_id not in valid_target_ids:
            self.get_logger().warning(
                f"Unknown target_id {self.target_id!r}; using 'yellow_golf'. "
                f"Valid values: {', '.join(sorted(valid_target_ids))}"
            )
            self.target_id = 'yellow_golf'
        self.get_logger().info(f'Competition target metadata: {self.target_id}')

        # Camera parameters (used by LineFollower sub-task via get_parameter)
        self.declare_parameter('down_image_width', 1280.0)
        self.declare_parameter('down_image_height', 960.0)

        # Task map (shared by _execute_task and _exec_task_cb)
        self.task_map = {
            'start': self._task_start,
            'setx': self._task_setx,
            'sety': self._task_sety,
            'setz': self._task_setz,
            'setrz': self._task_setrz,
            'setxy': self._task_setxy,
            'setxyz': self._task_setxyz,
            'setxyzrz': self._task_setxyzrz,
            'setxyrz': self._task_setxyrz,
            'bmovex': self._task_bmovex,
            'bmovey': self._task_bmovey,
            'bmovez': self._task_bmovez,
            'bmoverz': self._task_bmoverz,
            'bmovexy': self._task_bmovexy,
            'bmovexyz': self._task_bmovexyz,
            'wmovex': self._task_wmovex,
            'wmovey': self._task_wmovey,
            'wmovez': self._task_wmovez,
            'wmoverz': self._task_wmoverz,
            'wmovexy': self._task_wmovexy,
            'wmovexyz': self._task_wmovexyz,
            'wtravelx': self._task_wtravelx,
            'wtravely': self._task_wtravely,
            'wtravelz': self._task_wtravelz,
            'wtravelxy': self._task_wtravelxy,
            'wtravelxyz': self._task_wtravelxyz,
            'btravelx': self._task_btravelx,
            'btravely': self._task_btravely,
            'btravelz': self._task_btravelz,
            'btravelxy': self._task_btravelxy,
            'btravelxyz': self._task_btravelxyz,
            'navigate': self._task_navigate,
            'wait': self._task_wait,
            'follow_line': self._task_follow_line,
            'arrow_surface': self._task_arrow_surface,
            'hit_ball': self._task_hit_balls,
            'hit_balls': self._task_hit_balls,
            'drop_beacon': self._task_drop_beacon,
            'take_water_sample': self._task_take_water_sample,
            'release_sampler': self._task_release_sampler,
            'return_origin': self._task_return_origin,
        }

        # Action client
        self._action_client = ActionClient(self, BasicMotion, 'basic_motion')
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('BasicMotion action server not available!')

        # Subscribers
        self.create_subscription(
            ObjectPositionArray, '/perception/objects', self._objects_cb, 10)
        self.create_subscription(
            TargetPositionArray, '/perception/target_positions',
            self._target_positions_cb, 10)
        for cam in ('down_left', 'down_right'):
            self.create_subscription(
                DetectionArray, f'/perception/detection/{cam}',
                lambda msg, c=cam: self._det_cb(c, msg), 10)
        self.create_subscription(
            PoseInfo, '/basic_motion/pose_info', self._pose_cb, 10)

        # ZIT6 MCU 状态 (status check 用)
        self._mcu_status = ZitStatus()
        self._mcu_status_rcvd = False
        self.create_subscription(
            ZitStatus, '/zit6/state/status', self._mcu_status_cb, 10)

        # Publishers
        self.pub_status = self.create_publisher(TaskStatus, '/task/status', 10)
        self.pub_light = self.create_publisher(UInt8, '/zit6/cmd/light', 10)
        self.pub_servo = self.create_publisher(Float32, '/zit6/cmd/servo', 10)
        # The impact charge deliberately uses the existing ZIT6 velocity
        # setpoint wire format: mode=VEL (0x01) + body frame (0x10).
        self.pub_setpoint = self.create_publisher(
            ZitSetpoint, '/zit6/cmd/setpoint', 10)

        # Services
        self.create_service(RunTask, '/task/run', self._run_task_cb)
        self.create_service(Trigger, '/task/stop', self._stop_task_cb)
        self.create_service(ExecTask, '/task/exec', self._exec_task_cb)

        # Status timer
        self.create_timer(0.5, self._publish_status)

        self.get_logger().info('TaskRunner node started')
        self.get_logger().info(f'Debug mode: {self._debug_mode}')

    def _objects_cb(self, msg: ObjectPositionArray):
        with self._perception_lock:
            self.objects = msg

    def _target_positions_cb(self, msg: TargetPositionArray):
        with self._perception_lock:
            self.target_positions = msg

    def _det_cb(self, camera_name: str, msg: DetectionArray):
        with self._perception_lock:
            self._down_detections[camera_name] = (time.monotonic(), msg)

    def _pose_cb(self, msg: PoseInfo):
        with self._perception_lock:
            self._robot_pose = (msg.robot_x, msg.robot_y, msg.robot_z,
                                msg.robot_roll, msg.robot_pitch, msg.robot_yaw)

    def _mcu_status_cb(self, msg: ZitStatus):
        self._mcu_status = msg
        self._mcu_status_rcvd = True

    # ── 灯光 / 舵机控制 ────────────────────────────────────────────

    def set_light(self, color: int, label: str):
        msg = UInt8(data=color)
        self.pub_light.publish(msg)
        self.get_logger().info(f'💡 LIGHT ON: {label} (value={color})')

    def light_off(self):
        msg = UInt8(data=0)
        self.pub_light.publish(msg)
        self.get_logger().info('💡 LIGHT OFF')

    def set_servo(self, angle_rad: float, label: str):
        msg = Float32(data=float(angle_rad))
        self.pub_servo.publish(msg)
        self.get_logger().info(
            f'⚙️  SERVO: {label} (angle={angle_rad:.2f} rad)')

    # ── 下视对齐工具 ───────────────────────────────────────────────

    _SEARCH_DIRS = [
        (1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (-1.0, 1.0),
        (-1.0, 0.0), (-1.0, -1.0), (0.0, -1.0), (1.0, -1.0),
    ]

    def _best_down_detection(self, class_id: int):
        max_age = 0.60; now = time.monotonic(); candidates = []
        with self._perception_lock:
            arrays = list(self._down_detections.items())
        for _n, (t, a) in arrays:
            if now - t > max_age: continue
            for d in a.detections:
                if d.class_id == class_id: candidates.append(d)
        return max(candidates, key=lambda d: d.confidence) if candidates else None

    def _stereo_pair(self, class_id: int):
        max_age = 0.60; now = time.monotonic()
        with self._perception_lock:
            le = self._down_detections.get('down_left')
            re = self._down_detections.get('down_right')
        if le is None or re is None: return None
        lt, lm = le; rt, rm = re
        if now - lt > max_age or now - rt > max_age: return None
        bl = max((d for d in lm.detections if d.class_id == class_id),
                 key=lambda d: d.confidence, default=None)
        br = max((d for d in rm.detections if d.class_id == class_id),
                 key=lambda d: d.confidence, default=None)
        return (bl, br) if bl and br else None

    def _triangulate(self, class_id: int):
        pair = self._stereo_pair(class_id)
        with self._perception_lock: pose = self._robot_pose
        if pair is None or pose is None: return None
        ld, rd = pair
        rx, ry, rz, roll, pitch, yaw = pose
        R = _euler_to_rotation_matrix(roll, pitch, yaw)
        rp = np.array([rx, ry, rz])
        def _ray(px, py, off):
            vc = np.array([(px - _DOWN_CX) / _DOWN_FX, (py - _DOWN_CY) / _DOWN_FY, 1.0])
            vc /= np.linalg.norm(vc)
            vb = _DOWN_OPTICAL_TO_BODY @ vc
            vw = R @ vb; vw /= np.linalg.norm(vw)
            return rp + R @ off, vw
        lo, ld_ray = _ray(ld.pixel_x, ld.pixel_y, _DOWN_OFFSET_LEFT)
        ro, rd_ray = _ray(rd.pixel_x, rd.pixel_y, _DOWN_OFFSET_RIGHT)
        pos = _ray_intersection_midpoint(lo, ld_ray, ro, rd_ray)
        return (float(pos[0]), float(pos[1]), float(pos[2])) if pos is not None else None

    def _search_for_class(self, class_id: int, label: str) -> bool:
        sd = 0; ss = 0.08; sm = 3.0; sp = 0.30; mi = 0.01
        while ss <= sm:
            if self.stopped: return False
            if self._best_down_detection(class_id):
                self.get_logger().info(f'Search [{label}]: found'); return True
            dx, dy = self._SEARCH_DIRS[sd]
            td, tu = ss * dx, ss * dy
            dist = math.sqrt(td**2 + tu**2); n = max(1, int(dist / mi))
            sx, sy = td / n, tu / n
            for _ in range(n):
                if self.stopped: return False
                tx = self._cmd_x + sx; ty = self._cmd_y + sy
                self._send_action_goal(BasicMotion.Goal.SET,
                    [tx, ty, self._cmd_z, self._cmd_yaw],
                    'xy', timeout=0.01, quiet=True)
                self._cmd_x = tx; self._cmd_y = ty
                if self._best_down_detection(class_id):
                    self.get_logger().info(f'Search [{label}]: found!'); return True
            sd = (sd + 1) % 8
            if sd == 0: ss += sp
        return False

    def _align_to_class(self, class_id: int, label: str) -> bool:
        if self._best_down_detection(class_id) is None:
            self.get_logger().info(f'Align [{label}]: searching...')
            if not self._search_for_class(class_id, label):
                return False
        for i in range(200):
            if self.stopped: return False
            tgt = self._triangulate(class_id)
            if tgt is None: time.sleep(0.05); continue
            self._send_action_goal(BasicMotion.Goal.SET,
                [tgt[0], tgt[1], self._cmd_z, self._cmd_yaw],
                'xy', timeout=0.1, quiet=True)
            self._cmd_x = tgt[0]; self._cmd_y = tgt[1]
            self.get_logger().info(f'Align [{label}]: #{i}靠近，x:{tgt[0]},y:{tgt[1]}')
        self.get_logger().info(f'Align [{label}]: complete')
        return True

    # ========================================================================
    # Task loading
    # ========================================================================

    def load_tasks(self, path: str) -> list:
        """Load task list from JSON file."""
        if not os.path.exists(path):
            self.get_logger().error(f'Task file not found: {path}')
            return []

        with open(path, 'r') as f:
            data = json.load(f)

        tasks = data.get('tasks', [])
        self.get_logger().info(f'Loaded {len(tasks)} tasks from {path}')
        return tasks

    # ========================================================================
    # Task execution
    # ========================================================================

    def run_task_list(self):
        """Execute all tasks sequentially."""
        self.running = True
        self.stopped = False
        self.current_index = 0
        total = len(self.tasks)
        self.get_logger().info(f'=== Task list started ({total} tasks) ===')

        while self.current_index < total and not self.stopped:
            task = self.tasks[self.current_index]
            name = task.get('name', 'unknown')
            params = task.get('params', {})
            self._current_task_name = str(name)
            self._current_task_step = self.current_index + 1

            self.get_logger().info(
                f'[{self.current_index + 1}/{total}] {name} {params} '
                f'| cmd_pose=({self._cmd_x:.2f}, {self._cmd_y:.2f}, '
                f'{self._cmd_z:.2f}, {self._cmd_yaw:.1f}°)')

            try:
                success = self._execute_task(name, params)
                if not success:
                    self.get_logger().warn(
                        f'[{self.current_index + 1}/{total}] {name} FAILED')
            except Exception as e:
                self.get_logger().error(
                    f'[{self.current_index + 1}/{total}] {name} exception: {e}')

            self.current_index += 1

        self.running = False
        self._current_task_name = ''
        self._current_task_step = 0
        if self.stopped:
            self.get_logger().warn(f'=== Task list stopped at {self.current_index}/{total} ===')
        else:
            self.get_logger().info(f'=== Task list completed ({total}/{total}) ===')

    def _execute_task(self, name: str, params: dict) -> bool:
        """Execute a single task by name."""
        handler = self.task_map.get(name)
        if handler is None:
            self.get_logger().warn(f'Unknown task: {name}')
            return False

        return handler(params)

    # ========================================================================
    # Action helper
    # ========================================================================

    def _format_motion_context(self, purpose: str) -> str:
        """Create the standard task context attached to BasicMotion goals."""
        task_name = self._debug_task_name or self._current_task_name or 'unknown'
        step = self._current_task_step
        if step <= 0:
            step = self.current_index + 1 if self.tasks else 1
        purpose = str(purpose).strip() or '执行运动指令'
        return f'{task_name} task:{step}步骤-{purpose}'

    @staticmethod
    def _default_motion_purpose(cmd_type, axes: str) -> str:
        purposes = {
            BasicMotion.Goal.START: '初始化里程计原点',
            BasicMotion.Goal.SET: '执行绝对定位',
            BasicMotion.Goal.WMOVE: '执行世界坐标步进移动',
            BasicMotion.Goal.BMOVE: '执行机体坐标步进移动',
            BasicMotion.Goal.WTRAVEL: '执行世界坐标直线移动',
            BasicMotion.Goal.BTRAVEL: '执行机体坐标直线移动',
        }
        purpose = purposes.get(cmd_type, '执行运动指令')
        return f'{purpose}({axes or "all"})'

    def _send_action_goal(self, cmd_type, target, axes='', timeout=60.0,
                          quiet=False, task_context=''):
        """Send a BasicMotion action goal and wait for completion (blocking).

        Polls the future in a loop since this runs in a daemon thread while
        the main thread's SingleThreadedExecutor processes DDS events.

        Args:
            cmd_type: BasicMotion.Goal.{START,SET,WMOVE,BMOVE,WTRAVEL,BTRAVEL}
            target: list of 4 floats [x, y, z, yaw] (yaw in degrees)
            axes: which axes to move (empty = all)
            timeout: max time in seconds (0 = server default 60s)
            quiet: if True, suppress per-goal INFO logs (errors still logged)
            task_context: context shown by basic_motion; empty uses the standard
                current-task/current-step/action context.

        Returns:
            (success: bool, message: str)
        """
        type_names = {1: 'WMOVE', 2: 'BMOVE', 3: 'SET', 4: 'WTRAVEL', 5: 'BTRAVEL', 6: 'START'}
        type_name = type_names.get(cmd_type, f'UNKNOWN({cmd_type})')
        task_context = (str(task_context).strip() or self._format_motion_context(
            self._default_motion_purpose(cmd_type, axes)))

        # Debug mode timeout override
        effective_timeout = timeout
        if self._debug_timeout > 0:
            effective_timeout = self._debug_timeout

        if not quiet:
            t = [f'{v:.2f}' for v in target]
            self.get_logger().info(
                f'Send goal: {type_name} target=[{", ".join(t)}] '
                f'timeout={effective_timeout:.0f}s task_context="{task_context}"')

        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Action server not available')
            return False, 'Action server not available'

        goal = BasicMotion.Goal()
        goal.cmd_type = cmd_type
        goal.axes = axes
        goal.target = target
        goal.timeout = float(effective_timeout)
        goal.task_context = task_context

        send_future = self._action_client.send_goal_async(goal)
        while rclpy.ok() and not self.stopped and not send_future.done():
            time.sleep(0.01)
        if not rclpy.ok() or self.stopped:
            if not quiet:
                self.get_logger().warn(f'Goal interrupted (stopped={self.stopped})')
            return False, 'Stopped'
        if not send_future.done():
            self.get_logger().error('Goal send timeout')
            return False, 'Goal send timeout'

        goal_handle = send_future.result()
        self._active_goal_handle = goal_handle
        if not goal_handle.accepted:
            self._active_goal_handle = None
            self.get_logger().error(f'Goal rejected by server')
            return False, 'Goal rejected by server'

        if not quiet:
            self.get_logger().info(f'Goal accepted, waiting for result...')

        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not self.stopped and not result_future.done():
            time.sleep(0.01)
        self._active_goal_handle = None
        if not rclpy.ok() or self.stopped:
            if not quiet:
                self.get_logger().warn(f'Goal result interrupted (stopped={self.stopped})')
            return False, 'Stopped'
        if not result_future.done():
            self.get_logger().error('Result timeout')
            return False, 'Result timeout'

        result = result_future.result().result
        if not result.success:
            t_str = ', '.join(f'{v:.2f}' for v in target)
            self.get_logger().error(
                f'{type_name} FAILED: {result.message} '
                f'target=[{t_str}]')
        elif not quiet:
            self.get_logger().info(f'Goal result: SUCCESS')
        return result.success, result.message

    # ========================================================================
    # Task implementations
    # ========================================================================

    # --- START ---

    # ── 启动前状态检查 ──────────────────────────────────────────────

    def _sleep_or_skip(self, seconds: float, skip: threading.Event) -> bool:
        """Sleep, but return True early if skip is triggered by Enter press."""
        deadline = time.time() + seconds
        while time.time() < deadline and not skip.is_set():
            time.sleep(min(0.1, deadline - time.time()))
        return skip.is_set()

    def _task_start(self, p: dict) -> bool:
        # ── 后台监听 Enter 键跳过准备 ──
        skip = threading.Event()

        def _listen_skip():
            try:
                self.get_logger().info('按 回车 跳过检查和准备，直接发车')
                input()
                skip.set()
                self.get_logger().warn('⚠ 跳过准备，直接发车!')
            except EOFError:
                pass

        listener = threading.Thread(target=_listen_skip, daemon=True)
        listener.start()

        self.get_logger().info(f'YouLong_AUV_Control_System 准备启动，请做好拔缆准备')
        self.set_light(3, 'LED')
        if self._sleep_or_skip(1, skip):
            return self._do_start()
        self.light_off()
        self.get_logger().info(f'AUV 即将发动，请把缆或发布把缆命令')

        for i in range(1):
            if self._sleep_or_skip(0.5, skip):
                return self._do_start()
            self.set_light(1, 'LED')
            if self._sleep_or_skip(0.5, skip):
                return self._do_start()
            self.light_off()

        self.get_logger().info(f'AUV 将在6秒后启动，已经可以拔缆了')

        for i in range(7):
            if self._sleep_or_skip(0.25, skip):
                return self._do_start()
            self.set_light(2, 'LED')
            if self._sleep_or_skip(0.25, skip):
                return self._do_start()
            self.light_off()

        self.get_logger().info(f'AUV 将在两秒后启动，如果你能看到这一条信息，说明已经有点晚了')

        if self._sleep_or_skip(1, skip):
            return self._do_start()
        self.set_light(2, 'LED')
        if self._sleep_or_skip(1, skip):
            return self._do_start()
        self.light_off()

        return self._do_start()

    def _do_start(self) -> bool:
        """发送 START action goal，初始化 odom 原点。"""
        self.light_off()
        success, msg = self._send_action_goal(
            BasicMotion.Goal.START, [0.0, 0.0, 0.0, 0.0], timeout=0)
        if success:
            self._cmd_x = self._cmd_y = self._cmd_z = self._cmd_yaw = 0.0
        else:
            self.get_logger().error(f'START failed: {msg}')
        return success

    def _task_return_origin(self, p: dict) -> bool:
        """任务开始后回到 odom 原点，只修正水平位置和航向。

        START 会把当前所在位置定义为 odom 原点，因此通常这里无需再移动；
        保留该步骤是为了让任务时序明确，并处理 START 前仍有残余目标的情况。
        深度不参与归零，避免把当前深度误当成需要回到 z=0 的目标。
        """
        settle_time = max(0.0, float(p.get('state_settle_time', 0.3)))
        if settle_time > 0.0:
            time.sleep(settle_time)

        pose = self._latest_robot_pose()
        horizontal_error = math.hypot(pose[0], pose[1])
        yaw_error = abs(((pose[5] + 180.0) % 360.0) - 180.0)
        if horizontal_error <= 0.1 and yaw_error <= 5.0:
            self._cmd_x = self._cmd_y = self._cmd_yaw = 0.0
            self.get_logger().info(
                'return_origin: 已在 odom 原点附近，跳过重复定位 '
                f'(xy={horizontal_error:.3f}m, yaw={yaw_error:.1f}°)')
            return True

        timeout = max(1.0, float(p.get('timeout', 30.0)))
        success, msg = self._send_action_goal(
            BasicMotion.Goal.SET,
            [0.0, 0.0, 0.0, 0.0],
            'xyrz',
            timeout=timeout,
            task_context=self._format_motion_context(
                '回到里程计原点(保持当前深度)'),
        )
        self.get_logger().info(f'return_origin: {msg}')
        if success:
            self._cmd_x = self._cmd_y = self._cmd_yaw = 0.0
        return success

    # --- SET tasks (absolute positioning) ---

    def _task_setx(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.SET,
            [p['x'], self._cmd_y, self._cmd_z, self._cmd_yaw],
            "x")
        self.get_logger().info(f'setx: {msg}')
        if success:
            self._cmd_x = p['x']
        return success

    def _task_sety(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.SET,
            [self._cmd_x, p['y'], self._cmd_z, self._cmd_yaw],
            "y")
        self.get_logger().info(f'sety: {msg}')
        if success:
            self._cmd_y = p['y']
        return success

    def _task_setz(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.SET,
            [self._cmd_x, self._cmd_y, p['z'], self._cmd_yaw],
            "z")
        self.get_logger().info(f'setz: {msg}')
        if success:
            self._cmd_z = p['z']
        return success

    def _task_setrz(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.SET,
            [self._cmd_x, self._cmd_y, self._cmd_z, p['rz']],
            "rz")
        self.get_logger().info(f'setrz: {msg}')
        if success:
            self._cmd_yaw = p['rz']
        return success

    def _task_setxy(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.SET,
            [p['x'], p['y'], self._cmd_z, self._cmd_yaw],
            "xy")
        self.get_logger().info(f'setxy: {msg}')
        if success:
            self._cmd_x, self._cmd_y = p['x'], p['y']
        return success

    def _task_setxyz(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.SET,
            [p['x'], p['y'], p['z'], self._cmd_yaw],
            "xyz")
        self.get_logger().info(f'setxyz: {msg}')
        if success:
            self._cmd_x, self._cmd_y, self._cmd_z = p['x'], p['y'], p['z']
        return success

    def _task_setxyzrz(self, p: dict) -> bool:
        x, y, z, yaw = p['x'], p['y'], p['z'], p.get('rz', self._cmd_yaw)
        success, msg = self._send_action_goal(
            BasicMotion.Goal.SET, [x, y, z, yaw], "xyzrz")
        self.get_logger().info(f'setxyzrz: {msg}')
        if success:
            self._cmd_x, self._cmd_y, self._cmd_z = x, y, z
            self._cmd_yaw = yaw
        return success

    def _task_setxyrz(self, p: dict) -> bool:
        yaw = p.get('rz', self._cmd_yaw)
        success, msg = self._send_action_goal(
            BasicMotion.Goal.SET,
            [p['x'], p['y'], self._cmd_z, yaw],
            "xyrz")
        self.get_logger().info(f'setxyrz: {msg}')
        if success:
            self._cmd_x, self._cmd_y = p['x'], p['y']
            self._cmd_yaw = yaw
        return success

    # --- BMOVE tasks (body frame stepping) ---

    def _task_bmovex(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.BMOVE, [p['dx'], 0.0, 0.0, 0.0], "x")
        self.get_logger().info(f'bmovex: {msg}')
        if success:
            cy, sy = math.cos(math.radians(self._cmd_yaw)), math.sin(math.radians(self._cmd_yaw))
            self._cmd_x += cy * p['dx']
            self._cmd_y += sy * p['dx']
        return success

    def _task_bmovey(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.BMOVE, [0.0, p['dy'], 0.0, 0.0], "y")
        self.get_logger().info(f'bmovey: {msg}')
        if success:
            cy, sy = math.cos(math.radians(self._cmd_yaw)), math.sin(math.radians(self._cmd_yaw))
            self._cmd_x += -sy * p['dy']
            self._cmd_y += cy * p['dy']
        return success

    def _task_bmovez(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.BMOVE, [0.0, 0.0, p['dz'], 0.0], "z")
        self.get_logger().info(f'bmovez: {msg}')
        if success:
            self._cmd_z += p['dz']
        return success

    def _task_bmoverz(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.BMOVE, [0.0, 0.0, 0.0, p['drz']], "rz")
        self.get_logger().info(f'bmoverz: {msg}')
        if success:
            self._cmd_yaw += p['drz']
        return success

    def _task_bmovexy(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.BMOVE, [p['dx'], p['dy'], 0.0, 0.0], "xy")
        self.get_logger().info(f'bmovexy: {msg}')
        if success:
            cy, sy = math.cos(math.radians(self._cmd_yaw)), math.sin(math.radians(self._cmd_yaw))
            self._cmd_x += cy * p['dx'] - sy * p['dy']
            self._cmd_y += sy * p['dx'] + cy * p['dy']
        return success

    def _task_bmovexyz(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.BMOVE, [p['dx'], p['dy'], p['dz'], 0.0], "xyz")
        self.get_logger().info(f'bmovexyz: {msg}')
        if success:
            cy, sy = math.cos(math.radians(self._cmd_yaw)), math.sin(math.radians(self._cmd_yaw))
            self._cmd_x += cy * p['dx'] - sy * p['dy']
            self._cmd_y += sy * p['dx'] + cy * p['dy']
            self._cmd_z += p['dz']
        return success

    # --- WMOVE tasks (世界系步进：参数为绝对世界坐标，内部计算偏移) ---

    def _task_wmovex(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.WMOVE, [p['x'], 0.0, 0.0, 0.0], "x")
        self.get_logger().info(f'wmovex: {msg}')
        if success:
            self._cmd_x = p['x']
        return success

    def _task_wmovey(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.WMOVE, [0.0, p['y'], 0.0, 0.0], "y")
        self.get_logger().info(f'wmovey: {msg}')
        if success:
            self._cmd_y = p['y']
        return success

    def _task_wmovez(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.WMOVE, [0.0, 0.0, p['z'], 0.0], "z")
        self.get_logger().info(f'wmovez: {msg}')
        if success:
            self._cmd_z = p['z']
        return success

    def _task_wmoverz(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.WMOVE, [0.0, 0.0, 0.0, p['rz']], "rz")
        self.get_logger().info(f'wmoverz: {msg}')
        if success:
            self._cmd_yaw = p['rz']
        return success

    def _task_wmovexy(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.WMOVE, [p['x'], p['y'], 0.0, 0.0], "xy")
        self.get_logger().info(f'wmovexy: {msg}')
        if success:
            self._cmd_x, self._cmd_y = p['x'], p['y']
        return success

    def _task_wmovexyz(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.WMOVE, [p['x'], p['y'], p['z'], 0.0], "xyz")
        self.get_logger().info(f'wmovexyz: {msg}')
        if success:
            self._cmd_x, self._cmd_y, self._cmd_z = p['x'], p['y'], p['z']
        return success

    # --- WTRAVEL tasks (世界系直线：参数为绝对世界坐标) ---

    def _task_wtravelx(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.WTRAVEL, [p['x'], 0.0, 0.0, 0.0], "x")
        self.get_logger().info(f'wtravelx: {msg}')
        if success:
            self._cmd_x = p['x']
        return success

    def _task_wtravely(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.WTRAVEL, [0.0, p['y'], 0.0, 0.0], "y")
        self.get_logger().info(f'wtravely: {msg}')
        if success:
            self._cmd_y = p['y']
        return success

    def _task_wtravelz(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.WTRAVEL, [0.0, 0.0, p['z'], 0.0], "z")
        self.get_logger().info(f'wtravelz: {msg}')
        if success:
            self._cmd_z = p['z']
        return success

    def _task_wtravelxy(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.WTRAVEL, [p['x'], p['y'], 0.0, 0.0], "xy")
        self.get_logger().info(f'wtravelxy: {msg}')
        if success:
            self._cmd_x, self._cmd_y = p['x'], p['y']
        return success

    def _task_wtravelxyz(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.WTRAVEL, [p['x'], p['y'], p['z'], 0.0], "xyz")
        self.get_logger().info(f'wtravelxyz: {msg}')
        if success:
            self._cmd_x, self._cmd_y, self._cmd_z = p['x'], p['y'], p['z']
        return success

    # --- BTRAVEL tasks (body frame linear travel: body→world + turn + go) ---

    def _task_btravelx(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.BTRAVEL, [p['dx'], 0.0, 0.0, 0.0], "x")
        self.get_logger().info(f'btravelx: {msg}')
        if success:
            cy, sy = math.cos(math.radians(self._cmd_yaw)), math.sin(math.radians(self._cmd_yaw))
            self._cmd_x += cy * p['dx']
            self._cmd_y += sy * p['dx']
        return success

    def _task_btravely(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.BTRAVEL, [0.0, p['dy'], 0.0, 0.0], "y")
        self.get_logger().info(f'btravely: {msg}')
        if success:
            cy, sy = math.cos(math.radians(self._cmd_yaw)), math.sin(math.radians(self._cmd_yaw))
            self._cmd_x += -sy * p['dy']
            self._cmd_y += cy * p['dy']
        return success

    def _task_btravelz(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.BTRAVEL, [0.0, 0.0, p['dz'], 0.0], "z")
        self.get_logger().info(f'btravelz: {msg}')
        if success:
            self._cmd_z += p['dz']
        return success

    def _task_btravelxy(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.BTRAVEL, [p['dx'], p['dy'], 0.0, 0.0], "xy")
        self.get_logger().info(f'btravelxy: {msg}')
        if success:
            cy, sy = math.cos(math.radians(self._cmd_yaw)), math.sin(math.radians(self._cmd_yaw))
            self._cmd_x += cy * p['dx'] - sy * p['dy']
            self._cmd_y += sy * p['dx'] + cy * p['dy']
        return success

    def _task_btravelxyz(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.BTRAVEL, [p['dx'], p['dy'], p['dz'], 0.0], "xyz")
        self.get_logger().info(f'btravelxyz: {msg}')
        if success:
            cy, sy = math.cos(math.radians(self._cmd_yaw)), math.sin(math.radians(self._cmd_yaw))
            self._cmd_x += cy * p['dx'] - sy * p['dy']
            self._cmd_y += sy * p['dx'] + cy * p['dy']
            self._cmd_z += p['dz']
        return success

    # --- Special tasks ---

    def _move_to_nearest_object_xy(self, class_id: int) -> bool:
        """SET 绝对定位到 class_id 最近物体的 XY 坐标。

        使用 self.objects (ObjectPositionArray) 获取 3D 位置，
        成功后更新 self._cmd_x/_cmd_y。
        """
        nearest = None
        min_dist = float('inf')
        for obj in self.objects.objects:
            if obj.class_id == class_id:
                dx = obj.world_x - self._cmd_x
                dy = obj.world_y - self._cmd_y
                dist = math.sqrt(dx * dx + dy * dy)
                if dist < min_dist:
                    min_dist = dist
                    nearest = obj

        if nearest is None:
            self.get_logger().warn(
                f'_move_to_nearest_object_xy: no object class_id={class_id}')
            return False

        self.get_logger().info(
            f'_move_to_nearest_object_xy: class_id={class_id} '
            f'at ({nearest.world_x:.2f}, {nearest.world_y:.2f}) '
            f'dist={min_dist:.2f}m')

        success, msg = self._send_action_goal(
            BasicMotion.Goal.SET,
            [nearest.world_x, nearest.world_y, self._cmd_z, self._cmd_yaw],
            'xy', timeout=15.0, quiet=True)
        if success:
            self._cmd_x = nearest.world_x
            self._cmd_y = nearest.world_y
        else:
            self.get_logger().warn(
                f'_move_to_nearest_object_xy failed: {msg}')
        return success

    def _task_navigate(self, p: dict) -> bool:
        x = p.get('x', self._cmd_x)
        y = p.get('y', self._cmd_y)
        z = p.get('z', self._cmd_z)
        yaw = p.get('rz', self._cmd_yaw)
        success, msg = self._send_action_goal(
            BasicMotion.Goal.SET, [x, y, z, yaw], "xyzrz", timeout=120.0)
        self.get_logger().info(f'navigate: {msg}')
        if success:
            self._cmd_x, self._cmd_y, self._cmd_z, self._cmd_yaw = x, y, z, yaw
        return success

    def _task_wait(self, p: dict) -> bool:
        duration = p.get('duration', 1.0)
        time.sleep(duration)
        return True

    def _task_follow_line(self, p: dict) -> bool:
        """执行管道巡线任务 — 创建 LineFollower 子对象并运行。"""
        follower = LineFollower(self, p)
        try:
            return follower.execute()
        finally:
            follower.destroy()

    def _task_arrow_surface(self, p: dict) -> bool:
        """执行箭头对准+出水任务 — 创建 ArrowSurfacer 子对象并运行。"""
        surfacer = ArrowSurfacer(self, p)
        try:
            return surfacer.execute()
        finally:
            surfacer.destroy()

    # ── 撞球任务 ──────────────────────────────────────────────────

    @staticmethod
    def _normalize_impact_ball_name(value):
        """将撞球名称或 class_id 统一成定位器的物理类别名。"""
        if isinstance(value, (int, np.integer)):
            class_id = int(value)
            for name, known_id in _IMPACT_BALL_CLASS_IDS.items():
                if class_id == known_id:
                    return name
            return None
        text = str(value).strip().lower()
        if text.isdigit():
            return TaskRunnerNode._normalize_impact_ball_name(int(text))
        return _IMPACT_BALL_ALIASES.get(text)

    def _impact_ball_order(self, params: dict) -> list[str]:
        """Read the requested ball order; default is blue then red."""
        values = params.get('order')
        if values is None:
            values = params.get('ball_classes')
        if values is None:
            values = params.get('ball_class_ids')
        if values is None:
            values = ['impact_ball_blue', 'impact_ball_red']
        if isinstance(values, (str, int, np.integer)):
            values = [values]

        result = []
        for value in values:
            name = self._normalize_impact_ball_name(value)
            if name is not None and name not in result:
                result.append(name)
        if not result:
            self.get_logger().error(
                'hit_balls: no valid ball in order; use blue/red or class_id 5/6')
        return result

    def _best_impact_ball_target(self, name: str, params: dict):
        """Return a usable estimate for one suspended impact ball.

        Impact-ball targets are allowed to remain usable after the localizer
        marks them stale.  The estimate can still be valuable for the task;
        freshness is not a task-level validity condition.
        """
        class_id = _IMPACT_BALL_CLASS_IDS[name]
        min_confidence = float(params.get('min_confidence', 0.05))
        min_observations = int(params.get('min_observations', 1))
        with self._perception_lock:
            target_positions = list(self.target_positions.targets)
            compatibility_objects = list(self.objects.objects)

        candidates = []
        for target in target_positions:
            target_name = str(
                getattr(target, 'physical_class_name', '') or
                getattr(target, 'class_name', '')).strip().lower()
            if (int(getattr(target, 'class_id', -1)) != class_id
                    and target_name != name):
                continue
            # A front estimate is the normal source for suspended balls.  The
            # empty-source case keeps this task compatible with older bags.
            source = str(getattr(target, 'estimate_source', '')).strip().lower()
            if source not in ('', 'front'):
                continue
            status = int(getattr(target, 'status', TargetPosition.STATUS_STABLE))
            if status == TargetPosition.STATUS_UNINITIALIZED:
                continue
            confidence = float(getattr(target, 'confidence', 0.0))
            observations = int(getattr(target, 'num_observations', 0))
            if (not math.isfinite(confidence)
                    or confidence < min_confidence
                    or observations < min_observations):
                continue
            x = float(target.world_x)
            y = float(target.world_y)
            z = float(target.world_z)
            if not all(math.isfinite(value) for value in (x, y, z)):
                continue
            distance = math.hypot(x - self._cmd_x, y - self._cmd_y)
            # Prefer stable estimates, then the estimate nearest to the
            # current commanded position when duplicate tracks exist.
            candidates.append((
                status != TargetPosition.STATUS_STABLE,
                distance,
                -confidence,
                {'name': name, 'x': x, 'y': y, 'z': z,
                 'confidence': confidence, 'observations': observations,
                 'source': source or 'target_positions'},
            ))

        # Fallback for the legacy ObjectPositionArray producer.  The current
        # object_localizer publishes front targets on TargetPositionArray, but
        # this keeps hit_balls usable with old position-node recordings.
        for obj in compatibility_objects:
            if int(getattr(obj, 'class_id', -1)) != class_id:
                continue
            confidence = float(getattr(obj, 'confidence', 0.0))
            observations = int(getattr(obj, 'num_observations', 0))
            x = float(obj.world_x)
            y = float(obj.world_y)
            z = float(obj.world_z)
            if (not all(math.isfinite(value) for value in (x, y, z))
                    or confidence < min_confidence
                    or observations < min_observations):
                continue
            candidates.append((
                False,
                math.hypot(x - self._cmd_x, y - self._cmd_y),
                -confidence,
                {'name': name, 'x': x, 'y': y, 'z': z,
                 'confidence': confidence, 'observations': observations,
                 'source': 'objects'},
            ))

        if not candidates:
            return None
        candidates.sort(key=lambda item: item[:3])
        return candidates[0][3]

    def _wait_for_impact_ball(self, name: str, params: dict):
        """Wait for a usable target estimate while allowing task stop."""
        timeout = max(0.0, float(params.get('detect_timeout', 30.0)))
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not self.stopped:
            target = self._best_impact_ball_target(name, params)
            if target is not None:
                return target
            if time.monotonic() >= deadline:
                break
            time.sleep(0.1)
        self.get_logger().error(
            f'hit_balls: timed out waiting for {name} estimate '
            f'({timeout:.1f}s)')
        return None

    @staticmethod
    def _wrap_yaw_degrees(value: float) -> float:
        """Wrap a yaw command to the controller's conventional range."""
        return (float(value) + 180.0) % 360.0 - 180.0

    def _rotate_for_impact_scan(self, yaw: float, timeout: float) -> bool:
        """Rotate in place so the front stereo cameras scan their surroundings."""
        yaw = self._wrap_yaw_degrees(yaw)
        success, message = self._send_action_goal(
            BasicMotion.Goal.SET,
            [self._cmd_x, self._cmd_y, self._cmd_z, yaw],
            'rz',
            timeout=timeout,
            quiet=True,
            task_context=self._format_motion_context('主动旋转扫描红球'),
        )
        if success:
            self._cmd_yaw = yaw
        else:
            self.get_logger().warn(
                f'hit_balls: scan rotation to {yaw:.1f}° failed: {message}')
        return success

    def _active_localize_impact_balls(self, order: list[str], params: dict):
        """Actively scan 360 degrees and collect both ball estimates."""
        step = float(params.get('search_yaw_step_deg', 30.0))
        step = min(180.0, max(5.0, abs(step)))
        settle_time = max(0.0, float(params.get('search_settle_time', 0.4)))
        rotate_timeout = max(
            1.0, float(params.get('search_rotate_timeout', 10.0)))
        search_timeout = max(0.0, float(params.get('search_timeout', 60.0)))
        headings = max(1, int(math.ceil(360.0 / step)))
        start_yaw = self._cmd_yaw
        found = {}
        deadline = time.monotonic() + search_timeout

        self.get_logger().info(
            f'hit_balls: active localization scan started, '
            f'{headings} headings/{step:.1f}°')
        for index in range(headings):
            if not rclpy.ok() or self.stopped or time.monotonic() >= deadline:
                break
            heading = start_yaw + index * step
            # The first sample uses the current heading; subsequent samples
            # rotate in place so the front stereo pair observes all azimuths.
            if index > 0 and not self._rotate_for_impact_scan(
                    heading,
                    min(rotate_timeout,
                        max(1.0, deadline - time.monotonic()))):
                continue
            if settle_time > 0.0:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                time.sleep(min(settle_time, remaining))
            for name in order:
                if name not in found:
                    target = self._best_impact_ball_target(name, params)
                    if target is not None:
                        found[name] = target
                        self.get_logger().info(
                            f'hit_balls: active scan found {name} '
                            f'at ({target["x"]:.2f}, {target["y"]:.2f}, '
                            f'{target["z"]:.2f})')
            if len(found) == len(order):
                break

        self.get_logger().info(
            f'hit_balls: active localization scan finished, '
            f'found={list(found.keys())}, missing=' +
            f'{[name for name in order if name not in found]}')
        return found

    def _travel_to_impact_point(self, x: float, y: float, z: float,
                                timeout: float, label: str) -> bool:
        """Travel in a straight world-frame segment and update command pose."""
        dx = x - self._cmd_x
        dy = y - self._cmd_y
        if math.hypot(dx, dy) > 1e-6:
            yaw = math.degrees(math.atan2(dy, dx))
        else:
            yaw = self._cmd_yaw
        success, message = self._send_action_goal(
            BasicMotion.Goal.WTRAVEL,
            [x, y, z, yaw],
            'xyz',
            timeout=timeout,
            task_context=self._format_motion_context(label),
        )
        if success:
            self._cmd_x, self._cmd_y, self._cmd_z = x, y, z
            if math.hypot(dx, dy) > 1e-6:
                self._cmd_yaw = yaw
            self.get_logger().info(
                f'hit_balls: {label} complete at '
                f'({x:.2f}, {y:.2f}, {z:.2f})')
        else:
            self.get_logger().error(f'hit_balls: {label} failed: {message}')
        return success

    def _latest_robot_pose(self):
        """Return the latest measured odom pose, or the command pose fallback."""
        with self._perception_lock:
            pose = self._robot_pose
        if pose is not None and all(math.isfinite(float(value)) for value in pose):
            return tuple(float(value) for value in pose)
        return (
            float(self._cmd_x), float(self._cmd_y), float(self._cmd_z),
            0.0, 0.0, float(self._cmd_yaw),
        )

    def _impact_staging_pose(self, target: dict, staging_distance: float,
                             z_offset: float = 0.0):
        """Calculate a point before the ball and a yaw pointing at the ball."""
        pose = self._latest_robot_pose()
        dx = float(target['x']) - pose[0]
        dy = float(target['y']) - pose[1]
        distance = math.hypot(dx, dy)
        if distance > 1e-6:
            direction_x = dx / distance
            direction_y = dy / distance
            yaw = math.degrees(math.atan2(dy, dx))
        else:
            yaw = float(pose[5])
            yaw_rad = math.radians(yaw)
            direction_x = math.cos(yaw_rad)
            direction_y = math.sin(yaw_rad)
        return (
            float(target['x']) - direction_x * staging_distance,
            float(target['y']) - direction_y * staging_distance,
            float(target['z']) + z_offset,
            self._wrap_yaw_degrees(yaw),
        )

    def _hold_impact_alignment(self, name: str, target: dict, params: dict):
        """Continuously refresh the position/yaw target during the alignment hold."""
        duration = max(
            0.0, float(params.get('position_correction_duration', 30.0)))
        period = max(0.05, float(params.get('position_correction_period', 0.20)))
        command_timeout = max(
            0.20, float(params.get('position_correction_command_timeout', 10.0)))
        min_update_m = max(
            0.0, float(params.get('position_correction_min_update_m', 0.03)))
        min_update_yaw_deg = max(
            0.0, float(params.get('position_correction_min_update_deg', 1.0)))
        staging_distance = max(
            0.0, float(params.get('approach_distance', 0.5)))
        min_clearance = max(0.0, float(params.get('min_clearance', 0.05)))
        z_offset = float(params.get('z_offset', 0.2))

        deadline = time.monotonic() + duration
        last_target = target
        last_sent_staging = None
        while rclpy.ok() and not self.stopped:
            latest = self._best_impact_ball_target(name, params)
            if latest is not None:
                last_target = latest

            pose = self._latest_robot_pose()
            distance = math.hypot(
                float(last_target['x']) - pose[0],
                float(last_target['y']) - pose[1],
            )
            effective_distance = min(
                staging_distance,
                max(0.0, distance - min_clearance),
            )
            staging = self._impact_staging_pose(
                last_target, effective_distance, z_offset)
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break

            # BasicMotion keeps the last position target active.  Re-sending
            # an identical SET goal every 200 ms only creates action-server
            # work and can starve the simulator/perception executor.  Check
            # the target at the configured period, but send a new goal only
            # after a meaningful position or yaw change.
            if last_sent_staging is not None:
                position_delta = max(
                    abs(staging[index] - last_sent_staging[index])
                    for index in range(3))
                yaw_delta = abs(self._wrap_yaw_degrees(
                    staging[3] - last_sent_staging[3]))
                if (position_delta < min_update_m
                        and yaw_delta < min_update_yaw_deg):
                    time.sleep(min(period, remaining))
                    continue

            success, message = self._send_action_goal(
                BasicMotion.Goal.SET,
                list(staging),
                'xyzrz',
                timeout=min(command_timeout, max(0.20, remaining)),
                quiet=True,
                task_context=self._format_motion_context(
                    f'{name}持续位置姿态修正'),
            )
            if success:
                self._cmd_x, self._cmd_y, self._cmd_z, self._cmd_yaw = staging
                last_sent_staging = list(staging)
            elif not rclpy.ok() or self.stopped:
                return False
            else:
                self.get_logger().warning(
                    f'hit_balls: {name} alignment correction failed: {message}')

            remaining = deadline - time.monotonic()
            if remaining > 0.0:
                time.sleep(min(period, remaining))

        return rclpy.ok() and not self.stopped

    def _publish_body_velocity(self, forward_mps: float):
        """Publish one body-frame velocity-loop setpoint (forward/zero other axes)."""
        msg = ZitSetpoint()
        msg.control_key = 0x11  # VEL (0x01) | BODY (0x10)
        msg.type_mask = 0
        msg.x = float(forward_mps)
        msg.y = 0.0
        msg.z = 0.0
        msg.roll = 0.0
        msg.pitch = 0.0
        msg.yaw = 0.0
        msg.seq = 0
        self.pub_setpoint.publish(msg)

    def _charge_forward(self, params: dict) -> bool:
        """Run the body-X velocity loop for a fixed short impact charge."""
        duration = max(0.0, float(params.get('charge_duration', 5.0)))
        speed = max(0.0, float(params.get('charge_speed_mps', 0.15)))
        period = max(0.02, float(params.get('charge_publish_period', 0.05)))
        deadline = time.monotonic() + duration
        while rclpy.ok() and not self.stopped and time.monotonic() < deadline:
            self._publish_body_velocity(speed)
            time.sleep(min(period, max(0.0, deadline - time.monotonic())))
        # Leave velocity mode with a neutral command before switching back to
        # the position action for the return trip.
        self._publish_body_velocity(0.0)
        return rclpy.ok() and not self.stopped

    def _record_current_impact_pose(self):
        """Snapshot the measured pose after alignment for the return target."""
        pose = self._latest_robot_pose()
        return [pose[0], pose[1], pose[2], pose[5]]

    def _task_staged_charge_return(self, name: str, target: dict,
                                   params: dict) -> bool:
        """Align at the configured offset before one ball, charge, then return."""
        approach_distance = max(
            0.0, float(params.get('approach_distance', 0.5)))
        min_clearance = max(0.0, float(params.get('min_clearance', 0.05)))
        z_offset = float(params.get('z_offset', 0.2))
        approach_timeout = max(
            1.0, float(params.get('approach_timeout', 90.0)))
        return_timeout = max(
            1.0, float(params.get('return_timeout', 60.0)))

        pose = self._latest_robot_pose()
        distance = math.hypot(float(target['x']) - pose[0],
                              float(target['y']) - pose[1])
        effective_distance = min(
            approach_distance,
            max(0.0, distance - min_clearance),
        )
        staging = self._impact_staging_pose(
            target, effective_distance, z_offset)
        self.get_logger().info(
            f'hit_balls: {name} staging {effective_distance:.2f}m-before target '
            f'=({staging[0]:.2f}, {staging[1]:.2f}, {staging[2]:.2f}), '
            f'yaw={staging[3]:.1f}°')
        if not self._travel_to_impact_point(
                staging[0], staging[1], staging[2], approach_timeout,
                f'{name} {effective_distance:.2f}m staging'):
            return False

        # Refresh the ball estimate once at the staging point, then keep the
        # full position + yaw controller correcting for the configured hold.
        refreshed = self._best_impact_ball_target(name, params)
        if refreshed is not None:
            target = refreshed
        if not self._hold_impact_alignment(name, target, params):
            return False

        recorded_pose = self._record_current_impact_pose()
        self.get_logger().info(
            f'hit_balls: alignment complete; recorded pose '
            f'=({recorded_pose[0]:.2f}, {recorded_pose[1]:.2f}, '
            f'{recorded_pose[2]:.2f}, {recorded_pose[3]:.1f}°)')

        if not self._charge_forward(params):
            return False
        self.get_logger().info(
            f'hit_balls: forward velocity charge complete '
            f'({float(params.get("charge_duration", 5.0)):.1f}s)')

        success, message = self._send_action_goal(
            BasicMotion.Goal.SET,
            recorded_pose,
            'xyzrz',
            timeout=return_timeout,
            task_context=self._format_motion_context(
                f'{name}撞球后返回记录位置'),
        )
        if not success:
            self.get_logger().error(
                f'hit_balls: return to recorded pose failed: {message}')
            return False
        self._cmd_x, self._cmd_y, self._cmd_z, self._cmd_yaw = recorded_pose
        self.get_logger().info('hit_balls: red-ball task complete; returned to recorded pose')
        return True

    def _task_hit_balls(self, p: dict) -> bool:
        """Approach and pass through each detected suspended impact ball.

        The AUV stops a short distance before the current estimate, then
        travels through the ball to a point on the far side.  This creates a
        real collision path instead of merely moving to the ball's centre.
        The task is deliberately target-driven: it uses the front target
        localizer and does not depend on hard-coded scene coordinates.
        """
        order = self._impact_ball_order(p)
        if not order:
            return False
        approach_distance = max(0.0, float(p.get('approach_distance', 0.5)))
        pass_distance = max(0.0, float(p.get('pass_distance', 0.35)))
        min_clearance = max(0.0, float(p.get('min_clearance', 0.05)))
        z_offset = float(p.get('z_offset', 0.2))
        approach_timeout = max(1.0, float(p.get('approach_timeout', 90.0)))
        hit_timeout = max(1.0, float(p.get('hit_timeout', 45.0)))
        pause = max(0.0, float(p.get('between_balls_pause', 0.5)))

        self.get_logger().info(
            f'hit_balls: order={order}, approach={approach_distance:.2f}m, '
            f'pass={pass_distance:.2f}m')
        found = {}
        active_localization = bool(p.get('active_localization', True))
        if active_localization:
            found = self._active_localize_impact_balls(order, p)
            # Do not start an impact run with only one ball localized.  The
            # active scan provides the spatial search; this short completion
            # wait lets the detector/localizer finish the second estimate.
            for name in order:
                if name in found:
                    continue
                self.get_logger().info(
                    f'hit_balls: waiting to complete localization for {name}')
                target = self._wait_for_impact_ball(name, p)
                if target is None:
                    return False
                found[name] = target

        impact_mode = str(p.get('impact_mode', '')).strip().lower()
        if impact_mode == 'staged_charge_return':
            if len(order) != 1:
                self.get_logger().error(
                    'hit_balls: staged_charge_return requires exactly one ball')
                return False
            name = order[0]
            target = self._best_impact_ball_target(name, p) or found.get(name)
            if target is None:
                target = self._wait_for_impact_ball(name, p)
            if target is None:
                return False
            return self._task_staged_charge_return(name, target, p)

        for index, name in enumerate(order):
            target = self._best_impact_ball_target(name, p)
            if target is None:
                target = found.get(name)
            if target is None:
                target = self._wait_for_impact_ball(name, p)
            if target is None:
                return False

            dx = target['x'] - self._cmd_x
            dy = target['y'] - self._cmd_y
            distance = math.hypot(dx, dy)
            if distance > 1e-6:
                direction = np.array([dx / distance, dy / distance])
            else:
                yaw_rad = math.radians(self._cmd_yaw)
                direction = np.array([math.cos(yaw_rad), math.sin(yaw_rad)])

            # If already close, do not back away just to create a staging
            # point; leave at least min_clearance before the ball instead.
            staging_distance = min(
                approach_distance,
                max(0.0, distance - min_clearance),
            )
            staging_x = target['x'] - direction[0] * staging_distance
            staging_y = target['y'] - direction[1] * staging_distance
            hit_z = target['z'] + z_offset
            self.get_logger().info(
                f'hit_balls: [{index + 1}/{len(order)}] {name} '
                f'estimate=({target["x"]:.2f}, {target["y"]:.2f}, '
                f'{target["z"]:.2f}), confidence={target["confidence"]:.2f}')

            if not self._travel_to_impact_point(
                    staging_x, staging_y, hit_z, approach_timeout,
                    f'{name} approach'):
                return False

            # Refresh once after staging.  If the dynamic suspended ball has
            # moved, use its latest estimate for the pass-through segment.
            refreshed = self._best_impact_ball_target(name, p)
            if refreshed is not None:
                target = refreshed
            dx = target['x'] - self._cmd_x
            dy = target['y'] - self._cmd_y
            distance = math.hypot(dx, dy)
            if distance > 1e-6:
                direction = np.array([dx / distance, dy / distance])
            hit_x = target['x'] + direction[0] * pass_distance
            hit_y = target['y'] + direction[1] * pass_distance
            hit_z = target['z'] + z_offset
            if not self._travel_to_impact_point(
                    hit_x, hit_y, hit_z, hit_timeout,
                    f'{name} impact pass'):
                return False
            self.get_logger().info(f'hit_balls: {name} collision path finished')
            if index + 1 < len(order) and pause > 0.0:
                time.sleep(pause)
        return True

    # ── 投信标 / 采水 / 释放取水器 ─────────────────────────────────

    def _task_drop_beacon(self, p: dict) -> bool:
        angle = float(p.get('angle_rad', self.ANGLE_DROP_BEACON))
        self.set_servo(angle, 'drop beacon')
        self.get_logger().info('🔫 BEACON DROPPED!')
        return True

    def _task_take_water_sample(self, p: dict) -> bool:
        angle = float(p.get('angle_rad', self.ANGLE_SAMPLE_WATER))
        self.set_servo(angle, 'take water sample')
        self.get_logger().info('💧 WATER SAMPLE TAKEN!')
        return True

    def _task_release_sampler(self, p: dict) -> bool:
        """转向 → 对齐 START 标记 → 上浮靠岸 → 释放取水器。"""
        align_yaw = float(p.get('align_yaw', 180.0))
        start_cid = int(p.get('start_class_id', 4))
        approach_z = float(p.get('approach_z', -0.3))
        approach_x = float(p.get('approach_x', -0.3))
        approach_timeout = float(p.get('approach_timeout', 15.0))
        release_angle = float(p.get('release_angle_rad',
                                    self.ANGLE_RELEASE_SAMPLER))

        self.get_logger().info(
            f'🧭 release_sampler: turning to rz={align_yaw:.1f}°')
        self._send_action_goal(
            BasicMotion.Goal.WMOVE,
            [self._cmd_x, self._cmd_y, self._cmd_z, align_yaw],
            'rz', timeout=15.0)
        self._cmd_yaw = align_yaw

        self.get_logger().info(
            f'🎯 release_sampler: aligning to START marker (class={start_cid})')
        self._align_to_class(start_cid, 'START marker')

        self.get_logger().info(
            f'🌊🏖️  release_sampler: wmove z={approach_z} x={approach_x}')
        self._send_action_goal(
            BasicMotion.Goal.WMOVE,
            [approach_x, self._cmd_y, approach_z, self._cmd_yaw],
            'xz', timeout=approach_timeout)
        self._send_action_goal(
            BasicMotion.Goal.WMOVE,
            [approach_x, self._cmd_y - 1, approach_z, self._cmd_yaw],
            'xz', timeout=approach_timeout)
        self._cmd_x = approach_x; self._cmd_z = approach_z

        self.set_servo(release_angle, 'release water sampler')
        self.get_logger().info('🗑️  WATER SAMPLER RELEASED!')
        return True

    # ========================================================================
    # Service handlers
    # ========================================================================

    def _run_task_cb(self, request, response):
        if request.start:
            path = request.task_name
            if not os.path.isabs(path):
                default = os.path.join(
                    get_package_share_directory('uv_task'), 'config', 'tasks.json'
                )
                if os.path.exists(default):
                    path = default

            self.get_logger().info(f'Service /task/run: start tasks from {path}')
            self.tasks = self.load_tasks(path)
            if self.tasks:
                thread = threading.Thread(target=self.run_task_list, daemon=True)
                thread.start()
                response.success = True
                response.message = f'Started {len(self.tasks)} tasks'
            else:
                response.success = False
                response.message = 'No tasks loaded'
        else:
            self.get_logger().info('Service /task/run: stop requested')
            self.stopped = True
            response.success = True
            response.message = 'Stopped'
        return response

    def _stop_task_cb(self, request, response):
        self.get_logger().warn('Service /task/stop: emergency stop')
        self.stopped = True
        if self._active_goal_handle is not None:
            self.get_logger().info('Cancelling active action goal')
            cancel_future = self._action_client.async_cancel_goal(
                self._active_goal_handle)
            self._active_goal_handle = None
        response.success = True
        response.message = 'Tasks stopped'
        return response

    def _exec_task_cb(self, request, response):
        """Handle /task/exec: execute a single task (debug mode only)."""
        if not self._debug_mode:
            response.success = False
            response.message = 'ExecTask service only available in debug mode'
            self.get_logger().warn('/task/exec called but debug_mode is off')
            return response

        if self._debug_executing:
            response.success = False
            response.message = (
                f'Task "{self._debug_task_name}" is already running. '
                'Wait for it or call /task/stop.'
            )
            self.get_logger().warn(f'Rejected concurrent /task/exec: {self._debug_task_name}')
            return response

        task_name = request.task_name
        params_json = request.params_json
        timeout = request.timeout

        # Validate task_name against task_map
        if task_name not in self.task_map:
            response.success = False
            valid = ', '.join(sorted(self.task_map.keys()))
            response.message = f'Unknown task: {task_name}. Valid: {valid}'
            return response

        # Parse params JSON
        try:
            params = json.loads(params_json) if params_json.strip() else {}
        except json.JSONDecodeError as e:
            response.success = False
            response.message = f'Invalid params JSON: {e}'
            return response

        # Store timeout for _send_action_goal override
        params['_timeout'] = timeout

        self.get_logger().info(
            f'DEBUG EXEC: {task_name} params={params} timeout={timeout:.0f}s'
        )

        # Execute in daemon thread (same pattern as run_task_list)
        thread = threading.Thread(
            target=self._debug_exec_single, args=(task_name, params),
            daemon=True
        )
        thread.start()

        response.success = True
        response.message = f'Executing task: {task_name}'
        return response

    def _debug_exec_single(self, name: str, params: dict):
        """Run a single task in debug mode (runs in daemon thread)."""
        self._debug_executing = True
        self._debug_task_name = name
        self._current_task_name = name
        self._current_task_step = 1
        self.stopped = False
        self.running = True

        # Extract timeout override before calling execute
        timeout = params.pop('_timeout', -1.0)
        self._debug_timeout = float(timeout) if timeout > 0 else -1.0

        try:
            success = self._execute_task(name, params)
            if success:
                self.get_logger().info(f'DEBUG EXEC {name}: SUCCESS')
            else:
                self.get_logger().warn(f'DEBUG EXEC {name}: FAILED')
        except Exception as e:
            self.get_logger().error(f'DEBUG EXEC {name}: exception: {e}')
        finally:
            self._debug_executing = False
            self._debug_task_name = None
            self._current_task_name = ''
            self._current_task_step = 0
            self.running = False
            self._debug_timeout = -1.0

    # ========================================================================
    # Status publishing
    # ========================================================================

    def _publish_status(self):
        msg = TaskStatus()

        if self._debug_executing or (self._debug_mode and self.running):
            # Debug mode: single task executing
            msg.status = TaskStatus.STATUS_RUNNING
            msg.current_task_name = self._debug_task_name or 'unknown'
            msg.total_tasks = 1
            msg.current_task_index = 0
            msg.error_message = '[DEBUG MODE]'
        elif self.running:
            # Normal mode: task list executing
            msg.status = TaskStatus.STATUS_RUNNING
            msg.current_task_index = self.current_index
            msg.total_tasks = len(self.tasks)
            if self.current_index < len(self.tasks):
                msg.current_task_name = self.tasks[self.current_index].get('name', '')
            msg.error_message = ''
        elif self.stopped:
            msg.status = TaskStatus.STATUS_PAUSED
        else:
            msg.status = TaskStatus.STATUS_IDLE
            if self._debug_mode:
                msg.error_message = '[DEBUG MODE: idle, waiting for /task/exec]'

        self.pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TaskRunnerNode()

    if not node._debug_mode:
        # Normal mode: load default tasks and start immediately
        default_path = os.path.join(
            get_package_share_directory('uv_task'), 'config', 'tasks.json'
        )
        if os.path.exists(default_path):
            node.tasks = node.load_tasks(default_path)
            if node.tasks:
                thread = threading.Thread(target=node.run_task_list, daemon=True)
                thread.start()
                node.get_logger().info(f'Auto-started task list ({len(node.tasks)} tasks)')
    else:
        node.get_logger().info(
            'Debug mode active: auto-start skipped. '
            'Use /task/exec service to run single tasks.'
        )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
