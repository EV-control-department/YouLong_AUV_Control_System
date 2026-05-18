"""Task runner node: loads task list from JSON, executes tasks sequentially.

Each task calls basic_motion via the BasicMotion action server.
The task runner is the single source of truth for commanded position,
tracked locally (not from external topics).
"""

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

from uv_msgs.action import BasicMotion
from uv_msgs.msg import DetectionArray, ObjectPositionArray, PoseInfo, TaskStatus
from uv_msgs.srv import ExecTask, RunTask

from uv_task.arrow_surfacer import (
    _DOWN_CX, _DOWN_CY, _DOWN_FX, _DOWN_FY,
    _DOWN_OFFSET_LEFT, _DOWN_OFFSET_RIGHT, _DOWN_OPTICAL_TO_BODY,
    _euler_to_rotation_matrix, _ray_intersection_midpoint,
)
from uv_task.arrow_surfacer import ArrowSurfacer
from uv_task.line_follower import LineFollower


class TaskRunnerNode(Node):
    """Task runner: JSON task loader and sequential executor."""

    # ── 灯光常量 (/zit6/cmd/light) ─────────────────────────────────
    LIGHT_OFF = 0
    LIGHT_YELLOW = 1
    LIGHT_GREEN = 2
    LIGHT_RED = 3

    # ── 舵机角度 (/zit6/cmd/servo, rad) ────────────────────────────
    ANGLE_DROP_BEACON = math.pi / 2       # ~1.57 rad  投信标
    ANGLE_SAMPLE_WATER = 0.0               # 采水样
    ANGLE_RELEASE_SAMPLER = -math.pi / 2   # ~-1.57 rad  释放取水器

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

        # ── 下视感知（release_sampler 对齐用）──
        self._perception_lock = threading.RLock()
        self._down_detections = {}   # camera_name → (monotonic, DetectionArray)
        self._robot_pose = None      # (x, y, z, roll_deg, pitch_deg, yaw_deg)

        self.tasks = []
        self.current_index = 0
        self.running = False
        self.stopped = False
        self._active_goal_handle = None

        # Debug mode
        self.declare_parameter('debug_mode', False)
        self._debug_mode = self.get_parameter('debug_mode').get_parameter_value().bool_value
        self._debug_task_name = None
        self._debug_executing = False
        self._debug_timeout = -1.0

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
            'drop_beacon': self._task_drop_beacon,
            'take_water_sample': self._task_take_water_sample,
            'release_sampler': self._task_release_sampler,
        }

        # Action client
        self._action_client = ActionClient(self, BasicMotion, 'basic_motion')
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('BasicMotion action server not available!')

        # Subscribers
        self.create_subscription(
            ObjectPositionArray, '/perception/objects', self._objects_cb, 10)
        for cam in ('down_left', 'down_right'):
            self.create_subscription(
                DetectionArray, f'/perception/detection/{cam}',
                lambda msg, c=cam: self._det_cb(c, msg), 10)
        self.create_subscription(
            PoseInfo, '/basic_motion/pose_info', self._pose_cb, 10)

        # Publishers
        self.pub_status = self.create_publisher(TaskStatus, '/task/status', 10)
        self.pub_light = self.create_publisher(UInt8, '/zit6/cmd/light', 10)
        self.pub_servo = self.create_publisher(Float32, '/zit6/cmd/servo', 10)

        # Services
        self.create_service(RunTask, '/task/run', self._run_task_cb)
        self.create_service(Trigger, '/task/stop', self._stop_task_cb)
        self.create_service(ExecTask, '/task/exec', self._exec_task_cb)

        # Status timer
        self.create_timer(0.5, self._publish_status)

        self.get_logger().info('TaskRunner node started')
        self.get_logger().info(f'Debug mode: {self._debug_mode}')

    def _objects_cb(self, msg: ObjectPositionArray):
        self.objects = msg

    def _det_cb(self, camera_name: str, msg: DetectionArray):
        with self._perception_lock:
            self._down_detections[camera_name] = (time.monotonic(), msg)

    def _pose_cb(self, msg: PoseInfo):
        with self._perception_lock:
            self._robot_pose = (msg.robot_x, msg.robot_y, msg.robot_z,
                                msg.robot_roll, msg.robot_pitch, msg.robot_yaw)

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
            self.get_logger().info(f'Align [{label}]: #{i}靠近，x:{tgt[0]},y:{tgt[1]}')
            self._cmd_x = tgt[0]; self._cmd_y = tgt[1]
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

    def _send_action_goal(self, cmd_type, target, axes='', timeout=60.0,
                          quiet=False):
        """Send a BasicMotion action goal and wait for completion (blocking).

        Polls the future in a loop since this runs in a daemon thread while
        the main thread's SingleThreadedExecutor processes DDS events.

        Args:
            cmd_type: BasicMotion.Goal.{START,SET,WMOVE,BMOVE,WTRAVEL,BTRAVEL}
            target: list of 4 floats [x, y, z, yaw] (yaw in degrees)
            axes: which axes to move (empty = all)
            timeout: max time in seconds (0 = server default 60s)
            quiet: if True, suppress per-goal INFO logs (errors still logged)

        Returns:
            (success: bool, message: str)
        """
        type_names = {1: 'WMOVE', 2: 'BMOVE', 3: 'SET', 4: 'WTRAVEL', 5: 'BTRAVEL', 6: 'START'}
        type_name = type_names.get(cmd_type, f'UNKNOWN({cmd_type})')

        # Debug mode timeout override
        effective_timeout = timeout
        if self._debug_timeout > 0:
            effective_timeout = self._debug_timeout

        if not quiet:
            t = [f'{v:.2f}' for v in target]
            self.get_logger().info(
                f'Send goal: {type_name} target=[{", ".join(t)}] timeout={effective_timeout:.0f}s')

        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Action server not available')
            return False, 'Action server not available'

        goal = BasicMotion.Goal()
        goal.cmd_type = cmd_type
        goal.axes = axes
        goal.target = target
        goal.timeout = float(effective_timeout)

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

    def _task_start(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.START, [0.0, 0.0, 0.0, 0.0], timeout=0)
        if success:
            self._cmd_x = self._cmd_y = self._cmd_z = self._cmd_yaw = 0.0
        else:
            self.get_logger().error(f'START failed: {msg}')
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
