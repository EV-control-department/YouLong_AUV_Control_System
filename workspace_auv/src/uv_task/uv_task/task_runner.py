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

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

from uv_msgs.action import BasicMotion
from uv_msgs.msg import (
    DetectionArray,
    DeviceCommand,
    ObjectPositionArray,
    PoseInfo,
    TaskStatus,
)
from uv_msgs.srv import ExecTask, RunTask

from uv_task.line_follower import LineFollower


class TaskRunnerNode(Node):
    """Task runner: JSON task loader and sequential executor."""

    # Classes exported by uv_perception/vision.py's box model.
    CLASS_YELLOW_SECTOR = 0
    CLASS_RED_SECTOR = 1
    CLASS_GREEN_SECTOR = 2
    CLASS_ARROW = 3
    CLASS_START = 4
    CLASS_BASKET = 7
    CLASS_ARUCO_TAG = 8

    # Firmware-facing command values on /task/device_command.  The message
    # already exists in uv_msgs; hardware firmware only needs to subscribe to
    # this topic and translate these command values to its local drivers.
    CMD_SAMPLE_WATER = 1
    CMD_DROP_BEACON = 2
    CMD_LED_OFF = 0
    CMD_LED_YELLOW = 1
    CMD_LED_GREEN = 2
    CMD_LED_RED = 3

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
        self._perception_lock = threading.RLock()
        self._detections = {}
        self._front_stitched_image = None
        self._front_stitched_received_at = 0.0
        self._pose_info = None
        self._pose_info_received_at = 0.0
        self._aruco_marker_id = None
        self._selected_sector = None

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

        # Fixed WUURC world targets and depths are intentionally kept in
        # config/tasks.json.  Only reusable visual and actuator tuning stays
        # here as ROS parameters.
        self.declare_parameter('wuurc_surface_delta_z', 0.50)
        self.declare_parameter('wuurc_surface_dwell_s', 1.0)

        # Single down-camera visual centring.  The gains map image error to
        # small FRD BMOVE steps.  Signs are installation-specific and are
        # deliberately parameters, rather than hidden camera assumptions.
        self.declare_parameter('wuurc_align_confidence', 0.80)
        self.declare_parameter('wuurc_align_pixel_tolerance', 35.0)
        self.declare_parameter('wuurc_align_max_step_m', 0.12)
        self.declare_parameter('wuurc_align_m_per_pixel_x', 0.00055)
        self.declare_parameter('wuurc_align_m_per_pixel_y', 0.00055)
        self.declare_parameter('wuurc_align_body_x_sign', -1.0)
        self.declare_parameter('wuurc_align_body_y_sign', -1.0)
        self.declare_parameter('wuurc_align_timeout_s', 25.0)
        self.declare_parameter('wuurc_detection_max_age_s', 0.60)
        self.declare_parameter('wuurc_pose_max_age_s', 1.0)
        self.declare_parameter('wuurc_aruco_timeout_s', 12.0)
        self.declare_parameter('wuurc_led_duration_s', 3.0)

        # The command topic avoids a dependency on the placeholder hardware
        # manager.  These values are intentionally configurable to match the
        # STM32 command table when it becomes available.
        self.declare_parameter('wuurc_sample_command', self.CMD_SAMPLE_WATER)
        self.declare_parameter('wuurc_drop_command', self.CMD_DROP_BEACON)

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
            'sync_ins_pose': self._task_sync_ins_pose,
            'align_downward': self._task_align_downward,
            'surface_attempt': self._task_surface_attempt,
            'recognize_aruco': self._task_recognize_aruco,
            'select_aruco_sector': self._task_select_aruco_sector,
            'show_selected_sector_led': self._task_show_selected_sector_led,
            'drop_beacon': self._task_drop_beacon,
            'take_water_sample': self._task_take_water_sample,
        }

        # Action client
        self._action_client = ActionClient(self, BasicMotion, 'basic_motion')
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('BasicMotion action server not available!')

        # Subscribers
        self.create_subscription(
            ObjectPositionArray, '/perception/objects', self._objects_cb, 10)
        for camera_name in (
                'front_left', 'front_right', 'down_left', 'down_right'):
            self.create_subscription(
                DetectionArray,
                f'/perception/detection/{camera_name}',
                lambda msg, name=camera_name: self._detections_cb(name, msg),
                10)
        # ArUco IDs are not part of DetectionArray.  We use the same camera
        # stream consumed by vision.py, but decode the 4x4_1000 ID locally.
        self.create_subscription(
            Image, '/auv/front_cam/stitched', self._front_stitched_cb, 2)
        self.create_subscription(
            PoseInfo, '/basic_motion/pose_info', self._pose_info_cb, 10)

        # Publishers
        self.pub_status = self.create_publisher(TaskStatus, '/task/status', 10)
        self.pub_device_command = self.create_publisher(
            DeviceCommand, '/task/device_command', 10)

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

    def _detections_cb(self, camera_name: str, msg: DetectionArray):
        """Cache the newest vision.py output for low-latency task decisions."""
        with self._perception_lock:
            self._detections[camera_name] = (time.monotonic(), msg)

    def _front_stitched_cb(self, msg: Image):
        """Cache the raw front image used only for ArUco ID decoding."""
        with self._perception_lock:
            self._front_stitched_image = msg
            self._front_stitched_received_at = time.monotonic()

    def _pose_info_cb(self, msg: PoseInfo):
        """Cache the INS pose published by basic_motion (not position.py)."""
        with self._perception_lock:
            self._pose_info = msg
            self._pose_info_received_at = time.monotonic()

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
                    if params.get('stop_on_failure', False):
                        self.stopped = True
                        self.get_logger().error(
                            f'[{self.current_index + 1}/{total}] {name} failure '
                            'requested task-list stop')
            except Exception as e:
                self.get_logger().error(
                    f'[{self.current_index + 1}/{total}] {name} exception: {e}')
                if params.get('stop_on_failure', False):
                    self.stopped = True
                    self.get_logger().error(
                        f'[{self.current_index + 1}/{total}] {name} exception '
                        'requested task-list stop')

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
            self.get_logger().error('Goal rejected by server')
            return False, 'Goal rejected by server'

        if not quiet:
            self.get_logger().info('Goal accepted, waiting for result...')

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
            self.get_logger().info('Goal result: SUCCESS')
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
        dx = p['x'] - self._cmd_x
        success, msg = self._send_action_goal(
            BasicMotion.Goal.WMOVE, [dx, 0.0, 0.0, 0.0], "x")
        self.get_logger().info(f'wmovex: {msg}')
        if success:
            self._cmd_x = p['x']
        return success

    def _task_wmovey(self, p: dict) -> bool:
        dy = p['y'] - self._cmd_y
        success, msg = self._send_action_goal(
            BasicMotion.Goal.WMOVE, [0.0, dy, 0.0, 0.0], "y")
        self.get_logger().info(f'wmovey: {msg}')
        if success:
            self._cmd_y = p['y']
        return success

    def _task_wmovez(self, p: dict) -> bool:
        dz = p['z'] - self._cmd_z
        success, msg = self._send_action_goal(
            BasicMotion.Goal.WMOVE, [0.0, 0.0, dz, 0.0], "z")
        self.get_logger().info(f'wmovez: {msg}')
        if success:
            self._cmd_z = p['z']
        return success

    def _task_wmoverz(self, p: dict) -> bool:
        drz = p['rz'] - self._cmd_yaw
        success, msg = self._send_action_goal(
            BasicMotion.Goal.WMOVE, [0.0, 0.0, 0.0, drz], "rz")
        self.get_logger().info(f'wmoverz: {msg}')
        if success:
            self._cmd_yaw = p['rz']
        return success

    def _task_wmovexy(self, p: dict) -> bool:
        dx, dy = p['x'] - self._cmd_x, p['y'] - self._cmd_y
        success, msg = self._send_action_goal(
            BasicMotion.Goal.WMOVE, [dx, dy, 0.0, 0.0], "xy")
        self.get_logger().info(f'wmovexy: {msg}')
        if success:
            self._cmd_x, self._cmd_y = p['x'], p['y']
        return success

    def _task_wmovexyz(self, p: dict) -> bool:
        dx, dy, dz = p['x'] - self._cmd_x, p['y'] - self._cmd_y, p['z'] - self._cmd_z
        success, msg = self._send_action_goal(
            BasicMotion.Goal.WMOVE, [dx, dy, dz, 0.0], "xyz")
        self.get_logger().info(f'wmovexyz: {msg}')
        if success:
            self._cmd_x, self._cmd_y, self._cmd_z = p['x'], p['y'], p['z']
        return success

    # --- WTRAVEL tasks (世界系直线：参数为绝对世界坐标，内部计算偏移) ---

    def _task_wtravelx(self, p: dict) -> bool:
        dx = p['x'] - self._cmd_x
        success, msg = self._send_action_goal(
            BasicMotion.Goal.WTRAVEL, [dx, 0.0, 0.0, 0.0], "x")
        self.get_logger().info(f'wtravelx: {msg}')
        if success:
            self._cmd_x = p['x']
        return success

    def _task_wtravely(self, p: dict) -> bool:
        dy = p['y'] - self._cmd_y
        success, msg = self._send_action_goal(
            BasicMotion.Goal.WTRAVEL, [0.0, dy, 0.0, 0.0], "y")
        self.get_logger().info(f'wtravely: {msg}')
        if success:
            self._cmd_y = p['y']
        return success

    def _task_wtravelz(self, p: dict) -> bool:
        dz = p['z'] - self._cmd_z
        success, msg = self._send_action_goal(
            BasicMotion.Goal.WTRAVEL, [0.0, 0.0, dz, 0.0], "z")
        self.get_logger().info(f'wtravelz: {msg}')
        if success:
            self._cmd_z = p['z']
        return success

    def _resolve_wtravelxy_target(self, p: dict) -> tuple[float, float]:
        """Resolve a literal target or the sector chosen from an ArUco ID."""
        if p.get('target') != 'selected_sector':
            return float(p['x']), float(p['y'])
        if self._selected_sector is None:
            raise ValueError('No selected sector; run select_aruco_sector first')
        return self._selected_sector['x'], self._selected_sector['y']

    def _task_wtravelxy(self, p: dict) -> bool:
        x, y = self._resolve_wtravelxy_target(p)
        dx, dy = x - self._cmd_x, y - self._cmd_y
        success, msg = self._send_action_goal(
            BasicMotion.Goal.WTRAVEL, [dx, dy, 0.0, 0.0], "xy")
        self.get_logger().info(f'wtravelxy: {msg}')
        if success:
            self._cmd_x, self._cmd_y = x, y
        return success

    def _task_wtravelxyz(self, p: dict) -> bool:
        dx = p['x'] - self._cmd_x
        dy = p['y'] - self._cmd_y
        dz = p['z'] - self._cmd_z
        success, msg = self._send_action_goal(
            BasicMotion.Goal.WTRAVEL, [dx, dy, dz, 0.0], "xyz")
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

    # --- WUURC 2026 AUV mission, starting at the pipeline basket ---

    def _wuurc_param(self, name: str, overrides: dict):
        """Return a task override, otherwise the declared ROS parameter."""
        return overrides.get(name, self.get_parameter(f'wuurc_{name}').value)

    def _sync_command_pose_from_ins(self, max_age_s: float = 1.0) -> bool:
        """Rebase the task command tracker on current BasicMotion INS pose.

        LineFollower can receive a successful small action while the vehicle is
        physically constrained.  Its commanded-pose integration then differs
        from the real vehicle pose.  Before an absolute field transfer, use
        ``PoseInfo`` from basic_motion to prevent a stale large delta.
        """
        with self._perception_lock:
            pose = self._pose_info
            received_at = self._pose_info_received_at
        if pose is None or time.monotonic() - received_at > max_age_s:
            self.get_logger().error(
                'WUURC cannot synchronize command pose: '
                'basic_motion/pose_info is missing or stale')
            return False
        self._cmd_x = float(pose.robot_x)
        self._cmd_y = float(pose.robot_y)
        self._cmd_z = float(pose.robot_z)
        self._cmd_yaw = float(pose.robot_yaw)
        self.get_logger().info(
            f'WUURC INS handoff pose: '
            f'({self._cmd_x:.2f}, {self._cmd_y:.2f}, '
            f'{self._cmd_z:.2f}, {self._cmd_yaw:.1f}°)')
        return True

    def _latest_detection(self, class_id: int, camera_name: str,
                          min_confidence: float, max_age_s: float):
        """Return the freshest, highest-confidence detection of one class."""
        with self._perception_lock:
            cached = self._detections.get(camera_name)
        if cached is None:
            return None
        received_at, array = cached
        if time.monotonic() - received_at > max_age_s:
            return None
        candidates = [
            det for det in array.detections
            if det.class_id == class_id and det.confidence >= min_confidence
        ]
        return max(candidates, key=lambda det: det.confidence) if candidates else None

    def _apply_body_delta(self, dx: float, dy: float, dz: float = 0.0,
                          dyaw: float = 0.0):
        """Update the local command tracker after a successful FRD BMOVE."""
        yaw = math.radians(self._cmd_yaw)
        self._cmd_x += math.cos(yaw) * dx - math.sin(yaw) * dy
        self._cmd_y += math.sin(yaw) * dx + math.cos(yaw) * dy
        self._cmd_z += dz
        self._cmd_yaw = (self._cmd_yaw + dyaw + 180.0) % 360.0 - 180.0

    def _align_downward(self, class_id: int, label: str,
                        overrides: dict) -> bool:
        """Centre a YOLO box with one down-facing camera and FRD micro-steps.

        This deliberately uses ``vision.py``'s ``down_left`` DetectionArray,
        not the unreliable PositionNode.  Horizontal image error is corrected
        through body-Y and vertical image error through body-X.  The two signs
        are parameters because camera mounting orientation differs between the
        simulator and the vehicle after commissioning.
        """
        timeout_s = float(self._wuurc_param('align_timeout_s', overrides))
        deadline = time.monotonic() + timeout_s
        confidence = float(self._wuurc_param('align_confidence', overrides))
        max_age = float(self._wuurc_param('detection_max_age_s', overrides))
        tolerance = float(self._wuurc_param('align_pixel_tolerance', overrides))
        max_step = float(self._wuurc_param('align_max_step_m', overrides))
        image_x = float(self.get_parameter('down_image_width').value) / 2.0
        image_y = float(self.get_parameter('down_image_height').value) / 2.0

        while time.monotonic() < deadline and not self.stopped:
            det = self._latest_detection(
                class_id, 'down_left', confidence, max_age)
            if det is None:
                time.sleep(0.05)
                continue

            error_x = det.pixel_x - image_x
            error_y = det.pixel_y - image_y
            if abs(error_x) <= tolerance and abs(error_y) <= tolerance:
                self.get_logger().info(
                    f'WUURC down alignment complete: {label}, '
                    f'pixel_error=({error_x:.1f}, {error_y:.1f})')
                return True

            body_x = float(self._wuurc_param('align_body_x_sign', overrides))
            body_x *= error_y * float(self._wuurc_param(
                'align_m_per_pixel_y', overrides))
            body_y = float(self._wuurc_param('align_body_y_sign', overrides))
            body_y *= error_x * float(self._wuurc_param(
                'align_m_per_pixel_x', overrides))
            body_x = max(-max_step, min(max_step, body_x))
            body_y = max(-max_step, min(max_step, body_y))

            success, message = self._send_action_goal(
                BasicMotion.Goal.BMOVE, [body_x, body_y, 0.0, 0.0], 'xy',
                timeout=8.0, quiet=True)
            if not success:
                self.get_logger().warn(
                    f'WUURC down alignment failed for {label}: {message}')
                return False
            self._apply_body_delta(body_x, body_y)

        self.get_logger().warn(f'WUURC down alignment timeout: {label}')
        return False

    def _try_surface_and_restore(self, overrides: dict,
                                 restore: bool = True) -> bool:
        """Attempt the required 0.5 m ascent, safely restoring depth on failure.

        The current simulation vehicle may reject a complete surfacing target.
        A failed action is non-fatal: it restores the commanded depth and lets
        the remaining scoring tasks continue.  During task acquisition, a
        successful ascent is also restored because ArUco recognition happens
        after re-submerging.
        """
        original_depth = self._cmd_z
        ascent = abs(float(self._wuurc_param('surface_delta_z', overrides)))
        success, message = self._send_action_goal(
            BasicMotion.Goal.BMOVE, [0.0, 0.0, -ascent, 0.0], 'z',
            timeout=20.0)
        if not success:
            self.get_logger().warn(
                f'WUURC surface attempt unavailable: {message}; '
                f'restoring z={original_depth:.2f}')
            self._task_setz({'z': original_depth})
            return False

        self._cmd_z -= ascent
        time.sleep(float(self._wuurc_param('surface_dwell_s', overrides)))
        if restore:
            if not self._task_setz({'z': original_depth}):
                self.get_logger().error('WUURC failed to re-submerge after surface')
                return False
        return True

    def _publish_device_command(self, device_type: int, command: int,
                                value: float, description: str):
        """Publish one non-blocking hardware command without using hw_manager."""
        msg = DeviceCommand()
        msg.device_type = device_type
        msg.command = int(command)
        msg.value = float(value)
        self.pub_device_command.publish(msg)
        self.get_logger().info(
            f'WUURC device command: {description} '
            f'(type={device_type}, command={command}, value={value:.1f})')

    def _decode_aruco_id(self, max_age_s: float):
        """Decode one 4x4_1000 ArUco ID from the current front-left image.

        ``vision.py`` already confirms the board as YOLO class 8.  Its message
        intentionally has no marker-ID field, so the ID is decoded here from
        the original stitched stream without changing the perception package.
        """
        with self._perception_lock:
            image = self._front_stitched_image
            received_at = self._front_stitched_received_at
        if image is None or time.monotonic() - received_at > max_age_s:
            return None
        if image.encoding not in ('bgr8', 'rgb8') or image.width < 2:
            return None

        try:
            import cv2
            import numpy as np

            expected_row_bytes = image.width * 3
            if image.step < expected_row_bytes or len(image.data) < image.height * image.step:
                return None
            frame = np.frombuffer(image.data, dtype=np.uint8).reshape(
                image.height, image.step)
            frame = frame[:, :expected_row_bytes].reshape(
                image.height, image.width, 3)
            color_code = (
                cv2.COLOR_RGB2GRAY if image.encoding == 'rgb8'
                else cv2.COLOR_BGR2GRAY)
            dictionary = cv2.aruco.getPredefinedDictionary(
                cv2.aruco.DICT_4X4_1000)
            detector_params = cv2.aruco.DetectorParameters()
            # The simulated marker can be small in the front image.  Broaden
            # the perimeter range while retaining the standard 4x4 dictionary.
            detector_params.minMarkerPerimeterRate = 0.01
            detector_params.maxMarkerPerimeterRate = 4.0
            detector_params.adaptiveThreshWinSizeMin = 3
            detector_params.adaptiveThreshWinSizeMax = 51
            detector_params.adaptiveThreshWinSizeStep = 4
            detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            detector_params.detectInvertedMarker = True
            if hasattr(cv2.aruco, 'ArucoDetector'):
                detector = cv2.aruco.ArucoDetector(
                    dictionary, detector_params)

                def detect(gray):
                    return detector.detectMarkers(gray)
            else:
                def detect(gray):
                    return cv2.aruco.detectMarkers(
                        gray, dictionary, parameters=detector_params)

            def valid_ids(gray):
                """Run full-frame image preparations and return a task ID."""
                if min(gray.shape[:2]) < 8:
                    return None
                # The Stonefish texture fills the PNG and has no quiet-zone.
                # Add contrasting margins, then enlarge tiny camera markers.
                pad = max(8, int(min(gray.shape[:2]) * 0.08))
                padded = [
                    cv2.copyMakeBorder(
                        gray, pad, pad, pad, pad,
                        cv2.BORDER_CONSTANT, value=255),
                    cv2.copyMakeBorder(
                        gray, pad, pad, pad, pad,
                        cv2.BORDER_CONSTANT, value=0),
                ]
                images = [gray] + padded
                for source in padded:
                    images.append(cv2.resize(
                        source, None, fx=4.0, fy=4.0,
                        interpolation=cv2.INTER_CUBIC))
                    images.append(cv2.resize(
                        source, None, fx=8.0, fy=8.0,
                        interpolation=cv2.INTER_NEAREST))
                clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
                images.append(clahe.apply(gray))
                if min(gray.shape[:2]) >= 11:
                    images.append(cv2.adaptiveThreshold(
                        gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                        cv2.THRESH_BINARY, 11, 2))
                for candidate in images:
                    _corners, ids, _rejected = detect(candidate)
                    if ids is None:
                        continue
                    for marker in ids.reshape(-1):
                        marker_id = int(marker)
                        if marker_id in (1, 2, 3, 4, 5, 6):
                            return marker_id
                return None

            # The simulator publishes a left/right stitched image.  ArUco is
            # deliberately decoded from each complete half-frame; YOLO boxes
            # and position.py estimates are not used by this path.
            midpoint = image.width // 2
            for half in (frame[:, :midpoint], frame[:, midpoint:]):
                gray = cv2.cvtColor(half, color_code)
                marker_id = valid_ids(gray)
                if marker_id is not None:
                    return marker_id
                # Water lighting can make luminance grayscale low contrast;
                # retry the complete frame through individual B/G/R channels.
                for channel in cv2.split(half):
                    marker_id = valid_ids(channel)
                    if marker_id is not None:
                        return marker_id
            return None
        except Exception as exc:
            self.get_logger().warn(f'WUURC ArUco decoder unavailable: {exc}')
            return None

    def _recognize_aruco(self, overrides: dict):
        """Wait for a valid task ID (1..6) at the measured observation point."""
        timeout_s = float(self._wuurc_param('aruco_timeout_s', overrides))
        max_age_s = float(self._wuurc_param('detection_max_age_s', overrides))
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not self.stopped:
            marker_id = self._decode_aruco_id(max_age_s)
            if marker_id in (1, 2, 3, 4, 5, 6):
                self.get_logger().info(f'WUURC ArUco task received: id={marker_id}')
                return marker_id
            time.sleep(0.05)
        self.get_logger().error('WUURC failed to recognize ArUco ID 1..6')
        return None

    def _sector_for_aruco(self, marker_id: int):
        """Map regulation IDs 1..6 to a scene sector, class, and LED code."""
        if marker_id in (1, 2):
            return {
                'name': 'yellow_sector', 'x': 1.75, 'y': -2.80,
                'class_id': self.CLASS_YELLOW_SECTOR,
                'led_command': self.CMD_LED_YELLOW,
            }
        if marker_id in (3, 4):
            return {
                'name': 'green_sector', 'x': 3.50, 'y': -2.80,
                'class_id': self.CLASS_GREEN_SECTOR,
                'led_command': self.CMD_LED_GREEN,
            }
        if marker_id in (5, 6):
            return {
                'name': 'red_sector', 'x': 5.25, 'y': -2.80,
                'class_id': self.CLASS_RED_SECTOR,
                'led_command': self.CMD_LED_RED,
            }
        return None

    def _task_sync_ins_pose(self, p: dict) -> bool:
        """Synchronize the command tracker before a JSON world move."""
        max_age = float(self._wuurc_param('pose_max_age_s', p))
        return self._sync_command_pose_from_ins(max_age)

    def _task_align_downward(self, p: dict) -> bool:
        """Centre a literal YOLO class or the class of the selected sector."""
        if p.get('target') == 'selected_sector':
            if self._selected_sector is None:
                self.get_logger().error(
                    'Downward alignment skipped: no selected sector')
                return False
            return self._align_downward(
                self._selected_sector['class_id'],
                self._selected_sector['name'], p)
        class_id = int(p['class_id'])
        return self._align_downward(class_id, p.get('label', str(class_id)), p)

    def _task_surface_attempt(self, p: dict) -> bool:
        """Attempt a configured ascent, restoring depth unless requested otherwise."""
        return self._try_surface_and_restore(
            p, restore=bool(p.get('restore', True)))

    def _task_recognize_aruco(self, p: dict) -> bool:
        """Decode and retain the current 4x4_1000 ArUco task ID."""
        marker_id = self._recognize_aruco(p)
        fallback_id = p.get('fallback_id')
        if marker_id is None and fallback_id in (1, 2, 3, 4, 5, 6):
            marker_id = int(fallback_id)
            self.get_logger().warn(
                f'WUURC ArUco decode timed out; using explicit configured '
                f'fallback ID {marker_id}')
        self._aruco_marker_id = marker_id
        self._selected_sector = None
        return marker_id is not None

    def _task_select_aruco_sector(self, p: dict) -> bool:
        """Resolve the retained ArUco ID into a dynamic WTRAVELXY target."""
        sector = self._sector_for_aruco(self._aruco_marker_id)
        if sector is None:
            self.get_logger().error(
                'Sector selection failed: no valid ArUco ID')
            return False
        self._selected_sector = sector
        self.get_logger().info(
            f'Selected {sector["name"]} for ArUco {self._aruco_marker_id}')
        return True

    def _task_show_selected_sector_led(self, p: dict) -> bool:
        """Display the colour corresponding to the selected ArUco sector."""
        if self._selected_sector is None or self._aruco_marker_id is None:
            self.get_logger().error('Sector LED skipped: no selected sector')
            return False
        self._publish_device_command(
            DeviceCommand.DEVICE_LED,
            self._selected_sector['led_command'], float(self._aruco_marker_id),
            f'show task sector for ArUco {self._aruco_marker_id}')
        time.sleep(float(self._wuurc_param('led_duration_s', p)))
        self._publish_device_command(
            DeviceCommand.DEVICE_LED, self.CMD_LED_OFF, 0.0, 'turn LED off')
        return True

    def _task_drop_beacon(self, p: dict) -> bool:
        """Publish exactly one beacon drop command after sector alignment."""
        if self._selected_sector is None or self._aruco_marker_id is None:
            self.get_logger().error('Beacon drop skipped: no selected sector')
            return False
        self._publish_device_command(
            DeviceCommand.DEVICE_SERVO,
            int(self._wuurc_param('drop_command', p)),
            float(self._aruco_marker_id),
            f'drop beacon into {self._selected_sector["name"]}')
        return True

    def _task_take_water_sample(self, p: dict) -> bool:
        """Publish the one lower-computer command for environmental sampling."""
        self._publish_device_command(
            DeviceCommand.DEVICE_ARM,
            int(self._wuurc_param('sample_command', p)), 1.0,
            'take water sample')
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
            self._action_client.async_cancel_goal(
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
