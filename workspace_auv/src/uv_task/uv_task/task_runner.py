"""Task runner node: loads task list from JSON, executes tasks sequentially.

Each task unit calls navigation or basic_motion functions.
"""

import json
import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from uv_msgs.msg import AuvState, ObjectPositionArray, TaskStatus, MotionCommand
from uv_msgs.srv import RunTask


class TaskRunnerNode(Node):
    """Task runner: JSON task loader and sequential executor."""

    def __init__(self):
        super().__init__('task_runner')

        self.state = AuvState()
        self.objects = ObjectPositionArray()
        self._state_lock = threading.Lock()

        self.tasks = []
        self.current_index = 0
        self.running = False
        self.stopped = False

        # Publisher
        self.pub_motion = self.create_publisher(MotionCommand, '/auv/cmd/motion', 10)
        self.pub_status = self.create_publisher(TaskStatus, '/task/status', 10)

        # Subscribers
        self.create_subscription(AuvState, '/auv/state', self._state_cb, 10)
        self.create_subscription(ObjectPositionArray, '/perception/objects', self._objects_cb, 10)

        # Services
        self.create_service(RunTask, '/task/run', self._run_task_cb)
        self.create_service(Trigger, '/task/stop', self._stop_task_cb)

        # Status timer
        self.create_timer(0.5, self._publish_status)

        self.get_logger().info('TaskRunner node started')

    def _state_cb(self, msg: AuvState):
        with self._state_lock:
            self.state = msg

    def _objects_cb(self, msg: ObjectPositionArray):
        self.objects = msg

    def get_state(self) -> AuvState:
        with self._state_lock:
            return self.state

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

        while self.current_index < len(self.tasks) and not self.stopped:
            task = self.tasks[self.current_index]
            name = task.get('name', 'unknown')
            params = task.get('params', {})

            self.get_logger().info(
                f'Task {self.current_index + 1}/{len(self.tasks)}: {name} {params}'
            )

            try:
                success = self._execute_task(name, params)
                if not success:
                    self.get_logger().warn(f'Task {name} returned failure')
            except Exception as e:
                self.get_logger().error(f'Task {name} exception: {e}')

            self.current_index += 1

        self.running = False
        self.get_logger().info('Task list completed')

    def _execute_task(self, name: str, params: dict) -> bool:
        """Execute a single task by name."""
        task_map = {
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
            'navigate': self._task_navigate,
            'wait': self._task_wait,
        }

        handler = task_map.get(name)
        if handler is None:
            self.get_logger().warn(f'Unknown task: {name}')
            return False

        return handler(params)

    # ========================================================================
    # Task implementations
    # ========================================================================

    def _send_motion(self, mode: int, x: float, y: float, z: float, yaw: float):
        msg = MotionCommand()
        msg.mode = mode
        msg.type_mask = 0
        msg.x = x
        msg.y = y
        msg.z = z
        msg.yaw = yaw
        self.pub_motion.publish(msg)

    def _wait_reached(self, axes_mask: int = 0x0F, timeout: float = 60.0,
                      tol_x: float = 0.1, tol_y: float = 0.1,
                      tol_z: float = 0.1, tol_rz: float = 5.0) -> bool:
        """Wait until position reached."""
        rate = self.create_rate(10)
        start = self.get_clock().now()

        while rclpy.ok() and not self.stopped:
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > timeout:
                return False

            s = self.get_state()
            reached = True

            if axes_mask & 0x01 and abs(s.target_x - s.pos_x) > tol_x:
                reached = False
            if axes_mask & 0x02 and abs(s.target_y - s.pos_y) > tol_y:
                reached = False
            if axes_mask & 0x04 and abs(s.target_z - s.pos_z) > tol_z:
                reached = False
            if axes_mask & 0x08:
                yaw_err = abs(math.degrees(s.target_yaw - s.yaw))
                if yaw_err > 180:
                    yaw_err = 360 - yaw_err
                if yaw_err > tol_rz:
                    reached = False

            if reached:
                return True

            rate.sleep()

        return False

    # --- SET tasks ---

    def _task_setx(self, p: dict) -> bool:
        s = self.get_state()
        self._send_motion(MotionCommand.MODE_POS_WORLD, p['x'], s.pos_y, s.pos_z, s.yaw)
        return self._wait_reached(0x01)

    def _task_sety(self, p: dict) -> bool:
        s = self.get_state()
        self._send_motion(MotionCommand.MODE_POS_WORLD, s.pos_x, p['y'], s.pos_z, s.yaw)
        return self._wait_reached(0x02)

    def _task_setz(self, p: dict) -> bool:
        s = self.get_state()
        self._send_motion(MotionCommand.MODE_POS_WORLD, s.pos_x, s.pos_y, p['z'], s.yaw)
        return self._wait_reached(0x04)

    def _task_setrz(self, p: dict) -> bool:
        s = self.get_state()
        rz_rad = math.radians(p['rz'])
        self._send_motion(MotionCommand.MODE_POS_WORLD, s.pos_x, s.pos_y, s.pos_z, rz_rad)
        return self._wait_reached(0x08)

    def _task_setxy(self, p: dict) -> bool:
        s = self.get_state()
        self._send_motion(MotionCommand.MODE_POS_WORLD, p['x'], p['y'], s.pos_z, s.yaw)
        return self._wait_reached(0x03)

    def _task_setxyz(self, p: dict) -> bool:
        s = self.get_state()
        self._send_motion(MotionCommand.MODE_POS_WORLD, p['x'], p['y'], p['z'], s.yaw)
        return self._wait_reached(0x07)

    def _task_setxyzrz(self, p: dict) -> bool:
        rz_rad = math.radians(p.get('rz', 0.0))
        self._send_motion(MotionCommand.MODE_POS_WORLD, p['x'], p['y'], p['z'], rz_rad)
        return self._wait_reached(0x0F)

    def _task_setxyrz(self, p: dict) -> bool:
        s = self.get_state()
        rz_rad = math.radians(p.get('rz', 0.0))
        self._send_motion(MotionCommand.MODE_POS_WORLD, p['x'], p['y'], s.pos_z, rz_rad)
        return self._wait_reached(0x0B)

    # --- BMOVE tasks (body frame stepping) ---

    def _task_bmovex(self, p: dict) -> bool:
        s = self.get_state()
        yaw = s.yaw
        dx_w = p['dx'] * math.cos(yaw)
        dy_w = p['dx'] * math.sin(yaw)
        self._send_motion(MotionCommand.MODE_POS_WORLD,
                          s.pos_x + dx_w, s.pos_y + dy_w, s.pos_z, s.yaw)
        return self._wait_reached(0x03)

    def _task_bmovey(self, p: dict) -> bool:
        s = self.get_state()
        yaw = s.yaw
        dx_w = -p['dy'] * math.sin(yaw)
        dy_w = p['dy'] * math.cos(yaw)
        self._send_motion(MotionCommand.MODE_POS_WORLD,
                          s.pos_x + dx_w, s.pos_y + dy_w, s.pos_z, s.yaw)
        return self._wait_reached(0x03)

    def _task_bmovez(self, p: dict) -> bool:
        s = self.get_state()
        self._send_motion(MotionCommand.MODE_POS_WORLD,
                          s.pos_x, s.pos_y, s.pos_z + p['dz'], s.yaw)
        return self._wait_reached(0x04)

    def _task_bmoverz(self, p: dict) -> bool:
        s = self.get_state()
        rz_rad = s.yaw + math.radians(p['drz'])
        self._send_motion(MotionCommand.MODE_POS_WORLD,
                          s.pos_x, s.pos_y, s.pos_z, rz_rad)
        return self._wait_reached(0x08)

    def _task_bmovexy(self, p: dict) -> bool:
        s = self.get_state()
        yaw = s.yaw
        cy, sy = math.cos(yaw), math.sin(yaw)
        dx_w = cy * p['dx'] - sy * p['dy']
        dy_w = sy * p['dx'] + cy * p['dy']
        self._send_motion(MotionCommand.MODE_POS_WORLD,
                          s.pos_x + dx_w, s.pos_y + dy_w, s.pos_z, s.yaw)
        return self._wait_reached(0x03)

    def _task_bmovexyz(self, p: dict) -> bool:
        s = self.get_state()
        yaw = s.yaw
        cy, sy = math.cos(yaw), math.sin(yaw)
        dx_w = cy * p['dx'] - sy * p['dy']
        dy_w = sy * p['dx'] + cy * p['dy']
        self._send_motion(MotionCommand.MODE_POS_WORLD,
                          s.pos_x + dx_w, s.pos_y + dy_w, s.pos_z + p['dz'], s.yaw)
        return self._wait_reached(0x07)

    # --- WMOVE tasks (world frame stepping) ---

    def _task_wmovex(self, p: dict) -> bool:
        s = self.get_state()
        self._send_motion(MotionCommand.MODE_POS_WORLD,
                          s.pos_x + p['dx'], s.pos_y, s.pos_z, s.yaw)
        return self._wait_reached(0x01)

    def _task_wmovey(self, p: dict) -> bool:
        s = self.get_state()
        self._send_motion(MotionCommand.MODE_POS_WORLD,
                          s.pos_x, s.pos_y + p['dy'], s.pos_z, s.yaw)
        return self._wait_reached(0x02)

    def _task_wmovez(self, p: dict) -> bool:
        s = self.get_state()
        self._send_motion(MotionCommand.MODE_POS_WORLD,
                          s.pos_x, s.pos_y, s.pos_z + p['dz'], s.yaw)
        return self._wait_reached(0x04)

    def _task_wmoverz(self, p: dict) -> bool:
        s = self.get_state()
        rz_rad = s.yaw + math.radians(p['drz'])
        self._send_motion(MotionCommand.MODE_POS_WORLD,
                          s.pos_x, s.pos_y, s.pos_z, rz_rad)
        return self._wait_reached(0x08)

    def _task_wmovexy(self, p: dict) -> bool:
        s = self.get_state()
        self._send_motion(MotionCommand.MODE_POS_WORLD,
                          s.pos_x + p['dx'], s.pos_y + p['dy'], s.pos_z, s.yaw)
        return self._wait_reached(0x03)

    def _task_wmovexyz(self, p: dict) -> bool:
        s = self.get_state()
        self._send_motion(MotionCommand.MODE_POS_WORLD,
                          s.pos_x + p['dx'], s.pos_y + p['dy'], s.pos_z + p['dz'], s.yaw)
        return self._wait_reached(0x07)

    # --- Special tasks ---

    def _task_navigate(self, p: dict) -> bool:
        """Navigate to target using simple direct movement (A* would need navigator node)."""
        s = self.get_state()
        goal_x = p.get('x', s.pos_x)
        goal_y = p.get('y', s.pos_y)
        goal_z = p.get('z', s.pos_z)
        goal_rz = p.get('rz', math.degrees(s.yaw))
        rz_rad = math.radians(goal_rz)
        self._send_motion(MotionCommand.MODE_POS_WORLD, goal_x, goal_y, goal_z, rz_rad)
        return self._wait_reached(0x0F, timeout=120.0)

    def _task_wait(self, p: dict) -> bool:
        """Wait for specified duration."""
        duration = p.get('duration', 1.0)
        time.sleep(duration)
        return True

    # ========================================================================
    # Service handlers
    # ========================================================================

    def _run_task_cb(self, request, response):
        if request.start:
            path = request.task_name
            if not os.path.isabs(path):
                # Try default config path
                default = os.path.join(
                    os.path.dirname(__file__), '..', 'config', 'tasks.json'
                )
                if os.path.exists(default):
                    path = default

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
            self.stopped = True
            response.success = True
            response.message = 'Stopped'
        return response

    def _stop_task_cb(self, request, response):
        self.stopped = True
        response.success = True
        response.message = 'Tasks stopped'
        return response

    # ========================================================================
    # Status publishing
    # ========================================================================

    def _publish_status(self):
        msg = TaskStatus()
        if self.running:
            msg.status = TaskStatus.STATUS_RUNNING
        elif self.stopped:
            msg.status = TaskStatus.STATUS_PAUSED
        else:
            msg.status = TaskStatus.STATUS_IDLE

        msg.current_task_index = self.current_index
        msg.total_tasks = len(self.tasks)
        if self.current_index < len(self.tasks):
            msg.current_task_name = self.tasks[self.current_index].get('name', '')
        msg.error_message = ''
        self.pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TaskRunnerNode()

    # Load default tasks if available
    default_path = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'tasks.json'
    )
    if os.path.exists(default_path):
        node.tasks = node.load_tasks(default_path)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
