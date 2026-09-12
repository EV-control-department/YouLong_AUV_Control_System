"""Task runner node: loads YAML missions or standalone tasks sequentially.

Each task calls basic_motion via the BasicMotion action server.
The task runner is the single source of truth for commanded position,
tracked locally (not from external topics).
"""

from __future__ import annotations

from importlib import import_module
import json
import math
import os
from pathlib import Path
import threading
import time

import numpy as np
import rclpy
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
from uv_task.config_loader import (
    ConfigError,
    default_mission_path,
    load_mission_or_task,
)

from uv_task.arrow_surfacer import (
    _DOWN_CX, _DOWN_CY, _DOWN_FX, _DOWN_FY,
    _DOWN_OFFSET_LEFT, _DOWN_OFFSET_RIGHT, _DOWN_OPTICAL_TO_BODY,
    _euler_to_rotation_matrix, _ray_intersection_midpoint,
)
from uv_task.arrow_surfacer import ArrowSurfacer
# The competition task module names intentionally start with ``26rb_``.
# Such names cannot be used in a normal ``from package import module``
# statement, so load them through importlib.
RB26GrabBallTask = import_module('uv_task.26rb_grab_ball').RB26GrabBallTask
RB26GateTask = import_module('uv_task.26rb_gate_task').RB26GateTask
RB26HitBallsTask = import_module('uv_task.26rb_hit_balls').RB26HitBallsTask
RB26FindCollectionFrameTask = import_module(
    'uv_task.26rb_find_collection_frame').RB26FindCollectionFrameTask
RB26DropBeaconTask = import_module(
    'uv_task.26rb_drop_beacon').RB26DropBeaconTask
from uv_task.line_follower import LineFollower
from uv_camera.model_classes import model_class_id


_IMPACT_BALL_CLASS_IDS = {
    'impact_ball_blue': model_class_id('impact_ball_blue'),
    'impact_ball_red': model_class_id('impact_ball_red'),
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

# object_localizer.py publishes these as canonical physical classes on
# /perception/target_positions, while class_name can still contain the
# detector suffix (for example ``collection_frame_front``).  Keep the task
# tolerant of both forms because the localizer deliberately publishes front
# and down estimates separately.
_LOCALIZER_TARGET_CLASS_IDS = {
    model_class_id('collection_frame_down'): 'collection_frame',
    model_class_id('collection_frame_front'): 'collection_frame',
    model_class_id('target_rack_down'): 'target_rack',
    model_class_id('target_rack_front'): 'target_rack',
}
_TARGET_RACK_DOWN_CLASS_ID = model_class_id('target_rack_down')
_LOCALIZER_TARGET_ALIASES = {
    'collection_frame': 'collection_frame',
    'collection': 'collection_frame',
    'collection_platform': 'collection_frame',
    'platform': 'collection_frame',
    'placement_platform': 'collection_frame',
    '置物台': 'collection_frame',
    '置舞台': 'collection_frame',
    'target_rack': 'target_rack',
    'targetrack': 'target_rack',
    'rack': 'target_rack',
    'target': 'target_rack',
    'target_rack_platform': 'target_rack',
    'target-rack': 'target_rack',
    '货架': 'target_rack',
    '目标架': 'target_rack',
}


class TaskRunnerNode(Node):
    """Task runner: YAML mission loader and sequential executor."""

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
        self.declare_parameter('mission_file', '')
        self.mission_file = self.get_parameter(
            'mission_file').get_parameter_value().string_value
        self._debug_task_name = None
        self._debug_executing = False
        self._debug_timeout = -1.0

        # Competition metadata only.  The current cruise environment keeps all
        # targets visible; this value tells task logic which one is correct and
        # intentionally does not create scoring or grasping behaviour.
        self.declare_parameter('target_id', 'yellow_golf')
        self.target_id = self.get_parameter('target_id').get_parameter_value().string_value
        valid_target_ids = {
            name for name in ('yellow_golf', 'pink_golf', 'red_ring')
            if model_class_id(name, required=False) is not None
        }
        if self.target_id not in valid_target_ids:
            self.get_logger().warning(
                f"未知的 target_id {self.target_id!r}；将使用 'yellow_golf'。"
                f"有效值：{', '.join(sorted(valid_target_ids))}"
            )
            self.target_id = 'yellow_golf'
        self.get_logger().info(f'比赛目标元数据：{self.target_id}')

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
            '26rb_hit_balls': self._task_hit_balls,
            'pass_gate': self._task_pass_gates,
            'pass_gates': self._task_pass_gates,
            'go_through_gates': self._task_pass_gates,
            '26rb_gate_task': self._task_pass_gates,
            '26rb_pass_gates': self._task_pass_gates,
            'find_collection_frame': self._task_find_collection_frame,
            'find_platform_and_rack': self._task_find_collection_frame,
            'find_rack_and_platform': self._task_find_collection_frame,
            '26rb_find_collection_frame': self._task_find_collection_frame,
            'light_target_rack_return_origin':
                self._task_light_target_rack_return_origin,
            'light_frame_return': self._task_light_target_rack_return_origin,
            'visit_frame_light': self._task_light_target_rack_return_origin,
            'grab_ball': self._task_grab_ball,
            'grab_balls': self._task_grab_ball,
            '26rb_grab_ball': self._task_grab_ball,
            'drop_beacon': self._task_drop_beacon,
            '26rb_drop_beacon': self._task_drop_beacon,
            'take_water_sample': self._task_take_water_sample,
            'release_sampler': self._task_release_sampler,
            'return_origin': self._task_return_origin,
        }

        # Action client
        self._action_client = ActionClient(self, BasicMotion, 'basic_motion')
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('BasicMotion 动作服务器不可用！')

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

        self.get_logger().info('TaskRunner 节点已启动')
        self.get_logger().info(f'调试模式：{self._debug_mode}')

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
        self.get_logger().info(f'💡 灯光已打开：{label}（数值={color}）')

    def light_off(self):
        msg = UInt8(data=0)
        self.pub_light.publish(msg)
        self.get_logger().info('💡 灯光已关闭')

    def set_servo(self, angle_rad: float, label: str):
        msg = Float32(data=float(angle_rad))
        self.pub_servo.publish(msg)
        self.get_logger().info(
            f'⚙️  舵机：{label}（角度={angle_rad:.2f} rad）')

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

    def _down_visual_pair(self, class_id: int, max_age: float,
                          epipolar_tolerance: float):
        """Return a fresh, epipolar-consistent down-camera detection pair.

        The localizer's world position is intentionally not used here.  The
        light task closes its final horizontal loop on the two image centres.
        A pair ID is preferred when the detector provides one; otherwise the
        pair with the smallest normalized vertical mismatch is selected.
        """
        now = time.monotonic()
        with self._perception_lock:
            left_entry = self._down_detections.get('down_left')
            right_entry = self._down_detections.get('down_right')
        if left_entry is None or right_entry is None:
            return None

        left_time, left_msg = left_entry
        right_time, right_msg = right_entry
        if now - left_time > max_age or now - right_time > max_age:
            return None

        left_pair_id = int(getattr(left_msg, 'stereo_pair_id', 0) or 0)
        right_pair_id = int(getattr(right_msg, 'stereo_pair_id', 0) or 0)
        if (left_pair_id and right_pair_id
                and left_pair_id != right_pair_id):
            return None

        def candidates(message):
            result = []
            for detection in getattr(message, 'detections', []):
                if int(getattr(detection, 'class_id', -1)) != class_id:
                    continue
                try:
                    px = float(detection.pixel_x)
                    py = float(detection.pixel_y)
                    confidence = float(detection.confidence)
                except (AttributeError, TypeError, ValueError):
                    continue
                if all(math.isfinite(value)
                       for value in (px, py, confidence)):
                    result.append(detection)
            return result

        left_candidates = candidates(left_msg)
        right_candidates = candidates(right_msg)
        if not left_candidates or not right_candidates:
            return None

        pairs = []
        for left in left_candidates:
            left_v = (float(left.pixel_y) - _DOWN_CY) / _DOWN_FY
            for right in right_candidates:
                right_v = (float(right.pixel_y) - _DOWN_CY) / _DOWN_FY
                vertical_error = abs(left_v - right_v)
                if vertical_error <= epipolar_tolerance:
                    pairs.append((
                        vertical_error,
                        -(float(left.confidence) +
                          float(right.confidence)),
                        left, right,
                    ))
        if not pairs:
            return None
        _vertical_error, _confidence, left, right = min(
            pairs, key=lambda item: (item[0], item[1]))
        return left, right

    @staticmethod
    def _down_visual_error(left, right):
        """Return normalized image-centre and epipolar errors for a pair."""
        left_u = (float(left.pixel_x) - _DOWN_CX) / _DOWN_FX
        right_u = (float(right.pixel_x) - _DOWN_CX) / _DOWN_FX
        left_v = (float(left.pixel_y) - _DOWN_CY) / _DOWN_FY
        right_v = (float(right.pixel_y) - _DOWN_CY) / _DOWN_FY
        return (
            (left_u + right_u) * 0.5,
            (left_v + right_v) * 0.5,
            abs(left_v - right_v),
        )

    @staticmethod
    def _down_visual_body_step(du: float, dv: float, projection_depth: float,
                               gain: float, max_step: float):
        """Map down-view normalized image error to a bounded body XY step."""
        # The calibrated down optical frame maps to body (-y, +x, +z).
        body_dx = -float(dv) * float(projection_depth) * float(gain)
        body_dy = float(du) * float(projection_depth) * float(gain)
        norm = math.hypot(body_dx, body_dy)
        if norm > max_step:
            scale = float(max_step) / norm
            body_dx *= scale
            body_dy *= scale
        return body_dx, body_dy

    def _down_visual_servo_target_rack(self, p: dict, target_z: float) -> bool:
        """Center target_rack_down in the down stereo image before lighting."""
        servo_timeout = max(
            1.0, float(p.get('down_visual_servo_timeout',
                             p.get('horizontal_servo_timeout', 30.0))))
        stable_seconds = max(
            0.1, float(p.get('down_visual_servo_stable_seconds',
                             p.get('horizontal_servo_stable_seconds', 1.0))))
        detection_timeout = max(
            0.1, float(p.get('down_detection_timeout', 0.8)))
        pixel_tolerance = max(
            0.001, float(p.get('down_pixel_tolerance_fraction', 0.035)))
        epipolar_tolerance = max(
            0.001, float(p.get(
                'down_epipolar_vertical_tolerance_fraction', 0.04)))
        projection_depth = max(
            0.1, float(p.get('down_projection_depth_m', 0.8)))
        gain = max(0.05, float(p.get('down_visual_servo_gain', 0.8)))
        max_step = max(
            0.005, float(p.get('down_visual_servo_max_step_m', 0.08)))
        period = max(
            0.05, float(p.get('down_visual_servo_period',
                              p.get('horizontal_servo_period', 0.2))))
        command_timeout = max(
            0.2, float(p.get('down_visual_command_timeout',
                             p.get('horizontal_command_timeout', 10.0))))

        deadline = time.monotonic() + servo_timeout
        stable_since = None
        last_log = float('-inf')
        self.get_logger().info(
            'light_target_rack_return_origin：开始下视双目视觉伺服；'
            f'class_id={_TARGET_RACK_DOWN_CLASS_ID}，'
            f'像素容差={pixel_tolerance:.3f}，'
            f'极线容差={epipolar_tolerance:.3f}，'
            f'稳定时间={stable_seconds:.1f}s')

        while not self.stopped and time.monotonic() < deadline:
            now = time.monotonic()
            pair = self._down_visual_pair(
                _TARGET_RACK_DOWN_CLASS_ID,
                detection_timeout,
                epipolar_tolerance,
            )
            if pair is None:
                stable_since = None
                if now - last_log >= 1.0:
                    self.get_logger().warning(
                        'light_target_rack_return_origin：等待下视双目目标；'
                        '必须同时看到 target_rack_down 且左右目满足极线一致性')
                    last_log = now
                time.sleep(min(period, max(0.0, deadline - now)))
                continue

            left, right = pair
            du, dv, epipolar_error = self._down_visual_error(left, right)
            centered = (
                abs(du) <= pixel_tolerance
                and abs(dv) <= pixel_tolerance
                and epipolar_error <= epipolar_tolerance
            )
            if now - last_log >= 1.0:
                state = '已居中，等待稳定' if centered else '修正中'
                self.get_logger().info(
                    f'light_target_rack_return_origin：下视视觉伺服{state}；'
                    f'归一化误差=(du={du:+.4f},dv={dv:+.4f})，'
                    f'极线误差={epipolar_error:.4f}')
                last_log = now

            if centered:
                if stable_since is None:
                    stable_since = now
                elif now - stable_since >= stable_seconds:
                    self.get_logger().info(
                        'light_target_rack_return_origin：下视视觉伺服已连续稳定，'
                        '允许打开指示灯')
                    return True
            else:
                stable_since = None
                body_dx, body_dy = self._down_visual_body_step(
                    du, dv, projection_depth, gain, max_step)
                pose = self._latest_robot_pose()
                yaw = math.radians(float(pose[5]))
                cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
                world_dx = cos_yaw * body_dx - sin_yaw * body_dy
                world_dy = sin_yaw * body_dx + cos_yaw * body_dy
                target = [
                    pose[0] + world_dx,
                    pose[1] + world_dy,
                    float(target_z),
                    float(pose[5]),
                ]
                success, message = self._send_action_goal(
                    BasicMotion.Goal.SET,
                    target,
                    'xy',
                    timeout=command_timeout,
                    quiet=True,
                    task_context=self._format_motion_context(
                        'target_rack下视视觉伺服'))
                if not success:
                    self.get_logger().error(
                        'light_target_rack_return_origin：下视视觉伺服移动失败：'
                        f'{message}')
                    return False
                self._cmd_x = target[0]
                self._cmd_y = target[1]
                self._cmd_z = target[2]
                self._cmd_yaw = target[3]

            time.sleep(min(period, max(0.0, deadline - time.monotonic())))

        if self.stopped:
            return False
        self.get_logger().error(
            'light_target_rack_return_origin：下视视觉伺服超时，未打开指示灯')
        return False

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
                self.get_logger().info(f'搜索 [{label}]：已找到目标'); return True
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
                    self.get_logger().info(f'搜索 [{label}]：已找到目标！'); return True
            sd = (sd + 1) % 8
            if sd == 0: ss += sp
        return False

    def _align_to_class(self, class_id: int, label: str) -> bool:
        if self._best_down_detection(class_id) is None:
            self.get_logger().info(f'对准 [{label}]：正在搜索……')
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
            self.get_logger().info(f'对准 [{label}]：第 {i} 次接近，x={tgt[0]}，y={tgt[1]}')
        self.get_logger().info(f'对准 [{label}]：已完成')
        return True

    # ========================================================================
    # Task loading
    # ========================================================================

    def load_tasks(self, path: str) -> list:
        """Load a mission YAML or standalone task YAML."""
        tasks = load_mission_or_task(path)
        self.get_logger().info(f'已从 {path} 加载 {len(tasks)} 个任务')
        return tasks

    @staticmethod
    def _resolve_mission_path(value: str) -> str:
        """Resolve a mission/task path or an installed config filename."""
        text = str(value or '').strip()
        if not text:
            return str(default_mission_path())
        candidate = Path(os.path.expanduser(text))
        if candidate.is_absolute():
            return str(candidate)
        if candidate.exists():
            return str(candidate.resolve())
        missions_dir = default_mission_path().parent
        config_dirs = (missions_dir, missions_dir.parent / 'tasks')
        for config_dir in config_dirs:
            package_candidate = config_dir / candidate
            if package_candidate.exists():
                return str(package_candidate)
        if candidate.suffix == '':
            for config_dir in config_dirs:
                package_candidate = config_dir / f'{candidate.name}.yaml'
                if package_candidate.exists():
                    return str(package_candidate)
        return str(missions_dir / candidate)

    # ========================================================================
    # Task execution
    # ========================================================================

    def run_task_list(self):
        """Execute all tasks sequentially."""
        self.running = True
        self.stopped = False
        self.current_index = 0
        total = len(self.tasks)
        self.get_logger().info(f'=== 任务列表开始执行（共 {total} 个任务）===')

        while self.current_index < total and not self.stopped:
            task = self.tasks[self.current_index]
            name = task.get('name', 'unknown')
            params = task.get('params', {})
            self._current_task_name = str(name)
            self._current_task_step = self.current_index + 1

            self.get_logger().info(
                f'[{self.current_index + 1}/{total}] {name} 参数={params} '
                f'| 指令位姿=({self._cmd_x:.2f}, {self._cmd_y:.2f}, '
                f'{self._cmd_z:.2f}, {self._cmd_yaw:.1f}°)')

            try:
                success = self._execute_task(name, params)
                if not success:
                    self.get_logger().warn(
                        f'[{self.current_index + 1}/{total}] {name} 执行失败')
            except Exception as e:
                self.get_logger().error(
                    f'[{self.current_index + 1}/{total}] {name} 发生异常：{e}')

            self.current_index += 1

        self.running = False
        self._current_task_name = ''
        self._current_task_step = 0
        if self.stopped:
            self.get_logger().warn(f'=== 任务列表已停止，位置 {self.current_index}/{total} ===')
        else:
            self.get_logger().info(f'=== 任务列表执行完成（{total}/{total}）===')

    def _execute_task(self, name: str, params: dict) -> bool:
        """Execute a single task by name."""
        handler = self.task_map.get(name)
        if handler is None:
            self.get_logger().warn(f'未知任务：{name}')
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
                f'发送动作目标：{type_name}，目标=[{", ".join(t)}]，'
                f'超时={effective_timeout:.0f}s，任务上下文="{task_context}"')

        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('动作服务器不可用')
            return False, '动作服务器不可用'

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
                self.get_logger().warn(f'动作目标被中断（stopped={self.stopped}）')
            return False, '已停止'
        if not send_future.done():
            self.get_logger().error('发送动作目标超时')
            return False, '发送动作目标超时'

        goal_handle = send_future.result()
        self._active_goal_handle = goal_handle
        if not goal_handle.accepted:
            self._active_goal_handle = None
            self.get_logger().error('动作目标被服务器拒绝')
            return False, '动作目标被服务器拒绝'

        if not quiet:
            self.get_logger().info('动作目标已接受，等待执行结果……')

        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not self.stopped and not result_future.done():
            time.sleep(0.01)
        self._active_goal_handle = None
        if not rclpy.ok() or self.stopped:
            if not quiet:
                self.get_logger().warn(f'动作结果等待被中断（stopped={self.stopped}）')
            return False, '已停止'
        if not result_future.done():
            self.get_logger().error('等待动作结果超时')
            return False, '等待动作结果超时'

        result = result_future.result().result
        if not result.success:
            t_str = ', '.join(f'{v:.2f}' for v in target)
            self.get_logger().error(
                f'{type_name} 执行失败：{result.message}，'
                f'目标=[{t_str}]')
        elif not quiet:
            self.get_logger().info('动作执行结果：成功')
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
                self.get_logger().warn('⚠ 跳过准备，直接发车！')
            except EOFError:
                pass

        listener = threading.Thread(target=_listen_skip, daemon=True)
        listener.start()

        self.get_logger().info('YouLong_AUV_Control_System 准备启动，请做好拔缆准备')
        self.set_light(3, '启动指示灯')
        if self._sleep_or_skip(1, skip):
            return self._do_start()
        self.light_off()
        self.get_logger().info('AUV 即将发动，请拔缆或发布拔缆命令')

        for i in range(1):
            if self._sleep_or_skip(0.5, skip):
                return self._do_start()
            self.set_light(1, '启动指示灯')
            if self._sleep_or_skip(0.5, skip):
                return self._do_start()
            self.light_off()

        self.get_logger().info('AUV 将在 6 秒后启动，现在可以拔缆了')

        for i in range(7):
            if self._sleep_or_skip(0.25, skip):
                return self._do_start()
            self.set_light(2, '启动指示灯')
            if self._sleep_or_skip(0.25, skip):
                return self._do_start()
            self.light_off()

        self.get_logger().info('AUV 将在 2 秒后启动，如果看到这条信息，说明已经有点晚了')

        if self._sleep_or_skip(1, skip):
            return self._do_start()
        self.set_light(2, '启动指示灯')
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
            self.get_logger().error(f'START 执行失败：{msg}')
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
        self.get_logger().info(f'return_origin 执行结果：{msg}')
        if success:
            self._cmd_x = self._cmd_y = self._cmd_yaw = 0.0
        return success

    # --- SET tasks (absolute positioning) ---

    def _task_setx(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.SET,
            [p['x'], self._cmd_y, self._cmd_z, self._cmd_yaw],
            "x")
        self.get_logger().info(f'setx 执行结果：{msg}')
        if success:
            self._cmd_x = p['x']
        return success

    def _task_sety(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.SET,
            [self._cmd_x, p['y'], self._cmd_z, self._cmd_yaw],
            "y")
        self.get_logger().info(f'sety 执行结果：{msg}')
        if success:
            self._cmd_y = p['y']
        return success

    def _task_setz(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.SET,
            [self._cmd_x, self._cmd_y, p['z'], self._cmd_yaw],
            "z")
        self.get_logger().info(f'setz 执行结果：{msg}')
        if success:
            self._cmd_z = p['z']
        return success

    def _task_setrz(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.SET,
            [self._cmd_x, self._cmd_y, self._cmd_z, p['rz']],
            "rz")
        self.get_logger().info(f'setrz 执行结果：{msg}')
        if success:
            self._cmd_yaw = p['rz']
        return success

    def _task_setxy(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.SET,
            [p['x'], p['y'], self._cmd_z, self._cmd_yaw],
            "xy")
        self.get_logger().info(f'setxy 执行结果：{msg}')
        if success:
            self._cmd_x, self._cmd_y = p['x'], p['y']
        return success

    def _task_setxyz(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.SET,
            [p['x'], p['y'], p['z'], self._cmd_yaw],
            "xyz")
        self.get_logger().info(f'setxyz 执行结果：{msg}')
        if success:
            self._cmd_x, self._cmd_y, self._cmd_z = p['x'], p['y'], p['z']
        return success

    def _task_setxyzrz(self, p: dict) -> bool:
        x, y, z, yaw = p['x'], p['y'], p['z'], p.get('rz', self._cmd_yaw)
        success, msg = self._send_action_goal(
            BasicMotion.Goal.SET, [x, y, z, yaw], "xyzrz")
        self.get_logger().info(f'setxyzrz 执行结果：{msg}')
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
        self.get_logger().info(f'setxyrz 执行结果：{msg}')
        if success:
            self._cmd_x, self._cmd_y = p['x'], p['y']
            self._cmd_yaw = yaw
        return success

    # --- BMOVE tasks (body frame stepping) ---

    def _task_bmovex(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.BMOVE, [p['dx'], 0.0, 0.0, 0.0], "x")
        self.get_logger().info(f'bmovex 执行结果：{msg}')
        if success:
            cy, sy = math.cos(math.radians(self._cmd_yaw)), math.sin(math.radians(self._cmd_yaw))
            self._cmd_x += cy * p['dx']
            self._cmd_y += sy * p['dx']
        return success

    def _task_bmovey(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.BMOVE, [0.0, p['dy'], 0.0, 0.0], "y")
        self.get_logger().info(f'bmovey 执行结果：{msg}')
        if success:
            cy, sy = math.cos(math.radians(self._cmd_yaw)), math.sin(math.radians(self._cmd_yaw))
            self._cmd_x += -sy * p['dy']
            self._cmd_y += cy * p['dy']
        return success

    def _task_bmovez(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.BMOVE, [0.0, 0.0, p['dz'], 0.0], "z")
        self.get_logger().info(f'bmovez 执行结果：{msg}')
        if success:
            self._cmd_z += p['dz']
        return success

    def _task_bmoverz(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.BMOVE, [0.0, 0.0, 0.0, p['drz']], "rz")
        self.get_logger().info(f'bmoverz 执行结果：{msg}')
        if success:
            self._cmd_yaw += p['drz']
        return success

    def _task_bmovexy(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.BMOVE, [p['dx'], p['dy'], 0.0, 0.0], "xy")
        self.get_logger().info(f'bmovexy 执行结果：{msg}')
        if success:
            cy, sy = math.cos(math.radians(self._cmd_yaw)), math.sin(math.radians(self._cmd_yaw))
            self._cmd_x += cy * p['dx'] - sy * p['dy']
            self._cmd_y += sy * p['dx'] + cy * p['dy']
        return success

    def _task_bmovexyz(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.BMOVE, [p['dx'], p['dy'], p['dz'], 0.0], "xyz")
        self.get_logger().info(f'bmovexyz 执行结果：{msg}')
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
        self.get_logger().info(f'wmovex 执行结果：{msg}')
        if success:
            self._cmd_x = p['x']
        return success

    def _task_wmovey(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.WMOVE, [0.0, p['y'], 0.0, 0.0], "y")
        self.get_logger().info(f'wmovey 执行结果：{msg}')
        if success:
            self._cmd_y = p['y']
        return success

    def _task_wmovez(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.WMOVE, [0.0, 0.0, p['z'], 0.0], "z")
        self.get_logger().info(f'wmovez 执行结果：{msg}')
        if success:
            self._cmd_z = p['z']
        return success

    def _task_wmoverz(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.WMOVE, [0.0, 0.0, 0.0, p['rz']], "rz")
        self.get_logger().info(f'wmoverz 执行结果：{msg}')
        if success:
            self._cmd_yaw = p['rz']
        return success

    def _task_wmovexy(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.WMOVE, [p['x'], p['y'], 0.0, 0.0], "xy")
        self.get_logger().info(f'wmovexy 执行结果：{msg}')
        if success:
            self._cmd_x, self._cmd_y = p['x'], p['y']
        return success

    def _task_wmovexyz(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.WMOVE, [p['x'], p['y'], p['z'], 0.0], "xyz")
        self.get_logger().info(f'wmovexyz 执行结果：{msg}')
        if success:
            self._cmd_x, self._cmd_y, self._cmd_z = p['x'], p['y'], p['z']
        return success

    # --- WTRAVEL tasks (世界系直线：参数为绝对世界坐标) ---

    def _task_wtravelx(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.WTRAVEL, [p['x'], 0.0, 0.0, 0.0], "x")
        self.get_logger().info(f'wtravelx 执行结果：{msg}')
        if success:
            self._cmd_x = p['x']
        return success

    def _task_wtravely(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.WTRAVEL, [0.0, p['y'], 0.0, 0.0], "y")
        self.get_logger().info(f'wtravely 执行结果：{msg}')
        if success:
            self._cmd_y = p['y']
        return success

    def _task_wtravelz(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.WTRAVEL, [0.0, 0.0, p['z'], 0.0], "z")
        self.get_logger().info(f'wtravelz 执行结果：{msg}')
        if success:
            self._cmd_z = p['z']
        return success

    def _task_wtravelxy(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.WTRAVEL, [p['x'], p['y'], 0.0, 0.0], "xy")
        self.get_logger().info(f'wtravelxy 执行结果：{msg}')
        if success:
            self._cmd_x, self._cmd_y = p['x'], p['y']
        return success

    def _task_wtravelxyz(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.WTRAVEL, [p['x'], p['y'], p['z'], 0.0], "xyz")
        self.get_logger().info(f'wtravelxyz 执行结果：{msg}')
        if success:
            self._cmd_x, self._cmd_y, self._cmd_z = p['x'], p['y'], p['z']
        return success

    # --- BTRAVEL tasks (body frame linear travel: body→world + turn + go) ---

    def _task_btravelx(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.BTRAVEL, [p['dx'], 0.0, 0.0, 0.0], "x")
        self.get_logger().info(f'btravelx 执行结果：{msg}')
        if success:
            cy, sy = math.cos(math.radians(self._cmd_yaw)), math.sin(math.radians(self._cmd_yaw))
            self._cmd_x += cy * p['dx']
            self._cmd_y += sy * p['dx']
        return success

    def _task_btravely(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.BTRAVEL, [0.0, p['dy'], 0.0, 0.0], "y")
        self.get_logger().info(f'btravely 执行结果：{msg}')
        if success:
            cy, sy = math.cos(math.radians(self._cmd_yaw)), math.sin(math.radians(self._cmd_yaw))
            self._cmd_x += -sy * p['dy']
            self._cmd_y += cy * p['dy']
        return success

    def _task_btravelz(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.BTRAVEL, [0.0, 0.0, p['dz'], 0.0], "z")
        self.get_logger().info(f'btravelz 执行结果：{msg}')
        if success:
            self._cmd_z += p['dz']
        return success

    def _task_btravelxy(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.BTRAVEL, [p['dx'], p['dy'], 0.0, 0.0], "xy")
        self.get_logger().info(f'btravelxy 执行结果：{msg}')
        if success:
            cy, sy = math.cos(math.radians(self._cmd_yaw)), math.sin(math.radians(self._cmd_yaw))
            self._cmd_x += cy * p['dx'] - sy * p['dy']
            self._cmd_y += sy * p['dx'] + cy * p['dy']
        return success

    def _task_btravelxyz(self, p: dict) -> bool:
        success, msg = self._send_action_goal(
            BasicMotion.Goal.BTRAVEL, [p['dx'], p['dy'], p['dz'], 0.0], "xyz")
        self.get_logger().info(f'btravelxyz 执行结果：{msg}')
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
                f'_move_to_nearest_object_xy：未找到 class_id={class_id} 的物体')
            return False

        self.get_logger().info(
            f'_move_to_nearest_object_xy：class_id={class_id}，'
            f'位置=({nearest.world_x:.2f}, {nearest.world_y:.2f})，'
            f'距离={min_dist:.2f}m')

        success, msg = self._send_action_goal(
            BasicMotion.Goal.SET,
            [nearest.world_x, nearest.world_y, self._cmd_z, self._cmd_yaw],
            'xy', timeout=15.0, quiet=True)
        if success:
            self._cmd_x = nearest.world_x
            self._cmd_y = nearest.world_y
        else:
            self.get_logger().warn(
                f'_move_to_nearest_object_xy 执行失败：{msg}')
        return success

    def _task_navigate(self, p: dict) -> bool:
        x = p.get('x', self._cmd_x)
        y = p.get('y', self._cmd_y)
        z = p.get('z', self._cmd_z)
        yaw = p.get('rz', self._cmd_yaw)
        success, msg = self._send_action_goal(
            BasicMotion.Goal.SET, [x, y, z, yaw], "xyzrz", timeout=120.0)
        self.get_logger().info(f'navigate 执行结果：{msg}')
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
                'hit_balls：撞球顺序中没有有效目标；请使用 blue/red，'
                '或使用 robotcup20260901.yaml 中的有效 class_id')
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
            f'hit_balls：等待 {name} 目标估计超时（{timeout:.1f}s）')
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
                f'hit_balls：旋转到 {yaw:.1f}° 进行扫描失败：{message}')
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
            f'hit_balls：主动定位扫描开始，共 {headings} 个方向，'
            f'步进 {step:.1f}°')
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
                            f'hit_balls：主动扫描找到 {name}，'
                            f'位置=({target["x"]:.2f}, {target["y"]:.2f}, '
                            f'{target["z"]:.2f})')
            if len(found) == len(order):
                break

        self.get_logger().info(
            f'hit_balls：主动定位扫描完成，已找到={list(found.keys())}，'
            f'缺少={[name for name in order if name not in found]}')
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
                f'hit_balls：{label} 已完成，当前位置='
                f'({x:.2f}, {y:.2f}, {z:.2f})')
        else:
            self.get_logger().error(f'hit_balls：{label} 执行失败：{message}')
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
                    f'hit_balls：{name} 对准修正失败：{message}')

            remaining = deadline - time.monotonic()
            if remaining > 0.0:
                time.sleep(min(period, remaining))

        return rclpy.ok() and not self.stopped

    def _publish_body_velocity(self, forward_mps: float = 0.0,
                               lateral_mps: float = 0.0,
                               vertical_mps: float = 0.0,
                               yaw_rate_deg_s: float = 0.0):
        """Publish one body-frame velocity-loop setpoint.

        The wire protocol uses metres/second for the three linear axes and
        radians/second for yaw.  Keeping this helper on TaskRunner lets
        camera tasks use the same velocity path as the existing impact
        charge, without opening a second motion controller.
        """
        msg = ZitSetpoint()
        msg.control_key = 0x11  # VEL (0x01) | BODY (0x10)
        msg.type_mask = 0
        msg.x = float(forward_mps)
        msg.y = float(lateral_mps)
        msg.z = float(vertical_mps)
        msg.roll = 0.0
        msg.pitch = 0.0
        msg.yaw = math.radians(float(yaw_rate_deg_s))
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

    def _task_pass_gates(self, p: dict) -> bool:
        """仅用前视相机图像搜索、对准并连续通过多个门。"""
        gate_task = RB26GateTask(self, p)
        try:
            return gate_task.execute()
        finally:
            gate_task.destroy()

    def _task_hit_balls(self, p: dict) -> bool:
        """执行 26rb 撞球任务模块。"""
        task = RB26HitBallsTask(self, p)
        return task.execute()

    # ── 置物台 / target-rack 搜索任务 ─────────────────────────────

    @staticmethod
    def _normalize_localizer_target_name(value):
        """Normalize object_localizer labels to the two task target names."""
        if isinstance(value, (int, np.integer)):
            return _LOCALIZER_TARGET_CLASS_IDS.get(int(value))

        text = str(value or '').strip().lower()
        if not text:
            return None
        text = text.replace('-', '_').replace(' ', '_')
        # Detector labels are intentionally kept in class_name, whereas
        # physical_class_name is already canonical.
        for suffix in ('_front', '_down'):
            if text.endswith(suffix):
                text = text[:-len(suffix)]
                break
        return _LOCALIZER_TARGET_ALIASES.get(text)

    @classmethod
    def _localizer_target_name(cls, target):
        """Return a canonical name for one TargetPosition message."""
        physical_name = cls._normalize_localizer_target_name(
            getattr(target, 'physical_class_name', ''))
        if physical_name is not None:
            return physical_name
        class_name = cls._normalize_localizer_target_name(
            getattr(target, 'class_name', ''))
        if class_name is not None:
            return class_name
        return cls._normalize_localizer_target_name(
            getattr(target, 'class_id', -1))

    def _best_localizer_target(self, target_name: str, p: dict):
        """Get the best current world estimate from object_localizer.

        The localizer publishes independent ``front`` and ``down`` estimates
        for one physical target.  This task needs one usable world position,
        so it selects the freshest/highest-quality estimate rather than
        treating those two records as two different targets.
        """
        wanted = self._normalize_localizer_target_name(target_name)
        if wanted is None:
            return None

        min_confidence = float(p.get('min_confidence', 0.02))
        min_observations = max(1, int(p.get('min_observations', 1)))
        stale_status = int(getattr(TargetPosition, 'STATUS_STALE', 3))
        uninitialized_status = int(
            getattr(TargetPosition, 'STATUS_UNINITIALIZED', 0))
        stable_status = int(getattr(TargetPosition, 'STATUS_STABLE', 2))
        allow_stale = bool(p.get('allow_stale_targets', True))

        with self._perception_lock:
            candidates = list(self.target_positions.targets)

        valid = []
        for target in candidates:
            if self._localizer_target_name(target) != wanted:
                continue
            status = int(getattr(target, 'status', uninitialized_status))
            if status == uninitialized_status:
                continue
            if status == stale_status and not allow_stale:
                continue
            confidence = float(getattr(target, 'confidence', 0.0))
            observations = int(getattr(target, 'num_observations', 0))
            x = float(getattr(target, 'world_x', float('nan')))
            y = float(getattr(target, 'world_y', float('nan')))
            z = float(getattr(target, 'world_z', float('nan')))
            age = float(getattr(target, 'age_sec', 0.0))
            if (confidence < min_confidence or observations < min_observations
                    or not all(math.isfinite(value) for value in (x, y, z))):
                continue
            max_age = float(p.get('max_target_age_seconds', 0.0))
            if max_age > 0.0 and math.isfinite(age) and age > max_age:
                continue
            valid.append((
                1 if status == stable_status else 0,
                confidence,
                observations,
                -age if math.isfinite(age) else float('-inf'),
                1 if str(getattr(target, 'estimate_source', '')) == 'down' else 0,
                target,
            ))

        if not valid:
            return None

        target = max(valid, key=lambda item: item[:-1])[-1]
        return {
            'name': wanted,
            'x': float(target.world_x),
            'y': float(target.world_y),
            'z': float(target.world_z),
            'confidence': float(target.confidence),
            'observations': int(target.num_observations),
            'status': int(target.status),
            'age': float(getattr(target, 'age_sec', 0.0)),
            'source': str(getattr(target, 'estimate_source', 'unknown')),
            'instance_id': int(getattr(target, 'instance_id', 0)),
        }

    def _current_localizer_targets(self, p: dict):
        """Return at most one current estimate for each required target."""
        result = {}
        for name in ('collection_frame', 'target_rack'):
            target = self._best_localizer_target(name, p)
            if target is not None:
                result[name] = target
        return result

    def _look_at_localizer_target(self, target: dict, p: dict,
                                  label: str) -> bool:
        """Point the AUV at a world target using an absolute yaw SET goal."""
        pose = self._latest_robot_pose()
        dx = float(target['x']) - pose[0]
        dy = float(target['y']) - pose[1]
        if math.hypot(dx, dy) <= 1e-6:
            yaw = float(self._cmd_yaw)
        else:
            yaw = self._wrap_yaw_degrees(math.degrees(math.atan2(dy, dx)))

        self.get_logger().info(
            f'find_collection_frame：观察 {label} '
            f'({target["x"]:.2f}, {target["y"]:.2f}, {target["z"]:.2f})，'
            f'偏航角={yaw:.1f}°，来源={target["source"]}')
        success, message = self._send_action_goal(
            BasicMotion.Goal.SET,
            [self._cmd_x, self._cmd_y, self._cmd_z, yaw],
            'rz',
            timeout=max(1.0, float(p.get('rotate_timeout', 15.0))),
            task_context=self._format_motion_context(f'观察{label}'))
        if not success:
            self.get_logger().error(
                f'find_collection_frame：旋转观察 {label} 失败：{message}')
            return False
        self._cmd_yaw = yaw
        settle = max(0.0, float(p.get('look_settle_seconds', 0.5)))
        if settle > 0.0:
            time.sleep(settle)
        return True

    def _confirm_localizer_target(self, target_name: str, p: dict,
                                  deadline: float) -> bool:
        """Require one target estimate to remain available briefly."""
        hold_seconds = max(0.0, float(p.get('confirm_seconds', 0.5)))
        hold_start = None
        while not self.stopped and time.monotonic() < deadline:
            if self._best_localizer_target(target_name, p) is not None:
                if hold_start is None:
                    hold_start = time.monotonic()
                if time.monotonic() - hold_start >= hold_seconds:
                    return True
            else:
                hold_start = None
            time.sleep(0.05)
        return False

    def _scan_localizer_east_to_south(self, p: dict, deadline: float) -> bool:
        """Scan absolute yaw 90° (east) through 180° (south)."""
        start = float(p.get('scan_start_yaw_deg', 90.0))
        end = float(p.get('scan_end_yaw_deg', 180.0))
        step = abs(float(p.get('scan_yaw_step_deg', 15.0)))
        if step < 1e-3:
            step = 15.0
        if end < start:
            start, end = end, start

        headings = list(np.arange(start, end, step, dtype=float)) + [end]
        self.get_logger().info(
            f'find_collection_frame：初始未发现目标，'
            f'从东向 {start:.1f}° 至南向 {end:.1f}° 扫描')
        any_found = False
        for heading in headings:
            if self.stopped or time.monotonic() >= deadline:
                return False
            heading = self._wrap_yaw_degrees(float(heading))
            success, message = self._send_action_goal(
                BasicMotion.Goal.SET,
                [self._cmd_x, self._cmd_y, self._cmd_z, heading],
                'rz',
                timeout=min(
                    max(1.0, float(p.get('rotate_timeout', 15.0))),
                    max(1.0, deadline - time.monotonic())),
                quiet=True,
                task_context=self._format_motion_context(
                    f'东向至南向扫描{heading:.1f}°'))
            if not success:
                self.get_logger().warn(
                    f'find_collection_frame：扫描旋转至 {heading:.1f}° 失败：'
                    f'{message}')
                return False
            self._cmd_yaw = heading
            settle = max(0.0, float(p.get('scan_settle_seconds', 0.4)))
            if settle > 0.0:
                time.sleep(min(settle, max(0.0, deadline - time.monotonic())))
            found = self._current_localizer_targets(p)
            if found:
                any_found = True
                self.get_logger().info(
                    f'find_collection_frame：扫描在偏航角={heading:.1f}° '
                    f'发现 {", ".join(sorted(found))}')
                if len(found) >= 2:
                    return True
        return any_found

    def _task_find_collection_frame(self, p: dict) -> bool:
        """执行 26rb 置物台/台框定位任务模块。"""
        task = RB26FindCollectionFrameTask(self, p)
        return task.execute()

    def _task_light_target_rack_return_origin(self, p: dict) -> bool:
        """粗定位到目标架上方，下视视觉伺服对正后闪灯并返回。

        ``find_collection_frame`` has already confirmed the localizer targets
        before this task is normally called.  Its world position is used only
        for the initial coarse move.  The final alignment is closed on the
        ``target_rack_down`` detections from both down cameras, so the light
        command is issued only after the rack is visually centred and stable.
        The final return is the actual task chain origin, not the pose at task
        entry.
        """
        target_name = self._normalize_localizer_target_name(
            p.get('frame_name', p.get('target_name', 'target_rack')))
        if target_name is None:
            self.get_logger().error(
                'light_target_rack_return_origin：目标名称无效')
            return False

        target_timeout = max(1.0, float(p.get('target_timeout',
                                              p.get('timeout', 120.0))))
        deadline = time.monotonic() + target_timeout
        target = None
        last_wait_log = float('-inf')
        while not self.stopped and time.monotonic() < deadline:
            target = self._best_localizer_target(target_name, p)
            if target is not None:
                break
            now = time.monotonic()
            if now - last_wait_log >= 1.0:
                self.get_logger().info(
                    f'light_target_rack_return_origin：等待定位器提供 '
                    f'{target_name} 的位置')
                last_wait_log = now
            time.sleep(0.05)

        if self.stopped or target is None:
            self.get_logger().error(
                f'light_target_rack_return_origin：等待 {target_name} 超时')
            return False

        target_z = max(0.0, float(p.get('above_z_m', 0.20)))
        pose = self._latest_robot_pose()
        dx = float(target['x']) - pose[0]
        dy = float(target['y']) - pose[1]
        target_yaw = float(pose[5])
        if math.hypot(dx, dy) > 1e-6:
            target_yaw = self._wrap_yaw_degrees(math.degrees(math.atan2(dy, dx)))

        self.get_logger().info(
            f'light_target_rack_return_origin：移动到 {target_name} 中心上方 '
            f'({target["x"]:.2f}, {target["y"]:.2f}, {target_z:.2f})，'
            f'偏航角={target_yaw:.1f}°')
        success, message = self._send_action_goal(
            BasicMotion.Goal.SET,
            [float(target['x']), float(target['y']), target_z, target_yaw],
            'xyzrz',
            timeout=max(1.0, float(p.get('move_timeout', 120.0))),
            task_context=self._format_motion_context(
                f'移动到{target_name}正上方'))
        if not success:
            self.get_logger().error(
                f'light_target_rack_return_origin：移动失败：{message}')
            return False
        self._cmd_x = float(target['x'])
        self._cmd_y = float(target['y'])
        self._cmd_z = target_z
        self._cmd_yaw = target_yaw

        # 世界坐标只负责把目标送入下视相机视场，最终位置不再由
        # target_positions 的世界坐标闭环决定。
        if not self._down_visual_servo_target_rack(p, target_z):
            return False

        light_value = p.get('light_color', p.get('light', 'yellow'))
        if isinstance(light_value, str):
            light_value = {
                'off': self.LIGHT_OFF,
                'yellow': self.LIGHT_YELLOW,
                'green': self.LIGHT_GREEN,
                'red': self.LIGHT_RED,
            }.get(light_value.strip().lower(), self.LIGHT_YELLOW)
        try:
            light_value = int(light_value)
        except (TypeError, ValueError):
            light_value = self.LIGHT_YELLOW
        if light_value not in (self.LIGHT_OFF, self.LIGHT_YELLOW,
                               self.LIGHT_GREEN, self.LIGHT_RED):
            light_value = self.LIGHT_YELLOW

        self.set_light(light_value, f'{target_name} 中心')
        hold_seconds = max(0.0, float(p.get('light_hold_seconds', 1.0)))
        try:
            if hold_seconds > 0.0:
                end = time.monotonic() + hold_seconds
                while not self.stopped and time.monotonic() < end:
                    time.sleep(min(0.05, end - time.monotonic()))
            if self.stopped:
                return False

            self.get_logger().info(
                'light_target_rack_return_origin：返回任务链原点 '
                '（0.00, 0.00, 0.00, 0.0°）')
            success, message = self._send_action_goal(
                BasicMotion.Goal.SET,
                [0.0, 0.0, 0.0, 0.0],
                'xyzrz',
                timeout=max(1.0, float(p.get('return_timeout', 120.0))),
                task_context=self._format_motion_context(
                    f'从{target_name}返回任务链原点'))
            if not success:
                self.get_logger().error(
                    f'light_target_rack_return_origin：返回失败：{message}')
                return False
            self._cmd_x = self._cmd_y = self._cmd_z = self._cmd_yaw = 0.0
            self.get_logger().info(
                'light_target_rack_return_origin：已返回任务链原点')
            return True
        finally:
            self.light_off()

    def _task_grab_ball(self, p: dict) -> bool:
        """使用左下视相机完成单个指定颜色球的抓取动作。"""
        grab_task = RB26GrabBallTask(self, p)
        return grab_task.execute()

    # ── 投信标 / 采水 / 释放取水器 ─────────────────────────────────

    def _task_drop_beacon(self, p: dict) -> bool:
        """执行 26rb 丢球/投放任务模块。"""
        task = RB26DropBeaconTask(self, p)
        return task.execute()

    def _task_take_water_sample(self, p: dict) -> bool:
        angle = float(p.get('angle_rad', self.ANGLE_SAMPLE_WATER))
        self.set_servo(angle, '采集水样')
        self.get_logger().info('💧 水样已采集！')
        return True

    def _task_release_sampler(self, p: dict) -> bool:
        """转向 → 对齐 START 标记 → 上浮靠岸 → 释放取水器。"""
        align_yaw = float(p.get('align_yaw', 180.0))
        start_cid = model_class_id('guide_line')
        if 'start_class_id' in p:
            start_cid = int(p['start_class_id'])
        approach_z = float(p.get('approach_z', -0.3))
        approach_x = float(p.get('approach_x', -0.3))
        approach_timeout = float(p.get('approach_timeout', 15.0))
        release_angle = float(p.get('release_angle_rad',
                                    self.ANGLE_RELEASE_SAMPLER))

        self.get_logger().info(
            f'🧭 release_sampler：转向 rz={align_yaw:.1f}°')
        self._send_action_goal(
            BasicMotion.Goal.WMOVE,
            [self._cmd_x, self._cmd_y, self._cmd_z, align_yaw],
            'rz', timeout=15.0)
        self._cmd_yaw = align_yaw

        self.get_logger().info(
            f'🎯 release_sampler：对准 START 标记（class={start_cid}）')
        self._align_to_class(start_cid, 'START 标记')

        self.get_logger().info(
            f'🌊🏖️  release_sampler：执行 wmove，z={approach_z}，x={approach_x}')
        self._send_action_goal(
            BasicMotion.Goal.WMOVE,
            [approach_x, self._cmd_y, approach_z, self._cmd_yaw],
            'xz', timeout=approach_timeout)
        self._send_action_goal(
            BasicMotion.Goal.WMOVE,
            [approach_x, self._cmd_y - 1, approach_z, self._cmd_yaw],
            'xz', timeout=approach_timeout)
        self._cmd_x = approach_x; self._cmd_z = approach_z

        self.set_servo(release_angle, '释放取水器')
        self.get_logger().info('🗑️  取水器已释放！')
        return True

    # ========================================================================
    # Service handlers
    # ========================================================================

    def _run_task_cb(self, request, response):
        if request.start:
            path = self._resolve_mission_path(request.task_name)

            self.get_logger().info(f'服务 /task/run：从 {path} 开始执行任务')
            try:
                self.tasks = self.load_tasks(path)
            except ConfigError as exc:
                response.success = False
                response.message = f'任务配置无效：{exc}'
                self.get_logger().error(response.message)
                return response
            if self.tasks:
                thread = threading.Thread(target=self.run_task_list, daemon=True)
                thread.start()
                response.success = True
                response.message = f'已启动 {len(self.tasks)} 个任务'
            else:
                response.success = False
                response.message = '未加载任何任务'
        else:
            self.get_logger().info('服务 /task/run：收到停止请求')
            self.stopped = True
            response.success = True
            response.message = '已停止'
        return response

    def _stop_task_cb(self, request, response):
        self.get_logger().warn('服务 /task/stop：紧急停止')
        self.stopped = True
        if self._active_goal_handle is not None:
            self.get_logger().info('正在取消当前动作目标')
            self._action_client.async_cancel_goal(self._active_goal_handle)
            self._active_goal_handle = None
        response.success = True
        response.message = '任务已停止'
        return response

    def _exec_task_cb(self, request, response):
        """Handle /task/exec: execute a single task (debug mode only)."""
        if not self._debug_mode:
            response.success = False
            response.message = 'ExecTask 服务仅在调试模式下可用'
            self.get_logger().warn('/task/exec 被调用，但调试模式未开启')
            return response

        if self._debug_executing:
            response.success = False
            response.message = (
                f'任务“{self._debug_task_name}”已在运行。'
                '请等待任务结束，或调用 /task/stop。'
            )
            self.get_logger().warn(f'拒绝并发 /task/exec：{self._debug_task_name}')
            return response

        task_name = request.task_name
        params_json = request.params_json
        timeout = request.timeout

        # Validate task_name against task_map
        if task_name not in self.task_map:
            response.success = False
            valid = ', '.join(sorted(self.task_map.keys()))
            response.message = f'未知任务：{task_name}。有效任务：{valid}'
            return response

        # Parse params JSON
        try:
            params = json.loads(params_json) if params_json.strip() else {}
        except json.JSONDecodeError as e:
            response.success = False
            response.message = f'params_json 无效：{e}'
            return response

        # Store timeout for _send_action_goal override
        params['_timeout'] = timeout

        self.get_logger().info(
            f'调试执行：{task_name}，参数={params}，超时={timeout:.0f}s'
        )

        # Execute in daemon thread (same pattern as run_task_list)
        thread = threading.Thread(
            target=self._debug_exec_single, args=(task_name, params),
            daemon=True
        )
        thread.start()

        response.success = True
        response.message = f'正在执行任务：{task_name}'
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
                self.get_logger().info(f'调试执行 {name}：成功')
            else:
                self.get_logger().warn(f'调试执行 {name}：失败')
        except Exception as e:
            self.get_logger().error(f'调试执行 {name}：发生异常：{e}')
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
            msg.error_message = '[调试模式]'
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
                msg.error_message = '[调试模式：空闲，等待 /task/exec]'

        self.pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TaskRunnerNode()

    if not node._debug_mode:
        # Normal mode: load the selected YAML mission and start immediately.
        default_path = node.mission_file or str(default_mission_path())
        try:
            node.tasks = node.load_tasks(default_path)
        except ConfigError as exc:
            node.get_logger().fatal(f'无法启动任务执行器：{exc}')
            node.destroy_node()
            rclpy.try_shutdown()
            raise SystemExit(2) from exc
        if node.tasks:
            thread = threading.Thread(target=node.run_task_list, daemon=True)
            thread.start()
            node.get_logger().info(f'已自动启动任务列表（共 {len(node.tasks)} 个任务）')
    else:
        node.get_logger().info(
            '调试模式已开启：跳过自动启动。'
            '请使用 /task/exec 服务执行单个任务。'
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
