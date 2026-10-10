"""Task runner node: loads YAML missions or standalone tasks sequentially.

Each task calls basic_motion via the BasicMotion action server.
The task runner is the single source of truth for commanded position,
tracked locally (not from external topics).
"""

from __future__ import annotations

from collections import deque
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
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from std_msgs.msg import UInt8
from std_srvs.srv import Trigger

from zit6_interfaces.msg import ZitServo, ZitStatus
from auv_protocol.topics import (
    BASIC_MOTION, MODEL_CLASS_MAPPING, PERCEPTION_DETECTIONS, TRACKS, STATE_ODOM,
    ZIT6_STATUS, ZIT6_LIGHT, ZIT6_SERVO,
    MISSION_RUN, MISSION_STOP, MISSION_EXECUTE, MISSION_STATUS,
    LEGACY_TASK_RUN, LEGACY_TASK_STOP, LEGACY_TASK_EXECUTE,
    LEGACY_TASK_STATUS,
)

from uv_msgs.action import BasicMotion
from uv_msgs.msg import (
    DetectionArray,
    ModelClassMapping,
    ObjectTrack,
    ObjectTrackArray,
    PoseInfo,
    TaskStatus,
)
from uv_msgs.srv import ExecTask, RunTask
from uv_task.config_loader import (
    ConfigError,
    default_mission_path,
    load_mission_or_task,
)
from uv_task.mission_policy import (
    apply_failure_override,
    select_failure_override,
)
from uv_task.task_outcome import TaskOutcome
from uv_task.task_state import TaskState
from uv_task.basic_motion_test import RosMotionTest

from uv_task.arrow_surfacer import (
    _euler_to_rotation_matrix, _ray_intersection_midpoint,
)
from uv_task.arrow_surfacer import ArrowSurfacer
# The competition task module names intentionally start with ``26rb_``.
# Such names cannot be used in a normal ``from package import module``
# statement, so load them through importlib.
RB26GrabGolfTask = import_module('uv_task.26rb_grab_golf').RB26GrabGolfTask
RB26GrabBallRingTask = import_module('uv_task.26rb_grab_ball_ring').RB26GrabBallRingTask
RB26GateTask = import_module('uv_task.26rb_gate_task').RB26GateTask
RB26HitBallsTask = import_module('uv_task.26rb_hit_balls').RB26HitBallsTask
RB26FindCollectionFrameTask = import_module(
    'uv_task.26rb_find_collection_frame').RB26FindCollectionFrameTask
RB26DropBallTargetRackTask = import_module(
    'uv_task.26rb_drop_ball_target_rack').RB26DropBallTargetRackTask
RB26DropBeaconTask = import_module(
    'uv_task.26rb_drop_beacon').RB26DropBeaconTask
from uv_task.line_follower import LineFollower
from uv_camera.camera_tf import CameraExtrinsicsProvider, CameraExtrinsicsUnavailable
from uv_camera.perception_geometry import (
    TaskCameraGeometry, bind_detection_geometry, normalized_detection,
)
from uv_camera.camera_config import camera_mode_for_sim_mode, load_camera_config
from auv_protocol.model_mapping import ModelClassRegistry


# object_estimator publishes one persistent track per physical class on
# /auv/perception/tracks, while class_name remains the detector label
# including a view suffix (for example ``collection_frame_front``);
# physical_class_name is the canonical class when available.
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

    # ── 灯光常量 (/auv/hardware/zit6/cmd/light) ────────────────────
    LIGHT_OFF = 0
    LIGHT_YELLOW = 3
    LIGHT_GREEN = 2
    LIGHT_RED = 1
    # The canonical ROS UInt8 topic forwards the state byte unchanged. 4 is
    # reserved here for blue; the attached MCU/light controller must support it.
    LIGHT_BLUE = 4

    # Firmware logical servo channels.
    SERVO_ID_GOLF = 1
    SERVO_ID_RING = 2

    # ── 舵机角度 (/auv/hardware/zit6/cmd/servo, rad) ───────────────
    ANGLE_DROP_BEACON = 90       #   投信标
    ANGLE_SAMPLE_WATER = 0.0               # 采水样
    ANGLE_RELEASE_SAMPLER = 0   #   释放取水器
    ANGLE_INIT = 0.0

    def __init__(self):
        super().__init__('task_runner')

        # Shared across tasks; bias conversions remain opt-in until all motion
        # and perception entry points use the same task coordinate reference.
        self.state = TaskState()

        # Commanded position tracker (updated after every motion command).
        # Starts at (0,0,0,0) after START, which matches basic_motion's odom origin.
        # Used by single-axis SET tasks to fill non-targeted axes.
        self._cmd_x = 0.0
        self._cmd_y = 0.0
        self._cmd_z = 0.0
        self._cmd_yaw = 0.0
        self.object_tracks = ObjectTrackArray()
        self._model_mapping = ModelClassRegistry.empty()
        self._model_mapping_ready = threading.Event()
        self._localizer_target_class_ids = {}
        self._target_rack_down_class_id = None

        # ── 下视感知（release_sampler 对齐用）──
        self._perception_lock = threading.RLock()
        self._down_detections = {}   # camera_name → (monotonic, DetectionArray)
        # Preserve callback order while a task waits for a motion action.
        self._down_detection_sequence = 0
        self._down_detection_events = deque(maxlen=256)
        self._robot_pose = None      # (x, y, z, roll_deg, pitch_deg, yaw_deg)

        self.tasks = []
        self.current_index = 0
        self._current_task_name = ''
        self._current_task_step = 0
        self.running = False
        self.stopped = False
        self._active_goal_handle = None
        self._motion_stop_sent = False
        self._last_motion_failure_kind = ''
        self._last_motion_failure_message = ''
        self._light_state = None
        self._target_light_indications = set()
        self._last_failure_code = ''
        self._last_failure_message = ''
        self._mission_status_code = TaskStatus.STATUS_IDLE

        # Debug mode
        self.declare_parameter('debug_mode', False)
        self._debug_mode = self.get_parameter('debug_mode').get_parameter_value().bool_value
        self.declare_parameter('mission_file', '')
        self.mission_file = self.get_parameter(
            'mission_file').get_parameter_value().string_value
        self.declare_parameter('auto_start', True)
        auto_start_value = self.get_parameter('auto_start').value
        self._auto_start = (
            auto_start_value if isinstance(auto_start_value, bool)
            else str(auto_start_value).strip().lower()
            in ('1', 'true', 'yes', 'on'))
        self._debug_task_name = None
        self._debug_executing = False
        self._debug_timeout = -1.0
        self._last_basic_motion_test_outcome = None

        self.declare_parameter('camera_mode', 'auto')
        self.declare_parameter('camera_config_dir', '')
        self.declare_parameter('camera_base_frame', 'base_link')
        self.declare_parameter('camera_tf_timeout_sec', 5.0)
        self.declare_parameter('camera_tf_retry_period_sec', 0.1)
        camera_mode = camera_mode_for_sim_mode(
            False, self.get_parameter('camera_mode').value)
        camera_config_dir = str(
            self.get_parameter('camera_config_dir').value).strip() or None
        self.camera_configs = {
            camera: load_camera_config(camera, camera_mode, camera_config_dir)
            for camera in ('front', 'down')
        }
        self._detection_geometry = TaskCameraGeometry(self)
        self.camera_extrinsics_provider = CameraExtrinsicsProvider(
            self,
            base_frame=self.get_parameter('camera_base_frame').value,
            timeout_sec=float(self.get_parameter('camera_tf_timeout_sec').value),
            retry_period_sec=float(self.get_parameter('camera_tf_retry_period_sec').value))
        self.camera_extrinsics = {}
        self._camera_tf_warned = False
        self._camera_tf_timer = None
        down = self.camera_configs['down']
        down_left = down.side('left')
        self._down_fx = float(down_left.matrix[0, 0])
        self._down_fy = float(down_left.matrix[1, 1])
        self._down_cx = float(down_left.matrix[0, 2])
        self._down_cy = float(down_left.matrix[1, 2])
        self._down_offset_left = np.zeros(3, dtype=np.float64)
        self._down_offset_right = np.zeros(3, dtype=np.float64)
        self._down_optical_to_body = np.eye(3, dtype=np.float64)
        self._down_optical_to_body_right = np.eye(3, dtype=np.float64)

        # Task map (shared by _execute_task and _exec_task_cb)
        self.task_map = {
            'start': self._task_start,
            'basic_motion_test': self._task_basic_motion_test,
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
            'bline': self._task_bline,
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
            '26rb_drop_ball_target_rack': self._task_drop_ball_target_rack,
            'grab_golf': self._task_grab_golf,
            '26rb_grab_golf': self._task_grab_golf,
            '26rb_grab_ball_ring': self._task_grab_ball_ring,
            'drop_beacon': self._task_drop_beacon,
            '26rb_drop_beacon': self._task_drop_beacon,
            'take_water_sample': self._task_take_water_sample,
            'release_sampler': self._task_release_sampler,
            'return_origin': self._task_return_origin,
        }

        # Action client
        self._action_client = ActionClient(self, BasicMotion, BASIC_MOTION)
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('BasicMotion 动作服务器不可用！')

        self._basic_motion_test = RosMotionTest(self)

        # Subscribers
        mapping_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            ModelClassMapping, MODEL_CLASS_MAPPING,
            self._model_mapping_cb, mapping_qos)
        self.create_subscription(
            ObjectTrackArray, TRACKS, self._tracks_cb, 10)
        self.create_subscription(
            DetectionArray, PERCEPTION_DETECTIONS, self._det_cb, 10)
        self.create_subscription(
            PoseInfo, STATE_ODOM, self._pose_cb, 10)

        # ZIT6 MCU 状态 (status check 用)
        self._mcu_status = ZitStatus()
        self._mcu_status_rcvd = False
        self.create_subscription(
            ZitStatus, ZIT6_STATUS, self._mcu_status_cb, 10)

        # Publishers
        self.pub_status = self.create_publisher(TaskStatus, MISSION_STATUS, 10)
        self.pub_status_legacy = self.create_publisher(
            TaskStatus, LEGACY_TASK_STATUS, 10)
        self.pub_light = self.create_publisher(UInt8, ZIT6_LIGHT, 10)
        self.pub_servo = self.create_publisher(ZitServo, ZIT6_SERVO, 10)
        # A task runner can be interrupted while BasicMotion is still
        # executing a goal.  Cancel the goal; BasicMotion owns the neutral
        # velocity stop and its velocity lease watchdog.
        self.context.on_shutdown(self._stop_active_motion)

        # Services
        self.create_service(RunTask, MISSION_RUN, self._run_task_cb)
        self.create_service(Trigger, MISSION_STOP, self._stop_task_cb)
        self.create_service(ExecTask, MISSION_EXECUTE, self._exec_task_cb)
        # Thin compatibility aliases. New nodes use /auv/mission/*.
        self.create_service(RunTask, LEGACY_TASK_RUN, self._run_task_cb)
        self.create_service(Trigger, LEGACY_TASK_STOP, self._stop_task_cb)
        self.create_service(ExecTask, LEGACY_TASK_EXECUTE, self._exec_task_cb)

        # Status timer
        self.create_timer(0.5, self._publish_status)

        self.get_logger().info('TaskRunner 节点已启动')
        self.get_logger().info(f'调试模式：{self._debug_mode}')

    def _model_mapping_cb(self, message):
        try:
            registry = ModelClassRegistry.from_message(message)
            self._model_mapping = registry
            self._localizer_target_class_ids = {}
            for class_name, target_name in (
                    ('collection_frame_down', 'collection_frame'),
                    ('collection_frame_front', 'collection_frame'),
                    ('target_rack_down', 'target_rack'),
                    ('target_rack_front', 'target_rack')):
                class_id = registry.model_class_id(class_name, required=False)
                if class_id is not None:
                    self._localizer_target_class_ids[class_id] = target_name
            self._target_rack_down_class_id = registry.model_class_id(
                'target_rack_down', required=False)
            self._model_mapping_ready.set()
            self.get_logger().info(
                'task_runner received model mapping {}'.format(registry.model))
        except Exception as error:
            self.get_logger().error(
                'invalid model class mapping: {}'.format(error))

    def wait_for_model_mapping(self, timeout_sec=10.0):
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while not self._model_mapping_ready.is_set() and rclpy.ok():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            rclpy.spin_once(self, timeout_sec=min(0.1, remaining))
        return self._model_mapping_ready.is_set()

    def _tracks_cb(self, msg: ObjectTrackArray):
        with self._perception_lock:
            self.object_tracks = msg

    def _det_cb(self, msg: DetectionArray):
        if not bind_detection_geometry(self, msg):
            return
        camera_name = str(msg.camera_name).strip().lower()
        if camera_name not in ('down_left', 'down_right'):
            return
        with self._perception_lock:
            received_at = time.monotonic()
            self._down_detections[camera_name] = (received_at, msg)
            self._down_detection_sequence += 1
            self._down_detection_events.append((
                self._down_detection_sequence, received_at, camera_name, msg))

    def _pose_cb(self, msg: PoseInfo):
        with self._perception_lock:
            self._robot_pose = (msg.robot_x, msg.robot_y, msg.robot_z,
                                msg.robot_roll, msg.robot_pitch, msg.robot_yaw)

    def _mcu_status_cb(self, msg: ZitStatus):
        self._mcu_status = msg
        self._mcu_status_rcvd = True

    def _stop_active_motion(self):
        """Cancel an in-flight BasicMotion goal and neutralize ZIT6."""
        if self._motion_stop_sent:
            return
        self._motion_stop_sent = True
        self.stopped = True
        goal_handle = self._active_goal_handle
        self._active_goal_handle = None
        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
                self.get_logger().warning(
                    '任务执行器关闭：已请求取消当前 BasicMotion 目标')
            except Exception as exc:
                self.get_logger().warning(
                    f'任务执行器关闭：取消 BasicMotion 目标失败：{exc}')
        self.get_logger().warning(
            '任务执行器关闭：由 BasicMotion 负责速度租约超时和零速度保护')

    # ── 灯光 / 舵机控制 ────────────────────────────────────────────

    def set_light(self, color: int, label: str, *, log: bool = True):
        color = int(color)
        msg = UInt8(data=color)
        self.pub_light.publish(msg)
        self._light_state = color
        if log:
            self.get_logger().info(f'💡 灯光已打开：{label}（数值={color}）')

    def light_off(self):
        msg = UInt8(data=0)
        self.pub_light.publish(msg)
        self._light_state = self.LIGHT_OFF
        self.get_logger().info('💡 灯光已关闭')

    def _set_task_phase_light(self, color: int, label: str, *, log: bool = True):
        """Publish a task phase color only when it changes."""
        color = int(color)
        if self._light_state != color:
            self.set_light(color, label, log=log)

    def _pulse_task_light(self, color: int, label: str, *,
                          duration: float = 1.0,
                          restore_color: int = LIGHT_YELLOW):
        """Show a timed search result and restore the active phase color."""
        self.set_light(color, label)
        deadline = time.monotonic() + max(0.0, float(duration))
        while (rclpy.ok() and not self.stopped
               and time.monotonic() < deadline):
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
        if rclpy.ok() and not self.stopped:
            self._set_task_phase_light(
                restore_color, f'{label}提示结束，恢复阶段灯')

    def set_servo(self, angle_rad: float, label: str,
                  servo_id: int = SERVO_ID_GOLF):
        msg = ZitServo()
        msg.servo_id = int(servo_id)
        msg.angle = float(angle_rad)
        self.pub_servo.publish(msg)
        self.get_logger().info(
            f'⚙️  舵机：{label}（编号={msg.servo_id}，'
            f'目标值={angle_rad:.2f}）')

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

    def _refresh_camera_extrinsics(self):
        try:
            snapshot = self.camera_extrinsics_provider.snapshot()
        except CameraExtrinsicsUnavailable as error:
            self.camera_extrinsics = {}
            self._down_offset_left = np.zeros(3, dtype=np.float64)
            self._down_offset_right = np.zeros(3, dtype=np.float64)
            self._down_optical_to_body = np.eye(3, dtype=np.float64)
            self._down_optical_to_body_right = np.eye(3, dtype=np.float64)
            if not self._camera_tf_warned:
                self.get_logger().warning(
                    f'等待完整相机 TF 树，视觉任务暂不可用：{error}')
                self._camera_tf_warned = True
            return

        self.camera_extrinsics = snapshot
        left = snapshot['down_left']
        right = snapshot['down_right']
        self._down_offset_left = left.translation.copy()
        self._down_offset_right = right.translation.copy()
        self._down_optical_to_body = left.optical_to_body.copy()
        self._down_optical_to_body_right = right.optical_to_body.copy()
        self._camera_tf_warned = False

    def _ensure_camera_extrinsics(self) -> bool:
        """Lazily start TF retries when a camera task actually needs them."""
        if self.camera_extrinsics:
            return True
        if self._camera_tf_timer is None:
            self._camera_tf_timer = self.create_timer(
                float(self.get_parameter('camera_tf_retry_period_sec').value),
                self._refresh_camera_extrinsics)
        self._refresh_camera_extrinsics()
        return bool(self.camera_extrinsics)

    def _triangulate(self, class_id: int):
        if not self.camera_extrinsics:
            return None
        pair = self._stereo_pair(class_id)
        with self._perception_lock:
            pose = self._robot_pose
        if pair is None or pose is None:
            return None
        ld, rd = pair
        rx, ry, rz, roll, pitch, yaw = pose
        R = _euler_to_rotation_matrix(roll, pitch, yaw)
        rp = np.array([rx, ry, rz])

        def _ray(name, detection, off, optical_to_body):
            xy = normalized_detection(self, name, detection)
            vc = np.array([*xy, 1.0])
            vc /= np.linalg.norm(vc)
            vb = optical_to_body @ vc
            vw = R @ vb
            vw /= np.linalg.norm(vw)
            return rp + R @ off, vw

        lo, ld_ray = _ray(
            'down_left', ld, self._down_offset_left,
            self._down_optical_to_body)
        ro, rd_ray = _ray(
            'down_right', rd, self._down_offset_right,
            self._down_optical_to_body_right)
        pos = _ray_intersection_midpoint(lo, ld_ray, ro, rd_ray)
        return (float(pos[0]), float(pos[1]), float(pos[2])) \
            if pos is not None else None

    def _search_for_class(self, class_id: int, label: str) -> bool:
        if not self.camera_extrinsics:
            return None
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
        if not self._ensure_camera_extrinsics():
            return False
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
        tasks = load_mission_or_task(
            path, class_registry=self._model_mapping)
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

    @staticmethod
    def _pose_axis_selected(axes: str, axis: str) -> bool:
        text = str(axes or '').strip().lower()
        if not text:
            return True
        if axis == 'rz':
            return 'rz' in text
        return axis in text.replace('rz', '')

    def _update_command_tracker_from_pose(self, pose: dict):
        """Update the local command pose after an initial pose override."""
        command = str(pose['command']).upper()
        x, y, z, yaw = (float(value) for value in pose['target'])
        axes = pose.get('axes', '')

        if command in {'SET', 'WMOVE', 'WTRAVEL'}:
            if self._pose_axis_selected(axes, 'x'):
                self._cmd_x = x
            if self._pose_axis_selected(axes, 'y'):
                self._cmd_y = y
            if self._pose_axis_selected(axes, 'z'):
                self._cmd_z = z
            if self._pose_axis_selected(axes, 'rz'):
                self._cmd_yaw = yaw
            return

        # BMOVE/BTRAVEL targets are body-frame offsets.  BTRAVEL ignores yaw;
        # BMOVE applies all four values because its action endpoint does too.
        heading = math.radians(self._cmd_yaw)
        self._cmd_x += math.cos(heading) * x - math.sin(heading) * y
        self._cmd_y += math.sin(heading) * x + math.cos(heading) * y
        self._cmd_z += z
        if command == 'BMOVE':
            self._cmd_yaw = self._wrap_yaw_degrees(self._cmd_yaw + yaw)

    def _execute_initial_pose(self, task_name: str, pose: dict) -> TaskOutcome:
        """Execute a mission-level initial/failure-transferred pose."""
        if task_name == 'start':
            self.get_logger().info(
                'start：跳过初始位姿，先由 START 建立 odom 原点')
            return TaskOutcome.ok()

        command_types = {
            'SET': BasicMotion.Goal.SET,
            'WMOVE': BasicMotion.Goal.WMOVE,
            'BMOVE': BasicMotion.Goal.BMOVE,
            'WTRAVEL': BasicMotion.Goal.WTRAVEL,
            'BTRAVEL': BasicMotion.Goal.BTRAVEL,
        }
        command = str(pose['command']).upper()
        axes = str(pose.get('axes', ''))
        target = list(pose['target'])
        self.get_logger().info(
            f'{task_name}：执行初始位姿 {command}，axes={axes or "all"}，'
            f'target={[round(float(value), 3) for value in target]}')
        try:
            success, message = self._send_action_goal(
                command_types[command],
                target,
                axes,
                task_context=self._format_motion_context(
                    f'{task_name}初始位姿'),
                light_pattern=(self.LIGHT_RED, self.LIGHT_YELLOW),
                light_interval=1.0)
        except Exception as exc:
            return TaskOutcome.failed(
                f'{task_name}.exception', str(exc))
        finally:
            if rclpy.ok() and not self.stopped:
                self._set_task_phase_light(
                    self.LIGHT_YELLOW, f'{task_name}初始动作结束')
        if not success:
            code = (
                f'{task_name}.timeout'
                if self._last_motion_failure_kind == 'timeout'
                else f'{task_name}.motion')
            return TaskOutcome.failed(code, message)

        self._update_command_tracker_from_pose(pose)
        return TaskOutcome.ok()

    def _select_failure_override(self, task: dict, failure_code: str):
        return select_failure_override(task, failure_code)

    def _fallback_failure_outcome(
            self, task_name: str, stage: str = 'motion', message: str = ''):
        if self._last_motion_failure_kind == 'timeout':
            code = f'{task_name}.timeout'
        else:
            code = f'{task_name}.{stage}'
        return TaskOutcome.failed(code, message or self._last_motion_failure_message)

    def run_task_list(self):
        """Execute all tasks sequentially."""
        self.running = True
        self._mission_status_code = TaskStatus.STATUS_RUNNING
        self.stopped = False
        self.current_index = 0
        self._last_failure_code = ''
        self._last_failure_message = ''
        pending_failure_override = None
        total = len(self.tasks)
        self.get_logger().info(f'=== 任务列表开始执行（共 {total} 个任务）===')

        while self.current_index < total and not self.stopped:
            task = self.tasks[self.current_index]
            name = task.get('name', 'unknown')
            params, initial_pose = apply_failure_override(
                task.get('params', {}),
                task.get('initial_pose'),
                pending_failure_override,
            )
            self._current_task_name = str(name)
            self._current_task_step = self.current_index + 1

            self.get_logger().info(
                f'[{self.current_index + 1}/{total}] {name} 参数={params} '
                f'| 指令位姿=({self._cmd_x:.2f}, {self._cmd_y:.2f}, '
                f'{self._cmd_z:.2f}, {self._cmd_yaw:.1f}°)')

            try:
                outcome = self._execute_task(
                    name, params, initial_pose=initial_pose)
                if not outcome:
                    self._last_failure_code = outcome.failure_code
                    self._last_failure_message = outcome.message
                    self.get_logger().warn(
                        f'[{self.current_index + 1}/{total}] {name} 执行失败：'
                        f'code={outcome.failure_code}，{outcome.message}')
                    pending_failure_override = None
                    if self.current_index + 1 < total:
                        pending_failure_override = self._select_failure_override(
                            task, outcome.failure_code)
                        if pending_failure_override is not None:
                            outcome = outcome.with_transfer(
                                pending_failure_override)
                            self.get_logger().warn(
                                f'{name}：失败覆盖 {outcome.failure_code} '
                                '将传递给下一任务')
                else:
                    pending_failure_override = None
            except Exception as e:
                outcome = TaskOutcome.failed(
                    f'{name}.exception', str(e))
                self._last_failure_code = outcome.failure_code
                self._last_failure_message = outcome.message
                pending_failure_override = None
                self.get_logger().error(
                    f'[{self.current_index + 1}/{total}] {name} 发生异常：'
                    f'code={outcome.failure_code}，{outcome.message}')

                if self.current_index + 1 < total:
                    pending_failure_override = self._select_failure_override(
                        task, outcome.failure_code)
                    if pending_failure_override is not None:
                        outcome = outcome.with_transfer(
                            pending_failure_override)
                        self.get_logger().warn(
                            f'{name}：异常覆盖 {outcome.failure_code} '
                            '将传递给下一任务')

            self.current_index += 1

        self.running = False
        if self.stopped:
            self._mission_status_code = TaskStatus.STATUS_PAUSED
        elif self._last_failure_code:
            self._mission_status_code = TaskStatus.STATUS_ERROR
        else:
            self._mission_status_code = TaskStatus.STATUS_DONE
        self._current_task_name = ''
        self._current_task_step = 0
        if self.stopped:
            self.get_logger().warn(f'=== 任务列表已停止，位置 {self.current_index}/{total} ===')
        else:
            self.get_logger().info(f'=== 任务列表执行完成（{total}/{total}）===')

    def _execute_task(
            self, name: str, params: dict,
            initial_pose: dict | list[dict] | None = None) -> TaskOutcome:
        """Execute one task and normalize its result to ``TaskOutcome``."""
        self._last_motion_failure_kind = ''
        self._last_motion_failure_message = ''
        self._target_light_indications = set()
        self._task_failure_light_handled = False

        if initial_pose is not None:
            initial_poses = (
                initial_pose if isinstance(initial_pose, list)
                else [initial_pose])
            for pose in initial_poses:
                outcome = self._execute_initial_pose(name, pose)
                if not outcome:
                    self._set_task_phase_light(
                        self.LIGHT_RED, f'{name} 初始动作失败')
                    return outcome

        self._last_basic_motion_test_outcome = None
        handler = self.task_map.get(name)
        if handler is None:
            self.get_logger().warn(f'未知任务：{name}')
            outcome = TaskOutcome.failed(f'{name}.motion', '未知任务')
            self._set_task_phase_light(self.LIGHT_RED, f'{name} 执行失败')
            return outcome

        try:
            raw_outcome = handler(params)
        except Exception as exc:
            outcome = TaskOutcome.failed(f'{name}.exception', str(exc))
        else:
            if isinstance(raw_outcome, TaskOutcome):
                if raw_outcome.success or raw_outcome.failure_code:
                    outcome = raw_outcome
                else:
                    outcome = self._fallback_failure_outcome(name)
            elif raw_outcome:
                outcome = TaskOutcome.ok()
            else:
                outcome = self._fallback_failure_outcome(name)

        if not outcome and not self._task_failure_light_handled:
            self._set_task_phase_light(self.LIGHT_RED, f'{name} 执行失败')
        return outcome

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
            BasicMotion.Goal.BODY_VELOCITY: '执行机体速度租约',
        }
        purpose = purposes.get(cmd_type, '执行运动指令')
        return f'{purpose}({axes or "all"})'

    def _send_action_goal(self, cmd_type, target, axes='', timeout=60.0,
                          quiet=False, task_context='', velocity_lease=0.0,
                          light_color=None, light_pattern=None,
                          light_interval=1.0, cruise_speed=0.0, wait_deadline=None,
                          cancel_wait_timeout=0.0):
        """Send a BasicMotion action goal and wait for completion (blocking).

        Polls the future in a loop since this runs in a daemon thread while
        the main thread's SingleThreadedExecutor processes DDS events.

        Args:
            cmd_type: BasicMotion.Goal.{START,SET,WMOVE,BMOVE,WTRAVEL,BTRAVEL,
                BODY_VELOCITY}
            target: list of 4 floats [x, y, z, yaw] (yaw in degrees)
            axes: which axes to move (empty = all)
            timeout: max time in seconds (0 = server default 60s)
            velocity_lease: lease duration for BODY_VELOCITY commands
            quiet: if True, suppress per-goal INFO logs (errors still logged)
            task_context: context shown by basic_motion; empty uses the standard
                current-task/current-step/action context.

        Returns:
            (success: bool, message: str)
        """
        type_names = {
            1: 'WMOVE', 2: 'BMOVE', 3: 'SET', 4: 'WTRAVEL',
            5: 'BTRAVEL', 6: 'START', 7: 'BODY_VELOCITY', 8: 'BLINE',
        }
        type_name = type_names.get(cmd_type, f'UNKNOWN({cmd_type})')
        if cmd_type != BasicMotion.Goal.START and not light_pattern:
            phase_color = (
                self.LIGHT_YELLOW if light_color is None
                else int(light_color))
            self._set_task_phase_light(
                phase_color, f'{type_name} 运动阶段')
        self._last_motion_failure_kind = ''
        self._last_motion_failure_message = ''
        self._last_motion_final_target = None
        self._last_motion_cleanup_confirmed = True
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

        server_wait = 2.0 if wait_deadline is None else max(0.0, min(2.0, wait_deadline-time.monotonic()))
        if not self._action_client.wait_for_server(timeout_sec=server_wait):
            self.get_logger().error('动作服务器不可用')
            self._last_motion_failure_kind = 'motion'
            self._last_motion_failure_message = '动作服务器不可用'
            return False, '动作服务器不可用'

        if wait_deadline is not None:
            remaining = wait_deadline-time.monotonic()
            if remaining <= 0:
                self._last_motion_failure_kind = 'timeout'
                self._last_motion_failure_message = '动作等待截止时间已到'
                return False, self._last_motion_failure_message
            if effective_timeout > 0:
                effective_timeout = min(effective_timeout, remaining)

        goal = BasicMotion.Goal()
        goal.cmd_type = cmd_type
        goal.axes = axes
        goal.target = target
        goal.timeout = float(effective_timeout)
        goal.task_context = task_context
        goal.velocity_lease = float(velocity_lease)
        goal.cruise_speed = float(cruise_speed)

        send_future = self._action_client.send_goal_async(goal)
        self._last_motion_cleanup_confirmed = False
        blink_colors = tuple(int(color) for color in (light_pattern or ()))
        blink_interval = max(0.05, float(light_interval))
        blink_index = 0
        next_blink_at = time.monotonic() + blink_interval
        if blink_colors:
            self._set_task_phase_light(
                blink_colors[blink_index], f'{type_name} 初始动作闪灯',
                log=False)

        def update_blink_light():
            nonlocal blink_index, next_blink_at
            if blink_colors and time.monotonic() >= next_blink_at:
                blink_index = (blink_index + 1) % len(blink_colors)
                next_blink_at = time.monotonic() + blink_interval
                self._set_task_phase_light(
                    blink_colors[blink_index], f'{type_name} 初始动作闪灯',
                    log=False)

        def cancel_and_confirm(handle, result_future=None, cleanup_deadline=None):
            try:
                if not handle.accepted:
                    return True
                if result_future is None:
                    result_future = handle.get_result_async()
                if not result_future.done():
                    handle.cancel_goal_async()
                if cleanup_deadline is not None:
                    while rclpy.ok() and not result_future.done() and time.monotonic() < cleanup_deadline:
                        time.sleep(0.01)
                # A cancel acknowledgement alone is not a terminal result.
                if result_future.done():
                    result_future.result()
                    return True
            except Exception:
                pass
            return False

        def cancel_late_goal(future):
            try:
                # This callback may run on the executor: never block it.
                handle = future.result()
                if handle.accepted:
                    handle.cancel_goal_async()
            except Exception:
                pass

        def abandon_send():
            if cancel_wait_timeout <= 0:
                send_future.add_done_callback(cancel_late_goal)
                return
            cleanup_deadline = time.monotonic()+cancel_wait_timeout
            while rclpy.ok() and not send_future.done() and time.monotonic() < cleanup_deadline:
                time.sleep(0.01)
            if send_future.done():
                try:
                    handle = send_future.result()
                    self._active_goal_handle = handle if handle.accepted else None
                    self._last_motion_cleanup_confirmed = cancel_and_confirm(
                        handle, cleanup_deadline=cleanup_deadline)
                    if self._last_motion_cleanup_confirmed:
                        self._active_goal_handle = None
                except Exception:
                    self._last_motion_cleanup_confirmed = False
            else:
                send_future.add_done_callback(cancel_late_goal)

        def within_deadline():
            return wait_deadline is None or time.monotonic() < wait_deadline

        while rclpy.ok() and not self.stopped and within_deadline() and not send_future.done():
            update_blink_light()
            time.sleep(0.01)
        if not rclpy.ok() or self.stopped:
            abandon_send()
            if not quiet:
                self.get_logger().warn(f'动作目标被中断（stopped={self.stopped}）')
            self._last_motion_failure_kind = 'motion'
            self._last_motion_failure_message = '已停止'
            return False, '已停止'
        if not send_future.done():
            # Cancel a late acceptance too; a timed-out goal must not start later.
            abandon_send()
            self.get_logger().error('发送动作目标超时')
            self._last_motion_failure_kind = 'timeout'
            self._last_motion_failure_message = '发送动作目标超时'
            return False, '发送动作目标超时'

        goal_handle = send_future.result()
        self._active_goal_handle = goal_handle
        if not goal_handle.accepted:
            self._last_motion_cleanup_confirmed = True
            self._active_goal_handle = None
            self.get_logger().error('动作目标被服务器拒绝')
            self._last_motion_failure_kind = 'motion'
            self._last_motion_failure_message = '动作目标被服务器拒绝'
            return False, '动作目标被服务器拒绝'

        if not quiet:
            self.get_logger().info('动作目标已接受，等待执行结果……')

        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not self.stopped and within_deadline() and not result_future.done():
            update_blink_light()
            time.sleep(0.01)
        completed_in_time = result_future.done() and (cancel_wait_timeout <= 0 or within_deadline())
        interrupted = not rclpy.ok() or self.stopped
        if not result_future.done():
            self._last_motion_cleanup_confirmed = cancel_and_confirm(
                goal_handle, result_future,
                time.monotonic()+cancel_wait_timeout if cancel_wait_timeout > 0 else None)
        else:
            self._last_motion_cleanup_confirmed = True
        if self._last_motion_cleanup_confirmed or cancel_wait_timeout <= 0:
            self._active_goal_handle = None
        if interrupted:
            if not quiet:
                self.get_logger().warn(f'动作结果等待被中断（stopped={self.stopped}）')
            self._last_motion_failure_kind = 'motion'
            self._last_motion_failure_message = '已停止'
            return False, '已停止'
        if not completed_in_time:
            self.get_logger().error('等待动作结果超时')
            self._last_motion_failure_kind = 'timeout'
            self._last_motion_failure_message = '等待动作结果超时'
            return False, '等待动作结果超时'

        result = result_future.result().result
        if result.success:
            self._last_motion_final_target = list(result.final_target)
        if not result.success:
            t_str = ', '.join(f'{v:.2f}' for v in target)
            self.get_logger().error(
                f'{type_name} 执行失败：{result.message}，'
                f'目标=[{t_str}]')
            message = str(result.message)
            self._last_motion_failure_kind = (
                'timeout' if '超时' in message.lower()
                or 'timeout' in message.lower() else 'motion')
            self._last_motion_failure_message = message
        elif not quiet:
            self.get_logger().info('动作执行结果：成功')
        return result.success, result.message

    def _send_body_velocity(self, forward_mps: float = 0.0,
                            lateral_mps: float = 0.0,
                            vertical_mps: float = 0.0,
                            yaw_rate_deg_s: float = 0.0,
                            *, lease_s: float = 0.25,
                            quiet: bool = True,
                            task_context: str = '', light_color=None, wait_deadline=None,
                            cancel_wait_timeout=0.0):
        """Renew the BasicMotion body-velocity lease.

        Tasks deliberately do not construct or publish ``ZitSetpoint``
        messages.  BasicMotion owns the ZIT6 protocol and stops the command
        if this lease is not renewed in time.
        """
        return self._send_action_goal(
            BasicMotion.Goal.BODY_VELOCITY,
            [forward_mps, lateral_mps, vertical_mps, yaw_rate_deg_s],
            axes='xyzrz', timeout=0.0, quiet=quiet,
            task_context=task_context, velocity_lease=lease_s,
            light_color=light_color, wait_deadline=wait_deadline,
            cancel_wait_timeout=cancel_wait_timeout)

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

    def _task_basic_motion_test(self, p: dict) -> TaskOutcome:
        self._set_task_phase_light(
            self.LIGHT_YELLOW, 'basic_motion_test 运动阶段')
        outcome = self._basic_motion_test.run(p)
        self._last_basic_motion_test_outcome = outcome
        if not outcome:
            # End a containing mission as well; never continue after this test fails.
            self.stopped = True
            self.get_logger().error(
                f'basic_motion_test FAIL: {outcome.failure_code}: {outcome.message}')
        return outcome

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
            self.state.reset_pickup(start_xy=(0.0, 0.0))
        else:
            self.get_logger().error(f'START 执行失败：{msg}')
        return success

    def _task_return_origin(self, p: dict) -> bool:
        """开环上浮到目标 z：按起始深度和速度估算持续时间。"""
        target_z = float(p.get('ascent_target_z_m', -0.3))
        ascent_speed = abs(float(p.get('ascent_speed_mps', 0.05)))
        timeout = max(0.1, float(p.get('ascent_timeout', 30.0)))
        publish_period = max(
            0.02, float(p.get('ascent_publish_period', 0.05)))
        start_z = float(self._latest_robot_pose()[2])

        if (not all(math.isfinite(value) for value in
                    (target_z, ascent_speed, timeout, publish_period, start_z))
                or ascent_speed <= 0.0):
            self._last_motion_failure_kind = 'motion'
            self._last_motion_failure_message = 'return_origin 上浮参数无效'
            self.get_logger().error(self._last_motion_failure_message)
            return False

        ascent_distance = start_z - target_z
        if ascent_distance <= 0.0:
            self.get_logger().info(
                f'return_origin：当前 z={start_z:.3f}m 已到达或高于目标 '
                f'z={target_z:.3f}m，无需上浮')
            return True

        duration = ascent_distance / ascent_speed
        if duration > timeout:
            self._last_motion_failure_kind = 'timeout'
            self._last_motion_failure_message = (
                f'开环上浮预计需要 {duration:.1f}s，超过超时限制 {timeout:.1f}s')
            self.get_logger().error(self._last_motion_failure_message)
            return False

        lease_s = max(0.25, publish_period * 4.0)
        deadline = time.monotonic() + duration
        self.get_logger().info(
            f'return_origin：开环上浮 z={start_z:.3f}→{target_z:.3f}m，'
            f'速度={ascent_speed:.3f}m/s，预计 {duration:.1f}s')
        ascent_ok = True
        ascent_message = ''
        try:
            while not self.stopped and time.monotonic() < deadline:
                ascent_ok, ascent_message = self._send_body_velocity(
                    vertical_mps=-ascent_speed,
                    lease_s=lease_s,
                    task_context=self._format_motion_context(
                        'return_origin 开环上浮'))
                if not ascent_ok:
                    break
                remaining = deadline - time.monotonic()
                if remaining > 0.0:
                    time.sleep(min(publish_period, remaining))
        finally:
            stop_ok, stop_message = self._send_body_velocity(
                lease_s=lease_s,
                task_context=self._format_motion_context(
                    'return_origin 上浮结束，发送零速度'))

        if not ascent_ok:
            self.get_logger().error(
                f'return_origin：上浮速度指令失败：{ascent_message}')
            return False
        if not stop_ok:
            self._last_motion_failure_kind = 'motion'
            self._last_motion_failure_message = stop_message
            self.get_logger().error(
                f'return_origin：停止上浮速度失败：{stop_message}')
            return False
        if self.stopped:
            self._last_motion_failure_kind = 'motion'
            self._last_motion_failure_message = 'return_origin 上浮被停止'
            return False

        self._cmd_z = target_z
        self.get_logger().info(
            f'return_origin：开环上浮完成，估算目标 z={target_z:.3f}m')
        return True

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

    def _task_bline(self, p: dict) -> bool:
        success, message = self._send_action_goal(
            BasicMotion.Goal.BLINE,
            [float(p.get('dx', 1.0)), float(p.get('dy', 0.0)),
             float(p.get('dz', 0.0)), 0.0], 'xyz',
            timeout=float(p.get('timeout', 0.0)),
            cruise_speed=float(p.get('speed_mps', 0.15)))
        if success:
            target = self._last_motion_final_target
            if (target is None or len(target) != 4
                    or not all(math.isfinite(value) for value in target)):
                self._last_motion_failure_kind = 'motion'
                self._last_motion_failure_message = 'BLINE 返回的 final_target 无效'
                self.get_logger().error(self._last_motion_failure_message)
                return False
            self._cmd_x, self._cmd_y, self._cmd_z, self._cmd_yaw = target
        self.get_logger().info(f'bline 执行结果：{message}')
        return success

    def _move_to_nearest_object_xy(self, class_id: int) -> bool:
        """SET 绝对定位到 class_id 最近物体的 XY 坐标。

        使用 /auv/perception/tracks 中的有效 track 获取 3D 位置，
        成功后更新 self._cmd_x/_cmd_y。
        """
        nearest = None
        min_dist = float('inf')
        with self._perception_lock:
            tracks = list(self.object_tracks.tracks)
        for obj in tracks:
            if (obj.class_id == class_id
                    and int(obj.status) != int(ObjectTrack.STATUS_LOST)
                    and all(math.isfinite(float(value)) for value in
                            (obj.world_x, obj.world_y, obj.world_z))):
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
        if not self._ensure_camera_extrinsics():
            return False
        follower = LineFollower(self, p)
        try:
            return follower.execute()
        finally:
            follower.destroy()

    def _task_arrow_surface(self, p: dict) -> bool:
        """执行箭头对准+出水任务 — 创建 ArrowSurfacer 子对象并运行。"""
        if not self._ensure_camera_extrinsics():
            return False
        surfacer = ArrowSurfacer(self, p)
        try:
            return surfacer.execute()
        finally:
            surfacer.destroy()

    @staticmethod
    def _wrap_yaw_degrees(value: float) -> float:
        """Wrap a yaw command to the controller's conventional range."""
        return (float(value) + 180.0) % 360.0 - 180.0

    def _latest_robot_pose(self, require_measured=False):
        """Return measured odom; callers may explicitly disallow command fallback."""
        with self._perception_lock:
            pose = self._robot_pose
        if pose is not None and all(math.isfinite(float(value)) for value in pose):
            return tuple(float(value) for value in pose)
        if require_measured:
            raise ValueError('没有有效 odom 实测位姿')
        return (
            float(self._cmd_x), float(self._cmd_y), float(self._cmd_z),
            0.0, 0.0, float(self._cmd_yaw),
        )

    def _task_pass_gates(self, p: dict) -> TaskOutcome:
        """仅用前视相机图像搜索、对准并连续通过多个门。"""
        if not self._ensure_camera_extrinsics():
            return TaskOutcome.failed(
                '26rb_pass_gates.camera_tf', '前视相机 TF 未就绪')
        gate_task = RB26GateTask(self, p)
        try:
            return gate_task.execute()
        finally:
            gate_task.destroy()

    def _task_hit_balls(self, p: dict) -> TaskOutcome:
        """执行 26rb 撞球任务模块。"""
        task = RB26HitBallsTask(self, p)
        try:
            return task.execute()
        finally:
            task.destroy()

    # ── 置物台 / target-rack 搜索任务 ─────────────────────────────

    def _normalize_localizer_target_name(self, value):
        """Normalize object_localizer labels using the received class mapping."""
        if isinstance(value, (int, np.integer)):
            return self._localizer_target_class_ids.get(int(value))

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

    def _localizer_target_name(self, target):
        """Return a canonical name for one ObjectTrack message."""
        physical_name = self._normalize_localizer_target_name(
            getattr(target, 'physical_class_name', ''))
        if physical_name is not None:
            return physical_name
        class_name = self._normalize_localizer_target_name(
            getattr(target, 'class_name', ''))
        if class_name is not None:
            return class_name
        return self._normalize_localizer_target_name(
            getattr(target, 'class_id', -1))

    def _track_measurement_age(self, track):
        stamp = track.last_measurement_stamp
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if stamp_ns <= 0:
            return 0.0
        now_ns = int(self.get_clock().now().nanoseconds)
        if now_ns <= 0:
            return 0.0
        return max(0.0, (now_ns - stamp_ns) / 1e9)

    def _best_localizer_target(self, target_name: str, p: dict):
        """Get the best current world estimate from object_localizer.

        The estimator publishes one persistent track per physical target.
        Select its best current world estimate using status, confidence,
        observation count, and the age of its last measurement.
        """
        wanted = self._normalize_localizer_target_name(target_name)
        if wanted is None:
            return None

        min_confidence = float(p.get('min_confidence', 0.02))
        min_observations = max(1, int(p.get('min_observations', 1)))
        stale_statuses = (ObjectTrack.STATUS_STALE, ObjectTrack.STATUS_LOST)
        tentative_status = ObjectTrack.STATUS_TENTATIVE
        stable_status = ObjectTrack.STATUS_STABLE
        allow_stale = bool(p.get('allow_stale_targets', True))

        with self._perception_lock:
            candidates = list(self.object_tracks.tracks)

        valid = []
        for target in candidates:
            if self._localizer_target_name(target) != wanted:
                continue
            status = int(getattr(target, 'status', tentative_status))
            if status in stale_statuses and not allow_stale:
                continue
            confidence = float(getattr(target, 'confidence', 0.0))
            observations = int(getattr(target, 'measurement_count', 0))
            x = float(getattr(target, 'world_x', float('nan')))
            y = float(getattr(target, 'world_y', float('nan')))
            z = float(getattr(target, 'world_z', float('nan')))
            age = self._track_measurement_age(target)
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
                1 if 'down' in str(getattr(
                    target, 'estimate_source', '')).strip().lower().split('+') else 0,
                target,
            ))

        if not valid:
            return None

        target = max(valid, key=lambda item: item[:-1])[-1]
        result = {
            'name': wanted,
            'x': float(target.world_x),
            'y': float(target.world_y),
            'z': float(target.world_z),
            'confidence': float(target.confidence),
            'observations': int(target.measurement_count),
            'status': int(target.status),
            'age': self._track_measurement_age(target),
            'source': str(getattr(target, 'estimate_source', 'unknown')),
            'instance_id': int(target.track_id),
        }
        if wanted not in self._target_light_indications:
            self._target_light_indications.add(wanted)
            self._pulse_task_light(
                self.LIGHT_GREEN, f'发现 {wanted} 目标', duration=1.0)
        return result

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
            elif len(found) < 2:
                self._pulse_task_light(
                    self.LIGHT_RED,
                    f'扫描航向 {heading:.1f}° 未发现目标',
                    duration=1.0)
        return any_found

    def _task_find_collection_frame(self, p: dict) -> TaskOutcome:
        """执行 26rb 置物台/台框定位任务模块。"""
        task = RB26FindCollectionFrameTask(self, p)
        return task.execute()

    def _task_drop_ball_target_rack(self, p: dict) -> TaskOutcome:
        """从目标架上方开始视觉伺服，圆盘爪对准后释放球。"""
        return RB26DropBallTargetRackTask(self, p).execute()

    def _task_grab_ball_ring(self, p: dict) -> TaskOutcome:
        return RB26GrabBallRingTask(self, p).execute()

    def _task_grab_golf(self, p: dict) -> TaskOutcome:
        """使用左右下视相机竞争单目伺服，抓取指定颜色的高尔夫球并回位复检。"""
        grab_task = RB26GrabGolfTask(self, p)
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
        try:
            start_cid = self._model_mapping.configured_class_id(
                p, 'start_class_id', 'guide_line')
        except ValueError as error:
            self.get_logger().error(f'release_sampler：{error}')
            return False
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
            if self.running or self._debug_executing:
                response.success = False
                response.message = '已有任务正在执行；请等待结束或先停止'
                return response
            path = self._resolve_mission_path(request.task_name)

            self.get_logger().info(f'服务 /auv/mission/run：从 {path} 开始执行任务')
            try:
                self.tasks = self.load_tasks(path)
            except ConfigError as exc:
                response.success = False
                response.message = f'任务配置无效：{exc}'
                self.get_logger().error(response.message)
                return response
            if self.tasks:
                self.mission_file = path
                self.set_parameters([
                    Parameter('mission_file', value=path)])
                self._motion_stop_sent = False
                self.stopped = False
                self._mission_status_code = TaskStatus.STATUS_RUNNING
                self.running = True
                thread = threading.Thread(target=self.run_task_list, daemon=True)
                thread.start()
                response.success = True
                response.message = f'已启动 {len(self.tasks)} 个任务'
            else:
                response.success = False
                response.message = '未加载任何任务'
        else:
            self.get_logger().info('服务 /auv/mission/run：收到停止请求')
            self.stopped = True
            response.success = True
            response.message = '已停止'
        return response

    def _stop_task_cb(self, request, response):
        self.get_logger().warn('服务 /auv/mission/stop：紧急停止')
        self._stop_active_motion()
        response.success = True
        response.message = '任务已停止'
        return response

    def _exec_task_cb(self, request, response):
        """Handle /auv/mission/execute (debug mode only)."""
        if not self._debug_mode:
            response.success = False
            response.message = 'ExecTask 服务仅在调试模式下可用'
            self.get_logger().warn('/auv/mission/execute 被调用，但调试模式未开启')
            return response

        if self._debug_executing or self.running:
            response.success = False
            response.message = (
                f'任务“{self._debug_task_name}”已在运行。'
                '请等待任务结束，或调用 /auv/mission/stop。'
            )
            self.get_logger().warn(f'拒绝并发 /auv/mission/execute：{self._debug_task_name}')
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

        self._debug_executing = True
        self.running = True
        self._debug_task_name = task_name
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
        self._motion_stop_sent = False
        self._debug_task_name = name
        self._current_task_name = name
        self._current_task_step = 1
        self.stopped = False
        self.running = True

        # Extract timeout override before calling execute
        timeout = params.pop('_timeout', -1.0)
        self._debug_timeout = float(timeout) if timeout > 0 else -1.0

        try:
            outcome = self._execute_task(name, params)
            if outcome:
                self.get_logger().info(f'调试执行 {name}：成功')
            else:
                self.get_logger().warn(
                    f'调试执行 {name}：失败，code={outcome.failure_code}，'
                    f'{outcome.message}')
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

        if self._debug_executing:
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
            msg.error_message = (
                f'{self._last_failure_code}: {self._last_failure_message}'
                if self._last_failure_code else '')
        elif self._last_basic_motion_test_outcome is not None:
            outcome = self._last_basic_motion_test_outcome
            msg.status = (TaskStatus.STATUS_DONE if outcome.success
                          else TaskStatus.STATUS_ERROR)
            msg.current_task_name = 'basic_motion_test'
            msg.total_tasks = 1
            msg.error_message = ('' if outcome.success else
                                 f'{outcome.failure_code}: {outcome.message}')
        elif self.stopped:
            msg.status = TaskStatus.STATUS_PAUSED
            msg.current_task_index = self.current_index
            msg.total_tasks = len(self.tasks)
        else:
            msg.status = self._mission_status_code
            msg.current_task_index = self.current_index
            msg.total_tasks = len(self.tasks)
            if self._debug_mode:
                msg.error_message = '[调试模式：空闲，等待 /auv/mission/execute]'

        self.pub_status.publish(msg)
        self.pub_status_legacy.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TaskRunnerNode()
    if not node.wait_for_model_mapping():
        node.get_logger().fatal(
            '未收到 /auv/perception/model_classes；任务执行器拒绝启动')
        node.destroy_node()
        rclpy.try_shutdown()
        raise SystemExit(2)

    if not node._debug_mode:
        # Load and validate the selected mission before exposing its services.
        # The real bringup supervisor leaves auto_start disabled and releases
        # the mission only after all upstream readiness gates pass.
        default_path = node.mission_file or str(default_mission_path())
        try:
            node.tasks = node.load_tasks(default_path)
        except ConfigError as exc:
            node.get_logger().fatal(f'无法启动任务执行器：{exc}')
            node.destroy_node()
            rclpy.try_shutdown()
            raise SystemExit(2) from exc
        if node.tasks and node._auto_start:
            thread = threading.Thread(target=node.run_task_list, daemon=True)
            thread.start()
            node.get_logger().info(f'已自动启动任务列表（共 {len(node.tasks)} 个任务）')
        elif node.tasks:
            node.get_logger().info(
                f'已加载任务列表（共 {len(node.tasks)} 个任务），等待 /auv/mission/run')
    else:
        node.get_logger().info(
            '调试模式已开启：跳过自动启动。'
            '请使用 /auv/mission/execute 服务执行单个任务。'
        )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Run the motion stop while the action client and setpoint publisher
        # are still alive; the context callback is an idempotent fallback.
        node._stop_active_motion()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
