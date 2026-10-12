"""Gate sequence with visual alignment or preset-point BLINE stages.

Visual decisions select the highest fresh bbox across both eyes. No target IDs,
tracking, cross-eye association or reacquisition are required after alignment.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time

import numpy as np
from uv_camera.image_geometry import normalized_pixel
from uv_camera.perception_geometry import bind_detection_geometry
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from uv_msgs.action import BasicMotion
from uv_msgs.msg import DetectionArray
from auv_protocol.topics import PERCEPTION_DETECTIONS
from uv_task.gate_config import validate_gate_params
from uv_task.task_outcome import TaskOutcome


def wrap_degrees(value):
    return (float(value) + 180.0) % 360.0 - 180.0


def rotation(pose):
    """Body to world rotation; measured pose angles are in degrees."""
    r, p, y = map(math.radians, pose[3:6])
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def heading(vector):
    return math.degrees(math.atan2(vector[1], vector[0]))


@dataclass(frozen=True)
class GateFrame:
    eye: str
    sequence: int
    received: float
    ray: np.ndarray
    area_percent: float
    height_percent: float


class GateFailure(RuntimeError):
    def __init__(self, kind, message):
        super().__init__(message)
        self.kind = kind


class RB26GateTask:
    def __init__(self, node, params):
        self.node = node
        self.p = validate_gate_params(params, complete=True)
        self._now = time.monotonic
        self._sleep = time.sleep
        self._lock = threading.RLock()
        self.uses_vision = '0' in self.p['fixed_pass_code'][:self.p['gate_count']]
        if self.uses_vision:
            config = node.camera_configs['front']
            self.width, self.height = config.eye_resolution
            self.k = {e: config.side(e).matrix.copy() for e in ('left', 'right')}
            self.distortion = {e: np.asarray(config.side(e).distortion) for e in ('left', 'right')}
            self.extrinsics = {e: node.camera_extrinsics[f'front_{e}'] for e in ('left', 'right')}
            self.class_id = node._model_mapping.model_class_id('gate_front', required=False)
            if self.class_id is None:
                raise ValueError('模型映射中没有 gate_front')
        self.sequence = 0
        self._last_capture_id = dict(left=None, right=None)
        self._current_frames = dict(left=(), right=())
        self._height_stop_frame = None
        self._accept_frames = False
        self._capture_floor = 0.0
        self._velocity_active = False
        self._light_color = node.LIGHT_OFF
        self._bline_active = False
        self._deadline = math.inf
        self._phase_deadline = math.inf
        self.depth = self.p['observation_poses'][0][2]
        self._observation_pose = self.p['observation_poses'][0]
        self._cleanup_unconfirmed = False
        self._gate_index = 0
        self._observe_after_pass = False
        self._stage = '准备'
        self._sub = None
        if self.uses_vision:
            self._sub = node.create_subscription(
                DetectionArray, PERCEPTION_DETECTIONS, self._detection_cb,
                QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

    def _log(self, message, warn=False):
        logger = self.node.get_logger()
        if warn:
            logger.warn(message)
        else:
            logger.info(message)

    def _pose(self):
        try:
            raw = self.node._latest_robot_pose(require_measured=True)
            if raw is None:
                raise ValueError('missing pose')
            pose = tuple(float(x) for x in raw)
        except (ValueError, TypeError, AttributeError) as exc:
            raise GateFailure('odom', '无有效实测位姿: '+str(exc))
        if len(pose) != 6 or not all(math.isfinite(x) for x in pose):
            raise GateFailure('odom', '实测位姿无效')
        return pose

    def _check(self, deadline=None):
        if self.node.stopped or not rclpy.ok():
            raise GateFailure('cancelled', '任务已取消')
        if self._now() >= self._deadline:
            raise GateFailure('timeout', f'第 {self._gate_index+1} 个门总超时（{self._stage}）')
        if deadline is not None and self._now() >= deadline:
            raise GateFailure('timeout', f'当前过门阶段超时（{self._stage}）')

    def _limit(self, seconds):
        self._check()
        self._phase_deadline = min(self._deadline, self._now()+seconds)
        return self._phase_deadline

    def _tick(self, deadline=None):
        self._check(deadline)
        self._sleep(min(self.p['search_velocity_period'], self._deadline-self._now(),
                        max(0.0, deadline-self._now()) if deadline is not None else math.inf))

    def _fresh(self, frame, now):
        return frame is not None and now-frame.received <= self.p['search_detection_timeout']

    def _reset_gate(self, depth):
        with self._lock:
            self.depth = depth
            self._current_frames = dict(left=(), right=())
            self._height_stop_frame = None
            self._last_capture_id = dict(left=None, right=None)
            self._accept_frames = False

    def _largest_frame(self):
        """Select from each eye's latest message; never reuse a tracked target."""
        with self._lock:
            now = self._now()
            frames = (frame for candidates in self._current_frames.values()
                      for frame in candidates if self._fresh(frame, now))
            return max(frames,
                       key=lambda frame: (frame.height_percent, frame.area_percent, frame.sequence),
                       default=None)

    def _camera_ray(self, eye, center, pose, image_info=None):
        matrix = image_info.k if image_info is not None else self.k[eye]
        distortion = image_info.d if image_info is not None else self.distortion[eye]
        xy = normalized_pixel(matrix, distortion, *center)
        extrinsic = self.extrinsics[eye]
        transform = rotation(pose)
        ray = transform @ extrinsic.optical_to_body @ np.array([xy[0], xy[1], 1.0])
        ray /= np.linalg.norm(ray)
        return ray

    def _detection_cb(self, msg):
        if not bind_detection_geometry(self.node, msg):
            return
        geometry = getattr(self.node, '_detection_geometry', None)
        image_info = geometry.cache.resolve(msg) if geometry is not None else None
        eye = str(msg.camera_name)
        if eye.startswith('front_'):
            eye = eye[len('front_'):]
        if eye not in self._current_frames:
            return
        with self._lock:
            if not self._accept_frames:
                return
            now = self._now()
            capture_id = int(getattr(msg, 'capture_id', 0))
            if capture_id and capture_id == self._last_capture_id[eye]:
                return
            if capture_id:
                self._last_capture_id[eye] = capture_id
            self.sequence += 1
            stamp = float(msg.header.stamp.sec)+float(msg.header.stamp.nanosec)*1e-9
            observed_at = now
            if stamp > 0:
                if stamp < self._capture_floor:
                    return
                ros_now = self.node.get_clock().now().nanoseconds*1e-9
                if ros_now-stamp > self.p['search_detection_timeout'] or stamp-ros_now > self.p['search_detection_timeout']:
                    self._current_frames[eye] = ()
                    return
                # Account for transport latency when applying observation age.
                observed_at -= max(0.0, ros_now-stamp)
            try:
                pose = self._pose()
            except GateFailure:
                self._current_frames[eye] = ()
                return
            candidates = []
            for detection in msg.detections:
                if int(detection.class_id) != self.class_id:
                    continue
                confidence = float(detection.confidence)
                raw = tuple(float(getattr(detection, key)) for key in ('bbox_x1', 'bbox_y1', 'bbox_x2', 'bbox_y2'))
                if not all(math.isfinite(x) for x in (*raw, confidence)) or confidence < self.p['search_min_confidence']:
                    continue
                x1, y1, x2, y2 = raw
                if x2 <= x1 or y2 <= y1:
                    continue
                bbox = (max(0.0, min(self.width, x1)), max(0.0, min(self.height, y1)),
                        max(0.0, min(self.width, x2)), max(0.0, min(self.height, y2)))
                x1, y1, x2, y2 = bbox
                if min(x2-x1, y2-y1) < self.p['search_min_bbox_pixels']:
                    continue
                if image_info is not None:
                    self.k[eye] = np.asarray(image_info.k).reshape(3, 3)
                ray = self._camera_ray(eye, [(x1+x2)/2, (y1+y2)/2], pose, image_info)
                if np.linalg.norm(ray[:2]) < 1e-6:
                    continue
                candidate = GateFrame(eye, self.sequence, observed_at, ray,
                                      100*(x2-x1)*(y2-y1)/(self.width*self.height),
                                      100*(y2-y1)/self.height)
                candidates.append(candidate)
            # An empty/new message replaces all previous boxes for this eye.
            self._current_frames[eye] = tuple(candidates)

    def _yaw_error(self, frame):
        return wrap_degrees(heading(frame.ray)-self._pose()[5])

    def _light(self, color, label):
        self._light_color = color
        self.node._set_task_phase_light(color, label, log=False)

    def _velocity(self, horizontal=(0.0, 0.0), yaw_rate=0.0, color=None, wait_deadline=None):
        self._check()
        if self._bline_active:
            raise GateFailure('handoff', 'BLINE 执行中禁止旧伺服速度输出')
        pose = self._pose()
        vz = float(np.clip((self.depth-pose[2])*self.p['depth_kp'],
                           -self.p['depth_max_speed_mps'], self.p['depth_max_speed_mps']))
        world = np.array([*horizontal, vz])
        body = rotation(pose).T @ world
        self._velocity_active = True
        self._light_color = self.node.LIGHT_OFF if color is None else color
        deadline = min(self._deadline, self._phase_deadline if wait_deadline is None else wait_deadline)
        ok, message = self.node._send_body_velocity(
            *body.tolist(), yaw_rate_deg_s=float(yaw_rate),
            lease_s=max(0.25, 3*self.p['search_velocity_period']),
            task_context='26rb_gate_task 定深视觉伺服',
            light_color=self._light_color,
            wait_deadline=deadline, cancel_wait_timeout=2.0)
        if not getattr(self.node, '_last_motion_cleanup_confirmed', True):
            self._cleanup_unconfirmed = True
        if not ok:
            self._check(deadline)
            raise GateFailure('motion', message)

    def _position_hold_velocity(self, target_xy):
        """Keep the gate-search vehicle at its observation XY while yawing."""
        pose = self._pose()
        error = np.asarray(target_xy, dtype=float)-np.asarray(pose[:2], dtype=float)
        distance = float(np.linalg.norm(error))
        if distance <= self.p['search_position_hold_tolerance_m']:
            return (0.0, 0.0)
        velocity = error*self.p['search_position_hold_kp']
        speed = float(np.linalg.norm(velocity))
        if speed > self.p['search_position_hold_max_speed_mps']:
            velocity *= self.p['search_position_hold_max_speed_mps']/speed
        return float(velocity[0]), float(velocity[1])

    def _neutral(self, wait_deadline=None):
        if self._velocity_active:
            ok, message = self.node._send_body_velocity(
                task_context='26rb_gate_task 结束速度输出', light_color=self.node.LIGHT_OFF,
                wait_deadline=min(self._deadline, self._now()+2.0) if wait_deadline is None else wait_deadline,
                cancel_wait_timeout=2.0)
            self._velocity_active = False
            self._light_color = self.node.LIGHT_OFF
            if not getattr(self.node, '_last_motion_cleanup_confirmed', True):
                self._cleanup_unconfirmed = True
            if not ok:
                raise GateFailure('motion', f'停止速度失败: {message}')

    def _action(self, command, target, axes, deadline, **kwargs):
        self._check(deadline)
        kwargs.setdefault('light_color', self.node.LIGHT_OFF)
        ok, message = self.node._send_action_goal(
            command, [float(x) for x in target], axes=axes,
            timeout=max(0.001, deadline-self._now()), wait_deadline=deadline,
            task_context=f'26rb_gate_task 第 {self._gate_index+1} 门 {self._stage}',
            cancel_wait_timeout=2.0, **kwargs)
        if not getattr(self.node, '_last_motion_cleanup_confirmed', True):
            self._cleanup_unconfirmed = True
        self._check(deadline)
        if not ok:
            kind = 'timeout' if '超时' in message or 'timeout' in message.lower() else 'motion'
            raise GateFailure(kind, message)

    def _depth_hold(self, seconds, color=None):
        # Lamp duration is not an action acknowledgement timeout. Keep the
        # caller's phase deadline intact and allow each hold command up to 2s.
        until = min(self._deadline, self._now()+seconds)
        while self._now() < until:
            self._check()
            self._velocity(color=color, wait_deadline=self._now()+2.0)
            if self._now() < until:
                self._tick(until)

    def _flash(self, color, count, label="门框观察闪灯"):
        flash_async = getattr(self.node, '_flash_task_light', None)
        if callable(flash_async):
            flash_async(
                color, count, label,
                pulse_seconds=self.p['search_light_pulse_seconds'],
                gap_seconds=self.p['search_light_gap_seconds'],
                restore=False,
                initial_gap_seconds=self.p['search_light_gap_seconds'])
            return

        # Keep standalone adapters compatible when they lack the node light worker.
        self._neutral()
        self._light(self.node.LIGHT_OFF, '闪灯开始')
        self._depth_hold(self.p['search_light_gap_seconds'], self.node.LIGHT_OFF)
        for index in range(count):
            self._light(color, label)
            self._depth_hold(self.p['search_light_pulse_seconds'], color)
            self._light(self.node.LIGHT_OFF, '闪灯间隔')
            self._depth_hold(self.p['search_light_gap_seconds'], self.node.LIGHT_OFF)
        self._neutral()

    def _observe(self, seconds, deadline, color=None):
        self._phase_deadline = deadline
        if color is None:
            color = self.node.LIGHT_YELLOW
        hold_xy = self._observation_pose[:2]
        until = min(deadline, self._now()+seconds)
        while self._now() < until:
            self._check(deadline)
            self._velocity(horizontal=self._position_hold_velocity(hold_xy), color=color)
            self._tick(deadline)
        return self._largest_frame()

    def _reach_observation(self):
        self._stage = '预设观测位姿 SET'
        target = self._observation_pose
        self._log(f'第 {self._gate_index+1} 门：SET 到预设观测位姿 {target}')
        self._action(BasicMotion.Goal.SET, target, 'xyzrz',
                     self._limit(self.p['observation_set_timeout']))
        self.node._cmd_x, self.node._cmd_y, self.node._cmd_z, self.node._cmd_yaw = map(float, target)

    def _observe_next_gate_here(self):
        """Keep the BLINE endpoint hold and collect only new next-gate frames."""
        self._stage = '过门后原地观察下一门'
        seconds = self.p['pass_next_observe_seconds']
        self._log(f'上一门已通过：在 BLINE 终点保持位置，观察下一门 {seconds:.2f}s')
        self._light(self.node.LIGHT_YELLOW, '过门后原地观察下一门')
        with self._lock:
            self._capture_floor = self.node.get_clock().now().nanoseconds*1e-9
            self._accept_frames = True
        until = self._limit(seconds)
        # Do not switch to BODY_VELOCITY during this window: BasicMotion is
        # already holding the passed gate's BLINE endpoint in position mode.
        while self._now() < until:
            self._tick(until)
        self._check()
        selected = self._largest_frame()
        if selected is not None:
            pose = self._pose()
            self._observation_pose = [pose[0], pose[1], self.depth, pose[5]]
            self._log(f'第 {self._gate_index+1} 门：原地观察发现最大框 '
                      f'eye={selected.eye} height={selected.height_percent:.1f}%，跳过预设位姿 SET')
            self._flash(self.node.LIGHT_GREEN, 1, '过门后观察发现下一门')
            return selected
        self._log(f'第 {self._gate_index+1} 门：过门后 {seconds:.2f}s 未见有效门框，转到预设位姿 SET')
        self._reset_gate(self.depth)
        return None

    def _scan(self, sign, deadline):
        previous = self._pose()[5]
        hold_xy = self._observation_pose[:2]
        travelled = 0.0
        self._light(self.node.LIGHT_YELLOW, '左右扫描观测')
        while travelled < self.p['search_scan_angle_deg']:
            self._check(deadline)
            frame = self._largest_frame()
            if frame is not None:
                self._neutral()
                return frame
            self._velocity(horizontal=self._position_hold_velocity(hold_xy),
                           yaw_rate=sign*self.p['search_yaw_rate_deg_s'],
                           color=self.node.LIGHT_YELLOW)
            self._tick(deadline)
            current = self._pose()[5]
            travelled += sign*wrap_degrees(current-previous)
            previous = current
        self._neutral()
        return self._largest_frame()

    def _search(self):
        with self._lock:
            self._current_frames = dict(left=(), right=())
            self._capture_floor = self.node.get_clock().now().nanoseconds*1e-9
            self._accept_frames = True
        self._stage = '原地观察'
        self._light(self.node.LIGHT_YELLOW, '初始观察')
        selected = self._observe(self.p['search_observe_seconds'], self._deadline)
        if selected is not None:
            self._flash(self.node.LIGHT_GREEN, 1, '初始观察发现门框')
            return selected
        self._flash(self.node.LIGHT_RED, 1)
        deadline = self._limit(self.p['search_timeout'])
        self._stage = '右扫'
        selected = self._scan(1.0, deadline)
        if selected is None:
            self._stage = '扫描返回观测位姿'
            self._action(BasicMotion.Goal.SET, self._observation_pose, 'xyzrz',
                         min(deadline, self._now()+self.p['observation_set_timeout']))
            self.node._cmd_x, self.node._cmd_y, self.node._cmd_z, self.node._cmd_yaw = map(float, self._observation_pose)
            # Only current observations after this SET can start the left scan.
            with self._lock:
                self._current_frames = dict(left=(), right=())
                self._capture_floor = self.node.get_clock().now().nanoseconds*1e-9
            self._stage = '左扫'
            self._phase_deadline = deadline
            selected = self._scan(-1.0, deadline)
        if selected is None:
            self._flash(self.node.LIGHT_RED, 2, '搜索未找到门框')
            self.node._task_failure_light_handled = True
            raise GateFailure('search', '左右扫视结束仍未找到有效门框')
        self._flash(self.node.LIGHT_GREEN, 1, '扫描发现门框')
        return selected

    def _align(self, frame):
        """Turn directly to the largest current bbox's world bearing."""
        deadline = self._limit(self.p['search_align_timeout'])
        self._neutral()
        frame = self._largest_frame() or frame
        pose = self._pose()
        target = [pose[0], pose[1], self.depth, heading(frame.ray)]
        self._log(f'对准当前最大门框 eye={frame.eye} height={frame.height_percent:.1f}% '
                  f'target_yaw={target[3]:.2f}°')
        self._action(BasicMotion.Goal.SET, target, 'xyzrz', deadline)
        self.node._cmd_x, self.node._cmd_y, self.node._cmd_z, self.node._cmd_yaw = map(float, target)
        self._flash(self.node.LIGHT_GREEN, 1, '最大门框对准完成')
        self._log('已看到最大门框并完成对正，执行近距校准和穿门；后续无需重获门框')

    def _height_limit_reached(self):
        frame = self._largest_frame()
        if frame is not None and frame.height_percent > self.p['fore_aft_height_limit_percent']:
            self._height_stop_frame = frame
            return True
        return False

    def _fore_aft(self):
        deadline = self._limit(self.p['fore_aft_timeout'])
        self._stage = '门框近距 BLINE 校准'
        self._light(self.node.LIGHT_YELLOW, '门框近距 BLINE 校准')
        self._neutral()
        if self._height_limit_reached():
            self._log(f'当前最大框高 {self._height_stop_frame.height_percent:.1f}% > '
                      f'{self.p["fore_aft_height_limit_percent"]:.1f}%，直接进入最终对准和 pass')
            return
        pose = self._pose()
        start = np.asarray(pose[:2])
        distance = self.p['fore_aft_max_distance_m']
        self._log(f'近距 BLINE 单次前进 {distance:.2f}m，巡航 {self.p["fore_aft_speed_mps"]:.2f}m/s；'
                  f'所有当前框的最大高度 > {self.p["fore_aft_height_limit_percent"]:.1f}% 时停车')
        self._bline_active = True
        try:
            self._action(BasicMotion.Goal.BLINE,
                         [distance, 0.0, self.depth-pose[2], 0.0], 'xyz', deadline,
                         cruise_speed=self.p['fore_aft_speed_mps'],
                         position_tolerance=self.p['fore_aft_position_tolerance_m'],
                         cancel_predicate=self._height_limit_reached)
        finally:
            self._bline_active = False
        if getattr(self.node, '_last_motion_cancelled_by_condition', False):
            pose = self._pose()
            self.node._cmd_x, self.node._cmd_y, self.node._cmd_z, self.node._cmd_yaw = (
                float(pose[0]), float(pose[1]), float(pose[2]), float(pose[5]))
            self._log(f'途中最大框高 {self._height_stop_frame.height_percent:.1f}% > '
                      f'{self.p["fore_aft_height_limit_percent"]:.1f}%，BLINE 已确认停止')
            return
        target = getattr(self.node, '_last_motion_final_target', None)
        if target is not None and len(target) == 4 and all(math.isfinite(float(value)) for value in target):
            self.node._cmd_x, self.node._cmd_y, self.node._cmd_z, self.node._cmd_yaw = map(float, target)
        travelled = float(np.linalg.norm(np.asarray(self._pose()[:2])-start))
        self._log(f'近距 BLINE 完成，实测位移 {travelled:.2f}m，进入最终对准和 pass')

    def _final_yaw(self):
        """Best-effort alignment; no fresh bbox or no stable yaw still permits pass."""
        if self._largest_frame() is None:
            self._log('停车后没有当前有效门框，保留当前航向直接 pass')
            return
        start = self._now()
        deadline = min(self._deadline, start+self.p['pass_yaw_servo_max_seconds'])
        self._phase_deadline = deadline
        hold_xy = self._pose()[:2]
        stable_since = None
        while self._now() < deadline:
            self._check()
            frame = self._largest_frame()
            if frame is None:
                self._neutral()
                self._log('再次对准时已无当前门框，停止转向并直接 pass')
                return
            now = self._now()
            error = self._yaw_error(frame)
            if abs(error) <= self.p['pass_yaw_tolerance_deg']:
                stable_since = now if stable_since is None else stable_since
            else:
                stable_since = None
            if (stable_since is not None
                    and now-start >= self.p['pass_yaw_servo_min_seconds']
                    and now-stable_since >= self.p['pass_yaw_stable_seconds']):
                self._neutral()
                self._flash(self.node.LIGHT_GREEN, 1, '最终 yaw 对准完成')
                self._log(f'当前最大框再次对准完成 eye={frame.eye} error={error:+.2f}° '
                          f'elapsed={now-start:.2f}s，开始 pass')
                return
            rate = float(np.clip(error*self.p['pass_yaw_kp'],
                                 -self.p['pass_max_yaw_rate_deg_s'], self.p['pass_max_yaw_rate_deg_s']))
            try:
                self._velocity(horizontal=self._position_hold_velocity(hold_xy), yaw_rate=rate)
            except GateFailure as exc:
                # A servo-window boundary is optional. Communication failures,
                # unconfirmed cancellation and the overall deadline remain fatal.
                if (exc.kind != 'timeout' or self.node.stopped
                        or self._cleanup_unconfirmed or self._now() >= self._deadline):
                    raise
                break
            self._tick()
        self._neutral()
        self._log(f'再次对准在 {self.p["pass_yaw_servo_max_seconds"]:.2f}s 内未稳定，保留当前航向直接 pass', True)

    def _pass(self):
        self._neutral()
        pose = self._pose()
        deadline = self._limit(self.p['pass_timeout'])
        self._light(self.node.LIGHT_OFF, 'BLINE 过门')
        self._bline_active = True
        try:
            self._action(BasicMotion.Goal.BLINE,
                         [self.p['pass_distance_m'], 0.0, self.depth-pose[2], 0.0],
                         'xyz', deadline, cruise_speed=self.p['pass_speed_mps'])
        finally:
            self._bline_active = False
        target = getattr(self.node, '_last_motion_final_target', None)
        if target is None or len(target) != 4 or not all(math.isfinite(float(x)) for x in target):
            raise GateFailure('final_target', 'BLINE 返回的 final_target 无效')
        self.node._cmd_x, self.node._cmd_y, self.node._cmd_z, self.node._cmd_yaw = map(float, target)
        # Between gates the next iteration immediately observes here while
        # keeping this endpoint hold. Only the last gate needs the final pause.
        if self._gate_index+1 >= self.p['gate_count']:
            until = self._limit(self.p['pass_pause_seconds'])
            while self._now() < until:
                self._tick()
        return True

    def _park(self):
        """Confirm cleanup using a fresh bounded budget, even after timeout."""
        until = self._now()+6.0
        confirmed = not self._cleanup_unconfirmed
        with self._lock:
            self._accept_frames = False
        handle = getattr(self.node, '_active_goal_handle', None)
        if handle is not None:
            try:
                result = handle.get_result_async()
                handle.cancel_goal_async()
                while not result.done() and self._now() < until and rclpy.ok():
                    self._sleep(.01)
                confirmed = result.done()
                if confirmed:
                    result.result()
                    self._cleanup_unconfirmed = False
                    self.node._active_goal_handle = None
            except Exception as exc:
                self._log(f'取消动作未确认: {exc}', True)
                confirmed = False
        try:
            self._velocity_active = True
            self._neutral(wait_deadline=min(until, self._now()+2.0))
            if confirmed and not self.node.stopped and rclpy.ok():
                pose = self._pose()
                ok, _ = self.node._send_action_goal(
                    BasicMotion.Goal.SET, [pose[0], pose[1], pose[2], pose[5]], 'xyzrz',
                    timeout=max(.001, until-self._now()), wait_deadline=until,
                    quiet=True, task_context='26rb_gate_task 失败停车',
                    light_color=self.node.LIGHT_OFF, cancel_wait_timeout=2.0)
                confirmed = ok and getattr(self.node, '_last_motion_cleanup_confirmed', True)
        except Exception as exc:
            self._log(f'停车指令未确认: {exc}', True)
            confirmed = False
        return confirmed and not self._cleanup_unconfirmed

    def _fixed_motion_confirmed(self):
        if self._cleanup_unconfirmed or not getattr(self.node, '_last_motion_cleanup_confirmed', True):
            self._cleanup_unconfirmed = True
            raise GateFailure('handoff', '定点动作终态未确认，禁止下发下一条定位')

    def _fixed_set(self, target, axes, seconds):
        deadline = self._limit(seconds)
        self._action(BasicMotion.Goal.SET, target, axes, deadline)
        self._fixed_motion_confirmed()
        self.node._cmd_x, self.node._cmd_y, self.node._cmd_z, self.node._cmd_yaw = map(float, target)
        return deadline

    def _fixed_line(self, target_xy, depth, cancel_predicate=None, position_tolerance=None):
        """Reach an absolute XY using one measured-yaw body-relative BLINE."""
        pose = self._pose()
        dx, dy = target_xy[0]-pose[0], target_xy[1]-pose[1]
        if math.hypot(dx, dy) <= 1e-6:
            self._log('定点 BLINE：已到终点 XY，交由最终 SET 保持完整位姿')
            return
        yaw = math.radians(pose[5])
        body_target = [dx*math.cos(yaw)+dy*math.sin(yaw),
                       -dx*math.sin(yaw)+dy*math.cos(yaw), depth-pose[2], 0.0]
        deadline = self._limit(self.p['observation_travel_timeout'])
        kwargs = dict(cruise_speed=self.p['pass_speed_mps'], cancel_predicate=cancel_predicate)
        if position_tolerance is not None:
            kwargs['position_tolerance'] = position_tolerance
        self._log(f'定点 BLINE：终点 XY=({target_xy[0]:.3f},{target_xy[1]:.3f})m，'
                  f'保持深度={depth:.3f}m，机体系增量=({body_target[0]:+.3f},{body_target[1]:+.3f})m，'
                  f'速度={self.p["pass_speed_mps"]:.3f}m/s')
        self._bline_active = True
        try:
            self._action(BasicMotion.Goal.BLINE, body_target, 'xyz', deadline, **kwargs)
        finally:
            self._bline_active = False
        self._fixed_motion_confirmed()
        if getattr(self.node, '_last_motion_cancelled_by_condition', False):
            pose = self._pose()
            target = [pose[0], pose[1], pose[2], pose[5]]
        else:
            target = getattr(self.node, '_last_motion_final_target', None)
            if target is None or len(target) != 4 or not all(math.isfinite(float(x)) for x in target):
                raise GateFailure('final_target', '定点 BLINE 返回的 final_target 无效')
        self.node._cmd_x, self.node._cmd_y, self.node._cmd_z, self.node._cmd_yaw = map(float, target)

    def _fixed_depth(self, depth):
        """Keep the parked XY/yaw in POS until measured depth is within tolerance."""
        self._stage = '定点半径内调整深度 SET'
        pose = self._pose()
        self.depth = float(depth)
        target = [pose[0], pose[1], self.depth, pose[5]]
        self._log(f'已进入终点 XY 半径且 BLINE 已结束：保持停车位置，SET 深度到 {self.depth:.3f}m')
        deadline = self._fixed_set(target, 'z', self.p['depth_set_timeout'])
        last_log = float('-inf')
        while True:
            self._check(deadline)
            pose = self._pose()
            error = self.depth-pose[2]
            now = self._now()
            if now-last_log >= 0.5:
                self._log(f'[GATE FIXED DEPTH] gate={self._gate_index+1} '
                          f'z_m={pose[2]:7.3f} target_z_m={self.depth:7.3f} '
                          f'dz_m={error:+8.3f} tol_m={self.p["depth_tolerance_m"]:6.3f} '
                          f'remain_s={max(0.0, deadline-now):7.2f}')
                last_log = now
            if abs(error) <= self.p['depth_tolerance_m']:
                return
            # SET may complete using the controller's broader tolerance. Keep
            # its position hold active; do not replace it with velocity output.
            self._tick(deadline)

    def _run_fixed_gate(self):
        if self._gate_index == 0:
            self._reach_observation()
            self._fixed_motion_confirmed()
        self.depth = self._pose()[2]
        self._flash(self.node.LIGHT_GREEN, 1, '定点通过：瞄准下一条预设位姿')
        self._neutral()
        target_index = self._gate_index+1
        target = self.p['observation_poses'][target_index]
        radius = self.p['observation_xy_radius_m'][target_index]
        pose = self._pose()
        dx, dy = target[0]-pose[0], target[1]-pose[1]
        if math.hypot(dx, dy) > 1e-6:
            self._stage = '定点瞄准下一条预设位姿 SET yaw'
            aim = [pose[0], pose[1], pose[2], math.degrees(math.atan2(dy, dx))]
            self._fixed_set(aim, 'rz', self.p['search_align_timeout'])
        self.depth = self._pose()[2]
        entered_radius = False
        last_log = float('-inf')

        def inside_radius():
            nonlocal entered_radius, last_log
            self._check()
            pose = self._pose()
            distance = math.hypot(target[0]-pose[0], target[1]-pose[1])
            entered_radius = entered_radius or distance <= radius
            now = self._now()
            if now-last_log >= 0.5 or entered_radius:
                self._log(f'[GATE FIXED XY] gate={self._gate_index+1} pose={target_index+1} '
                          f'dist_xy_m={distance:7.3f} radius_m={radius:6.3f} '
                          f'z_m={pose[2]:7.3f} target_z_m={target[2]:7.3f} '
                          f'in_radius={int(entered_radius)} '
                          f'remain_s={max(0.0, self._phase_deadline-now):7.2f}')
                last_log = now
            return entered_radius

        self._stage = '定点 BLINE 保持当前深度接近终点半径'
        self._limit(self.p['observation_travel_timeout'])
        if not inside_radius():
            # The first BLINE's natural endpoint must also be inside the radius
            # when the terminal result wins the race with the radius predicate.
            self._fixed_line(target[:2], self.depth, cancel_predicate=inside_radius,
                             position_tolerance=radius/2.0)
            if not entered_radius and not inside_radius():
                raise GateFailure('motion', '定点 BLINE 已结束，但实测 XY 尚未进入终点半径')
        self._fixed_depth(target[2])
        self._stage = '定点 BLINE 目标深度走完剩余路程'
        self._fixed_line(target[:2], self.depth)
        self._stage = '定点终点完整位姿 SET'
        self._fixed_set(target, 'xyzrz', self.p['observation_set_timeout'])
        self._log(f'第 {self._gate_index+1} 门：定点通过完成，已 SET 到第 {target_index+1} 条位姿 {target}')

    def _run_gate(self):
        frame = self._observe_next_gate_here() if self._observe_after_pass else None
        if frame is None:
            self._reach_observation()
            frame = self._search()
        self._stage = '最大门框航向对准'
        self._align(frame)
        self._fore_aft()
        self._flash(self.node.LIGHT_GREEN, 1, '门框近距校准完成')
        self._stage = '可选最终 yaw 对准'
        self._final_yaw()
        self._stage = 'BLINE 穿门'
        self._pass()
        self._log(f'第 {self._gate_index+1} 门：已看到最大框并对正，穿门动作完成，记为通过')

    def execute(self):
        passed = skipped = 0
        previous_passed = False
        active_poses = self.p['observation_poses'][:self.p['gate_count']]
        for index, observation_pose in enumerate(active_poses):
            self._gate_index = index
            self.node._task_failure_light_handled = False
            self._stage = '准备'
            self._deadline = self._now()+self.p['timeout']
            self._phase_deadline = self._deadline
            self._reset_gate(observation_pose[2])
            self._observation_pose = observation_pose
            self._observe_after_pass = previous_passed
            previous_passed = False
            mode = self.p['fixed_pass_code'][index]
            self._log(f'第 {index+1}/{self.p["gate_count"]} 个门，模式={mode}，观测位姿 {observation_pose}，'
                      f'已通过 {passed}，已跳过 {skipped}/{self.p["max_failures"]}')
            try:
                if mode == '1':
                    self._run_fixed_gate()
                else:
                    self._run_gate()
                passed += 1
                previous_passed = True
            except GateFailure as exc:
                stopped = self._park()
                self._log(f'第 {index+1} 门失败 [{exc.kind}] {exc}；停车确认={stopped}', True)
                terminal = exc.kind in ('cancelled', 'odom') or not stopped
                if not terminal and index+1 < self.p['gate_count'] and skipped < self.p['max_failures']:
                    skipped += 1
                    self._log(f'跳过第 {index+1} 门，前往第 {index+2} 门（{skipped}/{self.p["max_failures"]}）', True)
                    continue
                reason = '' if stopped else '；旧动作/停车未确认，禁止进入下一门'
                return TaskOutcome.failed(f'26rb_gate_task.{exc.kind}',
                                          f'第 {index+1} 门: {exc}；通过 {passed}，跳过 {skipped}{reason}')
            except Exception as exc:
                self._park()
                return TaskOutcome.failed('26rb_gate_task.exception', str(exc))
        return TaskOutcome.ok(f'过门完成：通过 {passed}，跳过 {skipped}/{self.p["max_failures"]}')

    def destroy(self):
        if self._velocity_active:
            self._park()
        if self._sub is not None:
            self.node.destroy_subscription(self._sub)
