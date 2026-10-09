"""Fixed-depth gate sequence driven by front-eye detections and measured odom.

Camera ownership is independent of the motion phase. A handover never changes
lateral path geometry or the recorded fore/aft ray. Only the final yaw phase
uses calibrated stereo; all other visual phases use the owning eye.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import threading
import time

import cv2
import numpy as np
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
    stamp: float
    pair_id: int
    bbox: tuple
    pose: tuple
    origin: np.ndarray
    ray: np.ndarray
    area_percent: float

    @property
    def center(self):
        x1, y1, x2, y2 = self.bbox
        return np.array([(x1+x2)/2, (y1+y2)/2])


@dataclass(frozen=True)
class RecordedRay:
    origin: np.ndarray
    direction: np.ndarray


class GateFailure(RuntimeError):
    def __init__(self, kind, message):
        super().__init__(message)
        self.kind = kind


class RB26GateTask:
    def __init__(self, node, params):
        self.node = node
        self.p = validate_gate_params(params)
        self._now = time.monotonic
        self._sleep = time.sleep
        self._lock = threading.RLock()
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
        self.owner = None
        self.generation = 0
        self._race_floor = 0
        self._last_valid = 0.0
        self._lost_since = None
        self._latest = dict(left=None, right=None)
        self._history = {e: deque(maxlen=16) for e in ('left', 'right')}
        self._identity = dict(left=None, right=None)
        self._landmark = None
        self._reference = None
        self._recorded_ray = None
        self._accept_frames = False
        self._velocity_active = False
        self._light_color = node.LIGHT_YELLOW
        self._bline_active = False
        self._deadline = math.inf
        self._phase_deadline = math.inf
        self.depth = self.p['depth_front'][0]
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
        pose = tuple(float(x) for x in self.node._latest_robot_pose())
        if len(pose) != 6 or not all(math.isfinite(x) for x in pose):
            raise GateFailure('odom', '实测位姿无效')
        return pose

    def _check(self, deadline=None):
        if self.node.stopped or not rclpy.ok():
            raise GateFailure('cancelled', '任务已取消')
        if self._now() >= self._deadline:
            raise GateFailure('timeout', '过门任务总超时')
        if deadline is not None and self._now() >= deadline:
            raise GateFailure('timeout', '当前过门阶段超时')

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

    def _release_if_lost(self, now):
        if self.owner is not None and now-self._last_valid > self.p['search_priority_release_seconds']:
            old = self.owner
            self.owner = None
            self.generation += 1
            self._race_floor = self.sequence
            # The histories cannot win ownership. Only callbacks after this floor can.
            self._log(f'门框丢失超过 {self.p["search_priority_release_seconds"]:.1f}s，释放 {old} 优先权', True)

    def _reset_gate(self, depth):
        with self._lock:
            self.depth = depth
            self.owner = None
            self.generation += 1
            self._race_floor = self.sequence
            self._last_valid = 0.0
            self._lost_since = None
            self._latest = dict(left=None, right=None)
            self._identity = dict(left=None, right=None)
            for history in self._history.values():
                history.clear()
            self._reference = self._landmark = self._recorded_ray = None
            self._last_capture_id = dict(left=None, right=None)
            self._accept_frames = False

    def _camera_ray(self, eye, center, pose):
        pixel = np.asarray(center, dtype=np.float64).reshape(1, 1, 2)
        xy = cv2.undistortPoints(pixel, self.k[eye], self.distortion[eye]).reshape(2)
        extrinsic = self.extrinsics[eye]
        transform = rotation(pose)
        ray = transform @ extrinsic.optical_to_body @ np.array([xy[0], xy[1], 1.0])
        ray /= np.linalg.norm(ray)
        origin = np.asarray(pose[:3]) + transform @ extrinsic.translation
        return origin, ray

    def _matches(self, frame):
        """Conservative geometric association; detections have no track IDs."""
        if self._reference is None:
            return True
        # A stereo landmark permits reprojection across a moving camera.
        if self._landmark is not None:
            expected = self._landmark-frame.origin
            expected /= np.linalg.norm(expected)
            if math.degrees(math.acos(float(np.clip(expected @ frame.ray, -1, 1)))) > self.p['search_lock_bearing_tolerance_deg']:
                return False
        else:
            angle = math.degrees(math.acos(float(np.clip(self._reference.ray @ frame.ray, -1, 1))))
            if angle > self.p['search_lock_bearing_tolerance_deg']:
                return False
        old = self._identity[frame.eye]
        if old is not None:
            # Compensate rotation before checking image motion. Translation is
            # bounded by the generous image gate unless a landmark is known.
            if self._landmark is not None:
                delta = self._landmark-frame.origin
            else:
                delta = old.ray
            optical = self.extrinsics[frame.eye].optical_to_body.T @ rotation(frame.pose).T @ delta
            if optical[2] <= 0:
                return False
            predicted = np.array([self.k[frame.eye][0, 0]*optical[0]/optical[2]+self.k[frame.eye][0, 2],
                                  self.k[frame.eye][1, 1]*optical[1]/optical[2]+self.k[frame.eye][1, 2]])
            distance = np.linalg.norm((frame.center-predicted)/np.array([self.width, self.height]))
            if distance > self.p['search_lock_center_delta_fraction']:
                return False
        return True

    def _detection_cb(self, msg):
        eye = str(msg.camera_name)
        if eye.startswith('front_'):
            eye = eye[len('front_'):]
        if eye not in self._latest:
            return
        with self._lock:
            if not self._accept_frames:
                return
            now = self._now()
            self._release_if_lost(now)
            capture_id = int(getattr(msg, 'capture_id', 0))
            if capture_id and capture_id == self._last_capture_id[eye]:
                return
            if capture_id:
                self._last_capture_id[eye] = capture_id
            self.sequence += 1
            stamp = float(msg.header.stamp.sec)+float(msg.header.stamp.nanosec)*1e-9
            if stamp > 0:
                ros_now = self.node.get_clock().now().nanoseconds*1e-9
                if ros_now-stamp > self.p['search_detection_timeout'] or stamp-ros_now > self.p['search_detection_timeout']:
                    self._latest[eye] = None
                    return
            pose = self._pose()
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
                origin, ray = self._camera_ray(eye, [(x1+x2)/2, (y1+y2)/2], pose)
                if np.linalg.norm(ray[:2]) < 1e-6:
                    continue
                candidate = GateFrame(eye, self.sequence, now, stamp, int(msg.stereo_pair_id),
                                      bbox, pose, origin, ray,
                                      100*(x2-x1)*(y2-y1)/(self.width*self.height))
                if self._matches(candidate):
                    candidates.append(candidate)
            if not candidates:
                self._latest[eye] = None
                return
            # Once identified choose closest associated box, not a larger neighbour.
            reference = self._identity[eye]
            if reference is None:
                frame = max(candidates, key=lambda f: f.area_percent) if self._reference is None else min(
                    candidates, key=lambda f: np.linalg.norm(f.ray-self._reference.ray))
            else:
                frame = min(candidates, key=lambda f: np.linalg.norm(f.ray-reference.ray))
            self._latest[eye] = frame
            self._history[eye].append(frame)
            self._identity[eye] = frame
            if self.owner is None and frame.sequence > self._race_floor:
                self.owner = eye
                self.generation += 1
                self._log(f'{eye} 新帧获得门框伺服优先权')
            if self.owner == eye:
                self._last_valid = now
                self._reference = frame
            # Only accept a geometrically checked stereo landmark of this gate.
            center = self._stereo_center(now)
            if center is not None:
                self._landmark = center

    def _owner_frame(self):
        with self._lock:
            now = self._now()
            self._release_if_lost(now)
            frame = self._latest.get(self.owner)
            if self._fresh(frame, now):
                self._lost_since = None
                return frame
            if self._reference is None:
                return None
            if self._lost_since is None:
                self._lost_since = now
            if now-self._lost_since >= self.p['search_reacquire_timeout']:
                raise GateFailure('observation_lost', '门框重获超时')
            return None

    def _stereo_center(self, now):
        """Pair matching IDs (or stamps without IDs), then check ray geometry."""
        if not all(self._fresh(self._latest[e], now) for e in ('left', 'right')):
            return None
        pairs = []
        for left in reversed(self._history['left']):
            if not self._fresh(left, now):
                continue
            for right in reversed(self._history['right']):
                if not self._fresh(right, now):
                    continue
                if left.pair_id and right.pair_id:
                    if left.pair_id != right.pair_id:
                        continue
                elif not left.stamp or not right.stamp or abs(left.stamp-right.stamp) > self.p['pass_stereo_pair_slop']:
                    continue
                # Even equal IDs must describe a reasonably simultaneous capture.
                if abs(left.received-right.received) > self.p['pass_stereo_pair_slop']:
                    continue
                pairs.append((min(left.sequence, right.sequence), left, right))
        for _, left, right in sorted(pairs, key=lambda item: item[0], reverse=True):
            dot = float(np.clip(left.ray @ right.ray, -1, 1))
            if math.degrees(math.acos(dot)) < self.p['pass_stereo_min_angle_deg']:
                continue
            matrix = np.column_stack([left.ray, -right.ray])
            distances = np.linalg.lstsq(matrix, right.origin-left.origin, rcond=None)[0]
            if min(distances) <= 0 or max(distances) > self.p['pass_stereo_max_distance_m']:
                continue
            a = left.origin+distances[0]*left.ray
            b = right.origin+distances[1]*right.ray
            if np.linalg.norm(a-b) > self.p['pass_stereo_max_ray_gap_m']:
                continue
            # Reject implausible size mismatches between the same two gate boxes.
            if max(left.area_percent, right.area_percent)/min(left.area_percent, right.area_percent) > 2.5:
                continue
            center = (a+b)/2
            if self._reference is not None:
                direction = center-self._reference.origin
                direction /= np.linalg.norm(direction)
                if math.degrees(math.acos(float(np.clip(direction @ self._reference.ray, -1, 1)))) > self.p['search_lock_bearing_tolerance_deg']:
                    continue
            return center
        return None

    def _yaw_error(self, frame, stereo=False):
        pose = self._pose()
        if stereo:
            with self._lock:
                center = self._stereo_center(self._now())
            if center is not None:
                delta = center-np.asarray(pose[:3])
                if np.linalg.norm(delta[:2]) > 1e-6:
                    return wrap_degrees(heading(delta)-pose[5]), 'stereo'
        return wrap_degrees(heading(frame.ray)-pose[5]), frame.eye

    def _light(self, color, label):
        self._light_color = color
        self.node._set_task_phase_light(color, label, log=False)

    def _velocity(self, horizontal=(0.0, 0.0), yaw_rate=0.0, color=None):
        self._check()
        if self._bline_active:
            raise GateFailure('handoff', 'BLINE 执行中禁止旧伺服速度输出')
        pose = self._pose()
        vz = float(np.clip((self.depth-pose[2])*self.p['depth_kp'],
                           -self.p['depth_max_speed_mps'], self.p['depth_max_speed_mps']))
        world = np.array([*horizontal, vz])
        body = rotation(pose).T @ world
        self._velocity_active = True
        self._light_color = self.node.LIGHT_YELLOW if color is None else color
        ok, message = self.node._send_body_velocity(
            *body.tolist(), yaw_rate_deg_s=float(yaw_rate),
            lease_s=max(0.25, 3*self.p['search_velocity_period']),
            task_context='26rb_gate_task 定深视觉伺服',
            light_color=self._light_color,
            wait_deadline=min(self._deadline, self._phase_deadline))
        if not ok:
            self._check(self._phase_deadline)
            raise GateFailure('motion', message)

    def _neutral(self, wait_deadline=None):
        if self._velocity_active:
            ok, message = self.node._send_body_velocity(
                task_context='26rb_gate_task 结束速度输出', light_color=self._light_color,
                wait_deadline=min(self._deadline, self._now()+2.0) if wait_deadline is None else wait_deadline)
            self._velocity_active = False
            if not ok:
                raise GateFailure('motion', f'停止速度失败: {message}')

    def _action(self, command, target, axes, deadline, **kwargs):
        self._check(deadline)
        ok, message = self.node._send_action_goal(
            command, [float(x) for x in target], axes=axes,
            timeout=max(0.001, deadline-self._now()), wait_deadline=deadline,
            task_context='26rb_gate_task', **kwargs)
        self._check(deadline)
        if not ok:
            kind = 'timeout' if '超时' in message or 'timeout' in message.lower() else 'motion'
            raise GateFailure(kind, message)

    def _depth_hold(self, seconds, color=None):
        deadline = self._limit(seconds)
        while self._now() < deadline:
            self._check()
            self._velocity(color=color)
            self._tick()

    def _flash(self, color, count, label="门框观察闪灯"):
        self._neutral()
        for index in range(count):
            self._light(color, label)
            self._depth_hold(self.p['search_light_pulse_seconds'], color)
            self._light(self.node.LIGHT_OFF, '闪灯间隔')
            self._depth_hold(self.p['search_light_gap_seconds'], self.node.LIGHT_OFF)
        self._neutral()

    def _observe(self, seconds, deadline, require=False):
        self._phase_deadline = deadline
        until = min(deadline, self._now()+seconds)
        while self._now() < until:
            self._check(deadline)
            self._owner_frame()
            self._velocity()
            self._tick(deadline)
        frame = self._owner_frame()
        if require and frame is None:
            while frame is None:
                self._check(deadline)
                self._velocity()
                self._tick(deadline)
                frame = self._owner_frame()
        return frame

    def _search(self):
        # Only frames received after reaching depth can win this gate.
        with self._lock:
            self._accept_frames = True
        observed = self._observe(self.p['search_observe_seconds'], self._deadline)
        if observed is not None:
            self._flash(self.node.LIGHT_GREEN, 1)
            return
        self._flash(self.node.LIGHT_RED, 1)
        deadline = self._limit(self.p['search_timeout'])
        self._log('开始 -30° 起点的逐级扫视')
        base_yaw = self._pose()[5]
        for sweep in self.p['search_sweep_degrees']:
            # Positioning to the scan start is also velocity controlled, so a
            # first detection can immediately stop the turn.
            start = wrap_degrees(base_yaw+self.p['search_start_offset_deg'])
            while True:
                self._check(deadline)
                if self._owner_frame() is not None:
                    self._flash(self.node.LIGHT_GREEN, 1)
                    return
                error = wrap_degrees(start-self._pose()[5])
                if abs(error) <= 1.0:
                    break
                rate = float(np.clip(error*self.p['search_yaw_kp'],
                                     -self.p['search_yaw_rate_deg_s'], self.p['search_yaw_rate_deg_s']))
                self._velocity(yaw_rate=rate)
                self._tick(deadline)
            previous = self._pose()[5]
            travelled = 0.0
            while travelled < sweep:
                # Unwrap measured increments to handle scans crossing +/-180°.
                self._check(deadline)
                if self._owner_frame() is not None:
                    self._flash(self.node.LIGHT_GREEN, 1)
                    return
                self._velocity(yaw_rate=self.p['search_yaw_rate_deg_s'])
                self._tick(deadline)
                current = self._pose()[5]
                travelled += wrap_degrees(current-previous)
                previous = current
        raise GateFailure('search', '三轮扫视结束仍未找到门框')

    def _align(self, observe_seconds):
        deadline = self._limit(self.p['search_align_timeout'])
        frame = self._observe(observe_seconds, deadline, require=True)
        self._neutral()
        pose = self._pose()
        target_yaw = heading(frame.ray)
        self._action(BasicMotion.Goal.SET, [pose[0], pose[1], self.depth, target_yaw], 'xyzrz', deadline)
        self._light(self.node.LIGHT_GREEN, '门框对准完成')
        # A pre-turn pixel must never be applied to a post-turn pose.
        with self._lock:
            floor = self.sequence
        return floor

    def _fresh_after(self, floor):
        deadline = self._limit(self.p['search_reacquire_timeout'])
        while True:
            self._check(deadline)
            frame = self._owner_frame()
            if frame is not None and frame.sequence > floor:
                return frame
            self._velocity()
            self._tick(deadline)

    def _lateral(self, distance):
        if abs(distance) <= self.p['lateral_tolerance_m']:
            self._log('横移距离为零，跳过平移')
            return
        deadline = self._limit(self.p['lateral_timeout'])
        start = np.asarray(self._pose()[:2])
        yaw = math.radians(self._pose()[5])
        direction = np.array([-math.sin(yaw), math.cos(yaw)])
        # These variables stay fixed through all observation/owner losses.
        while True:
            self._check(deadline)
            frame = self._owner_frame()
            delta = np.asarray(self._pose()[:2])-start
            along = float(delta @ direction)
            error = distance-along
            cross = delta-along*direction
            if abs(error) <= self.p['lateral_tolerance_m'] and np.linalg.norm(cross) <= self.p['lateral_tolerance_m']:
                self._neutral()
                return
            if frame is None:
                self._velocity()
            else:
                speed = float(np.clip(error, -self.p['lateral_speed_mps'], self.p['lateral_speed_mps']))
                correction = -self.p['lateral_path_kp']*cross
                size = np.linalg.norm(correction)
                if size > self.p['lateral_max_path_speed_mps']:
                    correction *= self.p['lateral_max_path_speed_mps']/size
                yaw_error, _ = self._yaw_error(frame)
                rate = float(np.clip(yaw_error*self.p['search_yaw_kp'],
                                     -self.p['search_max_yaw_rate_deg_s'], self.p['search_max_yaw_rate_deg_s']))
                self._velocity(direction*speed+correction, rate)
            self._tick(deadline)

    def _record_ray(self, frame):
        direction = frame.ray[:2].copy()
        direction /= np.linalg.norm(direction)
        # Immutable horizontal camera ray projected onto the fixed depth plane.
        origin = np.array([frame.origin[0], frame.origin[1], self.depth])
        direction = np.array([direction[0], direction[1], 0.0])
        origin.setflags(write=False)
        direction.setflags(write=False)
        self._recorded_ray = RecordedRay(origin, direction)
        self._log(f'记录固定水平射线: origin={origin.tolist()}, direction={direction.tolist()}')

    def _fore_aft(self):
        deadline = self._limit(self.p['fore_aft_timeout'])
        ray = self._recorded_ray
        start = np.asarray(self._pose()[:2])
        stable_since = None
        generation = self.generation
        while True:
            self._check(deadline)
            frame = self._owner_frame()
            pose = self._pose()
            direction = ray.direction[:2]
            delta = np.asarray(pose[:2])-ray.origin[:2]
            cross = delta-float(delta @ direction)*direction
            travelled = abs(float((np.asarray(pose[:2])-start) @ direction))
            if travelled > self.p['fore_aft_max_travel_m']:
                raise GateFailure('area_unreachable', '面积目标未达到且沿射线位移已超过上限')
            if self.generation != generation:
                stable_since = None
                generation = self.generation
            if frame is None:
                stable_since = None
                self._velocity()
            else:
                error = self.p['fore_aft_target_area_percent']-frame.area_percent
                ready = (abs(error) <= self.p['fore_aft_area_tolerance_percent']
                         and np.linalg.norm(cross) <= self.p['fore_aft_line_tolerance_m']
                         and abs(self.depth-pose[2]) <= self.p['depth_tolerance_m'])
                if ready:
                    stable_since = self._now() if stable_since is None else stable_since
                    if self._now()-stable_since >= self.p['fore_aft_stable_seconds']:
                        self._neutral()
                        return
                else:
                    stable_since = None
                speed = 0.0 if abs(error) <= self.p['fore_aft_area_tolerance_percent'] else float(np.clip(
                    self.p['fore_aft_area_kp']*error/100,
                    -self.p['fore_aft_reverse_speed_mps'], self.p['fore_aft_speed_mps']))
                correction = -self.p['fore_aft_line_kp']*cross
                size = np.linalg.norm(correction)
                if size > self.p['fore_aft_max_cross_speed_mps']:
                    correction *= self.p['fore_aft_max_cross_speed_mps']/size
                rate = float(np.clip(wrap_degrees(heading(direction)-pose[5])*self.p['search_yaw_kp'],
                                     -self.p['search_max_yaw_rate_deg_s'], self.p['search_max_yaw_rate_deg_s']))
                self._velocity(direction*speed+correction, rate)
            self._tick(deadline)

    def _final_yaw(self):
        start = self._now()
        deadline = min(self._deadline, start+self.p['pass_yaw_servo_max_seconds'])
        self._phase_deadline = deadline
        stable_since = None
        generation = self.generation
        while True:
            self._check()
            frame = self._owner_frame()
            now = self._now()
            if self.generation != generation:
                stable_since = None
                generation = self.generation
            if frame is None:
                stable_since = None
                error, source = None, 'lost'
            else:
                error, source = self._yaw_error(frame, stereo=True)
                if abs(error) <= self.p['pass_yaw_tolerance_deg']:
                    stable_since = now if stable_since is None else stable_since
                else:
                    stable_since = None
            aligned = (frame is not None and stable_since is not None
                       and now-start >= self.p['pass_yaw_servo_min_seconds']
                       and now-stable_since >= self.p['pass_yaw_stable_seconds'])
            if aligned or now >= deadline:
                self._neutral()
                frame = self._owner_frame()
                if frame is None:
                    raise GateFailure('observation_lost', '最终 yaw 结束时没有有效门框，不发送 BLINE')
                error, source = self._yaw_error(frame, stereo=True)
                aligned = aligned and abs(error) <= self.p['pass_yaw_tolerance_deg']
                if aligned:
                    self._light(self.node.LIGHT_GREEN, '最终 yaw 稳定达标')
                self._log(f'最终 yaw {source}: 耗时 {now-start:.2f}s，剩余误差 {error:.2f}°，'
                          f'{"稳定达标" if aligned else "达到时限，按当前实测航向过门"}', not aligned)
                return
            rate = 0.0 if error is None else float(np.clip(error*self.p['pass_yaw_kp'],
                                 -self.p['pass_max_yaw_rate_deg_s'], self.p['pass_max_yaw_rate_deg_s']))
            self._velocity(yaw_rate=rate)
            self._tick()

    def _pass(self):
        self._neutral()
        if self._owner_frame() is None:
            raise GateFailure('observation_lost', 'BLINE 交接前门框已丢失')
        pose = self._pose()
        deadline = self._limit(self.p['pass_timeout'])
        self._light(self.node.LIGHT_YELLOW, 'BLINE 过门')
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
        # BasicMotion now holds the endpoint; no velocity output during this pause.
        until = self._limit(self.p['pass_pause_seconds'])
        while self._now() < until:
            self._tick()

    def _park(self):
        """Best effort cancellation/neutral and measured-pose hold on any failure."""
        handle = getattr(self.node, '_active_goal_handle', None)
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:
                pass
        try:
            # Explicit neutral also cancels an unfinished BLINE.
            self._velocity_active = True
            self._neutral(wait_deadline=self._now()+2.0)
            if not self.node.stopped and rclpy.ok():
                pose = self._pose()
                self.node._send_action_goal(
                    BasicMotion.Goal.SET, [pose[0], pose[1], pose[2], pose[5]], 'xyzrz',
                    timeout=2.0, wait_deadline=self._now()+2.0,
                    quiet=True, task_context='26rb_gate_task 失败停车', light_color=self._light_color)
        except Exception as exc:
            self._log(f'停车指令未确认: {exc}，速度租约将到期', True)

    def execute(self):
        self._deadline = self._now()+self.p['timeout']
        try:
            for index in range(self.p['gate_count']):
                self._reset_gate(self.p['depth_front'][index])
                self._log(f'第 {index+1}/{self.p["gate_count"]} 个门，深度 {self.depth:.2f}m')
                pose = self._pose()
                depth_deadline = self._limit(self.p['depth_set_timeout'])
                self._action(BasicMotion.Goal.SET, [pose[0], pose[1], self.depth, pose[5]],
                             'xyz', depth_deadline)
                while abs(self._pose()[2]-self.depth) > self.p['depth_tolerance_m']:
                    self._check(depth_deadline)
                    self._velocity()
                    self._tick(depth_deadline)
                self._neutral()
                self._flash(self.node.LIGHT_GREEN, 1, '上下对正完成')
                try:
                    self._search()
                except GateFailure as exc:
                    if exc.kind not in ('cancelled',) and self._now() < self._deadline:
                        self._flash(self.node.LIGHT_RED, 2)
                        self.node._task_failure_light_handled = True
                    raise
                self._align(self.p['search_align_observe_seconds'])
                self._lateral(self.p['lateral_front'][index])
                self._flash(self.node.LIGHT_GREEN, 1, '左右对正完成')
                floor = self._align(self.p['fore_aft_observe_seconds'])
                self._record_ray(self._fresh_after(floor))
                self._fore_aft()
                self._flash(self.node.LIGHT_GREEN, 1, '前后对正完成')
                self._final_yaw()
                self._pass()
            return TaskOutcome.ok('四阶段视觉伺服及 BLINE 过门完成')
        except GateFailure as exc:
            self._park()
            return TaskOutcome.failed(f'26rb_gate_task.{exc.kind}', str(exc))
        except Exception as exc:
            self._park()
            return TaskOutcome.failed('26rb_gate_task.exception', str(exc))

    def destroy(self):
        if self._velocity_active:
            self._park()
        self.node.destroy_subscription(self._sub)
