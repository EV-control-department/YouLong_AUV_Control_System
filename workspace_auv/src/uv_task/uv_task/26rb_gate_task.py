"""Fixed-depth gate sequence driven by front-eye detections and measured odom.

Camera ownership is independent of the motion phase. A handover never changes
lateral path geometry or the recorded fore/aft ray. Servo errors use the owning eye; validated stereo supports identity projection
and the final yaw correction.
"""
from __future__ import annotations

from collections import deque
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
from uv_task.gate_tracking import DetectionTracks
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
    height_percent: float
    image_space: int = 0

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
        self.p = validate_gate_params(params, complete=True)
        for key in ('fore_aft_target_area_percent', 'fore_aft_area_tolerance_percent', 'fore_aft_area_kp'):
            if key in params:
                self._log(f'{key} 已废弃且不参与控制；请使用 fore_aft 高度参数', True)
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
        self._fore_aft_start = None
        self._accept_frames = False
        self._capture_floor = 0.0
        self._velocity_active = False
        self._light_color = node.LIGHT_YELLOW
        self._bline_active = False
        self._deadline = math.inf
        self._phase_deadline = math.inf
        self.depth = self.p['observation_poses'][0][2]
        self._observation_pose = self.p['observation_poses'][0]
        self._target_id = None
        self._target_aliases = {}
        self._cross_pending = {}
        self._collecting = True
        self._tracks = self._new_tracks()
        self._cleanup_unconfirmed = False
        self._gate_index = 0
        self._stage = '准备'
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

    def _new_tracks(self):
        return DetectionTracks(self.width, self.height, self.p,
                               self._predict_box, self._tracking_box)

    def _release_if_lost(self, now):
        if self.owner is not None and now-self._last_valid > self.p['search_priority_release_seconds']:
            old = self.owner
            self.owner = None
            self.generation += 1
            self._race_floor = self.sequence
            self._log(f'释放 {old} 相机优先权，保留门框目标 ID={self._target_id}', True)

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
            self._fore_aft_start = None
            self._last_capture_id = dict(left=None, right=None)
            self._accept_frames = False
            self._target_id = None
            self._target_aliases = {}
            self._cross_pending = {}
            self._collecting = True
            self._tracks = self._new_tracks()

    def _camera_ray(self, eye, center, pose, image_info=None):
        matrix = image_info.k if image_info is not None else self.k[eye]
        distortion = image_info.d if image_info is not None else self.distortion[eye]
        xy = normalized_pixel(matrix, distortion, *center)
        extrinsic = self.extrinsics[eye]
        transform = rotation(pose)
        ray = transform @ extrinsic.optical_to_body @ np.array([xy[0], xy[1], 1.0])
        ray /= np.linalg.norm(ray)
        origin = np.asarray(pose[:3]) + transform @ extrinsic.translation
        return origin, ray

    def _tracking_box(self, frame):
        if frame.image_space == 1:
            return np.asarray(frame.bbox)
        x1, y1, x2, y2 = frame.bbox
        corners = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]], dtype=np.float64)
        normalized = [normalized_pixel(self.k[frame.eye], self.distortion[frame.eye], *point)
                      for point in corners]
        projected = self.k[frame.eye] @ np.column_stack((normalized, np.ones(4))).T
        points = (projected[:2] / projected[2]).T
        return np.r_[points.min(axis=0), points.max(axis=0)]

    def _predict_box(self, old, eye, pose, box):
        x1, y1, x2, y2 = box
        points = np.array([[x1, y1, 1], [x2, y1, 1], [x2, y2, 1], [x1, y2, 1]])
        optical = (np.linalg.inv(self.k[old.eye]) @ points.T)
        world = rotation(old.pose) @ self.extrinsics[old.eye].optical_to_body @ optical
        # Range belongs only to the selected identity, never to neighbouring
        # tracks in the bank. Project its local gate plane when stereo is valid.
        if (self._landmark is not None and not self._collecting
                and self._identity.get(old.eye) is old):
            old_transform = rotation(old.pose) @ self.extrinsics[old.eye].optical_to_body
            optical_depth = (old_transform.T @ (self._landmark-old.origin))[2]
            if optical_depth > 1e-6:
                new_origin = np.asarray(pose[:3])+rotation(pose)@self.extrinsics[eye].translation
                world = old.origin.reshape(3, 1)+world*optical_depth-new_origin.reshape(3, 1)
        current = self.extrinsics[eye].optical_to_body.T @ rotation(pose).T @ world
        if np.any(current[2] <= 1e-6):
            return None
        pixels = (self.k[eye] @ current)[:2]/current[2]
        return np.r_[pixels.min(axis=1), pixels.max(axis=1)]

    def _select_track(self, track, fresh):
        with self._lock:
            self._collecting = False
            self._target_id = track.id
            self._target_aliases = {track.eye: track.id}
            self._tracks.locked.add(track.id)
            self._reference = track.frame
            self._latest = dict(left=None, right=None)
            self._identity = dict(left=None, right=None)
            self.owner = None
            self._lost_since = self._now()
            if fresh:
                self._accept_target(track.frame)
            else:
                track.streak = 0
                track.heights.clear()
                track.velocity[:] = 0
            self._log(f'锁定门框 ID={track.id}，稳定高度 {track.peak_height:.1f}%')

    def _accept_target(self, frame):
        self._latest[frame.eye] = frame
        self._identity[frame.eye] = frame
        self._history[frame.eye].append(frame)
        if self.owner is None and frame.sequence > self._race_floor:
            self.owner = frame.eye
            self.generation += 1
            self._log(f'{frame.eye} 接管固定门框 ID={self._target_id}')
        if self.owner == frame.eye:
            self._last_valid = self._now()
            self._reference = frame
        center = self._stereo_center(self._now())
        if center is not None:
            self._landmark = center

    def _cross_eye_target(self, eye, frames):
        reference = self._reference
        if reference is None or not frames:
            self._cross_pending.pop(eye, None)
            return None
        predicted = self._predict_box(reference, eye, frames[0][1].pose, self._tracking_box(reference))
        if predicted is None:
            self._cross_pending.pop(eye, None)
            return None
        ranked = []
        for track_id, frame in frames:
            cost = self._tracks.cost(predicted, self._tracking_box(frame))
            angle = math.degrees(math.acos(float(np.clip(reference.ray @ frame.ray, -1, 1))))
            if cost is None or angle > self.p['search_lock_bearing_tolerance_deg']:
                continue
            # If simultaneous stereo is available, a bad triangulation is a
            # veto rather than permission to fall back to another gate.
            same_pair = (reference.pair_id == frame.pair_id if reference.pair_id and frame.pair_id
                         else bool(reference.stamp and frame.stamp))
            simultaneous = (same_pair
                            and abs(reference.stamp-frame.stamp) <= self.p['pass_stereo_pair_slop']
                            and abs(reference.received-frame.received) <= self.p['pass_stereo_pair_slop'])
            if simultaneous and self._triangulate(reference, frame) is None:
                continue
            ranked.append((cost, track_id, frame))
        ranked.sort(key=lambda item: item[0])
        if not ranked or (len(ranked) > 1 and ranked[1][0]-ranked[0][0] < self.p['tracking_ambiguity_margin']):
            self._cross_pending.pop(eye, None)
            return None
        _, track_id, frame = ranked[0]
        pending_id, count, last = self._cross_pending.get(eye, (None, 0, 0.0))
        continuous = pending_id == track_id and self._now()-last <= self.p['search_detection_timeout']
        count = count+1 if continuous else 1
        self._cross_pending[eye] = (track_id, count, self._now())
        if count < self.p['tracking_confirm_frames']:
            return None
        self._target_aliases[eye] = track_id
        self._tracks.locked.add(track_id)
        return frame

    def _detection_cb(self, msg):
        if not bind_detection_geometry(self.node, msg):
            return
        geometry = getattr(self.node, '_detection_geometry', None)
        image_info = geometry.cache.resolve(msg) if geometry is not None else None
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
                if stamp < self._capture_floor:
                    return
                ros_now = self.node.get_clock().now().nanoseconds*1e-9
                if ros_now-stamp > self.p['search_detection_timeout'] or stamp-ros_now > self.p['search_detection_timeout']:
                    self._latest[eye] = None
                    return
            try:
                pose = self._pose()
            except GateFailure:
                self._latest[eye] = None
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
                origin, ray = self._camera_ray(eye, [(x1+x2)/2, (y1+y2)/2], pose, image_info)
                if np.linalg.norm(ray[:2]) < 1e-6:
                    continue
                candidate = GateFrame(eye, self.sequence, now, stamp, int(msg.stereo_pair_id),
                                      bbox, pose, origin, ray,
                                      100*(x2-x1)*(y2-y1)/(self.width*self.height),
                                      100*(y2-y1)/self.height, int(getattr(msg, 'image_space', 0)))
                candidates.append(candidate)
            matched = self._tracks.update(eye, candidates, pose, now)
            if self._collecting:
                return
            self._latest[eye] = None
            alias = self._target_aliases.get(eye)
            target = matched.get(alias)
            track = self._tracks.tracks.get(alias)
            if target is not None and track.streak >= self.p['tracking_confirm_frames']:
                self._accept_target(target)
            elif eye not in self._target_aliases:
                target = self._cross_eye_target(eye, list(matched.items()))
                if target is not None:
                    self._accept_target(target)
            else:
                self._cross_pending.pop(eye, None)

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
            center = self._triangulate(left, right)
            if center is None:
                continue
            if self._reference is not None:
                direction = center-self._reference.origin
                direction /= np.linalg.norm(direction)
                if math.degrees(math.acos(float(np.clip(direction @ self._reference.ray, -1, 1)))) > self.p['search_lock_bearing_tolerance_deg']:
                    continue
            return center
        return None

    def _triangulate(self, left, right):
        dot = float(np.clip(left.ray @ right.ray, -1, 1))
        if math.degrees(math.acos(dot)) < self.p['pass_stereo_min_angle_deg']:
            return None
        matrix = np.column_stack([left.ray, -right.ray])
        distances = np.linalg.lstsq(matrix, right.origin-left.origin, rcond=None)[0]
        if min(distances) <= 0 or max(distances) > self.p['pass_stereo_max_distance_m']:
            return None
        a, b = left.origin+distances[0]*left.ray, right.origin+distances[1]*right.ray
        if np.linalg.norm(a-b) > self.p['pass_stereo_max_ray_gap_m']:
            return None
        if max(left.height_percent, right.height_percent)/min(left.height_percent, right.height_percent) > self.p['tracking_max_height_ratio']:
            return None
        return (a+b)/2

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
        self._light_color = self.node.LIGHT_YELLOW if color is None else color
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

    def _neutral(self, wait_deadline=None):
        if self._velocity_active:
            ok, message = self.node._send_body_velocity(
                task_context='26rb_gate_task 结束速度输出', light_color=self._light_color,
                wait_deadline=min(self._deadline, self._now()+2.0) if wait_deadline is None else wait_deadline,
                cancel_wait_timeout=2.0)
            self._velocity_active = False
            if not getattr(self.node, '_last_motion_cleanup_confirmed', True):
                self._cleanup_unconfirmed = True
            if not ok:
                raise GateFailure('motion', f'停止速度失败: {message}')

    def _action(self, command, target, axes, deadline, **kwargs):
        self._check(deadline)
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
        self._neutral()
        self._light(self.node.LIGHT_OFF, '闪灯开始')
        self._depth_hold(self.p['search_light_gap_seconds'], self.node.LIGHT_OFF)
        for index in range(count):
            self._light(color, label)
            self._depth_hold(self.p['search_light_pulse_seconds'], color)
            self._light(self.node.LIGHT_OFF, '闪灯间隔')
            self._depth_hold(self.p['search_light_gap_seconds'], self.node.LIGHT_OFF)
        self._neutral()

    def _observe(self, seconds, deadline, require=False, color=None):
        self._phase_deadline = deadline
        until = min(deadline, self._now()+seconds)
        while self._now() < until:
            self._check(deadline)
            self._owner_frame()
            self._velocity(color=color)
            self._tick(deadline)
        frame = self._owner_frame()
        if require and frame is None:
            while frame is None:
                self._check(deadline)
                self._velocity()
                self._tick(deadline)
                frame = self._owner_frame()
        return frame

    def _reach_observation(self):
        self._stage = '初始观测位姿 BTRAVEL'
        pose = self._pose()
        target = self._observation_pose
        dx, dy = target[0]-pose[0], target[1]-pose[1]
        yaw = math.radians(pose[5])
        body = [math.cos(yaw)*dx+math.sin(yaw)*dy,
                -math.sin(yaw)*dx+math.cos(yaw)*dy, target[2]-pose[2], 0.0]
        self._action(BasicMotion.Goal.BTRAVEL, body, 'xyz', self._limit(self.p['observation_travel_timeout']))
        self._stage = '初始观测位姿 SET'
        self._action(BasicMotion.Goal.SET, target, 'xyzrz', self._limit(self.p['observation_set_timeout']))
        self._light(self.node.LIGHT_GREEN, '到达初始观测位姿')

    def _scan(self, sign, deadline):
        previous = self._pose()[5]
        travelled = 0.0
        while travelled < self.p['search_scan_angle_deg']:
            self._check(deadline)
            self._velocity(yaw_rate=sign*self.p['search_yaw_rate_deg_s'])
            self._tick(deadline)
            current = self._pose()[5]
            travelled += sign*wrap_degrees(current-previous)
            previous = current
        self._neutral()

    def _search(self):
        with self._lock:
            self._capture_floor = self.node.get_clock().now().nanoseconds*1e-9
            self._accept_frames = True
        self._stage = '原地观察'
        self._observe(self.p['search_observe_seconds'], self._deadline, color=self.node.LIGHT_GREEN)
        with self._lock:
            selected = self._tracks.highest(self._now(), fresh_only=True)
            if selected is not None:
                self._select_track(selected, fresh=True)
        if selected is not None:
            self._flash(self.node.LIGHT_GREEN, 2)
            return
        self._flash(self.node.LIGHT_RED, 1)
        deadline = self._limit(self.p['search_timeout'])
        self._stage = '右扫 90°'
        self._scan(1.0, deadline)
        self._stage = '扫描返回观测位姿'
        self._action(BasicMotion.Goal.SET, self._observation_pose, 'xyzrz',
                     min(deadline, self._now()+self.p['observation_set_timeout']))
        self._stage = '左扫 90°'
        self._phase_deadline = deadline
        self._scan(-1.0, deadline)
        with self._lock:
            selected = self._tracks.highest(self._now(), fresh_only=False)
            if selected is not None:
                self._select_track(selected, fresh=False)
        if selected is None:
            self._flash(self.node.LIGHT_RED, 2, '完整搜索未找到门框')
            self.node._task_failure_light_handled = True
            raise GateFailure('search', '左右扫视结束仍未找到稳定门框')
        self._flash(self.node.LIGHT_GREEN, 2)
        self._stage = '扫描目标转向与重捕获'
        deadline = self._limit(self.p['search_reacquire_timeout'])
        self._lost_since = self._now()
        pose = self._pose()
        self._action(BasicMotion.Goal.SET, [pose[0], pose[1], self.depth, heading(selected.frame.ray)],
                     'xyzrz', deadline)
        self._fresh_after(self.sequence, deadline=deadline)

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

    def _fresh_after(self, floor, deadline=None):
        deadline = self._limit(self.p['search_reacquire_timeout']) if deadline is None else deadline
        self._phase_deadline = deadline
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
        self._fore_aft_start = np.asarray(self._pose()[:2]).copy()
        self._log(f'记录固定水平射线: origin={origin.tolist()}, direction={direction.tolist()}')

    def _fore_aft(self):
        deadline = self._limit(self.p['fore_aft_timeout'])
        ray = self._recorded_ray
        start = self._fore_aft_start
        stable_since = None
        generation = self.generation
        next_log = self._now()
        while True:
            self._check(deadline)
            frame = self._owner_frame()
            pose = self._pose()
            direction = ray.direction[:2]
            delta = np.asarray(pose[:2])-ray.origin[:2]
            cross = delta-float(delta @ direction)*direction
            travelled = abs(float((np.asarray(pose[:2])-start) @ direction))
            if travelled > self.p['fore_aft_max_travel_m']:
                raise GateFailure('height_unreachable', f'高度目标 {self.p["fore_aft_target_height_percent"]:.1f}% 未达到，当前 {frame.height_percent if frame else float("nan"):.1f}%，沿射线位移 {travelled:.2f}m 超过上限')
            if self.generation != generation:
                stable_since = None
                generation = self.generation
            if frame is None:
                stable_since = None
                self._velocity()
            else:
                error = self.p['fore_aft_target_height_percent']-frame.height_percent
                if self._now() >= next_log:
                    self._log(f'前后高度伺服: 当前 {frame.height_percent:.1f}%，'
                              f'目标 {self.p["fore_aft_target_height_percent"]:.1f}%，位移 {travelled:.2f}m')
                    next_log = self._now()+1.0
                ready = (abs(error) <= self.p['fore_aft_height_tolerance_percent']
                         and np.linalg.norm(cross) <= self.p['fore_aft_line_tolerance_m']
                         and abs(self.depth-pose[2]) <= self.p['depth_tolerance_m'])
                if ready:
                    stable_since = self._now() if stable_since is None else stable_since
                    if self._now()-stable_since >= self.p['fore_aft_stable_seconds']:
                        self._neutral()
                        return
                else:
                    stable_since = None
                speed = 0.0 if abs(error) <= self.p['fore_aft_height_tolerance_percent'] else float(np.clip(
                    self.p['fore_aft_height_kp']*error/100,
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
            aligned = (now <= deadline and frame is not None and stable_since is not None
                       and now-start >= self.p['pass_yaw_servo_min_seconds']
                       and now-stable_since >= self.p['pass_yaw_stable_seconds'])
            if aligned or now >= deadline:
                self._neutral()
                frame = self._owner_frame()
                if frame is None:
                    raise GateFailure('observation_lost', '最终 yaw 结束时没有有效门框，不发送 BLINE')
                error, source = self._yaw_error(frame, stereo=True)
                aligned = aligned and abs(error) <= self.p['pass_yaw_tolerance_deg']
                if not aligned:
                    raise GateFailure('timeout', '最终 yaw 在时限内未稳定对准，不发送 BLINE')
                if aligned:
                    self._light(self.node.LIGHT_GREEN, '最终 yaw 稳定达标')
                self._log(f'最终 yaw {source}: 耗时 {now-start:.2f}s，剩余误差 {error:.2f}°，'
                          '稳定达标')
                return
            rate = 0.0 if error is None else float(np.clip(error*self.p['pass_yaw_kp'],
                                 -self.p['pass_max_yaw_rate_deg_s'], self.p['pass_max_yaw_rate_deg_s']))
            self._velocity(yaw_rate=rate)
            self._tick()

    def _pass(self, require_height=True):
        self._neutral()
        frame = self._owner_frame()
        if frame is None:
            raise GateFailure('observation_lost', 'BLINE 交接前门框已丢失')
        if (require_height
                and abs(frame.height_percent-self.p['fore_aft_target_height_percent']) > self.p['fore_aft_height_tolerance_percent']):
            return False
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
                    light_color=self._light_color, cancel_wait_timeout=2.0)
                confirmed = ok and getattr(self.node, '_last_motion_cleanup_confirmed', True)
        except Exception as exc:
            self._log(f'停车指令未确认: {exc}', True)
            confirmed = False
        return confirmed and not self._cleanup_unconfirmed

    def _run_gate(self):
        self._reach_observation()
        self._search()
        self._stage = '门框航向对准'
        self._align(self.p['search_align_observe_seconds'])
        self._stage = '左右对正'
        self._lateral(self.p['lateral_front'][self._gate_index])
        self._flash(self.node.LIGHT_GREEN, 1, '左右对正完成')
        self._stage = '前后高度对正'
        floor = self._align(self.p['fore_aft_observe_seconds'])
        self._record_ray(self._fresh_after(floor))
        while True:
            self._stage = '前后高度对正'
            height_timed_out = False
            try:
                self._fore_aft()
            except GateFailure as exc:
                # Only the elapsed height phase can relax the height threshold.
                # Overall deadline, transport failures and unconfirmed motion
                # cleanup retain their original failure handling.
                if (exc.kind != 'timeout' or self._now() >= self._deadline
                        or self._now() < self._phase_deadline or self._cleanup_unconfirmed):
                    raise
                self._check()
                self._neutral()
                self._check()
                if self._cleanup_unconfirmed:
                    raise
                frame = self._owner_frame()
                if frame is None:
                    raise
                height_timed_out = True
                self._log(f'前后高度伺服超时，但固定门框 ID={self._target_id} 仍有效：'
                          f'当前高度 {frame.height_percent:.1f}%，停止前后修正，'
                          '最终 yaw 对准后直接 BLINE（本门不再检查高度阈值）', True)
            else:
                self._flash(self.node.LIGHT_GREEN, 1, '前后对正完成')
            self._stage = '最终 yaw 对准'
            self._final_yaw()
            floor = self.sequence
            frame = self._fresh_after(floor)
            if (height_timed_out
                    or abs(frame.height_percent-self.p['fore_aft_target_height_percent']) <= self.p['fore_aft_height_tolerance_percent']):
                self._stage = 'BLINE 穿门'
                passed = self._pass(require_height=False) if height_timed_out else self._pass()
                if passed:
                    return
            self._log('BLINE 前高度离开容差范围，返回高度伺服')

    def execute(self):
        passed = skipped = 0
        for index, observation_pose in enumerate(self.p['observation_poses']):
            self._gate_index = index
            self.node._task_failure_light_handled = False
            self._stage = '准备'
            self._deadline = self._now()+self.p['timeout']
            self._phase_deadline = self._deadline
            self._reset_gate(observation_pose[2])
            self._observation_pose = observation_pose
            self._log(f'第 {index+1}/{self.p["gate_count"]} 个门，观测位姿 {observation_pose}，'
                      f'已通过 {passed}，已跳过 {skipped}/{self.p["max_failures"]}')
            try:
                self._run_gate()
                passed += 1
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
        self.node.destroy_subscription(self._sub)
