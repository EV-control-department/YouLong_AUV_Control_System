"""Front yaw alignment and monitored BLINE approach before pickup or drop.

Front sightings can interrupt one cruise for a late yaw correction. Down-eye
sightings end the approach after a short delay and hand over to down-view servo.
The cruise and near-radius budgets accumulate BLINE time only. Visual
calibration, positional holds, flashes and goal-acceptance waits are excluded.
"""
from collections import deque
from dataclasses import dataclass
import math
import threading
import time

import cv2
import numpy as np
from uv_camera.perception_geometry import bind_detection_geometry, normalized_detection
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy

from auv_protocol.topics import PERCEPTION_DETECTIONS
from uv_msgs.action import BasicMotion
from uv_msgs.msg import DetectionArray
from uv_task.down_camera_servo import best_detection
from uv_task.golf_search_config import validate_search_params
from uv_task.task_outcome import TaskOutcome
from uv_task.drop_search_config import validate_drop_search_params
from uv_task.search_clock import BlineSearchClock


def _rotation(pose):
    r, p, y = map(math.radians, pose[3:6])
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                             math.sin(p), math.cos(y), math.sin(y))
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def _wrap(value):
    return (float(value)+180.0) % 360.0-180.0


@dataclass(frozen=True)
class Frame:
    camera: str
    sequence: int
    received: float
    stamp: float
    pair_id: int
    origin: np.ndarray
    ray: np.ndarray
    area: float


class SearchFailure(RuntimeError):
    pass


class FrameNotFound(SearchFailure):
    """The fallback scan completed without finding the collection frame."""


class FrontDownSearch:
    def __init__(self, node, params, *, validator=validate_search_params,
                 position_key='collection_frame_position',
                 front_class='collection_frame_front', down_class=None,
                 task_name='26rb_grab_golf', target_label='置物盘',
                 correct_depth=True):
        self.node = node
        self.p = validator(params)
        self.position = np.asarray(self.p[position_key])
        self.task_name = task_name
        self.target_label = target_label
        self.correct_depth = correct_depth
        self.depth = self.p['search_cruise_depth_m']
        self._now = time.monotonic
        self._sleep = time.sleep
        self._lock = threading.RLock()
        self._latest = {camera: None for camera in (
            'front_left', 'front_right', 'down_left', 'down_right')}
        self._history = {eye: deque(maxlen=16) for eye in ('front_left', 'front_right')}
        self._capture = {}
        self._sequence = 0
        self._search_clock = BlineSearchClock(
            self.p['search_timeout'], self.p['search_near_timeout'], lambda: self._now())
        self._bline_active = False
        self.first_down_seen_at = None
        self.first_down_indicated = False
        self.stereo_calibrated = False
        self._velocity_active = False
        self._sub = None
        self.front_class = node._model_mapping.model_class_id(
            front_class, required=False)
        self.down_class = node._model_mapping.model_class_id(
            down_class or params.get('collection_frame_class', 'collection_frame_down'), required=False)
        self._pulse = float(params.get('light_pulse_seconds', 0.35))
        self._gap = float(params.get('light_gap_seconds', 0.25))

    def _check(self):
        if self.node.stopped or not rclpy.ok():
            raise SearchFailure(f'寻找{self.target_label}已取消')

    def _pose(self):
        pose = tuple(float(x) for x in self.node._latest_robot_pose())
        if len(pose) != 6 or not all(math.isfinite(x) for x in pose):
            raise SearchFailure(f'寻找{self.target_label}时实测位姿无效')
        return pose

    def _sync_pose(self):
        pose = self._pose()
        (self.node._cmd_x, self.node._cmd_y, self.node._cmd_z,
         self.node._cmd_yaw) = (*pose[:3], pose[5])

    def _light(self, color, label):
        self.node._set_task_phase_light(color, f'{self.task_name}：{label}')

    def _wait(self, seconds):
        deadline = self._now()+seconds
        while self._now() < deadline:
            self._check()
            self._sleep(min(self.p['search_period'], max(0.0, deadline-self._now())))

    def _flash(self, color, count, label, restore=True):
        flash_async = getattr(self.node, '_flash_task_light', None)
        if not callable(flash_async):
            return not self.node.stopped
        return flash_async(
            color, count, f'{self.task_name}：{label}',
            pulse_seconds=self._pulse, gap_seconds=self._gap,
            restore=restore,
            restore_color=None)

    def _action(self, target, axes, label, timeout=None):
        self._check()
        if self._bline_active:
            raise SearchFailure('BLINE 未结束，禁止启动定点或对准运动')
        seconds = self.p['search_move_timeout'] if timeout is None else timeout
        deadline = self._now()+seconds
        ok, message = self.node._send_action_goal(
            BasicMotion.Goal.SET, list(map(float, target)), axes,
            timeout=seconds, wait_deadline=deadline,
            light_color=self.node.LIGHT_OFF,
            task_context=self.node._format_motion_context(label))
        if not ok:
            raise SearchFailure(f'{label}失败：{message}')
        self._sync_pose()

    def _hold(self):
        pose = self._pose()
        depth = self.depth if self.correct_depth else pose[2]
        axes = 'xyzrz' if self.correct_depth else 'xyrz'
        self._action([pose[0], pose[1], depth, pose[5]], axes,
                     f'寻找{self.target_label}：停车保持')

    def _fresh(self, frame):
        return frame is not None and self._now()-frame.received <= self.p['search_detection_timeout']

    def _note_down_frame(self, frames):
        if not frames:
            return False
        if self.first_down_seen_at is None:
            self.first_down_seen_at = min(frame.received for frame in frames)
        if not self.first_down_indicated:
            self.first_down_indicated = True
            self._flash(self.node.LIGHT_GREEN, 1, f'首次看到{self.target_label}')
        return True

    def _detection_cb(self, message):
        if not bind_detection_geometry(self.node, message):
            return
        camera = str(message.camera_name).strip().lower()
        if camera not in self._latest:
            return
        with self._lock:
            capture = int(getattr(message, 'capture_id', 0))
            if capture and self._capture.get(camera) == capture:
                return
            if capture:
                self._capture[camera] = capture
            self._sequence += 1
            now = self._now()
            stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
            if stamp and abs(self.node.get_clock().now().nanoseconds*1e-9-stamp) > self.p['search_detection_timeout']:
                self._latest[camera] = None
                return
            class_id = self.front_class if camera.startswith('front_') else self.down_class
            detection = best_detection(message, class_id)
            if detection is None or float(detection.confidence) < self.p['search_min_confidence']:
                self._latest[camera] = None
                return
            if camera.startswith('down_') and self._bline_active and self.first_down_seen_at is None:
                self.first_down_seen_at = now
            origin = ray = np.zeros(3)
            area = 1.0
            if camera.startswith('front_'):
                try:
                    xy = normalized_detection(self.node, camera, detection)
                    extrinsic = self.node.camera_extrinsics[camera]
                    pose = self._pose()
                    rotation = _rotation(pose)
                    ray = rotation @ extrinsic.optical_to_body @ np.array([*xy, 1.0])
                    ray /= np.linalg.norm(ray)
                    origin = np.asarray(pose[:3])+rotation @ extrinsic.translation
                    area = max(0.0, float(detection.bbox_x2-detection.bbox_x1)) * max(
                        0.0, float(detection.bbox_y2-detection.bbox_y1))
                    if not np.all(np.isfinite(ray)) or np.linalg.norm(ray[:2]) < 1e-6:
                        raise ValueError('无效前视射线')
                except (KeyError, ValueError, SearchFailure, cv2.error):
                    self._latest[camera] = None
                    return
            frame = Frame(camera, self._sequence, now, stamp,
                          int(getattr(message, 'stereo_pair_id', 0)), origin, ray, area)
            self._latest[camera] = frame
            if camera in self._history:
                self._history[camera].append(frame)

    def _frames(self, prefix, after=-1):
        with self._lock:
            return sorted((f for c, f in self._latest.items()
                           if c.startswith(prefix) and self._fresh(f) and f.sequence > after),
                          key=lambda f: f.sequence)

    def _stereo_center(self):
        with self._lock:
            if not all(self._fresh(self._latest[c]) for c in self._history):
                return None
            left = tuple(reversed(self._history['front_left']))
            right = tuple(reversed(self._history['front_right']))
        for a in left:
            for b in right:
                if not self._fresh(a) or not self._fresh(b):
                    continue
                if a.pair_id and b.pair_id:
                    if a.pair_id != b.pair_id:
                        continue
                elif not a.stamp or not b.stamp or abs(a.stamp-b.stamp) > self.p['search_stereo_pair_slop']:
                    continue
                if abs(a.received-b.received) > self.p['search_stereo_pair_slop']:
                    continue
                angle = math.degrees(math.acos(float(np.clip(a.ray @ b.ray, -1, 1))))
                if angle < self.p['search_stereo_min_angle_deg']:
                    continue
                distances = np.linalg.lstsq(np.column_stack((a.ray, -b.ray)),
                                           b.origin-a.origin, rcond=None)[0]
                if not np.all(np.isfinite(distances)) or min(distances) <= 0 or max(distances) > self.p['search_stereo_max_distance_m']:
                    continue
                x, y = a.origin+distances[0]*a.ray, b.origin+distances[1]*b.ray
                if np.linalg.norm(x-y) > self.p['search_stereo_max_ray_gap_m']:
                    continue
                if min(a.area, b.area) <= 0 or max(a.area, b.area)/min(a.area, b.area) > 2.5:
                    continue
                return (x+y)/2
        return None

    def _expired(self):
        if self._search_clock.near_entered_at is None:
            xy_distance = np.linalg.norm(np.asarray(self._pose()[:2])-self.position[:2])
            if xy_distance <= self.p['search_near_radius_m']:
                self._search_clock.enter_near()
                # Near-radius indication is asynchronous.  It must never
                # delay or gate the BLINE that just brought us into range.
                self._flash(self.node.LIGHT_YELLOW,
                            f'进入{self.target_label}近场半径')
                self.node.get_logger().info(
                    f'{self.task_name}：进入预设 {self.target_label} XY 二维半径；'
                    '近场计时仅累计 BLINE 运行时间')
        return self._search_clock.remaining() <= 0.0

    def _neutral(self):
        if not self._velocity_active:
            return
        ok, message = self.node._send_body_velocity(
            task_context=f'寻找{self.target_label}：结束 yaw 伺服速度',
            light_color=self.node.LIGHT_OFF,
            wait_deadline=self._now()+2.0)
        self._velocity_active = False
        if not ok:
            raise SearchFailure(f'结束前视伺服失败：{message}')

    def _align_front(self):
        """Try mono then stereo yaw within one three-second budget."""
        deadline = self._now()+self.p['search_front_align_seconds']
        owner = None
        stable_since = None
        stable_mode = None
        mono_aligned = stereo_aligned = False
        self._light(self.node.LIGHT_OFF, f'前视{self.target_label} yaw 对准')
        try:
            while self._now() < deadline:
                self._check()
                frames = self._frames('front_')
                frame = next((f for f in frames if f.camera == owner), None)
                if frame is None and frames:
                    frame = frames[0]
                    owner = frame.camera
                    stable_since = None
                pose = self._pose()
                center = self._stereo_center() if frame is not None else None
                mode = 'stereo' if center is not None else owner
                direction = center-np.asarray(pose[:3]) if center is not None else (
                    frame.ray if frame is not None else None)
                error = None if direction is None else _wrap(
                    math.degrees(math.atan2(direction[1], direction[0]))-pose[5])
                if error is not None and abs(error) <= self.p['search_yaw_tolerance_deg']:
                    if stable_since is None or stable_mode != mode:
                        stable_since, stable_mode = self._now(), mode
                    if self._now()-stable_since >= self.p['search_yaw_stable_seconds']:
                        if center is not None:
                            stereo_aligned = True
                            self.stereo_calibrated = True
                            break
                        mono_aligned = True
                else:
                    stable_since = None
                vz = 0.0
                if self.correct_depth:
                    vz = float(np.clip((self.depth-pose[2])*self.p['search_depth_gain'],
                        -self.p['search_max_vertical_speed_mps'], self.p['search_max_vertical_speed_mps']))
                world = np.array([0.0, 0.0, vz])
                body = _rotation(pose).T @ world
                rate = 0.0 if error is None else float(np.clip(
                    error*self.p['search_yaw_gain'], -self.p['search_max_yaw_rate_deg_s'],
                    self.p['search_max_yaw_rate_deg_s']))
                self._velocity_active = True
                ok, message = self.node._send_body_velocity(
                    *body.tolist(), yaw_rate_deg_s=rate,
                    lease_s=max(0.25, 4*self.p['search_period']),
                    wait_deadline=deadline, light_color=self.node.LIGHT_OFF,
                    task_context=f'寻找{self.target_label}：前视 yaw 伺服')
                if not ok:
                    if self._now() >= deadline:
                        break
                    raise SearchFailure(f'前视对准速度失败：{message}')
                self._sleep(min(self.p['search_period'], max(0.0, deadline-self._now())))
        finally:
            self._neutral()
        self._hold()
        if mono_aligned or stereo_aligned:
            self._flash(self.node.LIGHT_GREEN, 1, '前视 yaw 对准完成')
        else:
            self._flash(self.node.LIGHT_RED, 1, '前视未看到或未完成对准')
        return mono_aligned or stereo_aligned

    def _cancel(self, handle, result_future, allow_timeout=False):
        """Wait for terminal result before allowing any new motion command."""
        if not result_future.done():
            handle.cancel_goal_async()
        until = self._now()+self.p['search_cancel_timeout']
        while not result_future.done() and rclpy.ok() and self._now() < until:
            self._sleep(self.p['search_period'])
        if not result_future.done():
            raise SearchFailure('BLINE 取消后未结束，禁止启动后续运动')
        completed = result_future.result()
        if completed.status == 6 and not (
                allow_timeout and 'timeout' in completed.result.message.lower()):
            raise SearchFailure(f'BLINE 中止：{completed.result.message}')

    def _bline(self, monitor):
        self._check()
        if self._bline_active or self._velocity_active:
            raise SearchFailure('旧 BLINE 或伺服速度未结束')
        if self._expired():
            return 'timeout'
        remaining = self._search_clock.remaining()
        if not self.node._action_client.wait_for_server(timeout_sec=min(2.0, remaining)):
            raise SearchFailure(f'寻找{self.target_label}：BasicMotion 动作服务器不可用')
        goal = BasicMotion.Goal()
        goal.cmd_type = BasicMotion.Goal.BLINE
        goal.axes = 'xyz'
        # Stop by our accumulated runtime budget before reaching this far endpoint.
        dz = self.depth-self._pose()[2] if self.correct_depth else 0.0
        goal.target = [self.p['search_speed_mps']*remaining+1.0, 0.0, dz, 0.0]
        # This is a server failsafe for one goal, not the accumulated cruise clock.
        goal.timeout = remaining+self.p['search_cancel_timeout']+1.0
        goal.cruise_speed = self.p['search_speed_mps']
        goal.task_context = self.node._format_motion_context(f'寻找{self.target_label}：前向 BLINE 巡游')
        self._light(self.node.LIGHT_OFF, f'BLINE 前向寻找{self.target_label}')
        future = self.node._action_client.send_goal_async(goal)
        acceptance_deadline = self._now()+min(5.0, self.p['search_move_timeout'])
        while not future.done() and not self.node.stopped and rclpy.ok() and self._now() < acceptance_deadline:
            self._sleep(self.p['search_period'])
        if not future.done() or self.node.stopped or not rclpy.ok():
            def cancel_late(done):
                try:
                    handle = done.result()
                    if handle.accepted:
                        handle.cancel_goal_async()
                except Exception:
                    pass
            future.add_done_callback(cancel_late)
            self._check()
            raise SearchFailure('BLINE 目标接受超时，已登记迟到取消；停止后续运动')
        handle = future.result()
        if not handle.accepted:
            raise SearchFailure(f'寻找{self.target_label}：BLINE 被拒绝')
        self._bline_active = True
        self.node._active_goal_handle = handle
        result_future = handle.get_result_async()
        self._search_clock.resume()
        try:
            while True:
                self._check()
                if result_future.done():
                    result = result_future.result().result
                    if not result.success:
                        if self._expired() and 'timeout' in result.message.lower():
                            return 'timeout'
                        raise SearchFailure(f'寻找{self.target_label}：BLINE 失败：{result.message}')
                    if monitor(finished=True) == 'down':
                        self._sync_pose()
                        return 'down'
                    target = list(result.final_target)
                    if len(target) != 4 or not all(math.isfinite(x) for x in target):
                        raise SearchFailure('BLINE 返回无效 final_target')
                    (self.node._cmd_x, self.node._cmd_y, self.node._cmd_z,
                     self.node._cmd_yaw) = target
                    self.node._last_motion_final_target = target
                    return 'endpoint'
                reason = monitor()
                if reason:
                    self._cancel(handle, result_future, allow_timeout=reason == 'timeout')
                    self._sync_pose()
                    return reason
                self._sleep(self.p['search_period'])
        finally:
            try:
                if not result_future.done():
                    self._cancel(handle, result_future)
            finally:
                if result_future.done():
                    self._search_clock.pause()
                    self._bline_active = False
                    if self.node._active_goal_handle is handle:
                        self.node._active_goal_handle = None

    def _observe(self, prefix, seconds, after=-1, full_window=False):
        until = self._now()+seconds
        seen = False
        while self._now() < until:
            self._check()
            frames = self._frames(prefix, after)
            if frames:
                seen = True
                if prefix == 'down_':
                    self._note_down_frame(frames)
                if not full_window:
                    return True
            self._sleep(min(self.p['search_period'], max(0.0, until-self._now())))
        return seen or bool(self._frames(prefix, after))

    def _rotate_for_down_frame(self, after=-1):
        """Rotate in place at fixed depth while watching for the down-view frame."""
        if self._bline_active or self._velocity_active:
            raise SearchFailure('旧 BLINE 或速度伺服未结束，不能启动旋转搜索')
        speed = self.p['search_fallback_rotate_speed_deg_s']
        degrees = self.p['search_fallback_rotate_degrees']
        deadline = self._now()+self.p['search_fallback_rotate_timeout']
        previous_yaw = self._pose()[5]
        turned = 0.0
        found = False
        self._light(self.node.LIGHT_OFF, f'{self.target_label} 原地旋转搜索')
        try:
            while turned < degrees and self._now() < deadline:
                self._check()
                frames = self._frames('down_', after)
                if frames:
                    self._note_down_frame(frames)
                    found = True
                    break
                pose = self._pose()
                turned += _wrap(pose[5]-previous_yaw)
                previous_yaw = pose[5]
                if turned >= degrees:
                    break
                vz = (float(np.clip((self.depth-pose[2])*self.p['search_depth_gain'],
                      -self.p['search_max_vertical_speed_mps'],
                      self.p['search_max_vertical_speed_mps']))
                      if self.correct_depth else 0.0)
                body = _rotation(pose).T @ np.array([0.0, 0.0, vz])
                self._velocity_active = True
                ok, message = self.node._send_body_velocity(
                    *body.tolist(), yaw_rate_deg_s=speed,
                    lease_s=max(0.25, 4*self.p['search_period']),
                    wait_deadline=deadline, light_color=self.node.LIGHT_OFF,
                    task_context=f'{self.task_name}：{self.target_label}原地旋转搜索')
                if not ok:
                    raise SearchFailure(f'原地旋转搜索速度指令失败：{message}')
                self._sleep(min(self.p['search_period'], max(0.0, deadline-self._now())))
        finally:
            self._neutral()
        self._hold()
        if found:
            return True
        if turned < degrees:
            raise SearchFailure(
                f'原地旋转搜索超时：{turned:.1f}°/{degrees:.1f}°，'
                f'限时 {self.p["search_fallback_rotate_timeout"]:.1f}s')
        return False

    def execute(self):
        try:
            self._check()
            self._light(self.node.LIGHT_YELLOW, f'进入{self.target_label}寻找任务')
            if self.front_class is None or self.down_class is None:
                raise SearchFailure(f'模型映射缺少 {self.target_label} 的前视／下视类别')
            if not self.node._ensure_camera_extrinsics() or not any(
                    c in self.node.camera_extrinsics for c in self._history):
                raise SearchFailure(f'{self.target_label}前视寻找缺少相机 TF')
            self._sub = self.node.create_subscription(
                DetectionArray, PERCEPTION_DETECTIONS, self._detection_cb,
                QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
            pose = self._pose()
            self._action([*pose[:2], self.depth, pose[5]], 'z', f'寻找{self.target_label}：到达巡游水深',
                         timeout=self.p.get('depth_timeout', self.p['search_move_timeout']))
            pose = self._pose()
            delta = self.position[:2]-np.asarray(pose[:2])
            yaw = pose[5] if np.linalg.norm(delta) < 1e-6 else math.degrees(math.atan2(delta[1], delta[0]))
            self._action([*pose[:3], yaw], 'rz', f'寻找{self.target_label}：朝向预设目标位置')
            self._light(self.node.LIGHT_YELLOW, '前视相机观察')
            front_attempted = self._observe('front_', self.p['search_front_observe_seconds'], full_window=True)
            if front_attempted:
                self._align_front()
            else:
                self._flash(self.node.LIGHT_RED, 1, f'前视未看到{self.target_label}')
            # Both budgets count active BLINE time; stopped calibration pauses them.
            self._search_clock.reset()
            down_seen_at = None
            late_stereo_attempted = False

            def monitor(finished=False):
                nonlocal down_seen_at
                if down_seen_at is None and self._frames('down_'):
                    down_seen_at = self.first_down_seen_at
                    if down_seen_at is None:
                        down_seen_at = self._now()
                        self.first_down_seen_at = down_seen_at
                    self._note_down_frame(self._frames('down_'))
                if down_seen_at is not None:
                    if (finished or self._expired() or self._now()-down_seen_at
                            >= self.p['search_detection_stop_delay']):
                        return 'down'
                    # Finish the half-second delay unless the motion deadline ends first.
                    return None
                if self._expired():
                    return 'timeout'
                if not front_attempted and self._frames('front_'):
                    return 'front'
                if (front_attempted and not self.stereo_calibrated
                        and not late_stereo_attempted
                        and self._stereo_center() is not None):
                    return 'stereo'
                return None

            while not self._expired():
                reason = self._bline(monitor)
                if reason == 'down':
                    self._hold()
                    if getattr(self, 'rotate_fallback', False):
                        with self._lock:
                            after = self._sequence
                        self._light(self.node.LIGHT_YELLOW,
                                    f'下视相机观察{self.target_label}')
                        if self._observe(
                                'down_', self.p['search_fallback_observe_seconds'],
                                after, full_window=True):
                            return TaskOutcome.ok()
                        try:
                            found = self._rotate_for_down_frame(after)
                        except SearchFailure as scan_error:
                            if self.node.stopped or not rclpy.ok():
                                raise
                            raise FrameNotFound(
                                f'{self.target_label}原地旋转搜索未完成：'
                                f'{scan_error}') from scan_error
                        if found:
                            return TaskOutcome.ok()
                        raise FrameNotFound(
                            f'BLINE 中发现后观察 {self.p["search_fallback_observe_seconds"]:.1f}s，'
                            f'并原地旋转 '
                            f'{self.p["search_fallback_rotate_degrees"]:.0f}° 后，'
                            f'下视仍未检测到{self.target_label}')
                    return TaskOutcome.ok()
                if reason == 'front':
                    front_attempted = True
                    self._align_front()
                    continue
                if reason == 'stereo':
                    late_stereo_attempted = True
                    self._align_front()
                    continue
                if reason == 'timeout':
                    break
                # A successfully completed short line can be followed by another.
            self._hold()
            self._flash(self.node.LIGHT_RED, 1, f'{self.target_label} BLINE 寻找超时')
            pose = self._pose()
            depth = self.depth if self.correct_depth else pose[2]
            axes = 'xyz' if self.correct_depth else 'xy'
            self._action([*self.position[:2], depth, pose[5]], axes,
                         '寻找超时：定点到预设位置')
            with self._lock:
                after = self._sequence
            self._light(self.node.LIGHT_YELLOW, f'下视相机观察{self.target_label}')
            if self._observe(
                    'down_', self.p['search_fallback_observe_seconds'], after,
                    full_window=True):
                return TaskOutcome.ok()
            if getattr(self, 'rotate_fallback', False):
                try:
                    found = self._rotate_for_down_frame(after)
                except SearchFailure as scan_error:
                    if self.node.stopped or not rclpy.ok():
                        raise
                    raise FrameNotFound(
                        f'{self.target_label}原地旋转搜索未完成：{scan_error}') from scan_error
                if found:
                    return TaskOutcome.ok()
                raise FrameNotFound(
                    f'到达预设位置并原地旋转 {self.p["search_fallback_rotate_degrees"]:.0f}° 后，'
                    f'下视仍未检测到{self.target_label}')
            raise SearchFailure(f'到达预设目标位置后，下视仍未检测到{self.target_label}')
        except FrameNotFound as error:
            self.node.get_logger().error(f'{self.task_name}：{error}')
            if not self.node.stopped and rclpy.ok():
                self._flash(self.node.LIGHT_RED, 3, f'{self.target_label}寻找失败', restore=False)
                self.node._task_failure_light_handled = True
            return TaskOutcome.failed(f'{self.task_name}.no_frame', str(error))
        except SearchFailure as error:
            self.node.get_logger().error(f'{self.task_name}：{error}')
            if not self.node.stopped and rclpy.ok():
                self._flash(self.node.LIGHT_RED, 3, f'{self.target_label}寻找失败', restore=False)
                self.node._task_failure_light_handled = True
            return TaskOutcome.failed(f'{self.task_name}.search', str(error))
        finally:
            if self._sub is not None:
                self.node.destroy_subscription(self._sub)


class CollectionFrameSearch(FrontDownSearch):
    """Golf pickup approach with depth compensation during visual servo."""
    rotate_fallback = True


class TargetRackSearch(FrontDownSearch):
    """Drop-ball approach: set entry depth once, then preserve current depth."""
    # Rack has the same full-circle down-camera fallback as collection-frame
    # search.  The fallback is only entered after the complete observation
    # window at the preset/endpoint has elapsed.
    rotate_fallback = True

    def __init__(self, node, params):
        super().__init__(node, params, validator=validate_drop_search_params,
                         position_key='rack_position', front_class='target_rack_front',
                         down_class='target_rack_down', task_name='26rb_drop_ball_target_rack',
                         target_label='目标架', correct_depth=False)
