"""Sequential pickup with shared acquisition, check, servo and loss windows."""
from __future__ import annotations

from collections import deque
from importlib import import_module
import math
import time
from types import SimpleNamespace

import numpy as np

from uv_msgs.action import BasicMotion
from uv_task.collection_frame_search import CollectionFrameSearch
from uv_task.down_camera_servo import best_detection, normalized_image_error, body_to_world_rotation
from uv_task.task_outcome import TaskOutcome
from uv_task.task_state import TaskState
from uv_task.pickup_alignment import PickupWindow, target_world_xy, bounded_velocity

PickupController = import_module('uv_task.26rb_grab_golf').RB26GrabGolfTask


class PickupFailure(RuntimeError):
    pass


def validate_combined_params(params, *, complete=True):
    depth = params.get('work_depth_m', 0.2)
    if (isinstance(depth, bool) or not isinstance(depth, (int, float))
            or not math.isfinite(depth) or depth < 0):
        raise ValueError('work_depth_m 必须是有限非负数，NED向下为正，单位米')
    for kind in ('golf', 'ring'):
        key = kind+'_grab_depth_m'
        value = params.get(key, 0.44)
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or value < 0):
            raise ValueError(f'{key} 必须是有限非负数，表示机器人目标Z')
        if (complete or (key in params and 'work_depth_m' in params)) and value < depth:
            raise ValueError(f'{key} 必须不小于work_depth_m，表示机器人目标Z')
    step = params.get('retry_depth_step_m', 0.05)
    if (isinstance(step, bool) or not isinstance(step, (int, float))
            or not math.isfinite(step) or step < 0):
        raise ValueError('retry_depth_step_m 必须是有限非负数，单位米')
    if 'max_grab_retries' in params:
        raise ValueError('组合任务请分别配置golf.max_attempts和ring.max_attempts，次数包含首次')
    for key in ('golf_max_attempts', 'ring_max_attempts', 'ring_servo_repeat_count',
                'ring_orientation_min_samples'):
        value = params.get(key, 3)
        if isinstance(value, bool) or not isinstance(value, int) or value < 1:
            raise ValueError(f'{key} 必须是至少1的整数')
    for key, default in (('find_timeout', 15.0), ('check_timeout', 5.0),
                         ('horizontal_servo_timeout', 20.0), ('loss_timeout', 5.0),
                         ('ring_yaw_tolerance_deg', 3.0), ('ring_yaw_gain', 1.2),
                         ('ring_max_yaw_rate_deg_s', 10.0),
                         ('ring_orientation_spread_deg', 5.0), ('verification_timeout', 2.0),
                         ('return_timeout', 60.0), ('descent_timeout', 15.0),
                         ('descent_speed_mps', 0.03), ('vertical_publish_period', 0.05)):
        value = params.get(key, default)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'{key} 必须是有限正数')
    for key, default in (('ring_servo_repeat_period', .1), ('ring_servo_settle_seconds', 1.0)):
        value = params.get(key, default)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            raise ValueError(f'{key} 必须是有限非负数')
    for key, default in (('ring_open_angle_deg', 0.0), ('ring_close_angle_deg', 90.0)):
        value = params.get(key, default)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 <= value <= 270:
            raise ValueError(f'{key} 必须是[0,270]范围内的度数')
    if 'ring_orientation_timeout' in params:
        value = params['ring_orientation_timeout']
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError('旧ring_orientation_timeout必须是有限正数；组合任务现使用servo.timeout')
    quality = params.get('ring_orientation_min_quality', .8)
    if isinstance(quality, bool) or not isinstance(quality, (int, float)) or not math.isfinite(quality) or not 0 <= quality <= 1:
        raise ValueError('ring_orientation_min_quality 必须在[0,1]范围内')
    if params.get('ring_class', 'red_ring') != 'red_ring':
        raise ValueError('当前方向检测只支持 red_ring')


def wrap(angle):
    return (float(angle)+180.0) % 360.0-180.0


def choose_away_heading(axis_deg, xy, start_xy, current_yaw):
    """Choose one of the two ring-plane directions, facing away from START."""
    first, second = wrap(axis_deg), wrap(axis_deg+180.0)
    away = np.asarray(xy, dtype=float)-np.asarray(start_xy, dtype=float)
    direction = np.array([math.cos(math.radians(first)), math.sin(math.radians(first))])
    score = float(direction @ away)
    if abs(score) <= 1e-9:
        return min((first, second), key=lambda yaw: abs(wrap(yaw-current_yaw)))
    return first if score > 0 else second


class RB26GrabBallRingTask:
    def __init__(self, node, params):
        validate_combined_params(params)
        self.node, self.p = node, params
        self.work_depth = float(params.get('work_depth_m', 0.2))
        self.grab_depths = {kind: float(params.get(kind+'_grab_depth_m', 0.44))
                           for kind in ('golf', 'ring')}
        self.retry_depth_step = float(params.get('retry_depth_step_m', 0.05))
        self.descent_timeout = float(params.get('descent_timeout', 15.0))
        self.descent_speed = float(params.get('descent_speed_mps', 0.03))
        if not hasattr(node, 'state'):
            node.state = TaskState()
        self.state = node.state
        self.log = node.get_logger()
        self.ball = PickupController(node, params)
        self.ring = PickupController(
            node, params, target_name=params.get('ring_class', 'red_ring'),
            target_class_id=node._model_mapping.model_class_id('red_ring', required=False),
            gripper_frame='hairpin_claw_link', task_name='26rb_grab_ball_ring.ring')
        self.max_attempts = {kind: int(params.get(kind+'_max_attempts', 3)) for kind in ('golf', 'ring')}
        self.samples = deque(maxlen=64)
        self.sample_cursor = None
        self.find_timeout = float(params.get('find_timeout', 15.0))
        self.check_timeout = float(params.get('check_timeout', 5.0))
        self.servo_timeout = float(params.get('horizontal_servo_timeout', 20.0))
        self.loss_timeout = float(params.get('loss_timeout', 5.0))
        self._targets = {}
        self._ring_axis_at_alignment = None
        self._first_find_started = None
        self._frame_light_indicated = False
        self.yaw_tolerance = float(params.get('ring_yaw_tolerance_deg', 3.0))

    def _check(self):
        if self.node.stopped:
            raise PickupFailure('任务已取消')

    def _pose(self):
        # This task cannot use a stale commanded pose as a geometry measurement.
        pose = tuple(float(x) for x in self.node._latest_robot_pose(require_measured=True))
        if len(pose) != 6 or not all(math.isfinite(x) for x in pose):
            raise PickupFailure('实测位姿无效')
        return pose

    def _wait(self, seconds):
        until = time.monotonic()+seconds
        while time.monotonic() < until:
            self._check()
            time.sleep(min(.05, max(0.0, until-time.monotonic())))

    def _set_pose(self, target, label, *, axes='xyzrz', timeout=None):
        self._check()
        timeout = float(self.p.get('return_timeout', 60.0) if timeout is None else timeout)
        ok, message = self.node._send_action_goal(
            BasicMotion.Goal.SET, list(target), axes, timeout=timeout,
            wait_deadline=time.monotonic()+timeout,
            light_color=self.node.LIGHT_OFF, task_context=label)
        if not ok:
            raise PickupFailure(f'{label}失败：{message}')
        self.node._cmd_x, self.node._cmd_y, self.node._cmd_z, self.node._cmd_yaw = target

    def _restore_work_depth(self, label):
        """Complete a depth-only action before permitting horizontal recovery."""
        pose = self._pose()
        self._set_pose([pose[0], pose[1], self.work_depth, pose[5]], label,
                       axes='z', timeout=self.ball._command_timeout)

    def _return_frame(self, controller):
        """Return to this target's camera pose, without any visual servo."""
        kind = 'golf' if controller is self.ball else 'ring'
        anchor = getattr(self.state.pickup, kind+'_camera_pose')
        if anchor is None:
            raise PickupFailure('缺少本次目标下视对齐位姿')
        controller._servo_yaw_target = None
        self._restore_work_depth('组合抓取：抓取后恢复作业深度')
        self._set_pose(anchor, '组合抓取：返回'+kind+'下视对齐位姿', axes='xyrz')

    def _fresh_detection(self, class_id, controller):
        with self.node._perception_lock:
            entries = dict(self.node._down_detections)
        candidates = []
        now = time.monotonic()
        for camera in ('down_left', 'down_right'):
            entry = entries.get(camera)
            if entry is not None and 0 <= now-entry[0] <= controller._detection_timeout:
                detection = best_detection(entry[1], class_id)
                if detection is not None:
                    candidates.append((entry[0], camera, detection))
        if not candidates:
            return None, None
        # Keep one eye while it still has a valid observation, then switch to
        # the freshest other eye. Both eyes count as visibility for loss timing.
        active = getattr(controller, '_aligned_camera', None)
        selected = next((item for item in candidates if item[1] == active), max(candidates, key=lambda x: x[0]))
        return selected[1], selected[2]

    def _fallback_pose(self):
        recorded = self.state.pickup.last_servo_pose
        if recorded is not None:
            return recorded
        position = self.p['collection_frame_position']
        return (float(position[0]), float(position[1]), self.work_depth, self._pose()[5])

    def _record_target(self, kind, controller, camera, detection, *, timed_out):
        pose = self._pose()
        anchor = (*pose[:2], self.work_depth, pose[5])
        target = target_world_xy(self.node, camera, detection, pose, controller._projection_depth)
        self._targets[kind] = (target, pose, camera)
        if kind == 'ring':
            self._ring_axis_at_alignment = self._axis_estimate()
        controller._aligned_camera = camera
        self.state.update_pickup(**{kind+'_camera_pose': anchor, 'last_servo_pose': anchor})
        self.log.info(f'组合抓取：{kind} '+('servo超时，目标仍可见，采用当前观测抓取' if timed_out else '目标对齐完成'))
        if not timed_out:
            controller._flash_green(1, f'{kind}伺服成功')
        return 'ready'

    def _align_target(self, kind, controller, started, *, checking=False, exhausted=False):
        """One deadline survives frame/target/closed-loop-return transitions."""
        window = PickupWindow(started, self.check_timeout if checking else self.find_timeout,
                              self.servo_timeout, self.loss_timeout, checking=checking)
        controller._servo_depth = self.work_depth
        controller._servo_yaw_target = None
        controller._aligned_camera = None
        stable_since = None
        stable_camera = None
        frame_stable_since = None
        announced = None
        last_log = float('-inf')
        with self.node._perception_lock:
            earlier = [event[0] for event in self.node._down_detection_events if event[1] >= started]
            cursor = min(earlier)-1 if earlier and not checking else self.node._down_detection_sequence
        self.samples.clear()
        self.sample_cursor = None
        last_target_camera = None
        last_target_detection = None
        result = None
        try:
            while True:
                self._check()
                now = time.monotonic()
                with self.node._perception_lock:
                    events = tuple(event for event in self.node._down_detection_events if event[0] > cursor)
                if events and events[0][0] != cursor+1:
                    window.observations_valid = False
                for sequence, received, camera, message in events:
                    cursor = sequence
                    window.observe(received, camera, message, controller._golf_class_id,
                                   controller._detection_timeout)
                camera, detection = self._fresh_detection(controller._golf_class_id, controller)
                if detection is not None:
                    last_target_camera, last_target_detection = camera, detection
                # A fresh cached sighting may predate entry into a new ring stage.
                if not checking and detection is not None and now <= window.search_deadline:
                    window.seen = True
                result = window.absent_result(now, controller._detection_timeout)
                if result is not None:
                    self.log.info(f'组合抓取：{kind} '+('check' if checking else 'find')+'窗口结束，未看到目标：'+result)
                    break
                if exhausted and window.seen:
                    result = 'exhausted'
                    break
                frame_camera, frame = self._fresh_detection(controller._collection_class_id, controller)
                if window.transition(now, detection is not None, frame is not None):
                    stable_since = frame_stable_since = None
                if announced != window.mode:
                    announced = window.mode
                    self.log.info(f'组合抓取：{kind} 进入{window.mode}，servo剩余={max(0., window.deadline-now):.1f}s')
                if frame is not None and not self._frame_light_indicated:
                    self._frame_light_indicated = True
                    controller._flash_green(1, '首次看到frame')
                phase_color = (
                    self.node.LIGHT_YELLOW if window.mode == 'target'
                    or (window.mode == 'frame' and frame is None)
                    else self.node.LIGHT_OFF)
                self.node._set_task_phase_light(
                    phase_color, f'组合抓取：{window.mode}阶段')
                if now >= window.deadline:
                    if window.seen and last_target_detection is not None:
                        # Keep the last valid target sample for the mechanical
                        # attempt.  A timeout may coincide with the current
                        # frame aging out, but it must not skip the two-object
                        # pickup sequence after a real sighting.
                        result = self._record_target(
                            kind, controller, last_target_camera,
                            last_target_detection, timed_out=True)
                    else:
                        pose = self._pose()
                        self.state.update_pickup(last_servo_pose=(*pose[:2], self.work_depth, pose[5]))
                        result = 'unobserved'
                    break
                horizontal = np.zeros(2)
                controller._servo_yaw_target = None
                if window.mode == 'target' and detection is not None:
                    controller._aligned_camera = camera
                    pose, horizontal, du, dv = controller._horizontal_velocity(camera, detection)
                    if kind == 'ring':
                        self._sample_axis()
                    centred = abs(du) <= controller._pixel_tolerance and abs(dv) <= controller._pixel_tolerance
                    at_depth = abs(pose[2]-self.work_depth) <= controller._depth_hold_tolerance
                    if stable_camera != camera:
                        stable_since = None
                        stable_camera = camera
                    stable_since = (now if stable_since is None else stable_since) if centred and at_depth else None
                    if centred:
                        horizontal[:] = 0.
                    if stable_since is not None and now-stable_since >= controller._hold_seconds:
                        result = self._record_target(kind, controller, camera, detection, timed_out=False)
                        break
                elif window.mode == 'frame' and frame is not None:
                    controller._aligned_camera = frame_camera
                    pose, horizontal, du, dv = controller._horizontal_velocity(frame_camera, frame)
                    centred = abs(du) <= controller._pixel_tolerance and abs(dv) <= controller._pixel_tolerance
                    if centred:
                        horizontal[:] = 0.
                        frame_stable_since = now if frame_stable_since is None else frame_stable_since
                        if now-frame_stable_since >= controller._hold_seconds:
                            self.state.update_pickup(last_servo_pose=(*pose[:2], self.work_depth, pose[5]))
                    else:
                        frame_stable_since = None
                elif window.mode == 'return':
                    anchor = self._fallback_pose()
                    pose = self._pose()
                    horizontal = bounded_velocity(np.asarray(anchor[:2])-pose[:2], controller._servo_gain, controller._max_xy_speed)
                    controller._servo_yaw_target = anchor[3]
                else:
                    stable_since = frame_stable_since = None
                # Correction time is motion time, not an observation budget:
                # suspend both the confirmation and servo deadlines while a
                # non-zero visual correction is being sent.  Resume as soon
                # as the correction settles or the target is lost.
                if np.linalg.norm(horizontal) > 1e-6:
                    window.pause(now)
                else:
                    window.resume(now)
                if now-last_log >= controller._log_period:
                    self.log.info(f'组合抓取：{kind} {window.mode}速度伺服，世界XY速度=({horizontal[0]:+.3f},{horizontal[1]:+.3f})m/s')
                    last_log = now
                servo_light = (
                    self.node.LIGHT_YELLOW if window.mode == 'target'
                    or (window.mode == 'frame' and frame is None)
                    else self.node.LIGHT_OFF)
                controller._send_horizontal_velocity(
                    horizontal, window.deadline, light_color=servo_light)
                next_deadline = window.deadline
                if not window.seen:
                    next_deadline = min(next_deadline, window.search_deadline)
                self._wait(min(controller._servo_period, max(0., next_deadline-time.monotonic())))
        finally:
            # Cleanup holds the measured heading; it must not finish an old
            # fallback yaw move after the shared visual deadline has expired.
            controller._servo_yaw_target = None
            if controller._stop_horizontal_velocity(
                    light_color=self.node.LIGHT_OFF) is None:
                raise PickupFailure('组合抓取：视觉修正结束停车保持失败')
        return result

    def _sample_axis(self):
        camera = self.ring._aligned_camera
        with self.node._perception_lock:
            entry = self.node._down_detections.get(camera)
        if entry is None or (self.sample_cursor is not None and entry[0] <= self.sample_cursor) or time.monotonic()-entry[0] > self.ring._detection_timeout:
            return
        self.sample_cursor = entry[0]
        det = best_detection(entry[1], self.ring._golf_class_id)
        if (det is None or not getattr(det, 'orientation_valid', False)
                or not math.isfinite(float(getattr(det, 'orientation_quality', 0.0)))
                or float(det.orientation_quality) < self.p.get('ring_orientation_min_quality', .8)):
            self.samples.clear()
            return
        angle = float(det.orientation_axis_deg)
        if not math.isfinite(angle):
            self.samples.clear()
            return
        theta = math.radians(angle)
        delta = np.array([math.cos(theta), math.sin(theta)])*10.0
        points = [SimpleNamespace(pixel_x=det.pixel_x+sign*delta[0], pixel_y=det.pixel_y+sign*delta[1])
                  for sign in (-1, 1)]
        rays = [np.array([*normalized_image_error(self.node, camera, point, reference=det), 1.0]) for point in points]
        rotation = body_to_world_rotation(self._pose()) @ self.node.camera_extrinsics[camera].optical_to_body
        rays = [rotation @ ray for ray in rays]
        # Intersect both rays with the same horizontal plane; its unknown depth
        # scales the difference, but does not change this undirected XY axis.
        if min(ray[2] for ray in rays) < 1e-6:
            self.samples.clear()
            return
        axis = rays[1][:2]/rays[1][2]-rays[0][:2]/rays[0][2]
        if np.linalg.norm(axis) < 1e-8:
            self.samples.clear()
            return
        self.samples.append((entry[0], math.degrees(math.atan2(axis[1], axis[0])) % 180.0))

    def _axis_estimate(self):
        fresh = [(stamp, angle) for stamp, angle in self.samples
                 if time.monotonic()-stamp <= self.ring._detection_timeout]
        if len(fresh) < int(self.p.get('ring_orientation_min_samples', 3)):
            return None
        vector = np.mean([[math.cos(math.radians(2*angle)), math.sin(math.radians(2*angle))]
                          for _, angle in fresh], axis=0)
        if np.linalg.norm(vector) < 1e-6:
            return None
        mean = math.degrees(math.atan2(vector[1], vector[0]))/2.0 % 180.0
        if max(abs((angle-mean+90.0) % 180.0-90.0) for _, angle in fresh) > self.p.get('ring_orientation_spread_deg', 5.0):
            return None
        return mean

    def _ring_servo(self, angle, label):
        for index in range(int(self.p.get('ring_servo_repeat_count', 3))):
            self._check()
            self.node.set_servo(float(angle), f'抓环{label} ({index+1})', servo_id=2)
            if index+1 < int(self.p.get('ring_servo_repeat_count', 3)):
                self._wait(float(self.p.get('ring_servo_repeat_period', .1)))
        self._wait(float(self.p.get('ring_servo_settle_seconds', 1.0)))

    def _align_claw(self, kind, controller, yaw):
        target_xy, observed_pose, camera = self._targets[kind]
        provider = self.node.camera_extrinsics_provider
        try:
            tf = provider.lookup_transform(provider.base_frame, controller._gripper_frame)
            tf = tf.transform if hasattr(tf, 'transform') else tf
            claw = np.array([tf.translation.x, tf.translation.y, tf.translation.z], dtype=float)
        except Exception as error:
            raise PickupFailure('读取夹爪外参失败：'+str(error)) from error
        final_pose = list(self._pose())
        final_pose[5] = yaw
        offset = body_to_world_rotation(final_pose) @ claw
        if not np.all(np.isfinite(offset)):
            raise PickupFailure('夹爪外参无效')
        self._set_pose([target_xy[0]-offset[0], target_xy[1]-offset[1], self.work_depth, yaw],
                       '组合抓取：'+kind+'转向与夹爪对准同时执行', timeout=controller._command_timeout)

    def _orient_and_align_ring(self):
        # Samples were collected while servoing, without a second observation
        # timeout or a second camera-centering pass after turning.
        axis = self._ring_axis_at_alignment
        pose = self._pose()
        yaw = pose[5]
        if axis is not None:
            yaw = choose_away_heading(axis, pose[:2], self.state.pickup.start_xy, pose[5])
        else:
            self.log.warning('抓环：有效方向不足，保持当前航向对爪抓取')
        self._align_claw('ring', self.ring, yaw)
        return True

    def _grab_target_depth(self, kind):
        attempt = max(1, getattr(self.state.pickup, kind+'_attempts'))
        target = self.grab_depths[kind]+(attempt-1)*self.retry_depth_step
        if not math.isfinite(target):
            raise PickupFailure('重试抓取目标深度无效')
        return target

    def _descend_to_depth(self, kind, controller):
        """Descend in world Z until measured depth reaches this attempt's target."""
        target = self._grab_target_depth(kind)
        attempt = max(1, getattr(self.state.pickup, kind+'_attempts'))
        deadline = time.monotonic()+self.descent_timeout
        period = controller._vertical_period
        lease = max(0.25, 4*period)
        reached = False
        neutral_ok = False
        self.log.info(
            f'组合抓取：{kind} 第{attempt}次下潜，基础深度={self.grab_depths[kind]:.3f}m，'
            f'重试增量={(attempt-1)*self.retry_depth_step:.3f}m，目标深度={target:.3f}m，'
            f'速度上限={self.descent_speed:.3f}m/s，超时={self.descent_timeout:.1f}s')
        try:
            while not self.node.stopped:
                pose = self._pose()
                remaining = target-pose[2]
                now = time.monotonic()
                if remaining <= 1e-6 and now <= deadline:
                    reached = True
                    break
                if now >= deadline:
                    self.log.warning(f'组合抓取：{kind} 下潜超时，实测深度={pose[2]:.3f}m，目标={target:.3f}m')
                    break
                # Reduce the final step and compensate roll/pitch so descent
                # changes world Z without introducing world XY motion.
                vz = min(self.descent_speed, remaining/period)
                body = body_to_world_rotation(pose).T @ np.array([0.0, 0.0, vz])
                ok, message = self.node._send_body_velocity(
                    *body.tolist(), yaw_rate_deg_s=0.0, lease_s=lease,
                    wait_deadline=min(deadline, now+2.0),
                    light_color=self.node.LIGHT_OFF,
                    task_context=f'组合抓取：{kind} 下潜到{target:.3f}m')
                if not ok:
                    self.log.warning(f'组合抓取：{kind} 下潜速度指令失败：{message}')
                    break
                self._wait(min(period, max(0.0, deadline-time.monotonic())))
        finally:
            # Cleanup gets its own deadline even after the 15-second budget.
            neutral_ok, message = self.node._send_body_velocity(
                lease_s=lease, wait_deadline=time.monotonic()+2.0,
                light_color=self.node.LIGHT_OFF,
                task_context=f'组合抓取：{kind} 结束目标深度下潜')
            if not neutral_ok:
                self.log.warning(f'组合抓取：{kind} 下潜停车失败：{message}')
        if reached and neutral_ok and not self.node.stopped:
            self.log.info(f'组合抓取：{kind} 已到目标深度{target:.3f}m，停止下潜')
            return True
        return False

    def _mechanical_attempt(self, kind, controller):
        if kind == 'golf':
            if not controller._prepare_claw():
                raise PickupFailure('准备圆盘爪失败')
            self._align_claw(kind, controller, self._pose()[5])
        else:
            self._ring_servo(self.p.get('ring_open_angle_deg', 0.0), '松开')
            self._orient_and_align_ring()
        if not controller._wait_pre_descent_settle():
            raise PickupFailure('下潜前等待被取消')
        if not self._descend_to_depth(kind, controller):
            raise PickupFailure(kind+'下潜或停车失败')
        if kind == 'ring':
            self._ring_servo(self.p.get('ring_close_angle_deg', 90.0), '合爪')
        return True

    def _run_target(self, kind, controller):
        if controller._golf_class_id is None:
            self.state.update_pickup(**{kind+'_status': 'failed'})
            return
        started = self._first_find_started if kind == 'golf' else time.monotonic()
        if started is None:
            started = time.monotonic()
        checking = False
        performed = False
        attempt = 0
        while True:
            self._check()
            exhausted = attempt >= self.max_attempts[kind]
            result = self._align_target(kind, controller, started, checking=checking, exhausted=exhausted)
            if result == 'absent' and checking and performed:
                self.state.update_pickup(**{kind+'_status': 'success'})
                if kind == 'golf':
                    controller._flash_green(1, '球抓取确认完成，开始搜环')
                return
            if result != 'ready' or exhausted:
                self.state.update_pickup(**{kind+'_status': 'failed'})
                return
            attempt += 1
            self.state.update_pickup(**{kind+'_attempts': attempt})
            self.log.info(f'组合抓取：{kind} 第{attempt}/{self.max_attempts[kind]}次抓取')
            performed = self._mechanical_attempt(kind, controller)
            self._return_frame(controller)
            started = time.monotonic()
            checking = True

    def execute(self):
        self.state.reset_pickup()
        try:
            if self.ball._collection_class_id is None:
                raise PickupFailure('模型映射没有collection_frame_down')
            # The shared search helper retains its legacy key for other tasks;
            # this task supplies its single work-depth target to that helper.
            search_params = {**self.p, 'search_cruise_depth_m': self.work_depth}
            search = CollectionFrameSearch(self.node, search_params)
            search.task_name = '26rb_grab_ball_ring'
            search.target_label = '收集盘'
            approach = search.execute()
            self._frame_light_indicated = bool(
                getattr(search, 'first_down_indicated', False))
            self._first_find_started = getattr(search, 'first_down_seen_at', None)
            if self._first_find_started is None:
                self._first_find_started = time.monotonic()
            self.state.update_pickup(first_frame_seen_at=self._first_find_started)
            if not approach:
                return approach
            self._restore_work_depth('组合抓取：搜索结束恢复作业深度')
            pose = self._pose()
            anchor = (pose[0], pose[1], self.work_depth, pose[5])
            self.state.update_pickup(frame_pose=anchor, depth=self.work_depth)
            for controller in (self.ball, self.ring):
                controller._servo_depth = self.work_depth
                controller._servo_yaw_gain = float(self.p.get('ring_yaw_gain', 1.2))
                controller._servo_max_yaw_rate = float(self.p.get('ring_max_yaw_rate_deg_s', 10.0))
                controller._servo_yaw_tolerance = self.yaw_tolerance
            self._run_target('golf', self.ball)
            self._run_target('ring', self.ring)
            result = self.state.pickup
            if result.golf_status == result.ring_status == 'success':
                return TaskOutcome.ok('球和环均抓取成功')
            failed = [kind for kind in ('golf', 'ring') if getattr(result, kind+'_status') != 'success']
            code = failed[0]+'_exhausted' if len(failed) == 1 else 'both_exhausted'
            return TaskOutcome.failed('26rb_grab_ball_ring.'+code, '未确认抓取成功：'+','.join(failed))
        except (ValueError, KeyError, RuntimeError) as error:
            code = 'cancelled' if self.node.stopped else 'recovery'
            return TaskOutcome.failed('26rb_grab_ball_ring.'+code, str(error))
