"""Preset-position impact-ball workflow using front detections and BLINE.

Set task depth once on entry. Later commands adjust XY/yaw only, and every
BLINE has dz=0. Visual failures run a bounded blind approach and finish the
whole task; motion failures and cancellation do not start another motion.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import time

import numpy as np
import rclpy
from uv_msgs.action import BasicMotion
from uv_task.front_target_observer import FrontTargetObserver, wrap_degrees
from uv_task.hit_ball_config import validate_hit_params
from uv_task.task_outcome import TaskOutcome


class MotionFailure(RuntimeError):
    pass


class ObservationFailure(RuntimeError):
    pass


@dataclass(frozen=True)
class LineResult:
    reason: str
    drive_started: float | None


class RB26HitBallsTask:
    def __init__(self, node, params):
        self._node = node
        self._params = validate_hit_params(params)
        self._now = time.monotonic
        self._sleep = time.sleep
        self._observer = FrontTargetObserver(node, self._params)
        self._yaw_active = False
        self._bline_active = False
        self._name = ''
        self._position = None

    def destroy(self):
        self._observer.destroy()

    def _check(self):
        if self._node.stopped or not rclpy.ok():
            raise MotionFailure('撞球任务已取消')

    def _pose(self):
        pose = tuple(float(x) for x in self._node._latest_robot_pose())
        if len(pose) != 6 or not all(math.isfinite(x) for x in pose):
            raise MotionFailure('撞球任务实测位姿无效')
        return pose

    def _sync(self):
        pose = self._pose()
        (self._node._cmd_x, self._node._cmd_y, self._node._cmd_z,
         self._node._cmd_yaw) = (*pose[:3], pose[5])

    def _light(self, color, label):
        self._node._set_task_phase_light(color, f'26rb_hit_balls：{self._name} {label}')

    def _wait_until(self, deadline):
        while self._now() < deadline:
            self._check()
            self._sleep(min(self._params['search_period'], deadline-self._now()))

    def _flash(self, color, count, label):
        for index in range(count):
            self._light(color, f'{label} ({index+1}/{count})')
            self._wait_until(self._now()+self._params['light_pulse_seconds'])
            self._light(self._node.LIGHT_OFF, '闪灯间隔')
            if index+1 < count:
                self._wait_until(self._now()+self._params['light_gap_seconds'])

    def _set(self, target, axes, timeout, label):
        self._check()
        if self._bline_active:
            raise MotionFailure('BLINE 未结束，禁止发送 SET')
        ok, message = self._node._send_action_goal(
            BasicMotion.Goal.SET, list(map(float, target)), axes,
            timeout=timeout, wait_deadline=self._now()+timeout,
            task_context=self._node._format_motion_context(f'{self._name} {label}'))
        if not ok:
            raise MotionFailure(f'{label}失败：{message}')
        self._sync()

    def _hold(self):
        pose = self._pose()
        # No z axis here: do not restore depth from visual estimates or preset z.
        self._set([*pose[:3], pose[5]], 'xyrz',
                  self._params['search_rotate_timeout'], '停车保持当前水平位置和航向')

    def _turn(self, yaw, label):
        pose = self._pose()
        self._set([*pose[:3], wrap_degrees(yaw)], 'rz',
                  self._params['search_rotate_timeout'], label)

    def _look_at_preset(self):
        pose = self._pose()
        delta = self._position[:2]-np.asarray(pose[:2])
        yaw = pose[5] if np.linalg.norm(delta) < 1e-6 else math.degrees(math.atan2(delta[1], delta[0]))
        self._turn(yaw, '朝向当前颜色球的预设 XY')

    def _observe(self, seconds, full_window=False):
        # Each observation phase requires a callback after phase entry.
        after = self._observer.cursor()
        deadline = self._now()+seconds
        seen = False
        while self._now() < deadline:
            self._check()
            if self._observer.frames(after):
                seen = True
                if not full_window:
                    return True
            self._sleep(min(self._params['search_period'], deadline-self._now()))
        return seen or bool(self._observer.frames(after))

    def _neutral_yaw(self):
        if not self._yaw_active:
            return
        ok, message = self._node._send_body_velocity(
            task_context=self._node._format_motion_context(f'{self._name} 结束 yaw 伺服'),
            wait_deadline=self._now()+self._params['motion_cancel_timeout'])
        if not ok:
            raise MotionFailure(f'停止 yaw 伺服失败：{message}')
        self._yaw_active = False

    def _align(self, seconds, deadline=math.inf):
        """Zero translation, yaw-only servo; prefer geometrically valid stereo."""
        until = min(deadline, self._now()+seconds)
        after = self._observer.cursor()
        owner = self._observer.first_eye
        stable_mode = stable_since = None
        mono = stereo = False
        self._light(self._node.LIGHT_YELLOW, '前视单目／双目 yaw 对准')
        try:
            while self._now() < until:
                self._check()
                frames = self._observer.frames(after)
                frame = next((f for f in frames if f.eye == owner), None)
                if frame is None and frames:
                    frame = frames[0]
                    owner = frame.eye
                    stable_since = None
                pose = self._pose()
                center = self._observer.stereo_center(after) if frame is not None else None
                direction = center-np.asarray(pose[:3]) if center is not None else (
                    frame.ray if frame is not None else None)
                if direction is not None and np.linalg.norm(direction[:2]) < 1e-6:
                    direction = None
                error = None if direction is None else wrap_degrees(
                    math.degrees(math.atan2(direction[1], direction[0]))-pose[5])
                mode = 'stereo' if center is not None else owner
                if error is not None and abs(error) <= self._params['search_yaw_tolerance_deg']:
                    if stable_since is None or stable_mode != mode:
                        stable_since, stable_mode = self._now(), mode
                    if self._now()-stable_since >= self._params['search_yaw_stable_seconds']:
                        if center is not None:
                            stereo = True
                        else:
                            mono = True
                else:
                    stable_since = None
                rate = 0.0 if error is None else float(np.clip(
                    error*self._params['search_yaw_gain'],
                    -self._params['search_max_yaw_rate_deg_s'],
                    self._params['search_max_yaw_rate_deg_s']))
                self._yaw_active = True
                ok, message = self._node._send_body_velocity(
                    yaw_rate_deg_s=rate, lease_s=max(0.25, 4*self._params['search_period']),
                    wait_deadline=until,
                    task_context=self._node._format_motion_context(f'{self._name} 前视 yaw 伺服；无垂向修正'))
                if not ok:
                    if self._now() >= until:
                        break
                    raise MotionFailure(f'yaw 伺服失败：{message}')
                self._sleep(min(self._params['search_period'], max(0.0, until-self._now())))
        finally:
            self._neutral_yaw()
        self._hold()
        valid = bool(self._observer.frames(after))
        if valid and mono:
            self._flash(self._node.LIGHT_GREEN, 1, '单目对准完成')
        if valid and stereo:
            self._flash(self._node.LIGHT_GREEN, 2, '双目对准完成')
        if valid and not mono and not stereo:
            self._node.get_logger().warning(
                f'hit_balls：{self._name} 对准预算用尽仍未稳定，保留当前实测航向')
        # Flashing must not hide a target loss before the next motion starts.
        return bool(self._observer.frames(after))

    def _cancel_line(self, handle, result_future, allow_timeout=False):
        if not result_future.done():
            handle.cancel_goal_async()
        until = self._now()+self._params['motion_cancel_timeout']
        while not result_future.done() and rclpy.ok() and self._now() < until:
            self._sleep(self._params['search_period'])
        if not result_future.done():
            raise MotionFailure('BLINE 取消尚未确认结束，停止后续运动')
        completed = result_future.result()
        if completed.status == 6:  # ROS action STATUS_ABORTED, not our requested cancellation.
            if not (allow_timeout and 'timeout' in completed.result.message.lower()):
                raise MotionFailure(f'BLINE 中止：{completed.result.message}')

    def _run_bline(self, displacement, speed, timeout, label, monitor=None, duration=None):
        """Monitor a finite BLINE; timed phases start at positive speed feedback."""
        self._check()
        if self._yaw_active or self._bline_active:
            raise MotionFailure('旧速度或 BLINE 未结束，禁止启动新的 BLINE')
        deadline = self._now()+timeout
        client = self._node._action_client
        if not client.wait_for_server(timeout_sec=min(2.0, timeout)):
            raise MotionFailure('BasicMotion 动作服务器不可用')
        goal = BasicMotion.Goal()
        goal.cmd_type = BasicMotion.Goal.BLINE
        goal.axes = 'xyz'
        goal.target = [float(displacement[0]), float(displacement[1]), 0.0, 0.0]
        goal.cruise_speed = float(speed)
        goal.timeout = max(0.001, deadline-self._now())
        goal.task_context = self._node._format_motion_context(f'{self._name} {label}')
        drive = {'started': None}

        def feedback(message):
            f = message.feedback
            if (drive['started'] is None and f.phase in ('CAPTURE', 'CRUISE', 'BRAKE', 'TERMINAL')
                    and math.isfinite(f.along_speed_mps) and f.along_speed_mps > 0):
                drive['started'] = self._now()

        self._light(self._node.LIGHT_YELLOW, label)
        self._node._last_motion_final_target = None
        future = client.send_goal_async(goal, feedback_callback=feedback)
        accept_until = min(deadline, self._now()+self._params['motion_accept_timeout'])
        while not future.done() and not self._node.stopped and rclpy.ok() and self._now() < accept_until:
            self._sleep(self._params['search_period'])
        if not future.done() or self._node.stopped or not rclpy.ok():
            def cancel_late(done):
                try:
                    handle = done.result()
                    if handle.accepted:
                        handle.cancel_goal_async()
                except Exception:
                    pass
            future.add_done_callback(cancel_late)
            self._check()
            raise MotionFailure('BLINE 目标接受超时，已登记迟到取消')
        handle = future.result()
        if not handle.accepted:
            raise MotionFailure('BLINE 被拒绝')
        self._bline_active = True
        self._node._active_goal_handle = handle
        result_future = handle.get_result_async()
        start_until = min(deadline, self._now()+self._params['motion_start_timeout'])
        try:
            while True:
                self._check()
                if result_future.done():
                    result = result_future.result().result
                    if not result.success:
                        if self._now() >= deadline and 'timeout' in result.message.lower():
                            self._sync()
                            return LineResult('timeout', drive['started'])
                        raise MotionFailure(f'BLINE 失败：{result.message}')
                    target = list(result.final_target)
                    if len(target) != 4 or not all(math.isfinite(x) for x in target):
                        raise MotionFailure('BLINE 返回无效 final_target')
                    (self._node._cmd_x, self._node._cmd_y, self._node._cmd_z,
                     self._node._cmd_yaw) = target
                    self._node._last_motion_final_target = target
                    return LineResult('endpoint', drive['started'])
                reason = monitor() if monitor is not None else None
                if reason is None and duration is not None and drive['started'] is not None:
                    if self._now()-drive['started'] >= duration:
                        reason = 'duration'
                if reason is None and self._now() >= deadline:
                    reason = 'timeout'
                if reason is None and duration is not None and drive['started'] is None and self._now() >= start_until:
                    raise MotionFailure('BLINE 未在起步时限内产生前进反馈')
                if reason is not None:
                    self._cancel_line(handle, result_future, allow_timeout=reason == 'timeout')
                    self._sync()
                    return LineResult(reason, drive['started'])
                self._sleep(self._params['search_period'])
        finally:
            try:
                if not result_future.done():
                    self._cancel_line(handle, result_future)
            finally:
                if result_future.done():
                    self._bline_active = False
                    if self._node._active_goal_handle is handle:
                        self._node._active_goal_handle = None

    def _cruise_to_radius(self, front_seen):
        p = self._params
        deadline = self._now()+p['cruise_timeout']

        def monitor():
            if np.linalg.norm(np.asarray(self._pose()[:2])-self._position[:2]) <= p['cruise_radius_m']:
                return 'radius'
            if self._now() >= deadline:
                return 'timeout'
            if not front_seen and self._observer.frames():
                return 'front'
            return None

        while True:
            self._check()
            state = monitor()
            if state == 'radius':
                self._hold()
                return
            if state == 'timeout':
                raise ObservationFailure('靠近预设位置的 BLINE 总超时')
            if state == 'front':
                front_seen = True
                if not self._align(p['search_align_seconds'], deadline):
                    raise ObservationFailure('途中首次发现目标，但修正阶段结束时目标丢失')
                continue
            remaining = max(0.001, deadline-self._now())
            result = self._run_bline(
                [p['cruise_speed_mps']*remaining+1.0, 0.0], p['cruise_speed_mps'],
                remaining, 'BLINE 靠近预设位置水平半径', monitor=monitor)
            if result.reason == 'radius':
                self._hold()
                return
            if result.reason == 'timeout':
                raise ObservationFailure('靠近预设位置的 BLINE 总超时')
            if result.reason == 'front':
                front_seen = True
                if not self._align(p['search_align_seconds'], deadline):
                    raise ObservationFailure('途中首次发现目标，但修正阶段结束时目标丢失')

    def _near_observation(self):
        p = self._params
        base_yaw = self._pose()[5]
        if self._observe(p['observe_timeout']):
            return
        # Body/world yaw follows NED: negative is left, positive is right.
        for sign, label in ((-1, '左'), (1, '右')):
            self._turn(base_yaw+sign*p['observe_yaw_step_deg'],
                       f'近场向{label} {p["observe_yaw_step_deg"]:.1f}° 扫视')
            if self._observe(p['observe_direction_dwell_seconds']):
                return
        raise ObservationFailure('近场正前方及左右扫视均未发现当前颜色球')

    def _fallback(self, message):
        p = self._params
        self._node.get_logger().warning(f'hit_balls：{self._name} {message}；执行预设位置兜底并结束整个撞球任务')
        self._hold()
        self._flash(self._node.LIGHT_RED, 1, '观测失败，直接前往预设 XY')
        self._look_at_preset()
        pose = self._pose()
        world_delta = self._position[:2]-np.asarray(pose[:2])
        began = self._now()
        if np.linalg.norm(world_delta) > 1e-6:
            yaw = math.radians(pose[5])
            c, s = math.cos(yaw), math.sin(yaw)
            body_delta = [c*world_delta[0]+s*world_delta[1],
                          -s*world_delta[0]+c*world_delta[1]]
            result = self._run_bline(body_delta, p['fallback_speed_mps'],
                                     p['fallback_timeout'], '预设 XY 兜底 BLINE',
                                     duration=p['fallback_duration'])
            if result.reason == 'timeout':
                raise MotionFailure('兜底 BLINE 动作超时')
            began = result.drive_started if result.drive_started is not None else began
        self._hold()
        # Reaching the finite preset endpoint early never sends us beyond it.
        self._wait_until(began+p['fallback_duration'])
        self._light(self._node.LIGHT_OFF, '兜底结束，未通过视觉确认撞击')
        return TaskOutcome.ok(f'{self._name} 兜底前往预设 XY 后结束；未通过视觉确认撞击')

    def _one_ball(self):
        p = self._params
        self._look_at_preset()
        seen = self._observe(p['search_initial_observe_seconds'], full_window=True)
        if seen:
            if not self._align(p['search_align_seconds']):
                self._flash(self._node.LIGHT_RED, 1, '初始前视观测丢失，继续靠近')
        else:
            self._flash(self._node.LIGHT_RED, 1, '初始两秒未看到当前颜色球')
        self._cruise_to_radius(seen)
        self._near_observation()
        if not self._align(p['search_align_seconds']):
            raise ObservationFailure('近场对准结束时没有有效目标')
        result = self._run_bline([p['approach_distance_m'], 0.0],
                                 p['approach_speed_mps'], p['approach_timeout'],
                                 '对准后 BLINE 前进 0.5m')
        if result.reason != 'endpoint':
            raise MotionFailure('前进 0.5m 的 BLINE 超时')
        self._hold()
        if not self._align(p['charge_alignment_seconds']):
            raise ObservationFailure('撞击前两秒对准结束时没有有效目标')
        distance = p['charge_speed_mps']*p['charge_duration']+1.0
        result = self._run_bline([distance, 0.0], p['charge_speed_mps'],
                                 p['charge_timeout'], 'BLINE 前向定时撞击',
                                 duration=p['charge_duration'])
        if result.reason != 'duration':
            raise MotionFailure('定时撞击未完成指定前进时长')
        self._hold()
        self._light(self._node.LIGHT_GREEN, '定时撞击完成；不返回原位置')
        return TaskOutcome.ok()

    def execute(self):
        p = self._params
        node = self._node
        try:
            self._check()
            self._light(node.LIGHT_YELLOW, '进入撞球任务')
            for name in p['order']:
                key = 'targets_blue_position' if name == 'impact_ball_blue' else 'targets_red_position'
                if len(p[key]) != 3:
                    return TaskOutcome.failed('26rb_hit_balls.configuration', f'{name} 预设 [x,y,z] 尚未填写')
                if node._model_mapping.model_class_id(name, required=False) is None:
                    return TaskOutcome.failed('26rb_hit_balls.mapping', f'模型映射缺少 {name}')
            if not node._ensure_camera_extrinsics() or not any(
                    eye in node.camera_extrinsics for eye in ('front_left', 'front_right')):
                return TaskOutcome.failed('26rb_hit_balls.camera_tf', '撞球任务缺少前视相机 TF')
            pose = self._pose()
            self._set([*pose[:2], p['depth_task_depth_m'], pose[5]], 'z',
                      p['depth_timeout'], '入场定深一次')
            for index, name in enumerate(p['order']):
                self._name = name
                key = 'targets_blue_position' if name == 'impact_ball_blue' else 'targets_red_position'
                self._position = np.asarray(p[key])
                self._observer.select(node._model_mapping.model_class_id(name, required=False))
                node.get_logger().info(f'hit_balls：{name} 预设位置={list(self._position)}；后续无任务层深度修正')
                try:
                    self._one_ball()
                except ObservationFailure as error:
                    return self._fallback(str(error))
                if index+1 < len(p['order']):
                    self._wait_until(self._now()+p['between_balls_pause'])
            return TaskOutcome.ok('指定球的 BLINE 定时撞击流程完成')
        except MotionFailure as error:
            node.get_logger().error(f'hit_balls：{error}')
            code = '26rb_hit_balls.cancelled' if node.stopped or not rclpy.ok() else '26rb_hit_balls.motion'
            return TaskOutcome.failed(code, str(error))
