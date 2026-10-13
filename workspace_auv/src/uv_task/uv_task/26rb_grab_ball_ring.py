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
    pickup_order = params.get('pickup_order', ('golf', 'ring'))
    if (not isinstance(pickup_order, (list, tuple)) or len(pickup_order) != 2
            or any(not isinstance(kind, str) or kind not in ('golf', 'ring')
                   for kind in pickup_order)
            or len(set(pickup_order)) != 2):
        raise ValueError('pickup.order 必须恰好包含 golf 和 ring 各一次')
    final_tolerance = params.get('target_final_tolerance_fraction', 0.10)
    if (isinstance(final_tolerance, bool) or not isinstance(final_tolerance, (int, float))
            or not math.isfinite(final_tolerance) or not 0 < final_tolerance <= 0.25):
        raise ValueError('servo.final_tolerance_fraction 必须在(0,0.25]范围内')
    for kind in ('golf', 'ring'):
        tolerance_key = kind+'_pixel_tolerance_fraction'
        tolerance = params.get(tolerance_key, params.get('pixel_tolerance_fraction', .02))
        if tolerance_key in params:
            if (isinstance(tolerance, bool) or not isinstance(tolerance, (int, float))
                    or not math.isfinite(tolerance) or not 0.005 <= tolerance <= 0.25):
                raise ValueError(f'{kind}.pixel_tolerance_fraction 必须在[0.005,0.25]范围内')
        hold_key = kind+'_hold_seconds'
        if hold_key in params:
            value = params[hold_key]
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or value < 0):
                raise ValueError(f'{kind}.hold_seconds 必须是有限非负数')
        zone_key = kind+'_micro_servo_zone_fraction'
        if zone_key in params:
            value = params[zone_key]
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or not tolerance < value <= 1.0):
                raise ValueError(
                    f'{kind}.micro_servo.zone_fraction 必须大于中心容差且不超过1.0')
        for gain_key in (kind+'_servo_gain', kind+'_micro_servo_gain'):
            if gain_key in params:
                value = params[gain_key]
                if (isinstance(value, bool) or not isinstance(value, (int, float))
                        or not math.isfinite(value) or value <= 0):
                    raise ValueError(f'{gain_key} 必须是有限正数')
        for suffix, default in (('ki', 0.0), ('kd', 0.0),
                                ('integral_limit', 0.2)):
            key = kind+'_servo_pid_'+suffix
            value = params.get(key, default)
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or value < 0):
                raise ValueError(f'{kind}.servo.pid.{suffix} 必须是有限非负数')
    if 'frame_servo_gain' in params:
        value = params['frame_servo_gain']
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or value <= 0):
            raise ValueError('frame.servo_gain 必须是有限正数')
    for key, label in (('frame_yaw_gain', 'frame.yaw.gain'),
                       ('frame_yaw_max_rate_deg_s', 'frame.yaw.max_rate_deg_s')):
        if key in params:
            value = params[key]
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or value <= 0):
                raise ValueError(f'{label} 必须是有限正数')
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
                         ('horizontal_servo_timeout', 30.0), ('loss_timeout', 5.0),
                         ('target_loss_timeout', 5.0), ('frame_loss_timeout', 5.0),
                         ('ring_yaw_tolerance_deg', 3.0), ('ring_yaw_gain', 1.2),
                         ('claw_alignment_timeout', 30.0),
                         ('claw_alignment_position_tolerance_m', 0.15),
                         ('claw_alignment_yaw_tolerance_deg', 10.0),
                         ('ring_max_yaw_rate_deg_s', 10.0),
                         ('ring_orientation_spread_deg', 5.0), ('verification_timeout', 2.0),
                         ('return_timeout', 60.0), ('descent_timeout', 15.0),
                         ('descent_speed_mps', 0.03), ('vertical_publish_period', 0.05),
                         ('descent_xy_hold_gain', 0.8),
                         ('descent_xy_hold_max_speed_mps', 0.05),
                         ('descent_yaw_hold_gain', 1.2),
                         ('descent_yaw_hold_max_rate_deg_s', 10.0)):
        value = params.get(key, default)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'{key} 必须是有限正数')
    if params.get('claw_alignment_yaw_tolerance_deg', 10.0) > 180.0:
        raise ValueError('claw_alignment.yaw_tolerance_deg 不得超过180度')
    sticky_seconds = params.get('lost_observation_hold_seconds', 2.0)
    if (isinstance(sticky_seconds, bool) or not isinstance(sticky_seconds, (int, float))
            or not math.isfinite(sticky_seconds) or sticky_seconds < 0):
        raise ValueError('lost_observation_hold_seconds 必须是有限非负数')
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
            raise ValueError('ring.orientation_timeout 必须是有限正数，表示伺服完成后的方向采集秒数')
    quality = params.get('ring_orientation_min_quality', .8)
    if isinstance(quality, bool) or not isinstance(quality, (int, float)) or not math.isfinite(quality) or not 0 <= quality <= 1:
        raise ValueError('ring_orientation_min_quality 必须在[0,1]范围内')
    if params.get('ring_class', 'red_ring') != 'red_ring':
        raise ValueError('当前方向检测只支持 red_ring')


def wrap(angle):
    return (float(angle)+180.0) % 360.0-180.0


def choose_away_heading(axis_deg, xy, start_xy, current_yaw):
    """Choose the sign of an undirected world heading, facing away from START."""
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
        self.pickup_order = tuple(params.get('pickup_order', ('golf', 'ring')))
        self.work_depth = float(params.get('work_depth_m', 0.2))
        self.grab_depths = {kind: float(params.get(kind+'_grab_depth_m', 0.44))
                           for kind in ('golf', 'ring')}
        self.retry_depth_step = float(params.get('retry_depth_step_m', 0.05))
        self.descent_timeout = float(params.get('descent_timeout', 15.0))
        self.descent_speed = float(params.get('descent_speed_mps', 0.03))
        self.descent_period = float(params.get('vertical_publish_period', 0.05))
        self.descent_xy_gain = float(params.get('descent_xy_hold_gain', 0.8))
        self.descent_xy_max_speed = float(params.get('descent_xy_hold_max_speed_mps', 0.05))
        self.descent_yaw_gain = float(params.get('descent_yaw_hold_gain', 1.2))
        self.descent_yaw_max_rate = float(params.get('descent_yaw_hold_max_rate_deg_s', 10.0))
        self._descent_anchor = None
        self.claw_alignment_timeout = float(params.get('claw_alignment_timeout', 30.0))
        self.claw_alignment_position_tolerance = float(params.get('claw_alignment_position_tolerance_m', 0.15))
        self.claw_alignment_yaw_tolerance = float(params.get('claw_alignment_yaw_tolerance_deg', 10.0))
        if not hasattr(node, 'state'):
            node.state = TaskState()
        self.state = node.state
        self.log = node.get_logger()
        def target_servo_params(kind):
            target_params = dict(params)
            if kind+'_pixel_tolerance_fraction' in params:
                target_params['pixel_tolerance_fraction'] = params[
                    kind+'_pixel_tolerance_fraction']
            if kind+'_hold_seconds' in params:
                target_params['horizontal_hold_seconds'] = params[kind+'_hold_seconds']
            if kind+'_servo_gain' in params:
                target_params['horizontal_servo_gain'] = params[kind+'_servo_gain']
            for suffix in ('ki', 'kd', 'integral_limit'):
                key = kind+'_servo_pid_'+suffix
                if key in params:
                    target_params['target_servo_pid_'+suffix] = params[key]
            return target_params

        self.ball = PickupController(node, target_servo_params('golf'))
        self.ring = PickupController(
            node, target_servo_params('ring'), target_name=params.get('ring_class', 'red_ring'),
            target_class_id=node._model_mapping.model_class_id('red_ring', required=False),
            gripper_frame='hairpin_claw_link', task_name='26rb_grab_ball_ring.ring')
        self.max_attempts = {kind: int(params.get(kind+'_max_attempts', 3)) for kind in ('golf', 'ring')}
        self.samples = deque()
        self._last_ring_axis = None
        self.ring_orientation_seconds = float(params.get('ring_orientation_timeout', 2.0))
        self.find_timeout = float(params.get('find_timeout', 15.0))
        self.check_timeout = float(params.get('check_timeout', 5.0))
        self.servo_timeout = float(params.get('horizontal_servo_timeout', 30.0))
        self.final_target_tolerance = float(params.get('target_final_tolerance_fraction', 0.10))
        legacy_loss_timeout = float(params.get('loss_timeout', 5.0))
        self.target_loss_timeout = float(
            params.get('target_loss_timeout', legacy_loss_timeout))
        self.frame_loss_timeout = float(
            params.get('frame_loss_timeout', legacy_loss_timeout))
        self._targets = {}
        self._target_return_poses = {}
        self._blind_targets = {}
        self._blind_grabs_used = set()
        self._first_find_started = None
        self._frame_light_indicated = False
        self.yaw_tolerance = float(params.get('ring_yaw_tolerance_deg', 3.0))
        shared_gain = float(params.get('horizontal_servo_gain', 0.8))
        self.frame_servo_gain = float(params.get('frame_servo_gain', shared_gain))
        self.frame_yaw_gain = float(params.get('frame_yaw_gain', 0.2))
        self.frame_yaw_max_rate = float(params.get('frame_yaw_max_rate_deg_s', 3.0))
        self.target_servo_gains = {
            kind: float(params.get(kind+'_servo_gain', shared_gain))
            for kind in ('golf', 'ring')}
        self.micro_servo_gains = {
            kind: float(params.get(kind+'_micro_servo_gain',
                                   self.target_servo_gains[kind]*0.35))
            for kind in ('golf', 'ring')}
        self.micro_servo_zones = {
            kind: float(params.get(kind+'_micro_servo_zone_fraction', 0.30))
            for kind in ('golf', 'ring')}

    def _target_servo_gain(self, kind, du, dv):
        """Use the low gain inside the configured circular image-center zone."""
        if math.hypot(float(du), float(dv)) <= self.micro_servo_zones[kind]:
            return self.micro_servo_gains[kind]
        return self.target_servo_gains[kind]

    @staticmethod
    def _heading_to_xy(pose, target_xy):
        """Yaw toward a world-horizontal target projected from its bbox centre."""
        delta = np.asarray(target_xy, dtype=float)-np.asarray(pose[:2], dtype=float)
        if np.linalg.norm(delta) <= 1e-6:
            return float(pose[5])
        return math.degrees(math.atan2(float(delta[1]), float(delta[0])))

    def _check(self):
        if self.node.stopped:
            raise PickupFailure('任务已取消')

    def _pose(self, *, max_age=None):
        # This task cannot use a stale commanded pose as a geometry measurement.
        freshness = {} if max_age is None else {'max_age': max_age}
        pose = tuple(float(x) for x in self.node._latest_robot_pose(
            require_measured=True, **freshness))
        if len(pose) != 6 or not all(math.isfinite(x) for x in pose):
            raise PickupFailure('实测位姿无效')
        return pose

    def _wait(self, seconds):
        until = time.monotonic()+seconds
        while time.monotonic() < until:
            self._check()
            time.sleep(min(.05, max(0.0, until-time.monotonic())))

    def _set_pose(self, target, label, *, axes='xyzrz', timeout=None,
                  position_tolerance=0.0, yaw_tolerance=0.0):
        self._check()
        timeout = float(self.p.get('return_timeout', 60.0) if timeout is None else timeout)
        arrival = {}
        if position_tolerance > 0.0:
            arrival['position_tolerance'] = position_tolerance
        if yaw_tolerance > 0.0:
            arrival['yaw_tolerance'] = yaw_tolerance
        ok, message = self.node._send_action_goal(
            BasicMotion.Goal.SET, list(target), axes, timeout=timeout,
            wait_deadline=time.monotonic()+timeout, cancel_wait_timeout=2.0,
            light_color=self.node.LIGHT_OFF, task_context=label, **arrival)
        if not ok:
            raise PickupFailure(f'{label}失败：{message}')
        self.node._cmd_x, self.node._cmd_y, self.node._cmd_z, self.node._cmd_yaw = target

    def _restore_work_depth(self, label, *, hold_xy_yaw=None):
        """Restore depth before horizontal recovery, retaining the pickup anchor."""
        deadline = time.monotonic()+self.ball._command_timeout
        if hold_xy_yaw is None:
            pose = self._pose()
            target, axes = [pose[0], pose[1], self.work_depth, pose[5]], 'z'
        else:
            self._pose(max_age=1.0)
            x, y, yaw = hold_xy_yaw
            target, axes = [x, y, self.work_depth, yaw], 'xyzrz'
        self._set_pose(target, label, axes=axes, timeout=self.ball._command_timeout)
        if hold_xy_yaw is not None:
            # SET's generic arrival tolerance may be wider than the pickup
            # depth tolerance. Keep its full position target active until the
            # ascent really finishes before the xyrz camera-pose return.
            while True:
                self._check()
                pose = self._pose(max_age=1.0)
                now = time.monotonic()
                if now >= deadline:
                    raise PickupFailure(f'{label}深度确认超时：实测={pose[2]:.3f}m')
                if abs(pose[2]-self.work_depth) <= self.ball._depth_hold_tolerance:
                    break
                self._wait(min(self.descent_period, max(0.0, deadline-now)))

    def _return_frame(self, controller):
        """Restore depth, then return to this attempt's best target observation."""
        kind = 'golf' if controller is self.ball else 'ring'
        anchor = getattr(self.state.pickup, kind+'_camera_pose')
        if anchor is None:
            raise PickupFailure('缺少本次目标下视对齐位姿')
        controller._servo_yaw_target = None
        self._restore_work_depth('组合抓取：抓取后恢复作业深度',
                                 hold_xy_yaw=self._descent_anchor)
        self._descent_anchor = None
        self._set_pose(anchor, '组合抓取：返回'+kind+'目标观测位姿', axes='xyrz')

    def _fresh_detection(self, class_id, controller, *, active_camera=None):
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
            return None, None, None
        # Keep one eye while it still has a valid observation, then switch to
        # the freshest other eye. Both eyes count as visibility for loss timing.
        selected = next(
            (item for item in candidates if item[1] == active_camera),
            max(candidates, key=lambda x: x[0]))
        return selected[1], selected[2], selected[0]

    def _fallback_pose(self, kind=None):
        # Return to where this target was best seen, rather than letting a
        # later frame correction overwrite its observation point.
        target_pose = self._target_return_poses.get(kind)
        if target_pose is not None:
            return target_pose
        recorded = self.state.pickup.last_servo_pose
        if recorded is not None:
            return recorded
        position = self.p['collection_frame_position']
        return (float(position[0]), float(position[1]), self.work_depth, self._pose()[5])

    def _record_target(self, kind, controller, camera, detection, *, timed_out,
                       world_target=None, observed_pose=None):
        pose = self._pose() if observed_pose is None else observed_pose
        anchor = (*pose[:2], self.work_depth, pose[5])
        target = (target_world_xy(self.node, camera, detection, pose,
                                  controller._projection_depth)
                  if world_target is None else world_target)
        self._targets[kind] = (target, pose, camera)
        controller._aligned_camera = camera
        return_pose = self._target_return_poses.get(kind, anchor)
        self.state.update_pickup(**{kind+'_camera_pose': return_pose, 'last_servo_pose': anchor})
        if timed_out:
            self.log.warning(
                f'组合抓取：{kind} 进入过微调区后伺服超时，使用历史最靠近图像中心的有效世界位置'
                f'({target[0]:+.3f},{target[1]:+.3f})m盲抓一次')
        else:
            self.log.info(f'组合抓取：{kind} 目标对齐完成')
        if not timed_out:
            controller._flash_green(1, f'{kind}伺服成功')
        return 'ready'

    def _align_target(self, kind, controller, started, *, checking=False, exhausted=False):
        """One deadline survives frame/target/closed-loop-return transitions."""
        window = PickupWindow(
            started, self.check_timeout if checking else self.find_timeout,
            self.servo_timeout, self.target_loss_timeout, checking=checking,
            frame_loss_seconds=self.frame_loss_timeout)
        # This hard deadline ends visual corrections. A target that entered
        # its micro-servo zone may then use its one blind mechanical attempt.
        servo_deadline = started + self.servo_timeout
        controller._servo_depth = self.work_depth
        controller._servo_yaw_target = None
        controller._aligned_camera = None
        controller._servo_gain = self.target_servo_gains[kind]
        self._blind_targets.pop(kind, None)
        self._target_return_poses.pop(kind, None)
        base_tolerance = controller._pixel_tolerance
        tolerance_change_at = None
        tolerance_change_label = ('线性放宽' if self.final_target_tolerance >= base_tolerance
                                  else '线性收紧')
        self.log.info(
            f'组合抓取：{kind} target PID 参数：'
            f'Kp={self.target_servo_gains[kind]:.3f}，'
            f'微伺服 Kp={self.micro_servo_gains[kind]:.3f} '
            f'(区域={self.micro_servo_zones[kind]:.3f})，'
            f'Ki={controller._target_servo_ki:.3f}，'
            f'Kd={controller._target_servo_kd:.3f}，'
            f'I限幅={controller._target_servo_integral_limit:.3f}，'
            f'初始中心容差=±{base_tolerance:.4f}，'
            f'最终中心容差=±{self.final_target_tolerance:.4f}，'
            f'深度容差=±{controller._depth_hold_tolerance:.3f}m，'
            f'稳定保持={controller._hold_seconds:.2f}s，'
            f'计时 已用={max(0., time.monotonic()-started):.2f}s/'
            f'剩余={max(0., servo_deadline-time.monotonic()):.2f}s，I=(0.000,0.000)')
        self.log.info(
            '[PICKUP] 列说明：ex_n/ey_n=归一化X/Y残差，tol_n=当前容差；'
            'src=live新鲜观测/last历史观测/best历史最居中观测/none无数据；'
            'tol_phase=coarse常规/hold保持/ramp线性变化；'
            'elapsed_s/remain_s=已用/剩余秒，fallback=off未进入微调/armed可盲抓/used已用盲抓')

        def log_status(mode, eye, source, error, tolerance, phase, horizontal,
                       *, hold_since=None, received=None, feature='target'):
            """Keep ASCII field names, units and numeric widths fixed in every row."""
            status_now = time.monotonic()
            pose = self._pose()
            ex, ey = (None, None) if error is None else error
            def number(value, width=9, precision=4, signed=False):
                if value is None or not math.isfinite(float(value)):
                    return f'{"--":>{width}}'
                return format(float(value), f'{"+" if signed else ""}{width}.{precision}f')
            integral = controller._target_pid_integral
            fallback = ('used' if kind in self._blind_grabs_used else
                        'armed' if tolerance_change_at is not None else 'off')
            hold = 0.0 if hold_since is None else max(0.0, status_now-hold_since)
            change_in = (None if tolerance_change_at is None else
                         max(0.0, tolerance_change_at-status_now))
            age = None if received is None else max(0.0, status_now-received)
            self.log.info(
                f'[PICKUP] obj={kind:4s} mode={mode:9s} eye={eye or "-":10s} '
                f'feature={feature:6s} src={source:5s} tol_phase={phase:6s} '
                f'ex_n={number(ex, signed=True)} ey_n={number(ey, signed=True)} '
                f'tol_n={number(tolerance, 8)} '
                f'elapsed_s={number(max(0.0, status_now-started), 8, 2)} '
                f'remain_s={number(max(0.0, servo_deadline-status_now), 8, 2)} '
                f'hold_s={number(hold, 6, 2)} hold_need_s={number(controller._hold_seconds, 6, 2)} '
                f'adjust_in_s={number(change_in, 8, 2)} obs_age_s={number(age, 8, 2)} '
                f'fallback={fallback:5s} '
                f'dz_m={number(pose[2]-self.work_depth, 8, 3, True)} '
                f'dz_tol_m={number(controller._depth_hold_tolerance, 7, 3)} '
                f'Kp={number(controller._servo_gain, 7, 3)} '
                f'Ki={number(controller._target_servo_ki, 7, 3)} '
                f'Kd={number(controller._target_servo_kd, 7, 3)} '
                f'I_x={number(integral[0], signed=True)} I_y={number(integral[1], signed=True)} '
                f'I_max={number(controller._target_servo_integral_limit, 8, 4)} '
                f'vx_mps={number(horizontal[0], 8, 3, True)} '
                f'vy_mps={number(horizontal[1], 8, 3, True)}')

        stable_since = None
        stable_camera = None
        frame_stable_since = None
        announced = None
        last_log = float('-inf')
        with self.node._perception_lock:
            earlier = [event[0] for event in self.node._down_detection_events if event[1] >= started]
            cursor = min(earlier)-1 if earlier and not checking else self.node._down_detection_sequence
        last_target_camera = None
        last_target_error = None
        last_target_observation = None
        best_target_observation = None
        best_center_error = math.inf
        last_target_world = None
        last_target_world_camera = None
        last_target_received = float('-inf')
        last_frame_world = None
        last_frame_error = None
        last_frame_camera = None
        last_frame_received = float('-inf')
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
                    if kind == 'ring' and received <= now:
                        self._sample_axis(camera, received, message, collect=False)
                camera, detection, target_received = self._fresh_detection(
                    controller._golf_class_id, controller,
                    active_camera=(controller._aligned_camera
                                   if window.mode == 'target' else None))
                if detection is not None:
                    # A fresh cached detection may precede this stage's event
                    # cursor. Remember it as well, using the measured attitude.
                    if kind == 'ring':
                        self._sample_axis(
                            camera, target_received,
                            SimpleNamespace(detections=[detection]), collect=False)
                    last_target_camera = camera
                    last_target_error = normalized_image_error(
                        self.node, camera, detection)
                    if (camera != last_target_world_camera
                            or target_received > last_target_received):
                        pose = self._pose()
                        try:
                            world_target = target_world_xy(
                                self.node, camera, detection, pose,
                                controller._projection_depth)
                        except (AttributeError, KeyError, ValueError,
                                np.linalg.LinAlgError):
                            world_target = None
                        if world_target is not None:
                            last_target_world = world_target
                            last_target_world_camera = camera
                            last_target_received = target_received
                            last_target_observation = SimpleNamespace(
                                camera=camera, detection=detection, pose=pose,
                                world=world_target, received=target_received,
                                error=last_target_error)
                            # Keep the most centred valid observation for this
                            # servo attempt. Equal errors prefer newer data;
                            # a later drift/loss must not replace a better point.
                            center_error = math.hypot(*last_target_error)
                            if (best_target_observation is None
                                    or center_error < best_center_error-1e-9
                                    or (abs(center_error-best_center_error) <= 1e-9
                                        and target_received >= best_target_observation.received)):
                                best_target_observation = last_target_observation
                                best_center_error = center_error
                                self._target_return_poses[kind] = (
                                    pose[0], pose[1], self.work_depth, pose[5])
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
                frame_camera, frame, frame_received = self._fresh_detection(
                    controller._collection_class_id, controller,
                    active_camera=(controller._aligned_camera
                                   if window.mode == 'frame' else None))
                if frame is not None and (frame_camera != last_frame_camera
                        or frame_received > last_frame_received):
                    last_frame_error = normalized_image_error(self.node, frame_camera, frame)
                    pose = self._pose()
                    try:
                        world_frame = target_world_xy(
                            self.node, frame_camera, frame, pose,
                            controller._projection_depth)
                    except (AttributeError, KeyError, ValueError,
                            np.linalg.LinAlgError):
                        world_frame = None
                    if world_frame is not None:
                        last_frame_world = world_frame
                        last_frame_camera = frame_camera
                        last_frame_received = frame_received
                if window.transition(now, detection is not None, frame is not None):
                    stable_since = frame_stable_since = None
                if announced != window.mode:
                    announced = window.mode
                    self.log.info(f'组合抓取：{kind} 进入{window.mode}，servo剩余={max(0., servo_deadline-now):.1f}s')
                    if window.mode == 'return' and kind in self._target_return_poses:
                        anchor = self._target_return_poses[kind]
                        self.log.info(
                            f'组合抓取：{kind} 返回本轮target最居中观测点，'
                            f'XY=({anchor[0]:+.3f},{anchor[1]:+.3f})m，'
                            f'yaw={anchor[3]:+.2f}°，中心误差={best_center_error:.4f}')
                if frame is not None and not self._frame_light_indicated:
                    self._frame_light_indicated = True
                    controller._flash_green(1, '首次看到frame')
                phase_color = (
                    self.node.LIGHT_YELLOW if window.mode == 'target'
                    or (window.mode == 'frame' and frame is None)
                    else self.node.LIGHT_OFF)
                self.node._set_task_phase_light(
                    phase_color, f'组合抓取：{window.mode}阶段')
                # Entry fixes a wall-clock schedule; switching eyes or losing
                # the target never resets either the split or blind eligibility.
                target_tolerance = base_tolerance
                tolerance_phase = 'coarse'
                if tolerance_change_at is not None:
                    tolerance_phase = 'hold'
                    if now > tolerance_change_at:
                        span = servo_deadline-tolerance_change_at
                        progress = (1.0 if span <= 0.0 else
                                    min(1.0, (now-tolerance_change_at)/span))
                        target_tolerance += (self.final_target_tolerance-base_tolerance)*progress
                        tolerance_phase = 'ramp'
                horizontal = np.zeros(2)
                if now >= servo_deadline:
                    can_blind = (tolerance_change_at is not None
                                 and best_target_observation is not None
                                 and kind not in self._blind_grabs_used and not exhausted)
                    if can_blind:
                        observation = best_target_observation
                        # Freeze the best observed world point and retain the
                        # existing last-valid world-axis rule for ring heading.
                        # Reprojecting an old pixel from the timeout pose would
                        # move the blind target when the vehicle has drifted.
                        self._blind_targets[kind] = (
                            self._last_ring_axis if kind == 'ring' else None)
                        self._blind_grabs_used.add(kind)
                        log_status('blind', observation.camera, 'best', observation.error,
                                   target_tolerance, tolerance_phase, horizontal,
                                   received=observation.received)
                        result = self._record_target(
                            kind, controller, observation.camera, observation.detection,
                            timed_out=True, world_target=observation.world,
                            observed_pose=observation.pose)
                    else:
                        pose = self._pose()
                        self.state.update_pickup(
                            last_servo_pose=(*pose[:2], self.work_depth, pose[5]))
                        log_status('timeout', last_target_camera,
                                   'last' if last_target_error is not None else 'none',
                                   last_target_error, target_tolerance, tolerance_phase, horizontal,
                                   received=(None if last_target_observation is None else
                                             last_target_observation.received))
                        reason = ('本目标的盲抓次数已用完' if kind in self._blind_grabs_used else
                                  '没有有效目标世界位置' if last_target_observation is None else
                                  '本次未进入微调区')
                        self.log.warning(f'组合抓取：{kind} 视觉伺服超时，{reason}，跳过本次target')
                        result = 'timeout' if window.seen else 'unobserved'
                    break
                target_servo_error = None
                target_depth_error = None
                controller._servo_yaw_target = None
                if window.mode != 'target' or detection is None:
                    controller._reset_target_servo_pid()
                if window.mode == 'target' and detection is not None:
                    controller._aligned_camera = camera
                    du, dv = normalized_image_error(self.node, camera, detection)
                    if (tolerance_change_at is None
                            and math.hypot(du, dv) <= self.micro_servo_zones[kind]):
                        remaining = servo_deadline-now
                        tolerance_change_at = now+remaining/2.0
                        tolerance_phase = 'hold'
                        self.log.info(
                            f'组合抓取：{kind} 首次进入细调区，相机={camera}，'
                            f'servo剩余={remaining:.2f}s，'
                            f'前{remaining/2.0:.2f}s保持容差±{base_tolerance:.4f}，'
                            f'后{remaining/2.0:.2f}s{tolerance_change_label}到±{self.final_target_tolerance:.4f}')
                    controller._servo_gain = self._target_servo_gain(kind, du, dv)
                    pose, horizontal, du, dv = controller._horizontal_velocity(
                        camera, detection, target_pid=True)
                    target_servo_error = (du, dv)
                    centred = abs(du) <= target_tolerance and abs(dv) <= target_tolerance
                    target_depth_error = pose[2]-self.work_depth
                    at_depth = abs(target_depth_error) <= controller._depth_hold_tolerance
                    if stable_camera != camera:
                        stable_since = None
                        stable_camera = camera
                    stable_since = (now if stable_since is None else stable_since) if centred and at_depth else None
                    if centred:
                        controller._reset_target_servo_pid()
                        horizontal[:] = 0.
                    if stable_since is not None and now-stable_since >= controller._hold_seconds:
                        log_status('ready', camera, 'live', (du, dv), target_tolerance,
                                   tolerance_phase, horizontal, hold_since=stable_since,
                                   received=target_received)
                        result = self._record_target(kind, controller, camera, detection, timed_out=False)
                        break
                elif window.mode == 'frame' and frame is not None:
                    controller._aligned_camera = frame_camera
                    controller._servo_gain = self.frame_servo_gain
                    pose, horizontal, du, dv = controller._horizontal_velocity(frame_camera, frame)
                    if last_frame_world is not None:
                        controller._servo_yaw_target = self._heading_to_xy(
                            pose, last_frame_world)
                        controller._servo_yaw_gain = self.frame_yaw_gain
                        controller._servo_max_yaw_rate = self.frame_yaw_max_rate
                    centred = abs(du) <= controller._pixel_tolerance and abs(dv) <= controller._pixel_tolerance
                    if centred:
                        horizontal[:] = 0.
                        frame_stable_since = now if frame_stable_since is None else frame_stable_since
                        if now-frame_stable_since >= controller._hold_seconds:
                            self.state.update_pickup(last_servo_pose=(*pose[:2], self.work_depth, pose[5]))
                    else:
                        frame_stable_since = None
                elif window.mode == 'target' and last_target_world is not None:
                    age = now-last_target_received
                    if 0.0 <= age <= controller._lost_observation_hold_seconds:
                        pose = self._pose()
                        delta = np.asarray(last_target_world)-np.asarray(pose[:2])
                        tolerance = max(0.01, controller._projection_depth*target_tolerance)
                        if np.linalg.norm(delta) > tolerance:
                            gain = (self._target_servo_gain(kind, *last_target_error)
                                    if last_target_error is not None
                                    else self.target_servo_gains[kind])
                            horizontal = bounded_velocity(
                                delta, gain*0.5,
                                controller._max_xy_speed)
                        controller._aligned_camera = last_target_world_camera
                    stable_since = None
                elif window.mode == 'frame' and last_frame_world is not None:
                    age = now-last_frame_received
                    if 0.0 <= age <= controller._lost_observation_hold_seconds:
                        pose = self._pose()
                        delta = np.asarray(last_frame_world)-np.asarray(pose[:2])
                        tolerance = max(0.01, controller._projection_depth*controller._pixel_tolerance)
                        controller._servo_yaw_target = self._heading_to_xy(
                            pose, last_frame_world)
                        controller._servo_yaw_gain = self.frame_yaw_gain
                        controller._servo_max_yaw_rate = self.frame_yaw_max_rate
                        if np.linalg.norm(delta) > tolerance:
                            horizontal = bounded_velocity(
                                delta, self.frame_servo_gain*0.5,
                                controller._max_xy_speed)
                        controller._aligned_camera = last_frame_camera
                    frame_stable_since = None
                elif window.mode == 'return':
                    anchor = self._fallback_pose(kind)
                    pose = self._pose()
                    horizontal = bounded_velocity(np.asarray(anchor[:2])-pose[:2], self.frame_servo_gain, controller._max_xy_speed)
                    controller._servo_yaw_target = anchor[3]
                    controller._servo_yaw_gain = self.frame_yaw_gain
                    controller._servo_max_yaw_rate = self.frame_yaw_max_rate
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
                    if target_servo_error is not None:
                        log_status(window.mode, camera, 'live', target_servo_error,
                                   target_tolerance, tolerance_phase, horizontal,
                                   hold_since=stable_since, received=target_received)
                    elif window.mode == 'frame' and frame is not None:
                        log_status('frame', frame_camera, 'live', (du, dv), base_tolerance,
                                   'coarse', horizontal, hold_since=frame_stable_since,
                                   received=frame_received, feature='frame')
                    elif window.mode == 'frame' and last_frame_error is not None:
                        log_status('frame', last_frame_camera, 'last', last_frame_error,
                                   base_tolerance, 'coarse', horizontal,
                                   received=last_frame_received, feature='frame')
                    else:
                        log_status(window.mode, last_target_camera,
                                   'last' if last_target_error is not None else 'none',
                                   last_target_error, target_tolerance, tolerance_phase, horizontal,
                                   received=(None if last_target_observation is None else
                                             last_target_observation.received))
                    last_log = now
                servo_light = (
                    self.node.LIGHT_YELLOW if window.mode == 'target'
                    or (window.mode == 'frame' and frame is None)
                    else self.node.LIGHT_OFF)
                controller._send_horizontal_velocity(
                    horizontal, servo_deadline, light_color=servo_light)
                next_deadline = servo_deadline
                if not window.seen:
                    next_deadline = min(next_deadline, window.search_deadline)
                self._wait(min(controller._servo_period, max(0., next_deadline-time.monotonic())))
        finally:
            # Cleanup holds the measured heading; it must not finish an old
            # fallback yaw move after the shared visual deadline has expired.
            controller._servo_yaw_target = None
            controller._reset_target_servo_pid()
            if controller._stop_horizontal_velocity(
                    light_color=self.node.LIGHT_OFF) is None:
                raise PickupFailure('组合抓取：视觉修正结束停车保持失败')
        return result

    def _sample_axis(self, camera, received, message, *, collect=True):
        """Convert a valid image axis and retain the last world-frame direction."""
        if (not collect and self._last_ring_axis is not None
                and received <= self._last_ring_axis[0]):
            return
        det = best_detection(message, self.ring._golf_class_id)
        if (det is None or not getattr(det, 'orientation_valid', False)
                or not math.isfinite(float(getattr(det, 'orientation_quality', 0.0)))
                or float(det.orientation_quality) < self.p.get('ring_orientation_min_quality', .8)):
            return
        angle = float(det.orientation_axis_deg)
        if not math.isfinite(angle):
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
        if not all(np.all(np.isfinite(ray)) for ray in rays) or min(ray[2] for ray in rays) < 1e-6:
            return
        axis = rays[1][:2]/rays[1][2]-rays[0][:2]/rays[0][2]
        if np.linalg.norm(axis) < 1e-8:
            return
        sample = (received, math.degrees(math.atan2(axis[1], axis[0])) % 180.0)
        if self._last_ring_axis is None or received >= self._last_ring_axis[0]:
            self._last_ring_axis = sample
        if collect:
            self.samples.append(sample)

    def _collect_ring_axis(self):
        """Observe a fixed post-servo window, without extending it for samples."""
        self.samples.clear()
        with self.node._perception_lock:
            started = time.monotonic()
            cursor = self.node._down_detection_sequence
        deadline = started+self.ring_orientation_seconds
        self.log.info(
            f'抓环：水平伺服完成，定点采集方向{self.ring_orientation_seconds:.2f}s，'
            '优先本窗口新样本，无新样本沿用最后有效方向，采集结束立即转向')
        while True:
            self._check()
            with self.node._perception_lock:
                now = time.monotonic()
                events = tuple(event for event in self.node._down_detection_events
                               if event[0] > cursor)
            for sequence, received, camera, message in events:
                cursor = sequence
                if (camera in ('down_left', 'down_right')
                        and started <= received <= min(now, deadline)):
                    self._sample_axis(camera, received, message)
            if now >= deadline:
                break
            self._wait(min(.05, max(0.0, deadline-time.monotonic())))
        axis = self._axis_estimate()
        if axis is None and self._last_ring_axis is not None:
            received, axis = self._last_ring_axis
            self.log.warning(
                f'抓环：{self.ring_orientation_seconds:.2f}s方向采集结束，新有效样本=0，'
                f'沿用最后有效世界方向={axis:.2f}°，'
                f'历史方向年龄={max(0.0, time.monotonic()-received):.2f}s，立即转向')
        elif axis is None:
            self.log.warning(
                f'抓环：{self.ring_orientation_seconds:.2f}s方向采集结束，有效样本=0，'
                '也没有历史有效方向，保持当前航向对爪，不延长采集')
        else:
            self.log.info(
                f'抓环：{self.ring_orientation_seconds:.2f}s方向采集结束，'
                f'有效样本={len(self.samples)}，外参变换后世界方向={axis:.2f}°')
        return axis

    def _axis_estimate(self):
        # Every accepted sample belongs to this collection window. A sample
        # remains usable until the window ends, even if subsequent frames fail.
        if not self.samples:
            return None
        vector = np.mean([[math.cos(math.radians(2*angle)), math.sin(math.radians(2*angle))]
                          for _, angle in self.samples], axis=0)
        if np.linalg.norm(vector) < 1e-6:
            self.log.warning('抓环：窗口内方向均值相互抵消，采用最后一个有效方向立即转向')
            return self.samples[-1][1]
        return math.degrees(math.atan2(vector[1], vector[0]))/2.0 % 180.0

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
        self.log.info(
            f'组合抓取：{kind}对爪到位容差：XY=±{self.claw_alignment_position_tolerance:.3f}m，'
            f'yaw=±{self.claw_alignment_yaw_tolerance:.1f}deg，'
            f'超时={self.claw_alignment_timeout:.1f}s')
        self._set_pose([target_xy[0]-offset[0], target_xy[1]-offset[1], self.work_depth, yaw],
                       '组合抓取：'+kind+'转向与夹爪对准同时执行',
                       timeout=self.claw_alignment_timeout,
                       position_tolerance=self.claw_alignment_position_tolerance,
                       yaw_tolerance=self.claw_alignment_yaw_tolerance)

    def _orient_and_align_ring(self):
        pose = self._pose()
        yaw = pose[5]
        if 'ring' in self._blind_targets:
            history = self._blind_targets['ring']
            if history is None:
                self.log.warning('抓环：超时盲抓没有历史有效方向，保持当前航向对爪')
            else:
                received, axis = history
                # Apply the same world-axis heading convention as successful
                # alignment, using the history frozen at the servo timeout.
                yaw = choose_away_heading(axis, pose[:2], self.state.pickup.start_xy, pose[5])
                self.log.warning(
                    f'抓环：超时盲抓沿用最后有效世界轴方向，世界轴={axis:8.2f}deg，'
                    f'目标yaw={yaw:+8.2f}deg，'
                    f'观测年龄={max(0.0, time.monotonic()-received):8.2f}s')
        else:
            # Successful visual alignment retains its fixed two-second window.
            axis = self._collect_ring_axis()
            pose = self._pose()
            yaw = pose[5]
            if axis is not None:
                yaw = choose_away_heading(axis, pose[:2], self.state.pickup.start_xy, pose[5])
        self.log.info(
            f'抓环：开始转向对爪，当前yaw={pose[5]:+8.2f}deg，'
            f'目标yaw={yaw:+8.2f}deg，转角={wrap(yaw-pose[5]):+8.2f}deg')
        self._align_claw('ring', self.ring, yaw)
        return True

    def _grab_target_depth(self, kind):
        attempt = max(1, getattr(self.state.pickup, kind+'_attempts'))
        target = self.grab_depths[kind]+(attempt-1)*self.retry_depth_step
        if not math.isfinite(target):
            raise PickupFailure('重试抓取目标深度无效')
        return target

    def _stop_descent_velocity(self, kind):
        """Confirm zero output with a budget independent of the descent timer."""
        previous_confirmed = getattr(self.node, '_last_motion_cleanup_confirmed', True)
        try:
            ok, message = self.node._send_body_velocity(
                lease_s=0.25, wait_deadline=time.monotonic()+2.0,
                cancel_wait_timeout=0.0, light_color=self.node.LIGHT_OFF,
                task_context=f'组合抓取：{kind} 结束下压，确认零速度')
        except Exception as error:
            self.node.stopped = True
            raise PickupFailure(f'{kind} 下压停车失败，禁止继续运动：{error}') from error
        # A successful zero goal must not hide an older unconfirmed goal:
        # its late acceptance could otherwise overwrite the subsequent SET.
        if (not previous_confirmed or not ok
                or not getattr(self.node, '_last_motion_cleanup_confirmed', True)):
            self.node.stopped = True
            raise PickupFailure(f'{kind} 下压动作或停车结束未确认，禁止继续运动：{message}')

    def _descend_to_depth(self, kind, controller):
        """Return True at depth, False on normal timeout; control faults raise."""
        target = self._grab_target_depth(kind)
        attempt = max(1, getattr(self.state.pickup, kind+'_attempts'))
        deadline = time.monotonic()+self.descent_timeout
        reached = False
        try:
            self._check()
            # Freeze the completed claw-alignment command, never the drifting
            # measurement. Both recovery and ring closure reuse this anchor.
            anchor = (float(self.node._cmd_x), float(self.node._cmd_y),
                      float(self.node._cmd_yaw))
            if not all(math.isfinite(value) for value in anchor):
                raise PickupFailure('下压固定XY/yaw无效')
            self._descent_anchor = anchor
            self.log.info(
                f'组合抓取：{kind} 第{attempt}次定速下压，基础深度={self.grab_depths[kind]:.3f}m，'
                f'重试增量={(attempt-1)*self.retry_depth_step:.3f}m，目标深度={target:.3f}m，'
                f'世界Z速度={self.descent_speed:.3f}m/s，超时={self.descent_timeout:.1f}s，'
                f'固定XY=({anchor[0]:.3f},{anchor[1]:.3f})m，yaw={anchor[2]:.1f}°')
            last_log = float('-inf')
            while True:
                self._check()
                pose = self._pose(max_age=1.0)
                now = time.monotonic()
                error = target-pose[2]
                if now >= deadline:
                    self.log.warning(
                        f'组合抓取：{kind} 下压超时，实测深度={pose[2]:.3f}m，'
                        f'目标={target:.3f}m，残差={error:+.3f}m，结束本次下压后继续抓后流程')
                    break
                if error <= 0.0:
                    self.log.info(
                        f'组合抓取：{kind} 已到抓取深度，实测={pose[2]:.3f}m，目标={target:.3f}m')
                    reached = True
                    break
                horizontal = bounded_velocity(
                    np.asarray(anchor[:2])-np.asarray(pose[:2]),
                    self.descent_xy_gain, self.descent_xy_max_speed)
                yaw_error = wrap(anchor[2]-pose[5])
                yaw_rate = float(np.clip(yaw_error*self.descent_yaw_gain,
                                        -self.descent_yaw_max_rate, self.descent_yaw_max_rate))
                body = body_to_world_rotation(pose).T @ np.array(
                    [*horizontal, self.descent_speed])
                if now-last_log >= controller._log_period:
                    self.log.info(
                        f'[PICKUP DESCENT] obj={kind:<4} z_m={pose[2]:7.3f} '
                        f'target_z_m={target:7.3f} dz_m={error:+8.3f} '
                        f'dx_m={anchor[0]-pose[0]:+8.3f} dy_m={anchor[1]-pose[1]:+8.3f} '
                        f'dyaw_deg={yaw_error:+8.2f} remain_s={deadline-now:7.2f}')
                    last_log = now
                sent_at = time.monotonic()
                if sent_at >= deadline:
                    break
                ok, message = self.node._send_body_velocity(
                    *body.tolist(), yaw_rate_deg_s=yaw_rate,
                    lease_s=min(0.25, deadline-sent_at), wait_deadline=sent_at+2.0,
                    cancel_wait_timeout=2.0, light_color=self.node.LIGHT_OFF,
                    task_context=f'组合抓取：{kind} 保持XY/yaw定速下压')
                confirmed = getattr(self.node, '_last_motion_cleanup_confirmed', True)
                if not ok or not confirmed:
                    self._check()
                    raise PickupFailure(f'{kind} 下压速度指令失败：{message}')
                # Action acknowledgement time counts toward the control period
                # and the 15s budget; do not add a full sleep after each reply.
                self._wait(max(0.0, min(deadline, now+self.descent_period)-time.monotonic()))
        finally:
            self._stop_descent_velocity(kind)
        self._check()
        return reached

    def _mechanical_attempt(self, kind, controller):
        self._descent_anchor = None
        if kind == 'golf':
            if not controller._prepare_claw():
                raise PickupFailure('准备圆盘爪失败')
            self._align_claw(kind, controller, self._pose()[5])
        else:
            self._orient_and_align_ring()
            self._ring_servo(self.p.get('ring_open_angle_deg', 0.0), '松开')
        if not controller._wait_pre_descent_settle():
            raise PickupFailure('下潜前等待被取消')
        if not self._descend_to_depth(kind, controller):
            self.log.warning(
                f'组合抓取：{kind} 正常下压超时仍算一次抓取尝试，返回后由CHECK确认结果')
        if kind == 'ring':
            pose = self._pose(max_age=1.0)
            x, y, yaw = self._descent_anchor
            self._set_pose([x, y, pose[2], yaw], '组合抓取：抓环合爪前保持停止深度和固定XY/yaw',
                           timeout=controller._command_timeout)
            self._ring_servo(self.p.get('ring_close_angle_deg', 90.0), '合爪')
        # _return_frame owns the one ascent SET and the subsequent camera-pose
        # return. A normal timeout remains eligible for the existing CHECK.
        return True

    def _run_target(self, kind, controller, *, started=None):
        if controller._golf_class_id is None:
            self.state.update_pickup(**{kind+'_status': 'failed'})
            return
        if started is None:
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
        self._target_return_poses.clear()
        self._blind_targets.clear()
        self._blind_grabs_used.clear()
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
            controllers = {'golf': self.ball, 'ring': self.ring}
            for index, kind in enumerate(self.pickup_order):
                # The first golf stage can use the BLINE frame timestamp.
                # Every later target gets a fresh find/check/servo window.
                started = (self._first_find_started
                           if index == 0 and kind == 'golf'
                           else time.monotonic())
                self._run_target(kind, controllers[kind], started=started)
            result = self.state.pickup
            if result.golf_status == result.ring_status == 'success':
                return TaskOutcome.ok('球和环均抓取成功')
            failed = [kind for kind in self.pickup_order if getattr(result, kind+'_status') != 'success']
            code = failed[0]+'_exhausted' if len(failed) == 1 else 'both_exhausted'
            return TaskOutcome.failed('26rb_grab_ball_ring.'+code, '未确认抓取成功：'+','.join(failed))
        except (ValueError, KeyError, RuntimeError) as error:
            code = 'cancelled' if self.node.stopped else 'recovery'
            return TaskOutcome.failed('26rb_grab_ball_ring.'+code, str(error))
