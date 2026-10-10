"""使用 collection_frame 定位并完成带上浮复检的单个高尔夫球抓取。"""

from __future__ import annotations

import math
import time

import cv2
import numpy as np

from uv_msgs.action import BasicMotion
from uv_task.down_camera_servo import (
    DownCameraPriority, best_detection, body_image_step, normalized_image_error,
    body_to_world_rotation,
)
from uv_task.task_outcome import TaskOutcome
from uv_task.collection_frame_search import CollectionFrameSearch


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


class RB26GrabGolfTask:
    """Use one down camera at a time and TF offsets to pick up a golf ball."""

    def __init__(self, node, params: dict, *, target_name=None, target_class_id=None,
                 gripper_frame='disc_claw_link', task_name='26rb_grab_golf'):
        self._node = node
        self._params = params
        self._logger = node.get_logger()
        self._task_name = task_name
        self._gripper_frame = gripper_frame
        self._servo_yaw_target = None
        self._servo_yaw_gain = 1.2
        self._servo_max_yaw_rate = 10.0
        self._servo_yaw_tolerance = 3.0

        self._aligned_camera = None
        self._pending_golf_priority = None
        self._servo_depth = None

        golf_color = params.get('golf_color', 'pink_golf') if target_name is None else target_name
        self._golf_color = str(golf_color)
        self._golf_class_id = self._parse_golf_color(golf_color) if target_name is None else target_class_id
        collection_class = params.get(
            'collection_frame_class', 'collection_frame_down')
        self._collection_class_id = node._model_mapping.model_class_id(
            collection_class, required=False)

        self._detection_timeout = max(
            0.1, float(params.get('detection_timeout', 0.8)))
        self._priority_seconds = max(
            0.1, float(params.get('camera_priority_seconds', 3.0)))
        self._observe_seconds = max(
            0.0, float(params.get('collection_frame_observe_seconds', 2.0)))
        self._golf_observe_seconds = max(
            0.0, float(params.get('golf_observe_seconds', 2.0)))
        self._servo_timeout = max(
            1.0, float(params.get('horizontal_servo_timeout', 20.0)))
        self._servo_period = max(
            0.05, float(params.get('horizontal_servo_period', 0.15)))
        self._log_period = max(
            0.1, float(params.get('horizontal_servo_log_period', 0.5)))
        self._pixel_tolerance = _clamp(
            float(params.get('pixel_tolerance_fraction', 0.02)),
            0.005, 0.25)
        self._hold_seconds = max(
            0.0, float(params.get('horizontal_hold_seconds', 0.5)))
        self._projection_depth = max(
            0.1, float(params.get('projection_depth_m', 0.8)))
        self._servo_gain = max(
            0.05, float(params.get('horizontal_servo_gain', 0.8)))
        self._max_xy_speed = _clamp(
            float(params.get('horizontal_max_speed_mps', 0.08)), 0.005, 0.15)
        self._depth_hold_gain = max(0.05, float(params.get('depth_hold_gain', 0.8)))
        self._depth_hold_max_speed = _clamp(
            float(params.get('depth_hold_max_speed_mps', 0.08)), 0.005, 0.15)
        self._depth_hold_tolerance = max(
            0.005, float(params.get('depth_hold_tolerance_m', 0.03)))
        self._command_timeout = max(
            0.2, float(params.get('position_command_timeout', 10.0)))

        self._claw_prepare_angle = float(
            params.get('claw_prepare_angle_rad', 0.0))
        self._claw_prepare_repeat_count = max(
            1, int(params.get('claw_prepare_repeat_count', 3)))
        self._claw_prepare_repeat_period = max(
            0.0, float(params.get('claw_prepare_repeat_period', 0.1)))
        self._light_pulse_seconds = max(
            0.05, float(params.get('light_pulse_seconds', 0.35)))
        self._light_gap_seconds = max(
            0.0, float(params.get('light_gap_seconds', 0.25)))

        self._pre_descent_settle_seconds = max(
            0.0, float(params.get('pre_descent_settle_seconds', 1.0)))
        self._descent_speed = abs(float(params.get('descent_speed_mps', 0.1)))
        self._descent_duration = max(
            0.0, float(params.get('descent_duration_seconds', 5.0)))
        self._ascent_speed = abs(float(params.get('ascent_speed_mps', 0.1)))
        self._ascent_duration = max(
            0.0, float(params.get('ascent_duration_seconds', 5.0)))
        self._vertical_period = max(
            0.02, float(params.get('vertical_publish_period', 0.05)))
        self._return_timeout = max(
            1.0, float(params.get('return_timeout', 60.0)))
        self._verification_timeout = max(
            0.2, float(params.get('verification_timeout', 2.0)))
        self._verification_absence_hold_seconds = _clamp(
            float(params.get('verification_absence_hold_seconds', 0.8)),
            0.0, self._verification_timeout)
        # 2 retries after the first attempt means at most 3 pickup attempts.
        self._max_grab_retries = max(
            0, int(params.get('max_grab_retries', 2)))

    def _parse_golf_color(self, value) -> int | None:
        if not isinstance(value, str):
            return None
        name = value.strip()
        class_id = self._node._model_mapping.model_class_id(
            name, required=False)
        return class_id if name in {
            'pink_golf', 'yellow_golf'
        } and class_id is not None else None

    def _wait_for_detection(self, class_id: int, label: str,
                            duration: float):
        """Observe fresh detections from either down camera, without stereo."""
        priority = DownCameraPriority(
            self._node, class_id, priority_seconds=self._priority_seconds,
            detection_timeout=self._detection_timeout,
            label=f'{self._task_name} {label}观测')
        started = time.monotonic()
        deadline = started + max(0.0, float(duration))
        next_log = float('-inf')
        first_detection = None
        self._logger.info(
            f'{self._task_name}：开始观察 {label}，窗口={duration:.1f}s，'
            '相机=down_left/down_right')
        while not self._node.stopped and time.monotonic() < deadline:
            now = time.monotonic()
            priority.update()
            if priority.first_observation is not None and first_detection is None:
                camera_name, first_detection = priority.first_observation
                detection = first_detection
                self._logger.info(
                    f'{self._task_name}：观察到 {label}，相机={camera_name}；'
                    f'像素=({float(detection.pixel_x):.1f},'
                    f'{float(detection.pixel_y):.1f})，'
                    f'置信度={float(detection.confidence):.3f}，'
                    '继续完成观察窗口')
            if first_detection is None and now - next_log >= self._log_period:
                self._logger.info(
                    f'{self._task_name}：观察 {label} 中，尚未收到新鲜目标检测')
                next_log = now
            time.sleep(min(self._servo_period, max(0.0, deadline - now)))
        if self._node.stopped:
            return None
        if first_detection is not None:
            self._logger.info(
                f'{self._task_name}：{label} 的 {duration:.1f}s 观察窗口结束，'
                '窗口内曾检测到目标')
            return first_detection
        self._logger.warning(
            f'{self._task_name}：{duration:.1f}s 内未观测到 {label}')
        return None

    def _flash_green(self, count: int, label: str) -> bool:
        count = max(1, int(count))
        self._logger.info(
            f'{self._task_name}：{label}，绿灯闪烁 {count} 次（异步）')
        flash_async = getattr(self._node, '_flash_task_light', None)
        if callable(flash_async):
            return flash_async(
                self._node.LIGHT_GREEN, count, label,
                pulse_seconds=self._light_pulse_seconds,
                gap_seconds=self._light_gap_seconds,
                restore=True, restore_color=None)
        return not self._node.stopped

    @staticmethod
    def _body_to_world(dx: float, dy: float, yaw_deg: float):
        yaw = math.radians(float(yaw_deg))
        c, s = math.cos(yaw), math.sin(yaw)
        return c * dx - s * dy, s * dx + c * dy

    def _horizontal_velocity(self, camera_name, detection):
        """Project pixel error into bounded world-horizontal velocity (m/s)."""
        du, dv = normalized_image_error(self._node, camera_name, detection)
        camera = self._node.camera_extrinsics[camera_name]
        body_vx, body_vy = body_image_step(
            camera, du, dv, self._projection_depth,
            self._servo_gain, self._max_xy_speed)
        pose = self._servo_pose()
        world = body_to_world_rotation(pose) @ np.array([body_vx, body_vy, 0.0])
        return pose, world[:2], du, dv

    def _servo_pose(self):
        pose = tuple(float(value) for value in self._node._latest_robot_pose())
        if len(pose) != 6 or not all(math.isfinite(value) for value in pose):
            raise ValueError('水平伺服实测位姿无效')
        return pose

    def _send_horizontal_velocity(self, horizontal, deadline, *, light_color=None):
        pose = self._servo_pose()
        vz = _clamp((self._servo_depth-pose[2])*self._depth_hold_gain,
                    -self._depth_hold_max_speed, self._depth_hold_max_speed)
        # BODY_VELOCITY is in the body frame. Transform the complete world
        # velocity so horizontal motion has no world-Z component at any tilt.
        body = body_to_world_rotation(pose).T @ np.array([*horizontal, vz])
        yaw_error = 0.0 if self._servo_yaw_target is None else (self._servo_yaw_target-pose[5]+180.0) % 360.0-180.0
        yaw_rate = _clamp(yaw_error*self._servo_yaw_gain, -self._servo_max_yaw_rate, self._servo_max_yaw_rate)
        success, message = self._node._send_body_velocity(
            *body.tolist(), yaw_rate_deg_s=yaw_rate,
            lease_s=max(0.25, 4*self._servo_period),
            wait_deadline=min(deadline, time.monotonic()+2.0),
            task_context=f'{self._task_name} 定深水平速度伺服',
            light_color=light_color)
        if not success:
            raise RuntimeError(f'水平速度指令失败：{message}')

    def _stop_horizontal_velocity(self, *, light_color=None):
        """Stop the lease and hand measured XY / locked depth to position hold."""
        success, message = self._node._send_body_velocity(
            wait_deadline=time.monotonic()+2.0,
            task_context=f'{self._task_name} 结束水平速度伺服',
            light_color=light_color)
        if not success:
            self._logger.error(f'{self._task_name}：停止水平速度失败：{message}')
            return None
        if self._node.stopped:
            return None
        pose = self._servo_pose()
        yaw = pose[5] if self._servo_yaw_target is None else self._servo_yaw_target
        recorded = [pose[0], pose[1], self._servo_depth, yaw]
        success, message = self._node._send_action_goal(
            BasicMotion.Goal.SET, recorded, 'xyzrz',
            timeout=self._command_timeout,
            wait_deadline=time.monotonic()+self._command_timeout, quiet=True,
            light_color=light_color,
            task_context=f'{self._task_name} 水平伺服后保持位置和深度')
        if not success:
            self._logger.error(f'{self._task_name}：水平伺服后定深保持失败：{message}')
            return None
        (self._node._cmd_x, self._node._cmd_y,
         self._node._cmd_z, self._node._cmd_yaw) = recorded
        return recorded

    def _servo_horizontally(self, class_id: int, label: str, *, allow_target_handoff=True):
        """Servo image error with XY velocity while holding one measured depth."""
        if class_id is None:
            self._logger.error(f'{self._task_name}：模型映射中没有 {label} 类别')
            return None
        if not self._node._ensure_camera_extrinsics():
            self._logger.error(f'{self._task_name}：{label} 伺服缺少相机 TF')
            return None
        priority = self._pending_golf_priority if class_id == self._golf_class_id else None
        if priority is not None:
            self._pending_golf_priority = None
        else:
            priority = DownCameraPriority(
                self._node, class_id, priority_seconds=self._priority_seconds,
                detection_timeout=self._detection_timeout,
                label=f'{self._task_name} {label}伺服')
        ball_priority = None
        if class_id == self._collection_class_id and allow_target_handoff:
            ball_priority = DownCameraPriority(
                self._node, self._golf_class_id, priority_seconds=self._priority_seconds,
                detection_timeout=self._detection_timeout,
                label=f'{self._task_name} frame修正途中观察{self._golf_color}')
        self._aligned_camera = None
        deadline = time.monotonic()+self._servo_timeout
        hold_generation = None
        hold_since = None
        last_status_log = float('-inf')
        completed = False
        recorded = None
        self._node._set_task_phase_light(
            self._node.LIGHT_YELLOW, f'定深速度伺服至 {label}')
        try:
            if self._servo_depth is None:
                self._servo_depth = self._servo_pose()[2]
            self._logger.info(
                f'{self._task_name}：开始 {label} 定深速度伺服；'
                f'锁定深度={self._servo_depth:.3f}m，'
                f'水平限速={self._max_xy_speed:.3f}m/s，'
                f'深度容差={self._depth_hold_tolerance:.3f}m')
            while not self._node.stopped and time.monotonic() < deadline:
                if ball_priority is not None:
                    ball_camera, ball = ball_priority.update()
                    if ball is not None:
                        self._pending_golf_priority = ball_priority
                        self._logger.info(
                            f'{self._task_name}：frame 修正途中在 {ball_camera} 看到 '
                            f'{self._golf_color}，直接切换到目标水平伺服')
                        completed = True
                        break
                camera_name, detection = priority.update()
                if hold_generation != priority.generation:
                    hold_since = None
                    hold_generation = priority.generation
                now = time.monotonic()
                horizontal = np.zeros(2)
                if detection is None:
                    hold_since = None
                    if now-last_status_log >= self._log_period:
                        self._logger.info(
                            f'{self._task_name}：{label} 未收到有效检测，停止 XY 并继续定深')
                        last_status_log = now
                else:
                    pose, horizontal, du, dv = self._horizontal_velocity(camera_name, detection)
                    centered = abs(du) <= self._pixel_tolerance and abs(dv) <= self._pixel_tolerance
                    at_depth = abs(pose[2]-self._servo_depth) <= self._depth_hold_tolerance
                    if centered:
                        horizontal = np.zeros(2)
                    at_yaw = (self._servo_yaw_target is None or
                              abs((self._servo_yaw_target-pose[5]+180.0) % 360.0-180.0) <= self._servo_yaw_tolerance)
                    if centered and at_depth and at_yaw:
                        hold_since = now if hold_since is None else hold_since
                    else:
                        hold_since = None
                    if now-last_status_log >= self._log_period:
                        self._logger.info(
                            f'{self._task_name}：{label} 定深速度伺服，相机={camera_name}，'
                            f'误差=({du:+.4f},{dv:+.4f})，'
                            f'世界水平速度=({horizontal[0]:+.3f},{horizontal[1]:+.3f})m/s，'
                            f'深度={pose[2]:.3f}/{self._servo_depth:.3f}m')
                        last_status_log = now
                    if hold_since is not None and now-hold_since >= self._hold_seconds:
                        self._aligned_camera = camera_name
                        completed = True
                        break
                self._send_horizontal_velocity(horizontal, deadline)
                time.sleep(min(self._servo_period, max(0.0, deadline-time.monotonic())))
        except (KeyError, ValueError, RuntimeError, cv2.error, np.linalg.LinAlgError) as error:
            self._logger.error(f'{self._task_name}：{label} 定深速度伺服失败：{error}')
        finally:
            try:
                recorded = self._stop_horizontal_velocity()
            except Exception as error:
                self._logger.error(f'{self._task_name}：水平伺服停车失败：{error}')
        if not completed or recorded is None:
            self._aligned_camera = None
            self._pending_golf_priority = None
            self._logger.error(f'{self._task_name}：{label} 伺服未完成、超时或被中止')
            return None
        return recorded

    def _prepare_claw(self) -> bool:
        """Repeatedly command servo 1 to the requested open/pre-grab angle."""
        self._logger.info(
            f'{self._task_name}：夹爪进入准备状态；舵机 ID=1，'
            f'角度={self._claw_prepare_angle:.3f}rad，'
            f'重复发送 {self._claw_prepare_repeat_count} 次')
        for index in range(self._claw_prepare_repeat_count):
            if self._node.stopped:
                return False
            self._node.set_servo(
                self._claw_prepare_angle,
                f'抓高尔夫球准备位置 ({index + 1}/'
                f'{self._claw_prepare_repeat_count})', servo_id=1)
            if index + 1 < self._claw_prepare_repeat_count:
                deadline = (time.monotonic()
                            + self._claw_prepare_repeat_period)
                while not self._node.stopped and time.monotonic() < deadline:
                    time.sleep(min(
                        0.02, max(0.0, deadline - time.monotonic())))
        return not self._node.stopped

    def _apply_camera_gripper_offset(self):
        """Move the gripper over the camera-centred target using TF offsets."""
        if not self._node._ensure_camera_extrinsics():
            self._logger.error('26rb_grab_golf：下视相机外参尚未就绪')
            return None
        camera = self._node.camera_extrinsics.get(self._aligned_camera)
        if camera is None:
            self._logger.error('26rb_grab_golf：缺少伺服成功相机的外参')
            return None

        provider = self._node.camera_extrinsics_provider
        try:
            transform = provider.lookup_transform(
                provider.base_frame, self._gripper_frame)
            message = (transform.transform
                       if hasattr(transform, 'transform') else transform)
            claw_xyz = (
                float(message.translation.x),
                float(message.translation.y),
                float(message.translation.z))
        except Exception as error:
            self._logger.error(
                f'{self._task_name}：读取 disc_claw_link 外参失败：{error}')
            return None

        # When the target is centred in the successful eye, it lies on its
        # optical axis. Translate by camera_origin - claw_origin so the fixed
        # target is directly below the gripper instead of the camera.
        dx = float(camera.translation[0]) - claw_xyz[0]
        dy = float(camera.translation[1]) - claw_xyz[1]
        pose = self._node._latest_robot_pose()
        yaw = float(pose[5])
        world_dx, world_dy = self._body_to_world(dx, dy, yaw)
        self._logger.info(
            '26rb_grab_golf：按 TF 外参计算相机到圆盘爪的水平偏移；'
            f'{self._aligned_camera}=({camera.translation[0]:+.3f},'
            f'{camera.translation[1]:+.3f},'
            f'{camera.translation[2]:+.3f})m，'
            f'disc_claw=({claw_xyz[0]:+.3f},{claw_xyz[1]:+.3f},'
            f'{claw_xyz[2]:+.3f})m，'
            f'机体平移=({dx:+.3f},{dy:+.3f})m，'
            f'世界平移=({world_dx:+.3f},{world_dy:+.3f})m')

        if math.hypot(dx, dy) > 1e-6:
            success, message = self._node._send_action_goal(
                BasicMotion.Goal.BMOVE, [dx, dy, 0.0, 0.0], 'xy',
                timeout=self._command_timeout,
                task_context=self._node._format_motion_context(
                    f'按相机外参对准 {self._golf_color} 高尔夫球与圆盘爪'))
            if not success:
                self._logger.error(
                    f'{self._task_name}：相机到圆盘爪外参平移失败：{message}')
                return None
        # BMOVE starts at the measured position, not the preceding command.
        self._node._cmd_x = float(pose[0]) + world_dx
        self._node._cmd_y = float(pose[1]) + world_dy
        self._node._cmd_z = float(pose[2])
        self._node._cmd_yaw = yaw

        recorded = [
            float(self._node._cmd_x), float(self._node._cmd_y),
            float(self._node._cmd_z), float(self._node._cmd_yaw)]
        self._logger.info(
            f'{self._task_name}：圆盘爪已对准高尔夫球，记录下潜前位姿='
            f'({recorded[0]:.3f},{recorded[1]:.3f},{recorded[2]:.3f},'
            f'{recorded[3]:.1f}°)')
        return recorded

    def _wait_pre_descent_settle(self):
        if self._pre_descent_settle_seconds <= 0.0:
            return not self._node.stopped
        self._logger.info(
            f'{self._task_name}：等待艇体稳定 '
            f'{self._pre_descent_settle_seconds:.1f}s')
        deadline = time.monotonic() + self._pre_descent_settle_seconds
        while not self._node.stopped and time.monotonic() < deadline:
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
        return not self._node.stopped

    def _vertical_motion(self, speed_mps: float, duration: float,
                         label: str) -> bool:
        """Run a vertical body-speed phase, always releasing its velocity lease."""
        deadline = time.monotonic() + duration
        command_ok = True
        self._logger.info(
            f'{self._task_name}：{label}，速度={speed_mps:+.3f}m/s，'
            f'持续={duration:.1f}s（NED 正值向下）')
        try:
            while not self._node.stopped and time.monotonic() < deadline:
                success, message = self._node._send_body_velocity(
                    vertical_mps=speed_mps,
                    lease_s=max(0.25, self._vertical_period * 4.0),
                    task_context=self._node._format_motion_context(
                        f'{self._task_name} {label}速度控制'))
                if not success:
                    self._logger.error(
                        f'{self._task_name}：{label}速度指令失败：{message}')
                    command_ok = False
                    break
                time.sleep(min(
                    self._vertical_period,
                    max(0.0, deadline - time.monotonic())))
        finally:
            neutral_ok, neutral_message = self._node._send_body_velocity(
                lease_s=max(0.25, self._vertical_period * 4.0),
                task_context=self._node._format_motion_context(
                    f'{self._task_name} 结束{label}速度控制'))
            if not neutral_ok:
                self._logger.error(
                    f'{self._task_name}：{label}结束时发送中性速度失败：'
                    f'{neutral_message}')
                command_ok = False
        return command_ok and not self._node.stopped

    def _descend(self):
        return self._vertical_motion(
            self._descent_speed, self._descent_duration, '下潜')

    def _ascend(self):
        return self._vertical_motion(
            -self._ascent_speed, self._ascent_duration, '上浮')

    def _verify_golf_removed(self):
        """Observe the full window at the saved camera-centred pose, using fresh images."""
        camera_name = self._aligned_camera
        if camera_name is None:
            self._logger.error('26rb_grab_golf：复检缺少抓高尔夫球伺服成功相机')
            return False
        check_started = time.monotonic()
        deadline = check_started + self._verification_timeout
        absent_since = None
        fresh_message_seen = False
        last_processed_message_at = None
        last_status_log = float('-inf')
        target_present = False
        self._logger.info(
            f'{self._task_name}：已回到相机居中位置，检查 {self._golf_color} 是否仍在盘上；'
            f'复检相机={camera_name}，'
            f'完整观察窗口={self._verification_timeout:.1f}s，'
            f'连续消失确认={self._verification_absence_hold_seconds:.1f}s')
        while not self._node.stopped:
            now = time.monotonic()
            with self._node._perception_lock:
                entry = self._node._down_detections.get(camera_name)
            if (entry is None or entry[0] < check_started
                    or now - entry[0] > self._detection_timeout
                    or entry[0] == last_processed_message_at):
                if (entry is None
                        or now - entry[0] > self._detection_timeout):
                    absent_since = None
                if now - last_status_log >= self._log_period:
                    self._logger.info(
                        '26rb_grab_golf：复检等待回位后的新鲜视觉消息')
                    last_status_log = now
                if now >= deadline:
                    break
                time.sleep(min(self._servo_period, max(0.0, deadline - now)))
                continue

            fresh_message_seen = True
            if (last_processed_message_at is not None
                    and entry[0] - last_processed_message_at
                    > self._detection_timeout):
                absent_since = None
            last_processed_message_at = entry[0]
            detection = best_detection(entry[1], self._golf_class_id)
            if detection is None:
                if absent_since is None:
                    absent_since = entry[0]
                    self._logger.info(
                        f'{self._task_name}：当前未检测到 {self._golf_color}，'
                        '开始连续消失计时')
            else:
                target_present = True
                absent_since = None
                if now - last_status_log >= self._log_period:
                    self._logger.warning(
                        f'{self._task_name}：复检仍检测到盘上的 {self._golf_color}；'
                        f'像素=({float(detection.pixel_x):.1f},'
                        f'{float(detection.pixel_y):.1f})')
                    last_status_log = now
            if now >= deadline:
                break
            time.sleep(min(self._servo_period, max(0.0, deadline - now)))

        if self._node.stopped:
            return False
        if (absent_since is not None and last_processed_message_at is not None
                and time.monotonic()-last_processed_message_at <= self._detection_timeout
                and last_processed_message_at-absent_since >= self._verification_absence_hold_seconds):
            self._logger.info(
                f'{self._task_name}：{self._verification_timeout:.1f}s 复检窗口结束，'
                f'确认 {self._golf_color} 已不在盘上')
            return True
        if not fresh_message_seen:
            self._logger.error(
                '26rb_grab_golf：复检超时，回位后没有收到新鲜视觉消息')
        elif target_present:
            self._logger.warning(
                f'{self._task_name}：复检超时，盘上仍能看到 {self._golf_color}')
        else:
            self._logger.warning('26rb_grab_golf：复检超时，无法确认高尔夫球已离盘')
        return False

    def _return_to_recorded_pose(self, recorded_pose):
        """Return to the camera-centred pose saved before translating to the gripper."""
        success, message = self._node._send_action_goal(
            BasicMotion.Goal.SET, recorded_pose, 'xyzrz',
            timeout=self._return_timeout,
            task_context=self._node._format_motion_context(
                f'抓取{self._golf_color}高尔夫球后返回相机居中复检位置'))
        if not success:
            self._logger.error(
                f'{self._task_name}：返回记录位置失败：{message}')
            return False
        (self._node._cmd_x, self._node._cmd_y,
         self._node._cmd_z, self._node._cmd_yaw) = recorded_pose
        self._logger.info('26rb_grab_golf：已返回记录的相机居中复检位置')
        return True

    def _pickup_attempt(self, attempt: int, total_attempts: int):
        self._logger.info(
            f'{self._task_name}：开始第 {attempt}/{total_attempts} 次抓取；'
            f'视觉伺服目标={self._golf_color}')
        camera_pose = self._servo_horizontally(
            self._golf_class_id, self._golf_color)
        if camera_pose is None:
            return False, '水平视觉对准高尔夫球失败'
        if not self._flash_green(1, f'{self._golf_color} 伺服成功'):
            return False, '高尔夫球居中提示灯中断'

        gripper_pose = self._apply_camera_gripper_offset()
        if gripper_pose is None:
            return False, '相机到圆盘爪外参对准失败'
        if not self._wait_pre_descent_settle():
            return False, '下潜前稳定等待被中止'

        descent_ok = self._descend()
        # Attempt the ascent and positional recovery even if a descent velocity
        # renewal failed, so the vehicle still gets a chance to return upward.
        ascent_ok = self._ascend() if not self._node.stopped else False
        return_ok = (self._return_to_recorded_pose(camera_pose)
                     if not self._node.stopped else False)
        if not descent_ok or not ascent_ok or not return_ok:
            return False, '下潜、上浮或回到记录位置失败'

        if self._verify_golf_removed():
            self._logger.info(f'{self._task_name}：第 {attempt} 次抓取确认成功')
            return True, ''
        if attempt < total_attempts:
            self._logger.warning(
                f'{self._task_name}：第 {attempt} 次后高尔夫球仍在盘上，'
                f'剩余 {total_attempts - attempt} 次机会，将重新伺服并抓取')
        return False, '复检时高尔夫球仍在盘上'

    def execute(self) -> TaskOutcome:
        self._servo_depth = None
        self._pending_golf_priority = None
        if self._collection_class_id is None:
            return TaskOutcome.failed(
                '26rb_grab_golf.collection_frame',
                '模型映射中没有 collection_frame_down 类别')
        if self._golf_class_id is None:
            return TaskOutcome.failed(
                '26rb_grab_golf.golf', f'不支持的高尔夫球颜色 {self._golf_color!r}')

        approach = CollectionFrameSearch(self._node, self._params).execute()
        if not approach:
            return approach

        self._node._set_task_phase_light(
            self._node.LIGHT_YELLOW, '抓高尔夫球任务：搜索置物盘')
        if self._wait_for_detection(
                self._collection_class_id, 'collection_frame',
                self._observe_seconds) is None:
            return TaskOutcome.failed(
                '26rb_grab_golf.collection_frame',
                f'{self._observe_seconds:.1f}s 内未看到 collection_frame')
        if not self._flash_green(1, '已观察到 collection_frame'):
            return TaskOutcome.failed(
                '26rb_grab_golf.collection_frame', '置物盘提示灯中断')

        if not self._prepare_claw():
            return TaskOutcome.failed(
                '26rb_grab_golf.claw', '夹爪准备动作被中止')

        frame_pose = self._servo_horizontally(
            self._collection_class_id, 'collection_frame')
        if frame_pose is None:
            return TaskOutcome.failed(
                '26rb_grab_golf.collection_frame',
                '水平伺服到 collection_frame 正上方失败')

        if self._pending_golf_priority is None:
            if self._wait_for_detection(
                    self._golf_class_id, self._golf_color,
                    self._golf_observe_seconds) is None:
                return TaskOutcome.failed(
                    '26rb_grab_golf.golf',
                    f'伺服到 collection_frame 后，'
                    f'{self._golf_observe_seconds:.1f}s 内没有看到 {self._golf_color}')
            if not self._flash_green(1, f'已观察到 {self._golf_color}'):
                return TaskOutcome.failed(
                    '26rb_grab_golf.golf', '目标高尔夫球提示灯中断')

        total_attempts = self._max_grab_retries + 1
        for attempt in range(1, total_attempts + 1):
            success, message = self._pickup_attempt(attempt, total_attempts)
            if success:
                return TaskOutcome.ok()
            if self._node.stopped:
                return TaskOutcome.failed(
                    '26rb_grab_golf.stopped', '抓高尔夫球任务被中止')
            if attempt == total_attempts:
                self._logger.error(
                    f'{self._task_name}：已完成 {total_attempts} 次抓取，'
                    f'仍未确认 {self._golf_color} 离开置物盘')
                return TaskOutcome.failed(
                    '26rb_grab_golf.verification',
                    f'{total_attempts} 次抓取后高尔夫球仍在盘上：{message}')
            if message != '复检时高尔夫球仍在盘上':
                return TaskOutcome.failed(
                    '26rb_grab_golf.pickup', message)

        return TaskOutcome.failed(
            '26rb_grab_golf.verification', '抓高尔夫球任务未完成')
