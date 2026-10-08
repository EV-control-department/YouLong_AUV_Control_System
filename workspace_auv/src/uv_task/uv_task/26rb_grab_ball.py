"""使用 collection_frame 定位并完成带上浮复检的单球抓取。"""

from __future__ import annotations

import math
import time

from uv_msgs.action import BasicMotion
from uv_task.task_outcome import TaskOutcome


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


class RB26GrabBallTask:
    """Use the down-left camera and TF geometry to pick up one golf ball."""

    def __init__(self, node, params: dict):
        self._node = node
        self._params = params
        self._logger = node.get_logger()

        down_left = node.camera_configs['down'].side('left')
        self._fx = float(down_left.matrix[0, 0])
        self._fy = float(down_left.matrix[1, 1])
        self._cx = float(down_left.matrix[0, 2])
        self._cy = float(down_left.matrix[1, 2])

        color = params.get('ball_color', 'pink_golf')
        self._color = str(color)
        self._class_id = self._parse_ball_color(color)
        collection_class = params.get(
            'collection_frame_class', 'collection_frame_down')
        self._collection_class_id = node._model_mapping.model_class_id(
            collection_class, required=False)

        self._detection_timeout = max(
            0.1, float(params.get('detection_timeout', 0.8)))
        self._observe_seconds = max(
            0.0, float(params.get('collection_frame_observe_seconds', 2.0)))
        self._golf_observe_seconds = max(
            0.0, float(params.get('golf_observe_seconds', 2.0)))
        self._servo_timeout = max(
            1.0, float(params.get('horizontal_servo_timeout', 30.0)))
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
        self._max_xy_step = max(
            0.005, float(params.get('max_horizontal_step_m', 0.08)))
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
            0.2, float(params.get('verification_timeout', 5.0)))
        self._verification_absence_hold_seconds = _clamp(
            float(params.get('verification_absence_hold_seconds', 0.8)),
            0.0, self._verification_timeout)
        # 2 retries after the first attempt means at most 3 pickup attempts.
        self._max_grab_retries = max(
            0, int(params.get('max_grab_retries', 2)))

    def _parse_ball_color(self, value) -> int | None:
        if not isinstance(value, str):
            return None
        name = value.strip()
        class_id = self._node._model_mapping.model_class_id(
            name, required=False)
        return class_id if name in {
            'impact_ball_blue', 'impact_ball_red', 'pink_golf', 'yellow_golf'
        } and class_id is not None else None

    def _best_left_detection(self, class_id: int | None = None,
                             received_after: float | None = None):
        """Return the freshest/highest-quality target in down-left."""
        if class_id is None:
            class_id = self._class_id
        with self._node._perception_lock:
            entry = self._node._down_detections.get('down_left')
        if entry is None:
            return None
        received_at, message = entry
        if (received_after is not None and received_at < received_after) \
                or time.monotonic() - received_at > self._detection_timeout:
            return None

        candidates = []
        for detection in getattr(message, 'detections', []):
            if int(getattr(detection, 'class_id', -1)) != class_id:
                continue
            try:
                px = float(detection.pixel_x)
                py = float(detection.pixel_y)
                confidence = float(detection.confidence)
            except (AttributeError, TypeError, ValueError):
                continue
            if not all(math.isfinite(value) for value in (px, py, confidence)):
                continue
            x1 = float(getattr(detection, 'bbox_x1', px))
            x2 = float(getattr(detection, 'bbox_x2', px))
            y1 = float(getattr(detection, 'bbox_y1', py))
            y2 = float(getattr(detection, 'bbox_y2', py))
            area = max(0.0, x2 - x1) * max(0.0, y2 - y1)
            candidates.append((confidence, area, detection))
        if not candidates:
            return None
        return max(candidates, key=lambda item: (item[0], item[1]))[-1]

    def _wait_for_detection(self, class_id: int, label: str,
                            duration: float):
        """Watch a fresh down-left stream for the requested target."""
        started = time.monotonic()
        deadline = started + max(0.0, float(duration))
        next_log = float('-inf')
        first_detection = None
        self._logger.info(
            f'26rb_grab_ball：开始观察 {label}，窗口={duration:.1f}s，'
            '相机=down_left')
        while not self._node.stopped and time.monotonic() < deadline:
            now = time.monotonic()
            detection = self._best_left_detection(class_id, started)
            if detection is not None and first_detection is None:
                first_detection = detection
                self._logger.info(
                    f'26rb_grab_ball：观察到 {label}；'
                    f'像素=({float(detection.pixel_x):.1f},'
                    f'{float(detection.pixel_y):.1f})，'
                    f'置信度={float(detection.confidence):.3f}，'
                    '继续完成观察窗口')
            if first_detection is None and now - next_log >= self._log_period:
                self._logger.info(
                    f'26rb_grab_ball：观察 {label} 中，尚未收到新鲜目标检测')
                next_log = now
            time.sleep(min(self._servo_period, max(0.0, deadline - now)))
        if self._node.stopped:
            return None
        if first_detection is not None:
            self._logger.info(
                f'26rb_grab_ball：{label} 的 {duration:.1f}s 观察窗口结束，'
                '窗口内曾检测到目标')
            return first_detection
        self._logger.warning(
            f'26rb_grab_ball：{duration:.1f}s 内未观测到 {label}')
        return None

    def _flash_green(self, count: int, label: str) -> bool:
        """Flash green the requested number of times, then resume yellow."""
        count = max(1, int(count))
        self._logger.info(
            f'26rb_grab_ball：{label}，绿灯闪烁 {count} 次')
        for index in range(count):
            if self._node.stopped:
                return False
            self._node.set_light(
                self._node.LIGHT_GREEN, f'{label} ({index + 1}/{count})')
            deadline = time.monotonic() + self._light_pulse_seconds
            while not self._node.stopped and time.monotonic() < deadline:
                time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
            if self._node.stopped:
                return False
            self._node.set_light(self._node.LIGHT_OFF, f'{label} 灯间隔')
            if index + 1 < count:
                deadline = time.monotonic() + self._light_gap_seconds
                while not self._node.stopped and time.monotonic() < deadline:
                    time.sleep(min(
                        0.05, max(0.0, deadline - time.monotonic())))
        if not self._node.stopped:
            self._node._set_task_phase_light(
                self._node.LIGHT_YELLOW, f'{label}提示结束，恢复抓取阶段灯')
            return True
        return False

    @staticmethod
    def _body_to_world(dx: float, dy: float, yaw_deg: float):
        yaw = math.radians(float(yaw_deg))
        c, s = math.cos(yaw), math.sin(yaw)
        return c * dx - s * dy, s * dx + c * dy

    def _horizontal_step(self, detection):
        """Convert down-left pixel error into one bounded world XY step."""
        px = float(detection.pixel_x)
        py = float(detection.pixel_y)
        # Down optical x maps to +y_body and optical y maps to -x_body.
        du = (px - self._cx) / self._fx
        dv = (py - self._cy) / self._fy
        body_dx = -dv * self._projection_depth * self._servo_gain
        body_dy = du * self._projection_depth * self._servo_gain
        norm = math.hypot(body_dx, body_dy)
        if norm > self._max_xy_step:
            scale = self._max_xy_step / norm
            body_dx *= scale
            body_dy *= scale

        pose = self._node._latest_robot_pose()
        world_dx, world_dy = self._body_to_world(body_dx, body_dy, pose[5])
        return pose, body_dx, body_dy, world_dx, world_dy, du, dv

    def _servo_horizontally(self, class_id: int, label: str):
        """Move until the selected down-left detection is image-centred."""
        if class_id is None:
            self._logger.error(f'26rb_grab_ball：模型映射中没有 {label} 类别')
            return None

        deadline = time.monotonic() + self._servo_timeout
        hold_since = None
        last_command = None
        last_status_log = float('-inf')
        self._node._set_task_phase_light(
            self._node.LIGHT_YELLOW, f'水平伺服至 {label}')
        self._logger.info(
            f'26rb_grab_ball：开始水平视觉伺服至 {label}；'
            f'class_id={class_id}，容差={self._pixel_tolerance:.3f}，'
            f'投影深度={self._projection_depth:.2f}m')
        while not self._node.stopped and time.monotonic() < deadline:
            now = time.monotonic()
            detection = self._best_left_detection(class_id)
            if detection is None:
                hold_since = None
                if now - last_status_log >= self._log_period:
                    with self._node._perception_lock:
                        entry = self._node._down_detections.get('down_left')
                    if entry is None:
                        detail = '尚未收到 down_left 话题消息'
                    else:
                        age = now - entry[0]
                        detections = len(getattr(entry[1], 'detections', []))
                        detail = (
                            f'消息年龄={age:.2f}s/{self._detection_timeout:.2f}s，'
                            f'检测数量={detections}，目标类别={class_id}')
                    self._logger.info(
                        f'26rb_grab_ball：{label} 水平伺服等待检测；{detail}')
                    last_status_log = now
                time.sleep(min(
                    self._servo_period,
                    max(0.0, deadline - time.monotonic())))
                continue

            pose, body_dx, body_dy, world_dx, world_dy, du, dv = (
                self._horizontal_step(detection))
            centered = (abs(du) <= self._pixel_tolerance
                        and abs(dv) <= self._pixel_tolerance)
            if now - last_status_log >= self._log_period:
                state = '已居中' if centered else '修正中'
                self._logger.info(
                    f'26rb_grab_ball：{label} 水平伺服状态={state}；'
                    f'像素=({float(detection.pixel_x):.1f},'
                    f'{float(detection.pixel_y):.1f})，'
                    f'误差=(du={du:+.4f},dv={dv:+.4f})，'
                    f'机体步长=({body_dx:+.3f},{body_dy:+.3f})m，'
                    f'世界步长=({world_dx:+.3f},{world_dy:+.3f})m，'
                    f'位姿=({pose[0]:.3f},{pose[1]:.3f},{pose[2]:.3f},'
                    f'{pose[5]:.1f}°)')
                last_status_log = now

            if centered:
                if hold_since is None:
                    hold_since = time.monotonic()
                    self._logger.info(
                        f'26rb_grab_ball：{label} 已在左下视野居中，'
                        f'保持 {self._hold_seconds:.1f}s')
                if time.monotonic() - hold_since >= self._hold_seconds:
                    recorded = [
                        float(self._node._cmd_x), float(self._node._cmd_y),
                        float(self._node._cmd_z), float(self._node._cmd_yaw)]
                    self._logger.info(
                        f'26rb_grab_ball：{label} 水平伺服完成；'
                        f'记录相机居中位姿=({recorded[0]:.3f}, '
                        f'{recorded[1]:.3f}, {recorded[2]:.3f}, '
                        f'{recorded[3]:.1f}°)')
                    return recorded
            else:
                hold_since = None
                target_x = pose[0] + world_dx
                target_y = pose[1] + world_dy
                target = [target_x, target_y,
                          float(self._node._cmd_z), float(pose[5])]
                if (last_command is None
                        or math.hypot(target_x - last_command[0],
                                      target_y - last_command[1]) > 1e-4):
                    success, message = self._node._send_action_goal(
                        BasicMotion.Goal.SET, target, 'xy',
                        timeout=self._command_timeout, quiet=True,
                        task_context=self._node._format_motion_context(
                            f'抓取{label}水平伺服'))
                    if not success:
                        self._logger.warning(
                            f'26rb_grab_ball：{label} 水平修正失败；'
                            f'目标=({target_x:.3f},{target_y:.3f})，'
                            f'误差=({du:+.4f},{dv:+.4f})，消息={message}')
                    else:
                        self._node._cmd_x = target_x
                        self._node._cmd_y = target_y
                        self._node._cmd_yaw = float(pose[5])
                        last_command = [target_x, target_y]
                        self._logger.info(
                            f'26rb_grab_ball：{label} 水平修正已接受；'
                            f'目标=({target_x:.3f},{target_y:.3f})，'
                            f'世界步长=({world_dx:+.3f},{world_dy:+.3f})m')
            time.sleep(self._servo_period)

        self._logger.error(f'26rb_grab_ball：{label} 水平视觉伺服超时')
        return None

    def _prepare_claw(self) -> bool:
        """Repeatedly command servo 1 to the requested open/pre-grab angle."""
        self._logger.info(
            f'26rb_grab_ball：夹爪进入准备状态；舵机 ID=1，'
            f'角度={self._claw_prepare_angle:.3f}rad，'
            f'重复发送 {self._claw_prepare_repeat_count} 次')
        for index in range(self._claw_prepare_repeat_count):
            if self._node.stopped:
                return False
            self._node.set_servo(
                self._claw_prepare_angle,
                f'抓球准备位置 ({index + 1}/'
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
            self._logger.error('26rb_grab_ball：下视相机外参尚未就绪')
            return None
        camera = self._node.camera_extrinsics.get('down_left')
        if camera is None:
            self._logger.error('26rb_grab_ball：缺少 down_left 相机外参')
            return None

        provider = self._node.camera_extrinsics_provider
        try:
            transform = provider.lookup_transform(
                provider.base_frame, 'disc_claw_link')
            message = (transform.transform
                       if hasattr(transform, 'transform') else transform)
            claw_xyz = (
                float(message.translation.x),
                float(message.translation.y),
                float(message.translation.z))
        except Exception as error:
            self._logger.error(
                f'26rb_grab_ball：读取 disc_claw_link 外参失败：{error}')
            return None

        # When the target is centred in down-left, it lies on the camera's
        # optical axis. Translate by camera_origin - claw_origin so the fixed
        # target is directly below the gripper instead of the camera.
        dx = float(camera.translation[0]) - claw_xyz[0]
        dy = float(camera.translation[1]) - claw_xyz[1]
        yaw = float(self._node._cmd_yaw)
        world_dx, world_dy = self._body_to_world(dx, dy, yaw)
        self._logger.info(
            '26rb_grab_ball：按 TF 外参计算相机到圆盘爪的水平偏移；'
            f'down_left=({camera.translation[0]:+.3f},'
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
                    f'按相机外参对准 {self._color} 球与圆盘爪'))
            if not success:
                self._logger.error(
                    f'26rb_grab_ball：相机到圆盘爪外参平移失败：{message}')
                return None
            self._node._cmd_x += world_dx
            self._node._cmd_y += world_dy

        recorded = [
            float(self._node._cmd_x), float(self._node._cmd_y),
            float(self._node._cmd_z), float(self._node._cmd_yaw)]
        self._logger.info(
            f'26rb_grab_ball：圆盘爪已对准球，记录下潜前位姿='
            f'({recorded[0]:.3f},{recorded[1]:.3f},{recorded[2]:.3f},'
            f'{recorded[3]:.1f}°)')
        return recorded

    def _wait_pre_descent_settle(self):
        if self._pre_descent_settle_seconds <= 0.0:
            return not self._node.stopped
        self._logger.info(
            f'26rb_grab_ball：等待艇体稳定 '
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
            f'26rb_grab_ball：{label}，速度={speed_mps:+.3f}m/s，'
            f'持续={duration:.1f}s（NED 正值向下）')
        try:
            while not self._node.stopped and time.monotonic() < deadline:
                success, message = self._node._send_body_velocity(
                    vertical_mps=speed_mps,
                    lease_s=max(0.25, self._vertical_period * 4.0),
                    task_context=self._node._format_motion_context(
                        f'抓球{label}速度控制'))
                if not success:
                    self._logger.error(
                        f'26rb_grab_ball：{label}速度指令失败：{message}')
                    command_ok = False
                    break
                time.sleep(min(
                    self._vertical_period,
                    max(0.0, deadline - time.monotonic())))
        finally:
            neutral_ok, neutral_message = self._node._send_body_velocity(
                lease_s=max(0.25, self._vertical_period * 4.0),
                task_context=self._node._format_motion_context(
                    f'结束抓球{label}速度控制'))
            if not neutral_ok:
                self._logger.error(
                    f'26rb_grab_ball：{label}结束时发送中性速度失败：'
                    f'{neutral_message}')
                command_ok = False
        return command_ok and not self._node.stopped

    def _descend(self):
        return self._vertical_motion(
            self._descent_speed, self._descent_duration, '下潜')

    def _ascend(self):
        return self._vertical_motion(
            -self._ascent_speed, self._ascent_duration, '上浮')

    def _verify_ball_removed(self):
        """Require a fresh post-return image where the ball stays absent."""
        check_started = time.monotonic()
        deadline = check_started + self._verification_timeout
        absent_since = None
        fresh_message_seen = False
        last_processed_message_at = None
        last_status_log = float('-inf')
        target_present = False
        self._logger.info(
            f'26rb_grab_ball：已回到记录位置，检查 {self._color} 是否仍在盘上；'
            f'检查超时={self._verification_timeout:.1f}s，'
            f'连续消失确认={self._verification_absence_hold_seconds:.1f}s')
        while not self._node.stopped and time.monotonic() < deadline:
            now = time.monotonic()
            with self._node._perception_lock:
                entry = self._node._down_detections.get('down_left')
            if (entry is None or entry[0] < check_started
                    or entry[0] == last_processed_message_at):
                if now - last_status_log >= self._log_period:
                    self._logger.info(
                        '26rb_grab_ball：复检等待回位后的新鲜视觉消息')
                    last_status_log = now
                time.sleep(min(self._servo_period, max(0.0, deadline - now)))
                continue

            fresh_message_seen = True
            last_processed_message_at = entry[0]
            detection = self._best_left_detection(self._class_id, check_started)
            if detection is None:
                if absent_since is None:
                    absent_since = entry[0]
                    self._logger.info(
                        f'26rb_grab_ball：当前未检测到 {self._color}，'
                        '开始连续消失计时')
                if (entry[0] - absent_since
                        >= self._verification_absence_hold_seconds):
                    self._logger.info(
                        f'26rb_grab_ball：复检确认 {self._color} 已不在盘上')
                    return True
            else:
                target_present = True
                absent_since = None
                if now - last_status_log >= self._log_period:
                    self._logger.warning(
                        f'26rb_grab_ball：复检仍检测到盘上的 {self._color}；'
                        f'像素=({float(detection.pixel_x):.1f},'
                        f'{float(detection.pixel_y):.1f})')
                    last_status_log = now
            time.sleep(min(self._servo_period, max(0.0, deadline - now)))

        if not fresh_message_seen:
            self._logger.error(
                '26rb_grab_ball：复检超时，回位后没有收到新鲜视觉消息')
        elif target_present:
            self._logger.warning(
                f'26rb_grab_ball：复检超时，盘上仍能看到 {self._color}')
        else:
            self._logger.warning('26rb_grab_ball：复检超时，无法确认球已离盘')
        return False

    def _return_to_recorded_pose(self, recorded_pose):
        """Return to the pre-dive pose saved after camera/gripper alignment."""
        success, message = self._node._send_action_goal(
            BasicMotion.Goal.SET, recorded_pose, 'xyzrz',
            timeout=self._return_timeout,
            task_context=self._node._format_motion_context(
                f'抓取{self._color}球后返回下潜前位置'))
        if not success:
            self._logger.error(
                f'26rb_grab_ball：返回记录位置失败：{message}')
            return False
        (self._node._cmd_x, self._node._cmd_y,
         self._node._cmd_z, self._node._cmd_yaw) = recorded_pose
        self._logger.info('26rb_grab_ball：已返回记录的下潜前位置')
        return True

    def _pickup_attempt(self, attempt: int, total_attempts: int):
        self._logger.info(
            f'26rb_grab_ball：开始第 {attempt}/{total_attempts} 次抓取；'
            f'视觉伺服目标={self._color}')
        camera_pose = self._servo_horizontally(self._class_id, self._color)
        if camera_pose is None:
            return False, '水平视觉对准球失败'
        if not self._flash_green(3, f'{self._color} 已对准'):
            return False, '球居中提示灯中断'

        recorded_pose = self._apply_camera_gripper_offset()
        if recorded_pose is None:
            return False, '相机到圆盘爪外参对准失败'
        if not self._wait_pre_descent_settle():
            return False, '下潜前稳定等待被中止'

        descent_ok = self._descend()
        # Attempt the ascent and positional recovery even if a descent velocity
        # renewal failed, so the vehicle still gets a chance to return upward.
        ascent_ok = self._ascend() if not self._node.stopped else False
        return_ok = (self._return_to_recorded_pose(recorded_pose)
                     if not self._node.stopped else False)
        if not descent_ok or not ascent_ok or not return_ok:
            return False, '下潜、上浮或回到记录位置失败'

        if self._verify_ball_removed():
            self._logger.info(f'26rb_grab_ball：第 {attempt} 次抓取确认成功')
            return True, ''
        if attempt < total_attempts:
            self._logger.warning(
                f'26rb_grab_ball：第 {attempt} 次后球仍在盘上，'
                f'剩余 {total_attempts - attempt} 次机会，将重新伺服并抓取')
        return False, '复检时球仍在盘上'

    def execute(self) -> TaskOutcome:
        if self._collection_class_id is None:
            return TaskOutcome.failed(
                '26rb_grab_ball.collection_frame',
                '模型映射中没有 collection_frame_down 类别')
        if self._class_id is None:
            return TaskOutcome.failed(
                '26rb_grab_ball.golf', f'不支持的球颜色 {self._color!r}')

        self._node._set_task_phase_light(
            self._node.LIGHT_YELLOW, '抓球任务：搜索置物盘')
        if self._wait_for_detection(
                self._collection_class_id, 'collection_frame',
                self._observe_seconds) is None:
            return TaskOutcome.failed(
                '26rb_grab_ball.collection_frame',
                f'{self._observe_seconds:.1f}s 内未看到 collection_frame')
        if not self._flash_green(1, '已观察到 collection_frame'):
            return TaskOutcome.failed(
                '26rb_grab_ball.collection_frame', '置物盘提示灯中断')

        if not self._prepare_claw():
            return TaskOutcome.failed(
                '26rb_grab_ball.claw', '夹爪准备动作被中止')

        frame_pose = self._servo_horizontally(
            self._collection_class_id, 'collection_frame')
        if frame_pose is None:
            return TaskOutcome.failed(
                '26rb_grab_ball.collection_frame',
                '水平伺服到 collection_frame 正上方失败')

        if self._wait_for_detection(
                self._class_id, self._color,
                self._golf_observe_seconds) is None:
            return TaskOutcome.failed(
                '26rb_grab_ball.golf',
                f'伺服到 collection_frame 后，'
                f'{self._golf_observe_seconds:.1f}s 内没有看到 {self._color}')
        if not self._flash_green(2, f'已观察到 {self._color}'):
            return TaskOutcome.failed(
                '26rb_grab_ball.golf', '目标球提示灯中断')

        total_attempts = self._max_grab_retries + 1
        for attempt in range(1, total_attempts + 1):
            success, message = self._pickup_attempt(attempt, total_attempts)
            if success:
                return TaskOutcome.ok()
            if self._node.stopped:
                return TaskOutcome.failed(
                    '26rb_grab_ball.stopped', '抓球任务被中止')
            if attempt == total_attempts:
                self._logger.error(
                    f'26rb_grab_ball：已完成 {total_attempts} 次抓取，'
                    f'仍未确认 {self._color} 离开置物盘')
                return TaskOutcome.failed(
                    '26rb_grab_ball.verification',
                    f'{total_attempts} 次抓取后球仍在盘上：{message}')
            if message != '复检时球仍在盘上':
                return TaskOutcome.failed(
                    '26rb_grab_ball.pickup', message)

        return TaskOutcome.failed(
            '26rb_grab_ball.verification', '抓球任务未完成')
