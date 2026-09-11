"""使用下视相机完成带复检和重试的单球抓取。"""

from __future__ import annotations

import math
import time

import numpy as np

from uv_msgs.action import BasicMotion
from uv_camera.model_classes import model_class_id


_BALL_CLASS_IDS = {
    'blue': model_class_id('impact_ball_blue'),
    'impact_ball_blue': model_class_id('impact_ball_blue'),
    'blue_ball': model_class_id('impact_ball_blue'),
    # The active down-left detector labels the red ball as class 7.
    'red': model_class_id('pink_golf'),
    'impact_ball_red': model_class_id('pink_golf'),
    'red_ball': model_class_id('pink_golf'),
    'pink': model_class_id('pink_golf'),
    'pink_golf': model_class_id('pink_golf'),
    'pink_ball': model_class_id('pink_golf'),
    'yellow': model_class_id('yellow_golf'),
    'yellow_golf': model_class_id('yellow_golf'),
    'yellow_ball': model_class_id('yellow_golf'),
}


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


def _wrap_yaw(value: float) -> float:
    return (float(value) + 180.0) % 360.0 - 180.0


class RB26GrabBallTask:
    """Center a coloured ball under down-left, offset to the gripper, descend.

    The task intentionally uses only ``/perception/detection/down_left`` for
    the visual servo.  It does not use the object localizer position estimate,
    because the final pickup alignment is relative to the camera and gripper.
    """

    # These are the same down-left calibration values used by arrow_surfacer,
    # line_follower, and object_localizer.py.
    _IMAGE_WIDTH = 1280.0
    _IMAGE_HEIGHT = 960.0
    _HFOV_DEG = 87.19
    _FX = _IMAGE_WIDTH / (2.0 * math.tan(math.radians(_HFOV_DEG) / 2.0))
    _FY = _FX
    _CX = _IMAGE_WIDTH / 2.0
    _CY = _IMAGE_HEIGHT / 2.0

    def __init__(self, node, params: dict):
        self._node = node
        self._params = params
        self._logger = node.get_logger()

        color = params.get('ball_color', params.get('color', 'red'))
        self._class_id = self._parse_ball_color(color)
        self._color = str(color)
        self._detection_timeout = max(
            0.1, float(params.get('detection_timeout', 0.8)))
        self._servo_timeout = max(
            1.0, float(params.get('horizontal_servo_timeout', 30.0)))
        self._servo_period = max(
            0.05, float(params.get('horizontal_servo_period', 0.15)))
        self._log_period = max(
            0.1, float(params.get('horizontal_servo_log_period', 0.5)))
        self._pixel_tolerance = _clamp(
            float(params.get('pixel_tolerance_fraction', 0.035)),
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
            0.2, float(params.get('position_command_timeout', 2.0)))

        # Body-frame offset from the camera-centred point to the gripper.
        # +x is forward and +y is right in the AUV body frame.
        self._gripper_offset_x = float(
            params.get('gripper_offset_x_m', 0.05))
        self._gripper_offset_y = float(
            params.get('gripper_offset_y_m', 0.0))
        self._pre_descent_settle_seconds = max(
            0.0, float(params.get('pre_descent_settle_seconds', 1.5)))
        self._descent_speed = float(
            params.get('descent_speed_mps', 0.15))
        self._descent_duration = max(
            0.0, float(params.get('descent_duration_seconds', 10.0)))
        self._descent_period = max(
            0.02, float(params.get('descent_publish_period', 0.05)))
        self._return_timeout = max(
            1.0, float(params.get('return_timeout', 60.0)))
        self._verification_timeout = max(
            0.2, float(params.get('verification_timeout', 5.0)))
        self._verification_absence_hold_seconds = _clamp(
            float(params.get('verification_absence_hold_seconds', 0.8)),
            0.0, self._verification_timeout)
        self._max_grab_retries = max(
            0, int(params.get('max_grab_retries', 2)))

    @staticmethod
    def _parse_ball_color(value) -> int | None:
        if isinstance(value, (int, np.integer)):
            class_id = int(value)
            return class_id if class_id in set(_BALL_CLASS_IDS.values()) else None
        text = str(value or '').strip().lower().replace('-', '_').replace(' ', '_')
        if text.isdigit():
            return RB26GrabBallTask._parse_ball_color(int(text))
        return _BALL_CLASS_IDS.get(text)

    def _best_left_detection(self):
        """Return the freshest/highest-quality target in down-left."""
        with self._node._perception_lock:
            entry = self._node._down_detections.get('down_left')
        if entry is None or time.monotonic() - entry[0] > self._detection_timeout:
            return None

        candidates = []
        for detection in getattr(entry[1], 'detections', []):
            if int(getattr(detection, 'class_id', -1)) != self._class_id:
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

    @staticmethod
    def _body_to_world(dx: float, dy: float, yaw_deg: float):
        yaw = math.radians(float(yaw_deg))
        c, s = math.cos(yaw), math.sin(yaw)
        return c * dx - s * dy, s * dx + c * dy

    def _horizontal_step(self, detection):
        """Convert down-left pixel error into one bounded world XY step."""
        px = float(detection.pixel_x)
        py = float(detection.pixel_y)
        # Down-left optical axes are mapped to body (-y, +x, +z).  Therefore
        # a target to the right of image centre requires body +y motion, while
        # a target below image centre requires body -x motion.
        du = (px - self._CX) / self._FX
        dv = (py - self._CY) / self._FY
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

    def _servo_horizontally(self):
        if self._class_id is None:
            self._logger.error(
                f'26rb_grab_ball：不支持的球颜色 {self._color!r}')
            return None

        deadline = time.monotonic() + self._servo_timeout
        hold_since = None
        last_command = None
        last_status_log = float('-inf')
        self._logger.info(
            f'26rb_grab_ball：水平视觉伺服已启动；'
            f'话题=/perception/detection/down_left，'
            f'class_id={self._class_id}，容差={self._pixel_tolerance:.3f}，'
            f'投影深度={self._projection_depth:.2f}m')
        while (not self._node.stopped and time.monotonic() < deadline):
            now = time.monotonic()
            detection = self._best_left_detection()
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
                            f'检测数量={detections}，'
                            f'目标类别={self._class_id}')
                    self._logger.info(
                        f'26rb_grab_ball：水平伺服状态=等待检测；{detail}')
                    last_status_log = now
                time.sleep(min(self._servo_period, max(0.0, deadline - time.monotonic())))
                continue

            pose, body_dx, body_dy, world_dx, world_dy, du, dv = (
                self._horizontal_step(detection))
            centered = (abs(du) <= self._pixel_tolerance
                        and abs(dv) <= self._pixel_tolerance)
            if now - last_status_log >= self._log_period:
                state = '已居中' if centered else '修正中'
                self._logger.info(
                        f'26rb_grab_ball：水平伺服状态={state}；'
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
                        f'26rb_grab_ball：{self._color} 已在左下视野居中，'
                        f'保持 {self._hold_seconds:.1f}s')
                if time.monotonic() - hold_since >= self._hold_seconds:
                    recorded = self._node._latest_robot_pose()
                    result = [recorded[0], recorded[1], recorded[2], recorded[5]]
                    self._logger.info(
                        f'26rb_grab_ball：水平视觉伺服完成；'
                        f'记录位置=({result[0]:.3f}, {result[1]:.3f}, '
                        f'{result[2]:.3f}, {result[3]:.1f}°)')
                    return result
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
                            f'抓取{self._color}球水平伺服'))
                    if not success:
                        self._logger.warning(
                            f'26rb_grab_ball：水平修正失败；'
                            f'目标=({target_x:.3f},{target_y:.3f})，'
                            f'当前位置=({pose[0]:.3f},{pose[1]:.3f})，'
                            f'误差=({du:+.4f},{dv:+.4f})，消息={message}')
                    else:
                        self._node._cmd_x = target_x
                        self._node._cmd_y = target_y
                        last_command = [target_x, target_y]
                        self._logger.info(
                            f'26rb_grab_ball：水平修正已接受；'
                            f'目标=({target_x:.3f},{target_y:.3f})，'
                            f'世界步长=({world_dx:+.3f},{world_dy:+.3f})m')
            time.sleep(self._servo_period)

        self._logger.error(
            f'26rb_grab_ball：{self._color} 水平视觉伺服超时')
        return None

    def _apply_gripper_offset(self):
        """在下降前把相机对准点移动到爪子位置。"""
        dx = self._gripper_offset_x
        dy = self._gripper_offset_y
        if math.hypot(dx, dy) <= 1e-6:
            self._logger.info('26rb_grab_ball：夹爪偏置为零')
            return True

        success, message = self._node._send_action_goal(
            BasicMotion.Goal.BMOVE,
            [dx, dy, 0.0, 0.0], 'xy',
            timeout=self._command_timeout,
            task_context=self._node._format_motion_context(
                f'抓取{self._color}球爪子偏置'))
        if not success:
            self._logger.error(
                f'26rb_grab_ball：夹爪偏置移动失败：{message}')
            return False
        world_dx, world_dy = self._body_to_world(
            dx, dy, self._node._cmd_yaw)
        self._node._cmd_x += world_dx
        self._node._cmd_y += world_dy
        self._logger.info(
            f'26rb_grab_ball：夹爪偏置已应用，'
            f'机体坐标=({dx:.3f}, {dy:.3f})m')
        return True

    def _wait_pre_descent_settle(self):
        """等待偏置动作完成后的艇体和夹爪机械振动衰减。"""

        if self._pre_descent_settle_seconds <= 0.0:
            return not self._node.stopped
        self._logger.info(
            f'26rb_grab_ball：等待偏置移动稳定 '
            f'{self._pre_descent_settle_seconds:.1f}s，随后开始下降')
        deadline = time.monotonic() + self._pre_descent_settle_seconds
        while (not self._node.stopped and time.monotonic() < deadline):
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
        return not self._node.stopped

    def _descend(self):
        """以机体 z 方向速度下降，结束后发送中性速度。"""
        deadline = time.monotonic() + self._descent_duration
        while (not self._node.stopped and time.monotonic() < deadline):
            self._node._publish_body_velocity(
                vertical_mps=self._descent_speed)
            time.sleep(min(
                self._descent_period,
                max(0.0, deadline - time.monotonic())))
        self._node._publish_body_velocity()
        return not self._node.stopped

    def _verify_ball_removed(self):
        """返回后等待新鲜下视检测，确认目标球已经消失。

        不能把返回动作开始前的旧检测结果当作复检结果。至少收到一条
        返回之后的新消息后，目标连续消失 ``absence_hold_seconds`` 才
        认为抓取成功；如果目标仍在检测结果中直到超时，则交给上层重试。
        """

        check_started = time.monotonic()
        deadline = check_started + self._verification_timeout
        absent_since = None
        fresh_message_seen = False
        last_status_log = float('-inf')
        target_present = False

        self._logger.info(
            f'26rb_grab_ball：已返回，开始检查 {self._color} 是否仍存在；'
            f'检查超时={self._verification_timeout:.1f}s，'
            f'连续消失确认={self._verification_absence_hold_seconds:.1f}s')
        while not self._node.stopped and time.monotonic() < deadline:
            now = time.monotonic()
            with self._node._perception_lock:
                entry = self._node._down_detections.get('down_left')
            if entry is None or entry[0] < check_started:
                if now - last_status_log >= self._log_period:
                    self._logger.info(
                        '26rb_grab_ball：抓取结果复检等待返回后的新鲜视觉消息')
                    last_status_log = now
                time.sleep(min(self._servo_period, max(0.0, deadline - now)))
                continue

            fresh_message_seen = True
            detection = self._best_left_detection()
            if detection is None:
                if absent_since is None:
                    absent_since = now
                    self._logger.info(
                        f'26rb_grab_ball：返回后暂未检测到 {self._color}，'
                        '开始连续消失计时')
                if now - absent_since >= self._verification_absence_hold_seconds:
                    self._logger.info(
                        f'26rb_grab_ball：复检确认 {self._color} 已被抓走')
                    return True
            else:
                target_present = True
                absent_since = None
                if now - last_status_log >= self._log_period:
                    self._logger.warning(
                        f'26rb_grab_ball：复检仍检测到 {self._color}；'
                        f'像素=({float(detection.pixel_x):.1f},'
                        f'{float(detection.pixel_y):.1f})，准备判断是否重试')
                    last_status_log = now
            time.sleep(min(self._servo_period, max(0.0, deadline - now)))

        if not fresh_message_seen:
            self._logger.error(
                '26rb_grab_ball：抓取结果复检超时，返回后没有收到新鲜视觉消息')
        elif target_present:
            self._logger.warning(
                f'26rb_grab_ball：复检超时，仍能看到 {self._color}，抓取未确认成功')
        else:
            self._logger.warning('26rb_grab_ball：抓取结果复检超时，无法确认目标已消失')
        return False

    def _return_to_recorded_pose(self, recorded_pose):
        """下降后返回下降前的相机伺服位置。"""

        success, message = self._node._send_action_goal(
            BasicMotion.Goal.SET, recorded_pose, 'xyzrz',
            timeout=self._return_timeout,
            task_context=self._node._format_motion_context(
                f'抓取{self._color}球后返回水平伺服位置'))
        if not success:
            self._logger.error(
                f'26rb_grab_ball：返回记录的伺服位置失败：{message}')
            return False
        self._node._cmd_x, self._node._cmd_y, self._node._cmd_z, self._node._cmd_yaw = recorded_pose
        self._logger.info('26rb_grab_ball：已返回记录的伺服位置')
        return True

    def execute(self) -> bool:
        total_attempts = self._max_grab_retries + 1
        for attempt in range(1, total_attempts + 1):
            self._logger.info(
                f'26rb_grab_ball：开始第 {attempt}/{total_attempts} 次抓取；'
                f'在左下视野搜索 {self._color}（class_id={self._class_id}）')
            recorded_pose = self._servo_horizontally()
            if recorded_pose is None:
                return False
            if not self._apply_gripper_offset():
                return False
            if not self._wait_pre_descent_settle():
                return False
            self._logger.info(
                f'26rb_grab_ball：以 {self._descent_speed:.3f}m/s '
                f'下潜 {self._descent_duration:.1f}s')
            if not self._descend():
                return False
            if not self._return_to_recorded_pose(recorded_pose):
                return False
            if self._verify_ball_removed():
                self._logger.info(
                    f'26rb_grab_ball：第 {attempt} 次抓取确认成功')
                return True
            if attempt < total_attempts:
                self._logger.warning(
                    f'26rb_grab_ball：第 {attempt} 次抓取后仍检测到 {self._color}；'
                    f'将在剩余 {total_attempts - attempt} 次机会中重新抓取')
                continue
            self._logger.error(
                f'26rb_grab_ball：已达到最大重新尝试次数 '
                f'{self._max_grab_retries}，仍未确认 {self._color} 被抓走')
            return False
        return False
