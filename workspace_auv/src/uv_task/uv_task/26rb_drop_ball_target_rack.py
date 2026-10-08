"""前视寻找 rack，转入下视观测、像素伺服和圆盘爪定位，再释放球。"""

from __future__ import annotations

import math
import time

import cv2
import numpy as np

from uv_msgs.action import BasicMotion
from uv_task.task_outcome import TaskOutcome
from uv_task.collection_frame_search import TargetRackSearch
from uv_task.down_camera_servo import (
    DownCameraPriority, body_image_step, normalized_image_error,
)


class RB26DropBallTargetRackTask:
    """Approach the rack with front detections, then use down detections and TF to drop."""

    def __init__(self, node, params: dict):
        self._node = node
        self._params = params
        self._logger = node.get_logger()
        self._observe_seconds = max(
            0.1, float(params.get('observation_seconds', 2.0)))
        self._detection_timeout = max(
            0.1, float(params.get('down_detection_timeout', 0.8)))
        self._priority_seconds = max(
            0.1, float(params.get('down_camera_priority_seconds', 3.0)))
        self._servo_timeout = max(1.0, float(params.get(
            'down_visual_servo_timeout',
            params.get('horizontal_servo_timeout', 30.0))))
        self._stable_seconds = max(0.1, float(params.get(
            'down_visual_servo_stable_seconds',
            params.get('horizontal_servo_stable_seconds', 1.0))))
        self._pixel_tolerance = max(
            0.001, float(params.get('down_pixel_tolerance_fraction', 0.035)))
        # This is a constant pixel-servo scale, not a measured target range.
        self._projection_depth = max(
            0.1, float(params.get('down_projection_depth_m', 0.8)))
        self._gain = max(
            0.05, float(params.get('down_visual_servo_gain', 0.8)))
        self._max_step = max(
            0.005, float(params.get('down_visual_servo_max_step_m', 0.08)))
        self._command_timeout = max(0.2, float(params.get(
            'down_visual_command_timeout',
            params.get('horizontal_command_timeout', 10.0))))
        self._period = max(
            0.05, float(params.get('down_visual_servo_period', 0.2)))
        self._alignment_timeout = max(
            0.2, float(params.get('alignment_timeout', 10.0)))
        self._alignment_settle_seconds = max(
            0.0, float(params.get('alignment_settle_seconds', 0.5)))
        self._release_angle_deg = float(params.get('release_angle_deg', 90.0))
        if (not math.isfinite(self._release_angle_deg)
                or abs(self._release_angle_deg) > 180.0):
            raise ValueError('释放舵机角度必须是 [-180, 180] 范围内的度数')
        self._release_repeat_count = max(
            1, int(params.get('release_repeat_count', 3)))
        self._release_repeat_period = max(
            0.0, float(params.get('release_repeat_period', 0.1)))
        self._release_settle_seconds = max(
            0.0, float(params.get('release_settle_seconds', 1.0)))
        self._light_pulse_seconds = max(
            0.05, float(params.get('light_pulse_seconds', 0.35)))
        self._light_gap_seconds = max(
            0.0, float(params.get('light_gap_seconds', 0.25)))
        self._priority = None
        self._aligned_camera = None

    def _sleep(self, seconds: float) -> bool:
        deadline = time.monotonic() + seconds
        while not self._node.stopped and time.monotonic() < deadline:
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
        return not self._node.stopped

    def _observe_rack(self) -> bool:
        """Watch for a fresh rack in either eye and start camera arbitration."""
        self._priority = DownCameraPriority(
            self._node, self._node._target_rack_down_class_id,
            priority_seconds=self._priority_seconds,
            detection_timeout=self._detection_timeout,
            label='26rb_drop_ball_target_rack rack观测')
        deadline = time.monotonic() + self._observe_seconds
        found = False
        self._logger.info(
            f'26rb_drop_ball_target_rack：从当前位置观察 rack '
            f'{self._observe_seconds:.1f}s；相机=down_left/down_right')
        while not self._node.stopped and time.monotonic() < deadline:
            self._priority.update()
            if self._priority.first_observation is not None:
                if not found:
                    camera_name, detection = self._priority.first_observation
                    self._logger.info(
                        f'26rb_drop_ball_target_rack：观测到 rack；'
                        f'相机={camera_name}，'
                        f'像素=({detection.pixel_x:.1f},{detection.pixel_y:.1f})，'
                        f'置信度={detection.confidence:.3f}')
                found = True
            if not self._sleep(min(self._period, max(
                    0.0, deadline - time.monotonic()))):
                return False
        return found and not self._node.stopped

    def _servo_rack(self) -> bool:
        node = self._node
        self._priority = DownCameraPriority(
            node, node._target_rack_down_class_id,
            priority_seconds=self._priority_seconds,
            detection_timeout=self._detection_timeout,
            label='26rb_drop_ball_target_rack rack伺服')
        deadline = time.monotonic() + self._servo_timeout
        stable_since = None
        stable_generation = None
        last_log = float('-inf')
        self._logger.info(
            '26rb_drop_ball_target_rack：开始单目像素伺服；'
            f'相机优先权={self._priority_seconds:.1f}s，'
            f'归一化误差容差={self._pixel_tolerance:.3f}，'
            f'连续稳定时间={self._stable_seconds:.1f}s')
        while not node.stopped and time.monotonic() < deadline:
            camera_name, detection = self._priority.update()
            now = time.monotonic()
            if stable_generation != self._priority.generation:
                stable_since = None
                stable_generation = self._priority.generation
            if detection is None:
                stable_since = None
                if now - last_log >= 1.0:
                    self._logger.info(
                        '26rb_drop_ball_target_rack：等待任一下视相机的新 rack 观测')
                    last_log = now
            else:
                camera = node.camera_extrinsics.get(camera_name)
                if camera is None:
                    self._logger.error(f'缺少 {camera_name} 相机 TF')
                    return False
                try:
                    du, dv = normalized_image_error(node, camera_name, detection)
                except (ValueError, cv2.error, np.linalg.LinAlgError) as error:
                    self._logger.error(f'单目中心误差计算失败：{error}')
                    return False
                centered = (abs(du) <= self._pixel_tolerance
                            and abs(dv) <= self._pixel_tolerance)
                if now - last_log >= 1.0:
                    self._logger.info(
                        f'26rb_drop_ball_target_rack：采纳 {camera_name} rack；'
                        f'像素=({detection.pixel_x:.1f},{detection.pixel_y:.1f})，'
                        f'归一化误差=(du={du:+.4f},dv={dv:+.4f})，'
                        f'状态={"居中等待稳定" if centered else "修正中"}')
                    last_log = now
                if centered:
                    if stable_since is None:
                        stable_since = now
                    elif now - stable_since >= self._stable_seconds:
                        self._aligned_camera = camera_name
                        self._logger.info(
                            f'26rb_drop_ball_target_rack：{camera_name} 单目伺服成功；'
                            '接下来只执行相机到圆盘爪的固定外参平移')
                        return True
                else:
                    stable_since = None
                    # Optical XY axes encode the calibrated down-view image
                    # directions. No pair, disparity, or 3D rack point is used.
                    body_dx, body_dy = body_image_step(
                        camera, du, dv, self._projection_depth,
                        self._gain, self._max_step)
                    pose = node._latest_robot_pose()
                    yaw = math.radians(pose[5])
                    world_dx = math.cos(yaw) * body_dx - math.sin(yaw) * body_dy
                    world_dy = math.sin(yaw) * body_dx + math.cos(yaw) * body_dy
                    target = [pose[0] + float(world_dx),
                              pose[1] + float(world_dy),
                              float(node._cmd_z), float(pose[5])]
                    success, message = node._send_action_goal(
                        BasicMotion.Goal.SET, target, 'xy',
                        timeout=self._command_timeout, quiet=True,
                        task_context=node._format_motion_context(
                            f'{camera_name} rack单目像素伺服'))
                    if not success:
                        self._logger.error(f'rack单目伺服移动失败：{message}')
                        return False
                    node._cmd_x, node._cmd_y = target[:2]
            if not self._sleep(min(self._period, max(
                    0.0, deadline - time.monotonic()))):
                return False
        if not node.stopped:
            node._last_motion_failure_kind = 'timeout'
            node._last_motion_failure_message = 'rack单目视觉伺服超时'
            self._logger.error('rack单目视觉伺服超时，未执行丢球')
        return False

    def _flash_green(self, count: int, label: str,
                     restore_phase: bool = True) -> bool:
        node = self._node
        self._logger.info(
            f'26rb_drop_ball_target_rack：{label}，闪 {count} 次绿灯')
        for index in range(count):
            if node.stopped:
                return False
            node.set_light(node.LIGHT_GREEN, f'{label} ({index + 1}/{count})')
            if not self._sleep(self._light_pulse_seconds):
                return False
            node.set_light(node.LIGHT_OFF, f'{label} 闪烁间隔', log=False)
            if index + 1 < count and not self._sleep(self._light_gap_seconds):
                return False
        if restore_phase and not node.stopped:
            node._set_task_phase_light(node.LIGHT_YELLOW, '投球运动阶段')
        return not node.stopped

    def _align_disc_claw(self) -> bool:
        """Translate by the successful eye's fixed camera-to-claw XY offset."""
        node = self._node
        camera = node.camera_extrinsics.get(self._aligned_camera)
        if camera is None:
            self._logger.error('圆盘爪平移失败：没有单目伺服成功的相机')
            return False
        provider = node.camera_extrinsics_provider
        try:
            transform = provider.lookup_transform(
                provider.base_frame, 'disc_claw_link')
            message = (transform.transform
                       if hasattr(transform, 'transform') else transform)
            claw_body = np.array([
                float(message.translation.x), float(message.translation.y),
                float(message.translation.z)], dtype=np.float64)
            if not np.all(np.isfinite(claw_body)):
                raise ValueError('圆盘爪位置外参包含无效数值')
        except Exception as error:
            self._logger.error(
                f'26rb_drop_ball_target_rack：读取圆盘爪 TF 失败：{error}')
            return False

        if node.stopped:
            return False

        # Preserve the selected eye's baseline offset; do not substitute the
        # stereo midpoint or use detections again after successful centering.
        body_dx = float(camera.translation[0] - claw_body[0])
        body_dy = float(camera.translation[1] - claw_body[1])
        pose = node._latest_robot_pose()
        yaw = math.radians(pose[5])
        world_dx = math.cos(yaw) * body_dx - math.sin(yaw) * body_dy
        world_dy = math.sin(yaw) * body_dx + math.cos(yaw) * body_dy
        target = [pose[0] + world_dx, pose[1] + world_dy,
                  float(node._cmd_z), float(pose[5])]
        self._logger.info(
            f'26rb_drop_ball_target_rack：按 {self._aligned_camera} '
            '和圆盘爪固定外参平移；'
            f'相机机体系=({camera.translation[0]:+.3f},'
            f'{camera.translation[1]:+.3f},{camera.translation[2]:+.3f})m，'
            f'圆盘爪机体系=({claw_body[0]:+.3f},{claw_body[1]:+.3f},'
            f'{claw_body[2]:+.3f})m，'
            f'机体平移=({body_dx:+.3f},{body_dy:+.3f})m，'
            f'世界平移=({world_dx:+.3f},{world_dy:+.3f})m，'
            f'SET目标=({target[0]:.3f},{target[1]:.3f})m')
        success, message = node._send_action_goal(
            BasicMotion.Goal.SET, target, 'xy',
            timeout=self._alignment_timeout,
            task_context=node._format_motion_context('圆盘爪对准rack正上方'))
        if not success:
            self._logger.error(
                f'26rb_drop_ball_target_rack：圆盘爪定位失败：{message}')
            return False
        node._cmd_x, node._cmd_y = target[:2]
        return self._sleep(self._alignment_settle_seconds)

    def _release_ball(self) -> bool:
        angle_rad = math.radians(self._release_angle_deg)
        self._logger.info(
            f'26rb_drop_ball_target_rack：向 servo 1 发送释放指令，'
            f'角度={self._release_angle_deg:.1f}°={angle_rad:.4f}rad，'
            f'重复 {self._release_repeat_count} 次')
        for index in range(self._release_repeat_count):
            if self._node.stopped:
                return False
            self._node.set_servo(
                angle_rad, f'rack上方释放球 ({index + 1}/'
                f'{self._release_repeat_count})', servo_id=1)
            if (index + 1 < self._release_repeat_count
                    and not self._sleep(self._release_repeat_period)):
                return False
        return self._sleep(self._release_settle_seconds)

    def execute(self) -> TaskOutcome:
        node = self._node
        node._set_task_phase_light(node.LIGHT_YELLOW, 'rack观测与投球阶段')
        try:
            if not node._ensure_camera_extrinsics():
                return TaskOutcome.failed(
                    '26rb_drop_ball_target_rack.camera_tf', '下视相机 TF 未就绪')
            if node._target_rack_down_class_id is None:
                return TaskOutcome.failed(
                    '26rb_drop_ball_target_rack.mapping',
                    '共享类别映射中没有 target_rack_down')
            approach = TargetRackSearch(node, self._params).execute()
            if not approach:
                return approach
            if not self._observe_rack():
                return TaskOutcome.failed(
                    '26rb_drop_ball_target_rack.observation',
                    f'{self._observe_seconds:.1f}s 观测窗口内未看到rack或任务被中止')
            if not self._flash_green(1, '观测到rack'):
                return TaskOutcome.failed(
                    '26rb_drop_ball_target_rack.stopped', 'rack观测提示被中止')

            # Each SET xy preserves measured depth and yaw in basic_motion.
            if not self._servo_rack():
                return node._fallback_failure_outcome(
                    '26rb_drop_ball_target_rack', 'visual_servo')
            if not self._flash_green(2, 'rack视觉伺服成功'):
                return TaskOutcome.failed(
                    '26rb_drop_ball_target_rack.stopped', '伺服成功提示被中止')
            if not self._align_disc_claw():
                return node._fallback_failure_outcome(
                    '26rb_drop_ball_target_rack', 'alignment', '圆盘爪对准rack失败')
            if not self._release_ball():
                return TaskOutcome.failed(
                    '26rb_drop_ball_target_rack.stopped', '发送释放球指令被中止')
            if not self._flash_green(3, '球释放指令发送完成', restore_phase=False):
                return TaskOutcome.failed(
                    '26rb_drop_ball_target_rack.stopped', '释放完成提示被中止')
            self._logger.info(
                '26rb_drop_ball_target_rack：圆盘爪定位及释放指令流程完成')
            return TaskOutcome.ok()
        finally:
            if node._light_state != node.LIGHT_OFF:
                node.light_off()
