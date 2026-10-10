"""前视寻找rack，下视伺服后依次对准圆盘爪和发夹爪，释放球与环。"""

from __future__ import annotations

import math
import time

import cv2
import numpy as np

from uv_msgs.action import BasicMotion
from uv_task.task_outcome import TaskOutcome
from uv_task.collection_frame_search import TargetRackSearch
from uv_task.down_camera_servo import (
    DownCameraPriority, body_image_step, normalized_image_error, body_to_world_rotation,
)


def validate_ring_release_params(params):
    for key, default in (('release_angle_deg', 90.0), ('ring_release_angle_deg', 270.0)):
        angle = params.get(key, default)
        if (isinstance(angle, bool) or not isinstance(angle, (int, float))
                or not math.isfinite(angle) or not 0 <= angle <= 270.0):
            raise ValueError(f'{key} 必须为[0,270]范围内的有限度数')
    count = params.get('ring_release_repeat_count', 3)
    if isinstance(count, bool) or not isinstance(count, int) or count < 1:
        raise ValueError('ring_release.repeat_count 必须为至少1的整数')
    for key, default in (('ring_release_repeat_period', .1), ('ring_release_settle_seconds', 1.0)):
        value = params.get(key, default)
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or value < 0):
            raise ValueError(key+' 必须为有限非负数')


class RB26DropBallTargetRackTask:
    """Approach the rack with front detections, then use down detections and TF to drop."""

    def __init__(self, node, params: dict):
        validate_ring_release_params(params)
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
            params.get('horizontal_servo_timeout', 20.0))))
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
        self._max_speed = min(0.15, max(
            0.005, float(params.get('down_visual_servo_max_speed_mps', 0.08))))
        self._depth_gain = max(0.05, float(params.get('down_depth_hold_gain', 0.8)))
        self._max_vertical_speed = min(0.15, max(
            0.005, float(params.get('down_depth_hold_max_speed_mps', 0.08))))
        self._depth_tolerance = max(
            0.005, float(params.get('down_depth_hold_tolerance_m', 0.03)))
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
                or not 0 <= self._release_angle_deg <= 270.0):
            raise ValueError('释放舵机角度必须是 [0, 270] 范围内的度数')
        self._release_repeat_count = max(
            1, int(params.get('release_repeat_count', 3)))
        self._release_repeat_period = max(
            0.0, float(params.get('release_repeat_period', 0.1)))
        self._release_settle_seconds = max(
            0.0, float(params.get('release_settle_seconds', 1.0)))
        self._ring_release_angle_deg = float(params.get('ring_release_angle_deg', 270.0))
        self._ring_release_repeat_count = int(params.get('ring_release_repeat_count', 3))
        self._ring_release_repeat_period = float(params.get('ring_release_repeat_period', .1))
        self._ring_release_settle_seconds = float(params.get('ring_release_settle_seconds', 1.0))
        self._rack_camera_pose = None
        self._light_pulse_seconds = max(
            0.05, float(params.get('light_pulse_seconds', 0.35)))
        self._light_gap_seconds = max(
            0.0, float(params.get('light_gap_seconds', 0.25)))
        self._priority = None
        self._servo_depth = None
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

    def _servo_pose(self):
        pose = tuple(float(value) for value in self._node._latest_robot_pose())
        if len(pose) != 6 or not all(math.isfinite(value) for value in pose):
            raise ValueError('投球水平伺服实测位姿无效')
        return pose

    def _rack_velocity(self, camera_name, detection, pose):
        camera = self._node.camera_extrinsics[camera_name]
        du, dv = normalized_image_error(self._node, camera_name, detection)
        vx, vy = body_image_step(camera, du, dv, self._projection_depth,
                                 self._gain, self._max_speed)
        world = body_to_world_rotation(pose) @ np.array([vx, vy, 0.0])
        return world[:2], du, dv

    def _send_rack_velocity(self, horizontal, deadline, yaw_target=None):
        pose = self._servo_pose()
        vz = float(np.clip((self._servo_depth-pose[2])*self._depth_gain,
                           -self._max_vertical_speed, self._max_vertical_speed))
        yaw_error = 0.0 if yaw_target is None else (yaw_target-pose[5]+180.0) % 360.0-180.0
        yaw_rate = float(np.clip(yaw_error*1.2, -10.0, 10.0))
        # Preserve world depth even when body-horizontal axes are tilted.
        body = body_to_world_rotation(pose).T @ np.array([*horizontal, vz])
        success, message = self._node._send_body_velocity(
            *body.tolist(), yaw_rate_deg_s=yaw_rate,
            lease_s=max(0.25, 4*self._period),
            wait_deadline=min(deadline, time.monotonic()+2.0),
            task_context='26rb_drop_ball_target_rack 定深速度伺服',
            light_color=self._node.LIGHT_YELLOW)
        if not success:
            raise RuntimeError(f'rack速度指令失败：{message}')

    def _stop_rack_velocity(self):
        node = self._node
        success, message = node._send_body_velocity(
            wait_deadline=time.monotonic()+2.0,
            task_context='26rb_drop_ball_target_rack 结束速度伺服',
            light_color=node.LIGHT_OFF)
        if not success:
            raise RuntimeError(f'rack停止速度失败：{message}')
        if node.stopped:
            return False
        pose = self._servo_pose()
        target = [pose[0], pose[1], self._servo_depth, pose[5]]
        success, message = node._send_action_goal(
            BasicMotion.Goal.SET, target, 'xyzrz',
            timeout=self._command_timeout,
            wait_deadline=time.monotonic()+self._command_timeout, quiet=True,
            task_context='26rb_drop_ball_target_rack 保持水平位置和锁定深度',
            light_color=node.LIGHT_OFF)
        if not success:
            raise RuntimeError(f'rack停车定深失败：{message}')
        node._cmd_x, node._cmd_y, node._cmd_z, node._cmd_yaw = target
        return True

    def _servo_rack(self) -> bool:
        node = self._node
        self._aligned_camera = None
        self._priority = DownCameraPriority(
            node, node._target_rack_down_class_id,
            priority_seconds=self._priority_seconds,
            detection_timeout=self._detection_timeout,
            label='26rb_drop_ball_target_rack rack伺服')
        deadline = time.monotonic()+self._servo_timeout
        stable_since = None
        stable_generation = None
        last_log = float('-inf')
        aligned_camera = None
        stopped_ok = False
        failure_kind = 'motion'
        failure_message = 'rack定深速度伺服未完成'
        last_camera = None
        timed_out = False
        try:
            if self._servo_depth is None:
                self._servo_depth = self._servo_pose()[2]
            self._logger.info(
                '26rb_drop_ball_target_rack：开始单目定深速度伺服；'
                f'锁定深度={self._servo_depth:.3f}m，水平限速={self._max_speed:.3f}m/s')
            while not node.stopped and time.monotonic() < deadline:
                camera_name, detection = self._priority.update()
                now = time.monotonic()
                if stable_generation != self._priority.generation:
                    stable_since = None
                    stable_generation = self._priority.generation
                horizontal = np.zeros(2)
                pose = self._servo_pose()
                if detection is None:
                    stable_since = None
                    if now-last_log >= 1.0:
                        self._logger.info('rack检测丢失，停止XY速度并继续保持深度')
                        last_log = now
                else:
                    last_camera = camera_name
                    horizontal, du, dv = self._rack_velocity(camera_name, detection, pose)
                    centered = abs(du) <= self._pixel_tolerance and abs(dv) <= self._pixel_tolerance
                    at_depth = abs(pose[2]-self._servo_depth) <= self._depth_tolerance
                    if centered:
                        horizontal = np.zeros(2)
                    if centered and at_depth:
                        stable_since = now if stable_since is None else stable_since
                    else:
                        stable_since = None
                    if now-last_log >= 1.0:
                        self._logger.info(
                            f'rack定深速度伺服，相机={camera_name}，'
                            f'误差=({du:+.4f},{dv:+.4f})，'
                            f'世界水平速度=({horizontal[0]:+.3f},{horizontal[1]:+.3f})m/s，'
                            f'深度={pose[2]:.3f}/{self._servo_depth:.3f}m')
                        last_log = now
                    if stable_since is not None and now-stable_since >= self._stable_seconds:
                        aligned_camera = camera_name
                        break
                self._send_rack_velocity(horizontal, deadline)
                if not self._sleep(min(self._period, max(0.0, deadline-time.monotonic()))):
                    break
            if node.stopped:
                failure_message = 'rack定深速度伺服被中止'
            elif aligned_camera is None:
                failure_kind = 'timeout'
                timed_out = True
                failure_message = 'rack定深速度伺服超时，将按最后位姿继续完成放球放环'
        except (KeyError, ValueError, RuntimeError, cv2.error, np.linalg.LinAlgError) as error:
            failure_message = f'rack定深速度伺服失败：{error}'
        finally:
            try:
                stopped_ok = self._stop_rack_velocity()
            except Exception as error:
                failure_message += f'；停车失败：{error}'
        if not stopped_ok:
            node._last_motion_failure_kind = failure_kind
            node._last_motion_failure_message = failure_message
            self._logger.error(failure_message)
            return False
        if aligned_camera is None:
            aligned_camera = last_camera or next(
                (name for name in node.camera_extrinsics if name.startswith('down_')),
                None)
        if aligned_camera is None:
            node._last_motion_failure_kind = failure_kind
            node._last_motion_failure_message = failure_message + '；没有可用下视相机外参'
            self._logger.error(node._last_motion_failure_message)
            return False
        self._aligned_camera = aligned_camera
        if timed_out:
            self._logger.warning(
                f'{failure_message}；采用相机={aligned_camera}的当前位姿继续外参对准')
        else:
            self._logger.info(f'rack定深速度伺服完成，相机={aligned_camera}')
        return True

    def _flash_green(self, count: int, label: str,
                     restore_phase: bool = True) -> bool:
        node = self._node
        self._logger.info(
            f'26rb_drop_ball_target_rack：{label}，闪 {count} 次绿灯（异步）')
        flash_async = getattr(node, '_flash_task_light', None)
        if not callable(flash_async):
            return not node.stopped
        return flash_async(
            node.LIGHT_GREEN, count, label,
            pulse_seconds=self._light_pulse_seconds,
            gap_seconds=self._light_gap_seconds,
            restore=restore_phase,
            restore_color=node.LIGHT_YELLOW if restore_phase else None)

    def _align_disc_claw(self) -> bool:
        """Save the camera-centred anchor before the first claw translation."""
        self._rack_camera_pose = None
        try:
            camera_pose = self._servo_pose()
        except ValueError as error:
            self._logger.error(f'圆盘爪定位失败：{error}')
            return False
        if not self._align_claw('disc_claw_link', '圆盘爪', camera_pose):
            return False
        self._rack_camera_pose = camera_pose
        return True

    def _align_hairpin_claw(self) -> bool:
        if self._rack_camera_pose is None:
            self._logger.error('发夹爪定位失败：缺少本次rack相机居中位姿')
            return False
        # Use the same rack anchor as ball release. Applying another camera
        # offset to the disc-aligned current pose would shift the drop point.
        return self._align_claw('hairpin_claw_link', '发夹爪', self._rack_camera_pose)

    def _align_claw(self, frame, label, camera_pose) -> bool:
        """Place either claw over the same rack point from a camera-centred anchor."""
        node = self._node
        camera = node.camera_extrinsics.get(self._aligned_camera)
        if camera is None:
            self._logger.error(f'{label}平移失败：没有单目伺服成功的相机')
            return False
        provider = node.camera_extrinsics_provider
        try:
            transform = provider.lookup_transform(
                provider.base_frame, frame)
            message = (transform.transform
                       if hasattr(transform, 'transform') else transform)
            claw_body = np.array([
                float(message.translation.x), float(message.translation.y),
                float(message.translation.z)], dtype=np.float64)
            if not np.all(np.isfinite(claw_body)):
                raise ValueError(f'{label}位置外参包含无效数值')
        except Exception as error:
            self._logger.error(
                f'26rb_drop_ball_target_rack：读取{label} TF失败：{error}')
            return False

        if node.stopped:
            return False

        # Preserve the selected eye's baseline offset; do not substitute the
        # stereo midpoint or use detections again after successful centering.
        body_dx = float(camera.translation[0] - claw_body[0])
        body_dy = float(camera.translation[1] - claw_body[1])
        pose = camera_pose
        yaw = math.radians(pose[5])
        world_dx = math.cos(yaw) * body_dx - math.sin(yaw) * body_dy
        world_dy = math.sin(yaw) * body_dx + math.cos(yaw) * body_dy
        target = [pose[0] + world_dx, pose[1] + world_dy,
                  self._servo_depth if self._servo_depth is not None else float(pose[2]),
                  float(pose[5])]
        self._logger.info(
            f'26rb_drop_ball_target_rack：按 {self._aligned_camera} '
            f'和{label}固定外参平移；'
            f'相机机体系=({camera.translation[0]:+.3f},'
            f'{camera.translation[1]:+.3f},{camera.translation[2]:+.3f})m，'
            f'{label}机体系=({claw_body[0]:+.3f},{claw_body[1]:+.3f},'
            f'{claw_body[2]:+.3f})m，'
            f'机体平移=({body_dx:+.3f},{body_dy:+.3f})m，'
            f'世界平移=({world_dx:+.3f},{world_dy:+.3f})m，'
            f'直接SET目标=({target[0]:.3f},{target[1]:.3f},{target[2]:.3f},{target[3]:.1f})')
        success, message = node._send_action_goal(
            BasicMotion.Goal.SET, target, 'xyzrz',
            timeout=self._alignment_timeout,
            wait_deadline=time.monotonic()+self._alignment_timeout,
            quiet=True, light_color=node.LIGHT_OFF,
            task_context=f'26rb_drop_ball_target_rack：{label}外参直接对准')
        if not success:
            self._logger.error(f'{label}外参直接对准失败：{message}')
            return False
        if node.stopped or not self._sleep(self._alignment_settle_seconds):
            return False
        node._cmd_x, node._cmd_y, node._cmd_z, node._cmd_yaw = target
        return True

    def _release_ball(self) -> bool:
        self._logger.info(
            f'26rb_drop_ball_target_rack：向 servo 1 发送释放指令，'
            f'角度={self._release_angle_deg:.1f}°，'
            f'重复 {self._release_repeat_count} 次')
        for index in range(self._release_repeat_count):
            if self._node.stopped:
                return False
            self._node.set_servo(
                self._release_angle_deg, f'rack上方释放球 ({index + 1}/'
                f'{self._release_repeat_count})', servo_id=1)
            if (index + 1 < self._release_repeat_count
                    and not self._sleep(self._release_repeat_period)):
                return False
        return self._sleep(self._release_settle_seconds)

    def _release_ring(self) -> bool:
        self._logger.info(
            f'26rb_drop_ball_target_rack：向servo 2发送放环指令，'
            f'角度={self._ring_release_angle_deg:.1f}°，重复{self._ring_release_repeat_count}次')
        for index in range(self._ring_release_repeat_count):
            if self._node.stopped:
                return False
            self._node.set_servo(
                self._ring_release_angle_deg,
                f'rack上方释放环 ({index+1}/{self._ring_release_repeat_count})', servo_id=2)
            if (index+1 < self._ring_release_repeat_count
                    and not self._sleep(self._ring_release_repeat_period)):
                return False
        return self._sleep(self._ring_release_settle_seconds)

    def execute(self) -> TaskOutcome:
        self._servo_depth = None
        self._rack_camera_pose = None
        completion_flash_pending = False
        node = self._node
        node._set_task_phase_light(node.LIGHT_YELLOW, 'rack观测与放球放环阶段')
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
            node._set_task_phase_light(node.LIGHT_YELLOW, 'rack下视观测')
            if not self._observe_rack():
                return TaskOutcome.failed(
                    '26rb_drop_ball_target_rack.observation',
                    f'{self._observe_seconds:.1f}s 观测窗口内未看到rack或任务被中止')

            # XY velocity control and subsequent claw alignment share one depth target.
            if not self._servo_rack():
                return node._fallback_failure_outcome(
                    '26rb_drop_ball_target_rack', 'visual_servo')
            if not self._flash_green(1, 'rack视觉伺服成功'):
                return TaskOutcome.failed(
                    '26rb_drop_ball_target_rack.stopped', '伺服成功提示被中止')
            if not self._align_disc_claw():
                return node._fallback_failure_outcome(
                    '26rb_drop_ball_target_rack', 'alignment', '圆盘爪对准rack失败')
            if not self._release_ball():
                return TaskOutcome.failed(
                    '26rb_drop_ball_target_rack.stopped', '发送释放球指令被中止')
            if not self._flash_green(1, '放球完成'):
                return TaskOutcome.failed(
                    '26rb_drop_ball_target_rack.stopped', '放球提示被中止')
            if not self._align_hairpin_claw():
                return node._fallback_failure_outcome(
                    '26rb_drop_ball_target_rack', 'ring_alignment', '发夹爪对准rack失败')
            if not self._release_ring():
                return TaskOutcome.failed(
                    '26rb_drop_ball_target_rack.stopped', '发送释放环指令被中止')
            if not self._flash_green(1, '放环完成', restore_phase=False):
                return TaskOutcome.failed(
                    '26rb_drop_ball_target_rack.stopped', '释放完成提示被中止')
            completion_flash_pending = True
            self._logger.info(
                '26rb_drop_ball_target_rack：圆盘爪放球、发夹爪放环指令流程完成')
            return TaskOutcome.ok()
        finally:
            # Leave the final asynchronous green indication in the queue;
            # never cancel it from task cleanup.
            if not completion_flash_pending and node._light_state != node.LIGHT_OFF:
                node.light_off()
