"""海参抓取真机联调任务；通过 config/tasks/grab_sea_cucumber.yaml 配置。

必填：模型类别、实际左目尺寸、抓取区位姿（含扫描深度）、下压速度/时长/最大行程、收集区位姿、
舵机1固定0°抓取、90°释放。没有现场标定值就拒绝初始化，不使用抓球任务的危险默认值。
任务已注册到 task_runner，但不加入默认 robocup_26 任务链。
"""

from __future__ import annotations

from importlib import import_module
import math
import statistics
import time
from types import SimpleNamespace

from uv_msgs.action import BasicMotion
from uv_camera.down_calibration import load_real_down_json, real_down_calibration_path


RB26GrabBallTask = import_module('uv_task.26rb_grab_ball').RB26GrabBallTask


class GrabSeaCucumberTask(RB26GrabBallTask):
    """复用抓球视觉伺服和下压动作，以新鲜下视帧的数量变化复检。"""

    def __init__(self, node, params: dict):
        # 明确要求现场模型和投放机构参数；不能用抓球模型的默认类别误抓。
        required = (
            'sea_cucumber_class_id', 'image_width', 'image_height',
            'gripper_offset_x_m', 'gripper_offset_y_m',
            'descent_speed_mps', 'descent_duration_seconds',
            'max_press_distance_m', 'drop_pose', 'search_pose')
        missing = [key for key in required if key not in params]
        if missing:
            raise ValueError(
                f'抓海参缺少必填参数：{", ".join(missing)}；'
                '请在任务 YAML 或 params 中填写现场标定值')
        max_press_distance = float(params['max_press_distance_m'])
        speed = float(params['descent_speed_mps'])
        duration = float(params['descent_duration_seconds'])
        if (not all(map(math.isfinite, (speed, duration, max_press_distance)))
                or not 0 < speed <= 0.3
                or duration <= 0
                or not 0 < max_press_distance <= 0.5
                or speed * duration > max_press_distance):
            raise ValueError(
                f'下压配置不安全：速度={speed:g}m/s，时长={duration:g}s，'
                f'预计行程={speed * duration:g}m，上限={max_press_distance:g}m；'
                '要求速度≤0.3m/s、行程上限≤0.5m且预计行程不超过上限，请按实机标定')
        super().__init__(node, params)
        self._class_id = int(params['sea_cucumber_class_id'])
        if self._class_id < 0:
            raise ValueError('sea_cucumber_class_id 必须非负')
        self._color = '海参'
        self._search_pose = tuple(float(v) for v in params['search_pose'])
        if (len(self._search_pose) != 4
                or not all(map(math.isfinite, self._search_pose))):
            raise ValueError('search_pose 必须是有限数值 [x, y, z, yaw_deg]，使用 START 后的 odom 坐标')
        if self._search_pose[2] < 0:
            raise ValueError('search_pose 中的 z 必须是非负扫描目标深度，单位米、向下为正')
        self._search_travel_timeout = float(params.get('search_travel_timeout_seconds', 90.0))
        if not math.isfinite(self._search_travel_timeout) or self._search_travel_timeout <= 0:
            raise ValueError('search_travel_timeout_seconds 必须是有限的正数')
        self._drop_pose = tuple(float(v) for v in params['drop_pose'])
        if len(self._drop_pose) != 4 or not all(map(math.isfinite, self._drop_pose)):
            raise ValueError('drop_pose 必须是有限数值 [x, y, z, yaw_deg]')
        self._release_angle = math.pi / 2
        self._pickup_angle = 0.0
        # 兼容旧配置，但拒绝与实机定义相反的角度，不允许静默改变抓放方向。
        for key, expected in (('pickup_servo_angle_rad', self._pickup_angle),
                              ('release_servo_angle_rad', self._release_angle)):
            if key in params and not math.isclose(float(params[key]), expected, abs_tol=0.01):
                raise ValueError(f'{key} 与舵机1固定定义冲突：抓0 rad、放π/2 rad')
        self._total_timeout = float(params.get('total_timeout_seconds', 600.0))
        self._target_count = int(params.get('expected_count', 5))
        self._count_frames = int(params.get('count_frames', 3))
        self._count_timeout = float(params.get('count_timeout_seconds', 12.0))
        self._min_confidence = float(params.get('min_confidence', 0.35))
        self._drop_timeout = float(params.get('drop_timeout_seconds', 90.0))
        self._release_wait = float(params.get('release_wait_seconds', 2.0))
        self._ascent_step = float(params.get('ascent_step_m', 0.03))
        self._ascent_step_timeout = float(params.get('ascent_step_timeout_seconds', 8.0))
        self._ascent_pause = float(params.get('ascent_pause_seconds', 0.5))
        self._ascent_tolerance = float(params.get('ascent_tolerance_m', 0.015))
        self._max_failed_attempts = int(params.get('max_failed_attempts', 3))
        self._pixel_tolerance = float(params.get('pixel_tolerance_fraction', 0.035))
        width = float(params.get('image_width', 640.0))
        height = float(params.get('image_height', 480.0))
        if (width <= 0 or height <= 0 or self._target_count <= 0
                or self._count_frames < 2 or self._count_timeout <= 0
                or not math.isfinite(self._total_timeout)
                or self._total_timeout <= 0 or self._drop_timeout <= 0
                or self._max_failed_attempts < 0 or not 0 < self._min_confidence <= 1):
            raise ValueError('抓海参视觉、计数或超时参数无效')
        if (not 0.01 <= self._ascent_step <= 0.05
                or not 0 < self._ascent_tolerance < self._ascent_step
                or self._ascent_step_timeout <= 0
                or not 0 <= self._ascent_pause <= 5.0):
            raise ValueError('抓海参上浮步长、容差或等待参数无效')
        # 真机左目像素伺服必须使用与当前采集模式一致的实测 K，不再沿用
        # 抓球任务由名义 HFOV 推算的焦距。
        cal_width, cal_height, left_k, _, _, _, _, _ = \
            load_real_down_json(real_down_calibration_path())
        if (width, height) != (cal_width, cal_height):
            raise ValueError(
                f'抓海参图像应为每目 {cal_width}x{cal_height}，收到 {width}x{height}')
        self._FX, self._FY = float(left_k[0, 0]), float(left_k[1, 1])
        self._CX, self._CY = float(left_k[0, 2]), float(left_k[1, 2])
        self.confirmed_removed = 0
        self.delivery_commands = 0
        self._scan_received_after = None
        self._scan_capture_after_ns = None

    def _segmented_detections(self, message):
        return [d for d in getattr(message, 'detections', ())
                if int(getattr(d, 'class_id', -1)) == self._class_id
                and float(getattr(d, 'confidence', 0.0)) >= self._min_confidence
                and len(getattr(d, 'mask_x', ())) >= 3
                and len(getattr(d, 'mask_x', ())) == len(getattr(d, 'mask_y', ()))]

    def _best_left_detection(self):
        """选择最大可信掩膜，并用掩膜中心而非 bbox 中心压准目标。"""
        with self._node._perception_lock:
            entry = self._node._down_detections.get('down_left')
        if entry is None or time.monotonic() - entry[0] > self._detection_timeout:
            return None
        # 前往抓取区或返回后的扫描，只采用扫描开始后收到且拍摄的图像。
        received_after = getattr(self, '_scan_received_after', None)
        if received_after is not None and entry[0] <= received_after:
            return None
        capture_after_ns = getattr(self, '_scan_capture_after_ns', None)
        if capture_after_ns is not None:
            stamp = getattr(getattr(entry[1], 'header', None), 'stamp', None)
            stamp_ns = (int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
                        if stamp is not None else 0)
            if stamp_ns <= capture_after_ns:
                return None
        candidates = self._segmented_detections(entry[1])
        if not candidates:
            return None
        target = max(candidates, key=lambda d: (
            float(d.confidence), len(d.mask_x)))
        return SimpleNamespace(
            pixel_x=statistics.median(target.mask_x),
            pixel_y=statistics.median(target.mask_y),
            confidence=target.confidence,
        )

    def _count_visible(self, since: float, capture_after_ns: int,
                       deadline: float, conservative: str):
        """只数下视左目中不同的、动作完成后的新鲜分割帧。"""
        counts = []
        last_stamp = since
        end = min(deadline, time.monotonic() + self._count_timeout)
        while not self._node.stopped and time.monotonic() < end:
            with self._node._perception_lock:
                entry = self._node._down_detections.get('down_left')
            if entry is not None and entry[0] > last_stamp:
                last_stamp = entry[0]
                stamp = getattr(getattr(entry[1], 'header', None), 'stamp', None)
                stamp_ns = (int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
                            if stamp is not None else 0)
                if (stamp_ns > capture_after_ns
                        and time.monotonic() - last_stamp <= self._detection_timeout):
                    counts.append(len(self._segmented_detections(entry[1])))
                    if len(counts) >= self._count_frames:
                        # 抓前取最小、抓后取最大：偶发漏检不会假报抓取成功。
                        result = min(counts) if conservative == 'before' else max(counts)
                        self._logger.info(
                            f'抓海参：{conservative}计数帧={counts}，保守结果={result}')
                        return result
            time.sleep(min(0.1, max(0.0, end - time.monotonic())))
        self._logger.warning(f'抓海参：{conservative}计数缺少新鲜帧，已收{len(counts)}帧')
        return None

    def _travel(self, pose, label, deadline, timeout):
        motion_deadline = min(deadline, time.monotonic() + timeout)
        measured = self._measured_pose()
        if measured is None:
            self._logger.error(f'抓海参：{label}缺少实测位姿，拒绝移动')
            return False
        target = list(pose)
        if float(target[2]) < measured[2] - self._ascent_tolerance:
            if not self._ascend_to_depth(float(target[2]), motion_deadline):
                return False
            measured = self._measured_pose()
            if measured is None:
                return False
        if float(target[2]) < measured[2]:
            # WTRAVEL 不再发出向上的 z 目标；保留上浮后实测深度。
            target[2] = measured[2]
        remaining = motion_deadline - time.monotonic()
        if remaining <= 0 or self._node.stopped:
            return False
        success, message = self._node._send_action_goal(
            BasicMotion.Goal.WTRAVEL, target, 'xyzrz',
            timeout=remaining,
            task_context=self._node._format_motion_context(label))
        if not success:
            self._logger.error(f'抓海参：{label}失败：{message}')
            return False
        self._node._cmd_x, self._node._cmd_y, self._node._cmd_z, self._node._cmd_yaw = target
        return True

    def _measured_pose(self):
        """上浮只能依据实测位姿，不能使用任务节点的指令位姿兜底。"""
        with self._node._perception_lock:
            pose = self._node._robot_pose
        if pose is None or len(pose) < 6 or not all(math.isfinite(float(v)) for v in pose):
            return None
        return tuple(float(v) for v in pose)

    def _ascend_to_depth(self, target_z, deadline):
        """对所有上浮路径使用小幅 z-only BMOVE，等待每步实测到位。"""
        pose = self._measured_pose()
        if pose is None:
            self._logger.error('抓海参：没有实测位姿，拒绝盲目上浮')
            return False
        self._logger.info(
            f'抓海参：开始慢速步进上浮，当前z={pose[2]:.3f}m，'
            f'目标z={target_z:.3f}m，单步≤{self._ascent_step:.3f}m')

        # z 正方向向下。BMOVE 本身没有推力/速度限制，必须等实测深度
        # 达到当前小步目标后才允许发送下一步，避免指令在容差内快速累积。
        while not self._node.stopped and time.monotonic() < deadline:
            pose = self._measured_pose()
            if pose is None:
                self._logger.error('抓海参：上浮期间丢失实测位姿')
                return False
            current_z = pose[2]
            if current_z < target_z - self._ascent_step:
                self._logger.error('抓海参：上浮超出目标深度，停止后续步进')
                return False
            if current_z <= target_z + self._ascent_tolerance:
                break
            step_target_z = max(target_z, current_z - self._ascent_step)
            step_deadline = min(deadline, time.monotonic() + self._ascent_step_timeout)
            success, message = self._node._send_action_goal(
                BasicMotion.Goal.BMOVE,
                [0.0, 0.0, step_target_z - current_z, 0.0], 'z',
                timeout=max(0.1, step_deadline - time.monotonic()),
                task_context=self._node._format_motion_context('抓海参后小步上浮'))
            if not success:
                self._logger.error(f'抓海参：上浮 BMOVE 失败：{message}')
                return False
            while not self._node.stopped and time.monotonic() < step_deadline:
                pose = self._measured_pose()
                if pose is None:
                    self._logger.error('抓海参：等待上浮到位时丢失实测位姿')
                    return False
                if pose[2] <= step_target_z + self._ascent_tolerance:
                    break
                time.sleep(min(0.05, max(0.0, step_deadline - time.monotonic())))
            else:
                self._logger.error(
                    f'抓海参：单步上浮未到位，目标z={step_target_z:.3f}m；'
                    '停止发送后续上浮指令')
                return False
            self._logger.info(
                f'抓海参：上浮单步到位，实测z={pose[2]:.3f}m，'
                f'目标z={target_z:.3f}m')
            pause_end = min(deadline, time.monotonic() + self._ascent_pause)
            while not self._node.stopped and time.monotonic() < pause_end:
                time.sleep(min(0.05, pause_end - time.monotonic()))
        else:
            self._logger.error('抓海参：上浮超时或任务被停止')
            return False
        return True

    def _return_to_recorded_pose(self, recorded_pose):
        """先用 z-only BMOVE 小步上浮，再做不含 z 的水平回位。"""
        if len(recorded_pose) != 4 or not all(math.isfinite(float(v)) for v in recorded_pose):
            self._logger.error('抓海参：记录位姿无效，拒绝上浮')
            return False
        deadline = time.monotonic() + self._return_timeout
        if not self._ascend_to_depth(float(recorded_pose[2]), deadline):
            return False

        # SET 只控制水平位置和航向，z 保持当前实测值，不再一次性拉升。
        pose = self._measured_pose()
        if pose is None or self._node.stopped or time.monotonic() >= deadline:
            return False
        target = [float(recorded_pose[0]), float(recorded_pose[1]),
                  pose[2], float(recorded_pose[3])]
        success, message = self._node._send_action_goal(
            BasicMotion.Goal.SET, target, 'xyrz',
            timeout=max(0.1, deadline - time.monotonic()),
            task_context=self._node._format_motion_context('抓海参后水平回位'))
        if not success:
            self._logger.error(f'抓海参：上浮后水平回位失败：{message}')
            return False
        self._node._cmd_x, self._node._cmd_y = target[:2]
        self._node._cmd_z, self._node._cmd_yaw = target[2:]
        return True

    def _deliver(self, search_pose, deadline, return_to_search):
        if not self._travel(self._drop_pose, '海参运往收集区', deadline, self._drop_timeout):
            return False
        self._node.set_servo(self._release_angle, '海参收集区释放')
        end = min(deadline, time.monotonic() + self._release_wait)
        while not self._node.stopped and time.monotonic() < end:
            time.sleep(min(0.05, end - time.monotonic()))
        if self._node.stopped or time.monotonic() >= deadline:
            return False
        self.delivery_commands += 1
        self._logger.info('抓海参：已发释放指令；实际投放仍须靠视觉/现场验证')
        if not return_to_search:
            return True
        return self._travel(search_pose, '返回海参搜索区', deadline, self._drop_timeout)

    def execute(self) -> bool:
        deadline = time.monotonic() + self._total_timeout
        pose = self._measured_pose()
        if pose is None:
            self._logger.error('抓海参：前往抓取区缺少实测位姿，停止任务')
            return False
        # 四个分量均来自配置；x/y/z 已是 START 后 odom 系目标，不能再加上当前位姿。
        search_pose = self._search_pose
        self._logger.info(
            f'抓海参：先前往抓取区 ({search_pose[0]:.3f}, {search_pose[1]:.3f})m，'
            f'扫描目标深度={search_pose[2]:.3f}m，配置航向={search_pose[3]:.1f}°')
        if not self._travel(search_pose, '前往海参抓取区并调整扫描深度',
                            deadline, self._search_travel_timeout):
            self._logger.error('抓海参：未到达抓取区，停止扫描和抓取')
            return False
        failed_attempts = 0
        while not self._node.stopped and time.monotonic() < deadline:
            if self.confirmed_removed >= self._target_count:
                self._logger.info(f'抓海参：已确认减少 {self.confirmed_removed} 只')
                return True
            self._scan_received_after = time.monotonic()
            self._scan_capture_after_ns = self._node.get_clock().now().nanoseconds
            self._node.set_servo(self._pickup_angle, '海参抓取准备')
            self._servo_timeout = min(
                self._servo_timeout, max(0.1, deadline - time.monotonic()))
            recorded_pose = self._servo_horizontally()
            if recorded_pose is None:
                break
            before = self._count_visible(
                time.monotonic(), self._node.get_clock().now().nanoseconds,
                deadline, 'before')
            if before is None or before == 0:
                self._logger.error('抓海参：观察位无可信目标数量，停止下压')
                break
            if not self._apply_gripper_offset() or not self._wait_pre_descent_settle():
                break
            if time.monotonic() >= deadline:
                break
            self._return_timeout = min(
                self._return_timeout, max(0.1, deadline - time.monotonic()))
            self._descent_duration = min(self._descent_duration, deadline - time.monotonic())
            if not self._descend() or not self._return_to_recorded_pose(recorded_pose):
                break
            after = self._count_visible(
                time.monotonic(), self._node.get_clock().now().nanoseconds,
                deadline, 'after')
            if after is None:
                self._logger.error('抓海参：抓后无法可靠计数，停止以避免误送')
                break
            removed = max(0, before - after)
            if not removed:
                failed_attempts += 1
                self._logger.warning(
                    f'抓海参：数量未减少（{before}→{after}），'
                    f'重试 {failed_attempts}/{self._max_failed_attempts}')
                if failed_attempts > self._max_failed_attempts:
                    break
                continue
            failed_attempts = 0
            self.confirmed_removed += removed
            self._logger.info(f'抓海参：观察位数量 {before}→{after}，本次可能抓起 {removed} 只')
            if not self._deliver(
                    search_pose, deadline,
                    return_to_search=self.confirmed_removed < self._target_count):
                break
            if self.confirmed_removed >= self._target_count:
                self._logger.info(
                    f'抓海参：视觉确认减少 {self.confirmed_removed} 只，'
                    f'已发投放指令 {self.delivery_commands} 次')
                return True
        self._logger.error(
            f'抓海参：任务停止；视觉确认减少={self.confirmed_removed}/'
            f'{self._target_count}，已发投放指令={self.delivery_commands}')
        return False
