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
        speed = float(params['descent_speed_mps'])
        duration = float(params['descent_duration_seconds'])
        # 不恢复旧的0.3m/s和0.5m行程限幅；现场决定速度、持续时间。
        # 仍拒绝NaN、负速度或零时长，避免反向运动和无法终止的指令。
        if not math.isfinite(speed) or not math.isfinite(duration) or speed <= 0 or duration <= 0:
            raise ValueError('下压速度和时长必须是有限正数')
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
        pickup_deg = float(params.get('pickup_servo_angle_deg', 0.0))
        release_deg = float(params.get('release_servo_angle_deg', 90.0))
        if not math.isclose(pickup_deg, 0.0, abs_tol=0.01) or not math.isclose(release_deg, 90.0, abs_tol=0.01):
            raise ValueError('舵机1固定定义为抓0°、放90°')
        # 配置使用角度；任务内部沿用弧度接口，set_servo出口再转为真机角度协议。
        self._release_angle = math.radians(release_deg)
        self._pickup_angle = math.radians(pickup_deg)
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
        self._ascent_speed = float(params.get('ascent_speed_mps', 0.01))
        self._ascent_period = float(params.get('ascent_publish_period', 0.05))
        self._ascent_tolerance = float(params.get('ascent_tolerance_m', 0.01))
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
        # 真机左目像素伺服必须使用与当前采集模式一致的实测 K，不再沿用
        # 抓球任务由名义 HFOV 推算的焦距。
        cal_width, cal_height, left_k, _, _, _, _, _ = \
            load_real_down_json(real_down_calibration_path())
        if (width, height) != (cal_width, cal_height):
            raise ValueError(
                f'抓海参图像应为每目 {cal_width}x{cal_height}，收到 {width}x{height}')
        self._FX, self._FY = float(left_k[0, 0]), float(left_k[1, 1])
        self._CX, self._CY = float(left_k[0, 2]), float(left_k[1, 2])
        # camera发布的掩膜恢复到原始标定像素，不是监控窗口旋转后的像素。
        # 下视整体物理绕机体yaw倒装180°，原像素误差对应的body x/y均须反向。
        self._camera_mount_yaw = float(params.get('down_camera_mount_yaw_deg', 180.0))
        if not math.isfinite(self._camera_mount_yaw):
            raise ValueError('down_camera_mount_yaw_deg 必须为有限数值')
        self._logger.info(
            f'抓海参：下视安装yaw补偿={self._camera_mount_yaw:g}°；'
            '输入为camera还原后的标定像素，夹爪偏置保持机体系定义')
        self.confirmed_removed = 0
        self.delivery_commands = 0
        self._scan_received_after = None
        self._scan_capture_after_ns = None

    def _horizontal_step(self, detection):
        """像素 → 名义机体水平修正 → 安装yaw补偿 → odom修正。"""
        pose, dx, dy, _, _, du, dv = super()._horizontal_step(detection)
        body_dx, body_dy = self._body_to_world(dx, dy, self._camera_mount_yaw)
        world_dx, world_dy = self._body_to_world(body_dx, body_dy, pose[5])
        return pose, body_dx, body_dy, world_dx, world_dy, du, dv

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
        # 帧率波动时不要求必须收满默认3帧；至少2帧仍保留多帧复核。
        # 不接受旧帧、不用单帧或历史数量猜测当前是否抓到。
        if not self._node.stopped and time.monotonic() < deadline and len(counts) >= 2:
            result = min(counts) if conservative == 'before' else max(counts)
            self._logger.warning(
                f'抓海参：降级使用{len(counts)}个新鲜计数帧={counts}，保守结果={result}')
            return result
        return None

    def _retry_perception(self, reason, failures, deadline):
        """视觉失败只结束本轮，不立刻结束整项任务；仍受重试与总超时约束。"""
        if self._node.stopped or time.monotonic() >= deadline:
            return False
        if failures > self._max_failed_attempts:
            self._logger.error(f'抓海参：{reason}；连续视觉失败已超过重试上限{self._max_failed_attempts}')
            return False
        self._logger.warning(
            f'抓海参：{reason}；不盲目下压，等待新观测后重试 '
            f'{failures}/{self._max_failed_attempts}')
        end = min(deadline, time.monotonic() + 1.0)
        while not self._node.stopped and time.monotonic() < end:
            time.sleep(min(0.05, max(0.0, end-time.monotonic())))
        return not self._node.stopped and time.monotonic() < deadline

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
        # WTRAVEL 水平前进结束时保持行进航向，而非配置中的 rz。
        # 显式水平定位并归向，不能只修改 _cmd_yaw 就当作机器人已归位。
        remaining = motion_deadline - time.monotonic()
        measured = self._measured_pose()
        if remaining <= 0 or measured is None or self._node.stopped:
            return False
        target[2] = measured[2]  # 不用位置环拉升可能携带海参的机器人。
        self._logger.info(f'抓海参：{label}运输到位，开始水平定位及归向：'
                          f'x={target[0]:.3f}，y={target[1]:.3f}，yaw={target[3]:.1f}°')
        success, message = self._node._send_action_goal(
            BasicMotion.Goal.SET, target, 'xyrz', timeout=remaining,
            task_context=self._node._format_motion_context(label + '到位归向'))
        if not success:
            self._logger.error(f'抓海参：{label}到位归向失败：{message}')
            return False
        # 动作结果和实测位姿同时确认；旧指令位姿不能作为返回扫描区的证据。
        while not self._node.stopped and time.monotonic() < motion_deadline:
            measured = self._measured_pose()
            if measured is not None:
                yaw_error = abs((measured[5] - target[3] + 180.0) % 360.0 - 180.0)
                if (abs(measured[0] - target[0]) <= 0.1
                        and abs(measured[1] - target[1]) <= 0.1 and yaw_error <= 5.0):
                    break
            time.sleep(0.05)
        else:
            self._logger.error(f'抓海参：{label}实测位置/航向未归位，禁止释放或重新扫描')
            return False
        self._node._cmd_x, self._node._cmd_y, self._node._cmd_z, self._node._cmd_yaw = target
        self._logger.info(f'抓海参：{label}位置及航向已确认')
        return True

    def _measured_pose(self):
        """上浮只能依据实测位姿，不能使用任务节点的指令位姿兜底。"""
        with self._node._perception_lock:
            pose = self._node._robot_pose
            received = getattr(self._node, '_robot_pose_received', time.monotonic())
        if time.monotonic() - received > 1.0:
            return None
        if pose is None or len(pose) < 6 or not all(math.isfinite(float(v)) for v in pose):
            return None
        return tuple(float(v) for v in pose)

    def _ascend_to_depth(self, target_z, deadline):
        """绕过位置步进环，直接持续发布负body-z速度；不发送向下修正。"""
        pose = self._measured_pose()
        if pose is None:
            self._logger.error('抓海参：没有实测位姿，拒绝盲目上浮')
            return False
        self._logger.info(
            f'抓海参：直接速度上浮，当前z={pose[2]:.3f}m，目标z={target_z:.3f}m，'
            f'速度上限={self._ascent_speed:.3f}m/s；不使用BMOVE位置拉升')
        last_log = float('-inf')
        try:
            while not self._node.stopped and time.monotonic() < deadline:
                pose = self._measured_pose()
                if pose is None:
                    self._logger.error('抓海参：速度上浮期间丢失实测位姿')
                    return False
                remaining = pose[2] - target_z
                if remaining <= self._ascent_tolerance:
                    self._node._cmd_z = pose[2]
                    self._logger.info(f'抓海参：速度上浮到位，实测z={pose[2]:.3f}m；不向下纠偏')
                    return True
                # 距目标越近，指令越缓；不突然从下压切到大幅上浮速度。
                speed = min(self._ascent_speed, max(0.001, remaining * 0.1))
                self._node._publish_body_velocity(vertical_mps=-speed)
                if time.monotonic() - last_log >= 1.0:
                    last_log = time.monotonic()
                    self._logger.info(f'抓海参：上浮z={pose[2]:.3f}m，剩余={remaining:.3f}m，vz={-speed:.4f}m/s')
                time.sleep(min(self._ascent_period, max(0., deadline-time.monotonic())))
            self._logger.error('抓海参：上浮超时或任务被停止')
            return False
        finally:
            # 零速度仍是MCU速度环，不等于原始推进器零推力。
            self._node._publish_body_velocity()

    def _descend(self):
        """一次连续下压，不拆成位置步进；异常/停止时也发送零速度。"""
        self._logger.info(f'抓海参：连续下压，vz={self._descent_speed:g}m/s，'
                          f'持续={self._descent_duration:g}s；行程由现场配置决定')
        deadline = time.monotonic() + self._descent_duration
        try:
            while not self._node.stopped and time.monotonic() < deadline:
                self._node._publish_body_velocity(vertical_mps=self._descent_speed)
                time.sleep(min(self._descent_period, max(0., deadline-time.monotonic())))
            return not self._node.stopped
        finally:
            self._node._publish_body_velocity()

    def _return_to_recorded_pose(self, recorded_pose):
        """先直接低速上浮，再做不含 z 的水平回位。"""
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
        self._logger.info('抓海参：投放结束，返回配置扫描点；返回成功前不启动扫描')
        if not self._travel(search_pose, '返回海参搜索区', deadline,
                            self._search_travel_timeout):
            return False
        self._logger.info('抓海参：已返回扫描点并恢复扫描航向，允许开始下一轮识别')
        return True

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
        perception_failures = 0
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
                perception_failures += 1
                if self._retry_perception('水平对准本轮未完成', perception_failures, deadline):
                    continue
                break
            before = self._count_visible(
                time.monotonic(), self._node.get_clock().now().nanoseconds,
                deadline, 'before')
            if before is None or before == 0:
                perception_failures += 1
                if self._retry_perception('抓前计数缺失或暂未看到目标', perception_failures, deadline):
                    continue
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
                self._logger.warning(
                    '抓海参：抓后计数不确定，但已完成下压和回位；'
                    '先去收集区释放可能携带的海参，不增加确认数量，然后返回复查')
                # 无法确认时不能再带着可能已有的海参重复下压；先清空夹爪。
                if not self._deliver(search_pose, deadline, return_to_search=True):
                    break
                perception_failures += 1
                if self._retry_perception('投放后重新确认目标数量', perception_failures, deadline):
                    continue
                break
            perception_failures = 0
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
