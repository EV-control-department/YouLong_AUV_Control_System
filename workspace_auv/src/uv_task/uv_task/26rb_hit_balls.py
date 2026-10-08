"""26rb 撞球任务入口。

通用 ROS 状态、动作客户端和速度发布器由 TaskRunnerNode 提供；
撞球专属的观测缓存、目标筛选、扫描定位和运动策略由本模块负责。
"""

from __future__ import annotations

import math
import time

import numpy as np
import rclpy

from auv_protocol.topics import MEASUREMENTS
from uv_msgs.action import BasicMotion
from uv_msgs.msg import ObjectMeasurementArray, ObjectTrack
from uv_task.task_outcome import TaskOutcome


class RB26HitBallsTask:
    """Execute the configured suspended-ball impact task.

    Generic localization state and motion primitives remain on
    ``TaskRunnerNode``. Impact-ball observation handling, target selection,
    scanning, alignment, and per-ball workflow live in this module.
    """

    def __init__(self, node, params: dict):
        self._node = node
        self._params = params
        self._impact_ball_observations = {}
        self._impact_ball_positions = {}
        self._indicated_targets = set()
        self._measurements_subscription = node.create_subscription(
            ObjectMeasurementArray, MEASUREMENTS,
            self._measurements_cb, 10)

    def destroy(self):
        """Release the task-specific measurement subscription."""
        subscription = self._measurements_subscription
        self._measurements_subscription = None
        if subscription is not None:
            self._node.destroy_subscription(subscription)

    def _measurements_cb(self, msg: ObjectMeasurementArray):
        """Cache best front rays and positions for configured targets."""
        node = self._node
        values = self._params.get(
            'order', ['impact_ball_blue', 'impact_ball_red'])
        if isinstance(values, (str, int, np.integer)):
            values = [values]
        target_names_by_class = {}
        for value in values:
            name = self._normalize_impact_ball_name(value)
            if name is None:
                continue
            class_id = node._model_mapping.model_class_id(
                name, required=False)
            if class_id is not None:
                target_names_by_class[int(class_id)] = name

        received_at = time.monotonic()
        min_confidence = float(self._params.get('min_confidence', 0.05))
        latest_by_class = {}
        latest_positions_by_class = {}
        for measurement in msg.measurements:
            class_id = int(measurement.class_id)
            if class_id not in target_names_by_class:
                continue
            source = str(measurement.source_camera).strip().lower()
            if not source.startswith('front'):
                continue
            confidence = float(measurement.confidence)
            if not math.isfinite(confidence) or confidence < min_confidence:
                continue

            if measurement.has_position:
                position = (
                    float(measurement.world_x),
                    float(measurement.world_y),
                    float(measurement.world_z),
                )
                if all(math.isfinite(value) for value in position):
                    candidate = {
                        'received_at': received_at,
                        'confidence': confidence,
                        'source': source,
                        'measurement_form': int(measurement.measurement_form),
                        'position': position,
                    }
                    previous = latest_positions_by_class.get(class_id)
                    if (previous is None
                            or confidence > previous['confidence']):
                        latest_positions_by_class[class_id] = candidate

            if not measurement.has_ray:
                continue
            ray_direction = (
                float(measurement.ray_direction_x),
                float(measurement.ray_direction_y),
                float(measurement.ray_direction_z),
            )
            if (not all(math.isfinite(value) for value in ray_direction)
                    or math.hypot(ray_direction[0], ray_direction[1]) <= 1e-6):
                continue
            observation = {
                'received_at': received_at,
                'confidence': confidence,
                'source': source,
                'measurement_form': int(measurement.measurement_form),
                'ray_direction': ray_direction,
            }
            previous = latest_by_class.get(class_id)
            if previous is None or confidence > previous['confidence']:
                latest_by_class[class_id] = observation

        if not latest_by_class and not latest_positions_by_class:
            return
        with node._perception_lock:
            self._impact_ball_observations.update(latest_by_class)
            self._impact_ball_positions.update(latest_positions_by_class)

        for class_id, observation in latest_by_class.items():
            ray_x, ray_y, ray_z = observation['ray_direction']
            node.get_logger().info(
                f'hit_balls：采纳 {target_names_by_class[class_id]} 观测 '
                f'(id={class_id}, camera={observation["source"]}, '
                f'form={observation["measurement_form"]})，'
                f'confidence={observation["confidence"]:.3f}，'
                f'ray_dir=({ray_x:.3f},{ray_y:.3f},{ray_z:.3f})')

    def _normalize_impact_ball_name(self, value):
        """Accept only canonical impact-ball names from the model registry."""
        if not isinstance(value, str):
            return None
        name = value.strip()
        return (name if name in {'impact_ball_blue', 'impact_ball_red'}
                and self._node._model_mapping.model_class_id(
                    name, required=False) is not None else None)

    def _indicate_target_found(self, name: str):
        if name in self._indicated_targets:
            return
        self._indicated_targets.add(name)
        self._node._pulse_task_light(
            self._node.LIGHT_GREEN, f'观测到 {name}', duration=1.0)

    def _impact_ball_order(self, params: dict) -> list[str]:
        """Read the requested canonical detector class-name order."""
        node = self._node
        values = params.get('order', ['impact_ball_blue', 'impact_ball_red'])
        if isinstance(values, (str, int, np.integer)):
            values = [values]

        result = []
        for value in values:
            name = self._normalize_impact_ball_name(value)
            if name is not None and name not in result:
                result.append(name)
        if not result:
            node.get_logger().error(
                'hit_balls：撞球顺序中没有有效目标；请使用 '
                '共享模型映射中存在的 canonical class name')
        return result

    def _latest_impact_ball_observation(
            self, name: str, params: dict, *, after_received=None):
        """Return a front measurement ray and its world-frame yaw."""
        node = self._node
        class_id = node._model_mapping.model_class_id(name, required=False)
        if class_id is None:
            return None
        min_confidence = float(params.get('min_confidence', 0.05))
        with node._perception_lock:
            cached = self._impact_ball_observations.get(int(class_id))
        if cached is None:
            return None
        received_at = float(cached['received_at'])
        max_age = max(0.0, float(params.get(
            'search_observation_max_age', 5.0)))
        too_old = time.monotonic() - received_at > max_age
        if (too_old
                or (after_received is not None
                    and received_at <= float(after_received))
                or cached['confidence'] < min_confidence):
            return None

        ray_x, ray_y, _ray_z = cached['ray_direction']
        if (not math.isfinite(ray_x) or not math.isfinite(ray_y)
                or math.hypot(ray_x, ray_y) <= 1e-6):
            return None
        yaw = math.degrees(math.atan2(ray_y, ray_x))
        return {**cached, 'yaw_deg': node._wrap_yaw_degrees(yaw)}

    def _latest_impact_ball_position(
            self, name: str, params: dict, *, reference_received=None):
        """Return a fresh direct front-camera XYZ position, if available."""
        node = self._node
        class_id = node._model_mapping.model_class_id(name, required=False)
        if class_id is None:
            return None
        with node._perception_lock:
            cached = self._impact_ball_positions.get(int(class_id))
        if cached is None:
            return None
        received_at = float(cached['received_at'])
        max_age = max(0.0, float(params.get(
            'search_observation_max_age', 5.0)))
        if (time.monotonic() - received_at > max_age
                or (reference_received is not None
                    and received_at + 1e-6 < float(reference_received))
                or cached['confidence'] < float(
                    params.get('min_confidence', 0.05))):
            return None
        if not all(math.isfinite(value) for value in cached['position']):
            return None
        return cached

    def _wait_for_impact_ball_observation(
            self, name: str, params: dict, *, timeout: float,
            after_received=None):
        """Wait for a front observation at the current heading."""
        node = self._node
        deadline = time.monotonic() + max(0.0, float(timeout))
        while rclpy.ok() and not node.stopped:
            observation = self._latest_impact_ball_observation(
                name, params, after_received=after_received)
            if observation is not None:
                return observation
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            time.sleep(min(0.05, remaining))
        return None

    def _best_impact_ball_target(self, name: str, params: dict):
        """Choose the best usable estimate for one suspended impact ball."""
        node = self._node
        class_id = node._model_mapping.model_class_id(name)
        min_confidence = float(params.get('min_confidence', 0.05))
        min_observations = int(params.get('min_observations', 1))
        with node._perception_lock:
            tracks = list(node.object_tracks.tracks)

        candidates = []
        for target in tracks:
            target_name = str(
                getattr(target, 'physical_class_name', '') or
                getattr(target, 'class_name', '')).strip().lower()
            if (int(getattr(target, 'class_id', -1)) != class_id
                    and target_name != name):
                continue
            source = str(getattr(target, 'estimate_source', '')).strip().lower()
            if source and not source.startswith('front'):
                continue
            status = int(getattr(
                target, 'status', ObjectTrack.STATUS_TENTATIVE))
            confidence = float(getattr(target, 'confidence', 0.0))
            observations = int(getattr(target, 'measurement_count', 0))
            if (not math.isfinite(confidence)
                    or confidence < min_confidence
                    or observations < min_observations):
                continue
            x = float(target.world_x)
            y = float(target.world_y)
            z = float(target.world_z)
            if not all(math.isfinite(value) for value in (x, y, z)):
                continue
            distance = math.hypot(x - node._cmd_x, y - node._cmd_y)
            candidates.append((
                status != ObjectTrack.STATUS_STABLE,
                distance,
                -confidence,
                {'name': name, 'x': x, 'y': y, 'z': z,
                 'confidence': confidence, 'observations': observations,
                 'source': source or 'tracks'},
            ))

        if not candidates:
            return None
        candidates.sort(key=lambda item: item[:3])
        return candidates[0][3]

    def _wait_for_impact_ball(self, name: str, params: dict):
        """Wait for a usable target estimate while allowing task stop."""
        node = self._node
        timeout = max(0.0, float(params.get('detect_timeout', 30.0)))
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not node.stopped:
            target = self._best_impact_ball_target(name, params)
            if target is not None:
                return target
            if time.monotonic() >= deadline:
                break
            time.sleep(0.1)
        node.get_logger().error(
            f'hit_balls：等待 {name} 目标估计超时（{timeout:.1f}s）')
        return None

    def _rotate_for_impact_scan(self, yaw: float, timeout: float) -> bool:
        """Rotate in place so the front stereo cameras scan their surroundings."""
        node = self._node
        yaw = node._wrap_yaw_degrees(yaw)
        success, message = node._send_action_goal(
            BasicMotion.Goal.SET,
            [node._cmd_x, node._cmd_y, node._cmd_z, yaw],
            'rz',
            timeout=timeout,
            quiet=True,
            task_context=node._format_motion_context('主动旋转扫描撞球目标'),
        )
        if success:
            node._cmd_yaw = yaw
        else:
            node.get_logger().warn(
                f'hit_balls：旋转到 {yaw:.1f}° 进行扫描失败：{message}')
        return success

    def _active_localize_impact_balls(self, order: list[str], params: dict):
        """Search a left/right arc, align to a ray, and localize each ball."""
        node = self._node
        node._set_task_phase_light(
            node.LIGHT_YELLOW, '撞球 15° 旋转与目标搜索阶段')
        step = float(params.get('search_yaw_step_deg', 15.0))
        step = min(180.0, max(5.0, abs(step)))
        settle_time = max(0.0, float(params.get('search_settle_time', 5.0)))
        direction_dwell_time = max(
            0.0, float(params.get('search_direction_dwell_time', 5.0)))
        rotate_timeout = max(
            1.0, float(params.get('search_rotate_timeout', 10.0)))
        search_timeout = max(0.0, float(params.get('search_timeout', 60.0)))
        found = {}
        for name in order:
            search_deadline = time.monotonic() + search_timeout
            start_yaw = node._wrap_yaw_degrees(
                float(node._latest_robot_pose()[5]))
            observation = self._latest_impact_ball_observation(
                name, params)
            if observation is None and settle_time > 0.0:
                observation = self._wait_for_impact_ball_observation(
                    name, params, timeout=settle_time)
            if observation is not None:
                self._indicate_target_found(name)

            if observation is None:
                node.get_logger().info(
                    f'hit_balls：当前方向未看到 {name}，开始左右各 '
                    f'{step:.1f}°搜索')
                for offset_deg in (step, 0.0, -step, 0.0):
                    if (not rclpy.ok() or node.stopped
                            or time.monotonic() >= search_deadline):
                        break
                    heading = node._wrap_yaw_degrees(start_yaw + offset_deg)
                    if not self._rotate_for_impact_scan(
                            heading, rotate_timeout):
                        continue
                    arrived_at = time.monotonic()
                    remaining = max(0.0, search_deadline - arrived_at)
                    observation_timeout = (
                        direction_dwell_time
                        if abs(offset_deg) > 1e-9 else settle_time)
                    observation = self._wait_for_impact_ball_observation(
                        name, params,
                        timeout=min(observation_timeout, remaining))
                    if observation is not None:
                        self._indicate_target_found(name)
                        break
                    node._pulse_task_light(
                        node.LIGHT_RED,
                        f'{name} 在 {offset_deg:+.1f}° 未观测到目标',
                        duration=1.0)

            if observation is None:
                node.set_light(node.LIGHT_RED, f'{name} 搜索失败')
                node.get_logger().error(
                    f'hit_balls：左右各 {step:.1f}°扫描后仍未看到 {name}')
                return None

            ray_yaw = float(observation['yaw_deg'])
            if not self._rotate_for_impact_scan(ray_yaw, rotate_timeout):
                node.get_logger().error(
                    f'hit_balls：无法将艇首对准 {name} 的观测射线')
                return None
            node.get_logger().info(
                f'hit_balls：看到 {name}，观测射线偏航角='
                f'{ray_yaw:.1f}°，保持观察 2 秒')
            observe_deadline = time.monotonic() + 2.0
            while rclpy.ok() and not node.stopped:
                remaining = observe_deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                time.sleep(min(0.1, remaining))
            if not rclpy.ok() or node.stopped:
                return None

            target = self._best_impact_ball_target(name, params)
            if target is None:
                target = self._wait_for_impact_ball(name, params)
            if target is None:
                node.set_light(node.LIGHT_RED, f'{name} 定位失败')
                return None
            found[name] = target
            node.get_logger().info(
                f'hit_balls：{name} 定位完成，位置='
                f'({target["x"]:.2f}, {target["y"]:.2f}, '
                f'{target["z"]:.2f})')

        node.get_logger().info(
            f'hit_balls：搜索与定位完成，已找到={list(found.keys())}')
        return found

    def _travel_to_impact_point(self, x: float, y: float, z: float,
                                timeout: float, label: str,
                                light_color=None) -> bool:
        """Travel in a straight world-frame segment and update command pose."""
        node = self._node
        dx = x - node._cmd_x
        dy = y - node._cmd_y
        if math.hypot(dx, dy) > 1e-6:
            yaw = math.degrees(math.atan2(dy, dx))
        else:
            yaw = node._cmd_yaw
        success, message = node._send_action_goal(
            BasicMotion.Goal.WTRAVEL,
            [x, y, z, yaw], 'xyz', timeout=timeout,
            task_context=node._format_motion_context(label),
            light_color=light_color)
        if success:
            node._cmd_x, node._cmd_y, node._cmd_z = x, y, z
            if math.hypot(dx, dy) > 1e-6:
                node._cmd_yaw = yaw
            node.get_logger().info(
                f'hit_balls：{label} 已完成，当前位置='
                f'({x:.2f}, {y:.2f}, {z:.2f})')
        else:
            node.get_logger().error(f'hit_balls：{label} 执行失败：{message}')
        return success

    def _impact_staging_pose(self, target: dict, staging_distance: float,
                             z_offset: float = 0.0):
        """Calculate a point before the ball and a yaw pointing at the ball."""
        node = self._node
        pose = node._latest_robot_pose()
        dx = float(target['x']) - pose[0]
        dy = float(target['y']) - pose[1]
        distance = math.hypot(dx, dy)
        if distance > 1e-6:
            direction_x, direction_y = dx / distance, dy / distance
            yaw = math.degrees(math.atan2(dy, dx))
        else:
            yaw = float(pose[5])
            yaw_rad = math.radians(yaw)
            direction_x, direction_y = math.cos(yaw_rad), math.sin(yaw_rad)
        return (
            float(target['x']) - direction_x * staging_distance,
            float(target['y']) - direction_y * staging_distance,
            float(target['z']) + z_offset,
            node._wrap_yaw_degrees(yaw),
        )

    def _hold_impact_alignment(self, name: str, params: dict, *,
                               after_received=None):
        """Align yaw from fresh measurement rays while holding position."""
        node = self._node
        duration = max(
            0.0, float(params.get('position_correction_duration', 30.0)))
        period = max(0.05, float(params.get('position_correction_period', 0.20)))
        command_timeout = max(
            0.20, float(params.get('position_correction_command_timeout', 10.0)))
        min_update_yaw_deg = max(
            0.0, float(params.get('position_correction_min_update_deg', 1.0)))

        deadline = time.monotonic() + duration
        last_observation_received = after_received
        last_sent_yaw = None
        while rclpy.ok() and not node.stopped:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            observation = self._wait_for_impact_ball_observation(
                name, params,
                timeout=min(period, remaining),
                after_received=last_observation_received)
            if observation is None:
                continue
            last_observation_received = observation['received_at']
            target_position = self._latest_impact_ball_position(
                name, params, reference_received=last_observation_received)
            if target_position is not None:
                pose = node._latest_robot_pose()
                target_x, target_y, target_z = target_position['position']
                horizontal_distance = math.hypot(
                    target_x - pose[0], target_y - pose[1])
                node.get_logger().info(
                    f'hit_balls：{name} 实时目标位置(odom)='
                    f'({target_x:.3f}, {target_y:.3f}, {target_z:.3f})，'
                    f'水平距离={horizontal_distance:.3f}m，'
                    f'camera={target_position["source"]}，'
                    f'confidence={target_position["confidence"]:.3f}')
            yaw = float(observation['yaw_deg'])
            if (last_sent_yaw is not None
                    and abs(node._wrap_yaw_degrees(
                        yaw - last_sent_yaw)) < min_update_yaw_deg):
                continue
            success, message = node._send_action_goal(
                BasicMotion.Goal.SET,
                [node._cmd_x, node._cmd_y, node._cmd_z, yaw],
                'rz',
                timeout=min(command_timeout, max(0.20, remaining)),
                quiet=True,
                task_context=node._format_motion_context(
                    f'{name} measurement 射线偏航对准'),
                light_color=node.LIGHT_BLUE)
            if success:
                node._cmd_yaw = yaw
                last_sent_yaw = yaw
            elif not rclpy.ok() or node.stopped:
                return False
            else:
                node.get_logger().warning(
                    f'hit_balls：{name} 射线偏航对准失败：{message}')

        return rclpy.ok() and not node.stopped and last_sent_yaw is not None

    def _charge_forward(self, params: dict) -> bool:
        """Run the body-X velocity loop for a fixed short impact charge."""
        node = self._node
        duration = max(0.0, float(params.get('charge_duration', 5.0)))
        speed = max(0.0, float(params.get('charge_speed_mps', 0.15)))
        period = max(0.02, float(params.get('charge_publish_period', 0.05)))
        lease = max(0.25, period * 4.0)
        deadline = time.monotonic() + duration
        while rclpy.ok() and not node.stopped and time.monotonic() < deadline:
            success, message = node._send_body_velocity(
                forward_mps=speed, lease_s=lease,
                task_context=node._format_motion_context('撞球持续前进'))
            if not success:
                node.get_logger().error(f'撞球速度指令发送失败：{message}')
                return False
            time.sleep(min(period, max(0.0, deadline - time.monotonic())))
        node._send_body_velocity(
            lease_s=lease,
            task_context=node._format_motion_context('结束撞球持续前进'))
        return rclpy.ok() and not node.stopped

    def _record_current_impact_pose(self):
        """Snapshot the measured pose after alignment for the return target."""
        pose = self._node._latest_robot_pose()
        return [pose[0], pose[1], pose[2], pose[5]]

    def execute(self) -> TaskOutcome:
        node = self._node
        params = self._params
        order = self._impact_ball_order(params)
        if not order:
            return TaskOutcome.failed(
                '26rb_hit_balls.localization', '没有有效的撞球目标')

        approach_distance = max(
            0.0, float(params.get('approach_distance', 0.5)))
        pass_distance = max(
            0.0, float(params.get('pass_distance', 0.35)))
        min_clearance = max(
            0.0, float(params.get('min_clearance', 0.05)))
        z_offset = float(params.get('z_offset', 0.2))
        approach_timeout = max(
            1.0, float(params.get('approach_timeout', 90.0)))
        hit_timeout = max(1.0, float(params.get('hit_timeout', 45.0)))
        pause = max(0.0, float(params.get('between_balls_pause', 0.5)))

        node.get_logger().info(
            f'hit_balls：撞球顺序={order}，接近距离={approach_distance:.2f}m，'
            f'穿过距离={pass_distance:.2f}m')
        found = {}
        active_localization = bool(params.get('active_localization', True))
        if active_localization:
            found = self._active_localize_impact_balls(order, params)
            if found is None or any(name not in found for name in order):
                return TaskOutcome.failed(
                    '26rb_hit_balls.localization',
                    '左右各 15°扫描后未能完成撞球目标定位')

        impact_mode = str(params.get('impact_mode', '')).strip().lower()
        if impact_mode == 'staged_charge_return':
            if len(order) != 1:
                node.get_logger().error(
                    'hit_balls：分段冲撞返回模式要求恰好配置一个球')
                return TaskOutcome.failed(
                    '26rb_hit_balls.impact',
                    '分段冲撞返回模式要求恰好配置一个球')
            name = order[0]
            target = self._best_impact_ball_target(name, params) or found.get(name)
            if target is None:
                target = self._wait_for_impact_ball(name, params)
            if target is None:
                return TaskOutcome.failed(
                    '26rb_hit_balls.localization',
                    f'无法获得 {name} 的撞球位置')
            self._indicate_target_found(name)
            return self._staged_charge_return(name, target, params)

        for index, name in enumerate(order):
            target = self._best_impact_ball_target(name, params)
            if target is None:
                target = found.get(name)
            if target is None:
                target = self._wait_for_impact_ball(name, params)
            if target is None:
                return TaskOutcome.failed(
                    '26rb_hit_balls.localization',
                    f'无法获得 {name} 的撞球位置')
            self._indicate_target_found(name)

            dx = target['x'] - node._cmd_x
            dy = target['y'] - node._cmd_y
            distance = (dx * dx + dy * dy) ** 0.5
            if distance > 1e-6:
                direction = (dx / distance, dy / distance)
            else:
                import math
                yaw_rad = math.radians(node._cmd_yaw)
                direction = (math.cos(yaw_rad), math.sin(yaw_rad))

            staging_distance = min(
                approach_distance,
                max(0.0, distance - min_clearance),
            )
            staging_x = target['x'] - direction[0] * staging_distance
            staging_y = target['y'] - direction[1] * staging_distance
            hit_z = target['z'] + z_offset
            node.get_logger().info(
                f'hit_balls：[{index + 1}/{len(order)}] {name} '
                f'估计位置=({target["x"]:.2f}, {target["y"]:.2f}, '
                f'{target["z"]:.2f})，置信度={target["confidence"]:.2f}')

            if not self._travel_to_impact_point(
                    staging_x, staging_y, hit_z, approach_timeout,
                    f'{name} approach', light_color=node.LIGHT_BLUE):
                return TaskOutcome.failed(
                    '26rb_hit_balls.approach',
                    f'{name} 接近位置移动失败')

            refreshed = self._best_impact_ball_target(name, params)
            if refreshed is not None:
                target = refreshed
            dx = target['x'] - node._cmd_x
            dy = target['y'] - node._cmd_y
            distance = (dx * dx + dy * dy) ** 0.5
            if distance > 1e-6:
                direction = (dx / distance, dy / distance)
            hit_x = target['x'] + direction[0] * pass_distance
            hit_y = target['y'] + direction[1] * pass_distance
            hit_z = target['z'] + z_offset
            if not self._travel_to_impact_point(
                    hit_x, hit_y, hit_z, hit_timeout,
                    f'{name} impact pass'):
                return TaskOutcome.failed(
                    '26rb_hit_balls.impact',
                    f'{name} 撞击移动失败')
            node.get_logger().info(
                f'hit_balls：{name} 撞击路径已完成')
            if index + 1 < len(order) and pause > 0.0:
                import time
                time.sleep(pause)
        return TaskOutcome.ok()

    def _staged_charge_return(self, name: str, target: dict,
                              params: dict) -> TaskOutcome:
        """Align before one ball, charge through it, then return."""
        node = self._node
        approach_distance = max(
            0.0, float(params.get('approach_distance', 0.5)))
        min_clearance = max(0.0, float(params.get('min_clearance', 0.05)))
        z_offset = float(params.get('z_offset', 0.2))
        approach_timeout = max(
            1.0, float(params.get('approach_timeout', 90.0)))
        return_timeout = max(
            1.0, float(params.get('return_timeout', 60.0)))

        pose = node._latest_robot_pose()
        distance = math.hypot(float(target['x']) - pose[0],
                              float(target['y']) - pose[1])
        effective_distance = min(
            approach_distance,
            max(0.0, distance - min_clearance),
        )
        staging = self._impact_staging_pose(
            target, effective_distance, z_offset)
        node.get_logger().info(
            f'hit_balls：{name} 在目标前方 {effective_distance:.2f}m 处待命，'
            f'位置=({staging[0]:.2f}, {staging[1]:.2f}, {staging[2]:.2f})，'
            f'偏航角={staging[3]:.1f}°')
        if not self._travel_to_impact_point(
                staging[0], staging[1], staging[2], approach_timeout,
                f'{name} {effective_distance:.2f}m staging',
                light_color=node.LIGHT_BLUE):
            return TaskOutcome.failed(
                '26rb_hit_balls.approach', '撞球接近位置移动失败')

        arrived_at = time.monotonic()
        if not self._hold_impact_alignment(
                name, params, after_received=arrived_at):
            return TaskOutcome.failed(
                '26rb_hit_balls.approach', '撞球对准失败')

        recorded_pose = self._record_current_impact_pose()
        node.get_logger().info(
            f'hit_balls：对准完成，已记录位置 '
            f'=({recorded_pose[0]:.2f}, {recorded_pose[1]:.2f}, '
            f'{recorded_pose[2]:.2f}, {recorded_pose[3]:.1f}°)')

        if not self._charge_forward(params):
            return TaskOutcome.failed(
                '26rb_hit_balls.impact', '撞球前冲失败')
        node.get_logger().info(
            f'hit_balls：前向速度冲撞完成，持续 '
            f'{float(params.get("charge_duration", 5.0)):.1f}s')

        success, message = node._send_action_goal(
            BasicMotion.Goal.SET,
            recorded_pose,
            'xyzrz',
            timeout=return_timeout,
            task_context=node._format_motion_context(
                f'{name}撞球后返回记录位置'))
        if not success:
            node.get_logger().error(
                f'hit_balls：返回记录位置失败：{message}')
            return TaskOutcome.failed(
                '26rb_hit_balls.return', message)
        node._cmd_x, node._cmd_y, node._cmd_z, node._cmd_yaw = recorded_pose
        node.get_logger().info(
            'hit_balls：红球任务完成，已返回记录位置')
        return TaskOutcome.ok()
