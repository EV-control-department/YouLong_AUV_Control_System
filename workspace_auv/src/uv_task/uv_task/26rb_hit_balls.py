"""26rb 撞球任务入口。

具体的 ROS 感知缓存、动作客户端和速度发布器由 TaskRunnerNode 提供；
本模块只负责把撞球流程作为一个独立任务对象暴露出来。
"""

from __future__ import annotations

import math

from uv_msgs.action import BasicMotion


class RB26HitBallsTask:
    """Execute the configured suspended-ball impact task.

    The low-level target-cache and motion primitives remain shared services on
    ``TaskRunnerNode``.  The ordering, mode selection, and per-ball workflow
    live here so this module is the owner of the actual task behaviour.
    """

    def __init__(self, node, params: dict):
        self._node = node
        self._params = params

    def execute(self) -> bool:
        node = self._node
        params = self._params
        order = node._impact_ball_order(params)
        if not order:
            return False

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
            found = node._active_localize_impact_balls(order, params)
            # Complete localization before starting the impact run.
            for name in order:
                if name in found:
                    continue
                node.get_logger().info(
                    f'hit_balls：等待完成 {name} 的定位')
                target = node._wait_for_impact_ball(name, params)
                if target is None:
                    return False
                found[name] = target

        impact_mode = str(params.get('impact_mode', '')).strip().lower()
        if impact_mode == 'staged_charge_return':
            if len(order) != 1:
                node.get_logger().error(
                    'hit_balls：分段冲撞返回模式要求恰好配置一个球')
                return False
            name = order[0]
            target = node._best_impact_ball_target(name, params) or found.get(name)
            if target is None:
                target = node._wait_for_impact_ball(name, params)
            if target is None:
                return False
            return self._staged_charge_return(name, target, params)

        for index, name in enumerate(order):
            target = node._best_impact_ball_target(name, params)
            if target is None:
                target = found.get(name)
            if target is None:
                target = node._wait_for_impact_ball(name, params)
            if target is None:
                return False

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

            if not node._travel_to_impact_point(
                    staging_x, staging_y, hit_z, approach_timeout,
                    f'{name} approach'):
                return False

            refreshed = node._best_impact_ball_target(name, params)
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
            if not node._travel_to_impact_point(
                    hit_x, hit_y, hit_z, hit_timeout,
                    f'{name} impact pass'):
                return False
            node.get_logger().info(
                f'hit_balls：{name} 撞击路径已完成')
            if index + 1 < len(order) and pause > 0.0:
                import time
                time.sleep(pause)
        return True

    def _staged_charge_return(self, name: str, target: dict,
                              params: dict) -> bool:
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
        staging = node._impact_staging_pose(
            target, effective_distance, z_offset)
        node.get_logger().info(
            f'hit_balls：{name} 在目标前方 {effective_distance:.2f}m 处待命，'
            f'位置=({staging[0]:.2f}, {staging[1]:.2f}, {staging[2]:.2f})，'
            f'偏航角={staging[3]:.1f}°')
        if not node._travel_to_impact_point(
                staging[0], staging[1], staging[2], approach_timeout,
                f'{name} {effective_distance:.2f}m staging'):
            return False

        refreshed = node._best_impact_ball_target(name, params)
        if refreshed is not None:
            target = refreshed
        if not node._hold_impact_alignment(name, target, params):
            return False

        recorded_pose = node._record_current_impact_pose()
        node.get_logger().info(
            f'hit_balls：对准完成，已记录位置 '
            f'=({recorded_pose[0]:.2f}, {recorded_pose[1]:.2f}, '
            f'{recorded_pose[2]:.2f}, {recorded_pose[3]:.1f}°)')

        if not node._charge_forward(params):
            return False
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
            return False
        node._cmd_x, node._cmd_y, node._cmd_z, node._cmd_yaw = recorded_pose
        node.get_logger().info(
            'hit_balls：红球任务完成，已返回记录位置')
        return True
