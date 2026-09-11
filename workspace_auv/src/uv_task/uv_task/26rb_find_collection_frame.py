"""26rb 置物台/target-rack 定位任务入口。"""

from __future__ import annotations

from uv_msgs.action import BasicMotion


class RB26FindCollectionFrameTask:
    """Find and confirm the collection-frame targets, then move above the platform."""

    def __init__(self, node, params: dict):
        self._node = node
        self._params = params

    def execute(self) -> bool:
        node = self._node
        params = self._params
        import time

        timeout = max(5.0, float(params.get('timeout', 120.0)))
        deadline = time.monotonic() + timeout
        platform_name = node._normalize_localizer_target_name(
            params.get('platform_name', 'collection_frame')) \
            or 'collection_frame'
        rack_name = node._normalize_localizer_target_name(
            params.get('rack_name', 'target_rack')) or 'target_rack'
        required = (platform_name, rack_name)
        look_order = [
            node._normalize_localizer_target_name(name)
            for name in params.get('look_order', list(required))
        ]
        look_order = [name for name in look_order if name in required]
        for name in required:
            if name not in look_order:
                look_order.append(name)

        node.get_logger().info(
            'find_collection_frame：等待定位器提供 '
            f'{platform_name} 和 {rack_name} 的位置')
        targets = node._current_localizer_targets(params)
        if not targets:
            if not node._scan_localizer_east_to_south(params, deadline):
                node.get_logger().error(
                    'find_collection_frame：从东向南扫描未找到目标')
                return False

        looked = set()
        while not node.stopped and time.monotonic() < deadline:
            targets = node._current_localizer_targets(params)
            for name in look_order:
                if name in looked or name not in targets:
                    continue
                if not node._look_at_localizer_target(targets[name], params, name):
                    return False
                confirm_deadline = min(
                    deadline,
                    time.monotonic() + max(
                        0.5, float(params.get('confirm_timeout', 5.0))))
                if not node._confirm_localizer_target(
                        name, params, confirm_deadline):
                    node.get_logger().error(
                        f'find_collection_frame：{name} 位置确认失败')
                    return False
                looked.add(name)
                node.get_logger().info(
                    f'find_collection_frame：定位器已确认 {name} 位置')

            if all(name in targets for name in required):
                break
            time.sleep(0.05)

        if node.stopped:
            return False
        targets = node._current_localizer_targets(params)
        if not all(name in targets for name in required):
            missing = [name for name in required if name not in targets]
            node.get_logger().error(
                'find_collection_frame：等待定位器位置超时，'
                f'缺少={missing}')
            return False

        platform = targets[platform_name]
        target_z = max(0.0, float(params.get('collection_depth_m', 0.20)))
        pose = node._latest_robot_pose()
        dx = float(platform['x']) - pose[0]
        dy = float(platform['y']) - pose[1]
        target_yaw = node._cmd_yaw
        if (dx * dx + dy * dy) ** 0.5 > 1e-6:
            import math
            target_yaw = node._wrap_yaw_degrees(math.degrees(math.atan2(dy, dx)))

        node.get_logger().info(
            f'find_collection_frame：两个位置均已确认，移动到置物台上方 '
            f'({platform["x"]:.2f}, {platform["y"]:.2f}, {target_z:.2f})，'
            f'固定在水面下 {target_z:.2f}m，偏航角={target_yaw:.1f}°')
        success, message = node._send_action_goal(
            BasicMotion.Goal.SET,
            [float(platform['x']), float(platform['y']), target_z, target_yaw],
            'xyzrz',
            timeout=max(1.0, float(params.get('move_timeout', 120.0))),
            task_context=node._format_motion_context('移动到置物台上方'))
        if not success:
            node.get_logger().error(
                f'find_collection_frame：移动到置物台上方失败：{message}')
            return False
        node._cmd_x = float(platform['x'])
        node._cmd_y = float(platform['y'])
        node._cmd_z = target_z
        node._cmd_yaw = target_yaw
        node.get_logger().info(
            'find_collection_frame：已到达置物台上方位置')
        return True
