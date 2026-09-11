"""26rb 丢球/投放任务入口。"""

from __future__ import annotations


class RB26DropBeaconTask:
    """Trigger the configured drop-servo action."""

    def __init__(self, node, params: dict):
        self._node = node
        self._params = params

    def execute(self) -> bool:
        node = self._node
        angle = float(self._params.get('angle_rad', node.ANGLE_DROP_BEACON))
        node.set_servo(angle, '投放信标')
        node.get_logger().info('🔫 信标已投放！')
        return True
