#!/usr/bin/env python3
"""DDS-only 3-D viewer for the mapping task.

Run after sourcing the ROS 2 and workspace environments:
    python3 visualization/mapping_visualizer.py
"""

import json
import threading

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


class MappingData(Node):
    def __init__(self):
        super().__init__('mapping_visualizer')
        self.lock = threading.Lock()
        self.payload = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/task/mapping/map', self._map_cb, qos)

    def _map_cb(self, message):
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                return
            with self.lock:
                self.payload = payload
        except json.JSONDecodeError as error:
            self.get_logger().warning(f'invalid mapping JSON: {error}')


def main():
    rclpy.init()
    node = MappingData()
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    figure = plt.figure('YouLong mapping DDS viewer')
    axis = figure.add_subplot(111, projection='3d')
    axis.set_xlabel('x / m')
    axis.set_ylabel('y / m')
    axis.set_zlabel('-z / m')
    axis.set_title('waiting for /task/mapping/map')

    def redraw(_frame):
        with node.lock:
            payload = node.payload
        if payload is None:
            return
        grid = payload.get('grid', {})
        center = grid.get('center', [0.0, 0.0, 0.0])
        side = float(grid.get('side_m', 2.0))
        cells = payload.get('cells', [])
        axis.clear()
        axis.set_xlabel('x / m')
        axis.set_ylabel('y / m')
        axis.set_zlabel('-z / m')
        axis.set_title(f"mapping state: {payload.get('state', 'unknown')}")

        for cell in cells:
            x, y, z = cell['center']
            color = '#777777'
            if cell.get('label') == 'square_cone':
                color = '#e67e22'
            elif cell.get('label') == 'round_cone':
                color = '#3498db'
            elif cell.get('visited'):
                color = '#aaaaaa'
            axis.scatter([x], [y], [-z], marker='s', s=80,
                         facecolors='none', edgecolors=color)
            axis.text(x, y, -z, str(cell['id']), color=color)
            if cell.get('position'):
                px, py, pz = cell['position']
                axis.scatter([px], [py], [-pz], color=color, s=55)
                axis.plot([x, px], [y, py], [-z, -pz], color=color, linewidth=1)

        tag = payload.get('tag')
        if tag and tag.get('position'):
            x, y, z = tag['position']
            axis.scatter([x], [y], [-z], marker='*', s=180,
                         color='#f1c40f', label='tag')

        axis.set_xlim(center[0] - side, center[0] + side)
        axis.set_ylim(center[1] - side, center[1] + side)
        axis.set_zlim(-float(center[2]) - 1.0, -float(center[2]) + 1.0)
        axis.legend(loc='upper left')

    animation = FuncAnimation(figure, redraw, interval=500, cache_frame_data=False)
    try:
        plt.show()
    finally:
        del animation
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
