"""Small, readable trajectory/map/log indexes and non-actuating replay plots."""

import bisect
import json
import math
import threading
import time
from pathlib import Path

import cv2
import numpy as np


class TelemetryRecorder:
    """Write incrementally; never retain an entire session in RAM."""

    def __init__(self, root):
        self.root = Path(root)
        self.files = {}
        self.last = {}
        self.stop_event = threading.Event()
        self.thread = None
        self.node = None
        self.owns_context = False

    def start(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from rcl_interfaces.msg import Log
        from std_msgs.msg import String
        from uv_msgs.msg import PoseInfo

        self.rclpy = rclpy
        if not rclpy.ok():
            rclpy.init(args=[])
            self.owns_context = True
        self.node = rclpy.create_node('uv_record_telemetry')
        for kind, relative in (('pose', 'metadata/trajectory.jsonl'),
                               ('map', 'metadata/mapping.jsonl'), ('log', 'logs/rosout.jsonl')):
            self.files[kind] = (self.root / relative).open('a', encoding='utf-8', buffering=1)
        for topic in ('/basic_motion/pose_info', '/auv/basic_motion/pose_info'):
            self.node.create_subscription(PoseInfo, topic, self._pose, qos_profile_sensor_data)
        self.node.create_subscription(String, '/task/mapping/map', self._map, qos_profile_sensor_data)
        self.node.create_subscription(Log, '/rosout', self._log, qos_profile_sensor_data)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self._spin, name='uv-record-telemetry', daemon=True)
        self.thread.start()

    def _write(self, kind, value, rate=0):
        now = time.monotonic()
        if rate and now - self.last.get(kind, float('-inf')) < 1.0/rate:
            return
        self.last[kind] = now
        value['receive_time_unix_ns'] = time.time_ns()
        value['timeline_ns'] = self.node.get_clock().now().nanoseconds
        self.files[kind].write(json.dumps(value, ensure_ascii=False, allow_nan=False)+'\n')

    def _pose(self, msg):
        stamp = int(msg.stamp.sec)*1_000_000_000 + int(msg.stamp.nanosec)
        pose = {axis: float(getattr(msg, 'robot_'+axis)) for axis in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')}
        if all(math.isfinite(value) for value in pose.values()):
            self._write('pose', {'stamp_ns': stamp, 'pose': pose}, rate=5)

    def _map(self, msg):
        try:
            value = json.loads(msg.data)
            # Full evidence remains in rosbag. Viewer snapshots retain only
            # the fused map, not every point in the ever-growing observation pool.
            cells = value.get('cells', {})
            items = cells.items() if isinstance(cells, dict) else enumerate(cells)
            compact = {}
            for key, cell in items:
                if isinstance(cell, dict):
                    compact[str(key)] = {k: v for k, v in cell.items()
                                         if k not in ('raw_observations', 'observations', 'points', 'raw_points')}
                    if 'measurements' in compact[str(key)]:
                        compact[str(key)]['measurements'] = cell['measurements'][-20:]
            value['cells'] = compact
            value.pop('events', None)
            value.pop('measurement_points', None)
            self._write('map', value, rate=1)
        except (ValueError, TypeError):
            pass

    def _log(self, msg):
        self._write('log', {'stamp_ns': int(msg.stamp.sec)*1_000_000_000+int(msg.stamp.nanosec),
                            'name': msg.name, 'level': int(msg.level), 'message': msg.msg})

    def _spin(self):
        try:
            while not self.stop_event.is_set() and self.rclpy.ok():
                self.executor.spin_once(timeout_sec=0.1)
        except Exception as error:
            print('uv_record: telemetry stopped: '+str(error), flush=True)

    def stop(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=2.0)
        if self.node:
            if getattr(self, 'executor', None) is not None:
                self.executor.remove_node(self.node)
            self.node.destroy_node()
        for handle in self.files.values():
            handle.close()
        if self.owns_context and self.rclpy.ok():
            self.rclpy.shutdown()


class ReplayTelemetry:
    """Disk-offset indexes: map payloads/logs are loaded only when displayed."""

    def __init__(self, root):
        root = Path(root)
        self.indexes = {}
        for kind, relative in (('pose', 'metadata/trajectory.jsonl'),
                               ('map', 'metadata/mapping.jsonl'), ('log', 'logs/rosout.jsonl')):
            path = root / relative
            entries = []
            if path.is_file():
                with path.open('rb') as stream:
                    while True:
                        offset = stream.tell()
                        line = stream.readline()
                        if not line:
                            break
                        try:
                            value = json.loads(line)
                            entries.append((int(value.get('receive_time_unix_ns', 0)), offset))
                        except (ValueError, TypeError):
                            continue
            entries.sort()
            self.indexes[kind] = (path, [entry[0] for entry in entries], [entry[1] for entry in entries])

    def at(self, kind, timestamp):
        path, times, offsets = self.indexes[kind]
        index = bisect.bisect_right(times, timestamp)-1
        if index < 0:
            return None
        with path.open('rb') as stream:
            stream.seek(offsets[index])
            return json.loads(stream.readline())

    def render(self, timestamp):
        canvas = np.full((480, 640, 3), 28, np.uint8)
        path, times, offsets = self.indexes['pose']
        count = bisect.bisect_right(times, timestamp)
        poses = []
        if count:
            with path.open('rb') as stream:
                for index in range(0, count, max(1, count//1200)):
                    stream.seek(offsets[index])
                    poses.append(json.loads(stream.readline())['pose'])
        current = self.at('pose', timestamp)
        mapping = self.at('map', timestamp) or {}
        cells = mapping.get('cells', {})
        targets = []
        for cell_id, cell in cells.items():
            point = cell.get('position') or cell.get('filtered_position')
            if point is not None and len(point) >= 2:
                targets.append((cell_id, point, cell))
        points = [[p['x'], p['y']] for p in poses] + [item[1][:2] for item in targets]
        centers = [cell.get('center') for cell in cells.values() if cell.get('center') is not None]
        tag = mapping.get('tag') or {}
        tag_position = tag.get('position')
        points += [point[:2] for point in centers]
        if tag_position is not None:
            points.append(tag_position[:2])
        if points:
            values = np.asarray(points, float)
            lower, upper = values.min(axis=0)-0.4, values.max(axis=0)+0.4
            scale = min(560/max(upper[0]-lower[0], 1), 350/max(upper[1]-lower[1], 1))
            def pixel(point):
                return (int(40+(point[0]-lower[0])*scale), int(425-(point[1]-lower[1])*scale))
            if len(poses) > 1:
                cv2.polylines(canvas, [np.array([pixel([p['x'], p['y']]) for p in poses])], False, (220, 190, 50), 2)
            for point in centers:
                center = pixel(point)
                cv2.rectangle(canvas, (center[0]-5, center[1]-5), (center[0]+5, center[1]+5), (85, 85, 85), 1)
            if tag_position is not None:
                cv2.drawMarker(canvas, pixel(tag_position), (0, 220, 255), cv2.MARKER_STAR, 20, 2)
            for cell_id, point, cell in targets:
                center = pixel(point)
                color = ((30, 170, 245) if cell.get('label') == 'square_cone' else (240, 180, 60))
                if cell.get('source') == 'fallback':
                    color = (120, 120, 120)
                for measurement in cell.get('measurements', []):
                    measured = measurement.get('position')
                    if measured is not None and len(measured) >= 2:
                        cv2.circle(canvas, pixel(measured), 2, color if measurement.get('accepted') else (80, 80, 80), -1)
                if cell.get('label') == 'square_cone':
                    cv2.rectangle(canvas, (center[0]-9, center[1]-9), (center[0]+9, center[1]+9), color, 2)
                else:
                    cv2.circle(canvas, center, 9, color, 2)
                cv2.putText(canvas, str(cell_id), (center[0]+12, center[1]), 0, .5, (240, 240, 240), 1)
            if current:
                p = current['pose']
                center = pixel([p['x'], p['y']])
                yaw = math.radians(p['yaw'])
                end = (center[0]+int(25*math.cos(yaw)), center[1]-int(25*math.sin(yaw)))
                cv2.arrowedLine(canvas, center, end, (100, 255, 100), 3)
                cv2.putText(canvas, 'x=%.2f y=%.2f z=%.2f yaw=%.1f' % (p['x'], p['y'], p['z'], p['yaw']),
                            (15, 40), 0, .5, (230, 230, 230), 1)
        cv2.putText(canvas, 'Map: '+str(mapping.get('state', 'no map'))+
                    (' [FALLBACK]' if mapping.get('fallback_used') else ''),
                    (15, 20), 0, .5, (230, 230, 230), 1)
        log = self.at('log', timestamp) or {}
        text = '[%s] %s' % (log.get('name', ''), log.get('message', ''))
        return canvas, text
