"""Position node: monocular ray intersection for 3D object positioning.

Subscribes to YOLO detection results + robot pose.
Implements ray accumulation, RANSAC filtering, and pairwise intersection.
Tracks and remembers object world positions.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node

from uv_msgs.msg import DetectionArray, ObjectPosition, ObjectPositionArray, PoseInfo


# Camera intrinsic + extrinsic parameters (from xunyun_fixed.scn)
# Each camera has its own body offset; optical_to_body is shared per pair
CAM_PARAMS = {
    'front_left': {
        'width': 1280, 'height': 960, 'hfov': 34.19,
        'offset': np.array([0.23, -0.05, 0.76]),
        'optical_to_body': np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]]),
    },
    'front_right': {
        'width': 1280, 'height': 960, 'hfov': 34.19,
        'offset': np.array([0.23, 0.05, 0.76]),
        'optical_to_body': np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]]),
    },
    'down_left': {
        'width': 1280, 'height': 960, 'hfov': 32.18,
        'offset': np.array([-0.13, -0.05, 0.645]),
        'optical_to_body': np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]),
    },
    'down_right': {
        'width': 1280, 'height': 960, 'hfov': 32.18,
        'offset': np.array([-0.13, 0.05, 0.645]),
        'optical_to_body': np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]),
    },
}

MIN_BASELINE = 0.1  # minimum baseline between ray origins (meters)
MAX_DISTANCE = 50.0  # maximum object distance (meters)
MIN_RAYS_FOR_INTERSECTION = 3


def _euler_to_rotation_matrix(rx_deg: float, ry_deg: float, rz_deg: float) -> np.ndarray:
    """ZYX Euler angles (degrees) to rotation matrix."""
    rx, ry, rz = math.radians(rx_deg), math.radians(ry_deg), math.radians(rz_deg)
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)

    R = np.array([
        [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
        [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
        [-sy,     cy * sx,                 cy * cx],
    ])
    return R


CLASS_NAMES = {
    0: "blue_drump", 1: "blue_stick", 2: "gate_green_stick",
    3: "gate_red_stick", 4: "red_drump", 5: "red_stick",
    6: "yellow_flag", 7: "yellow_stick",
}


class PositionNode(Node):
    """Position node: 3D object localization via monocular ray intersection."""

    def __init__(self):
        super().__init__('position')

        self.declare_parameter('max_history', 30)
        self._max_history = self.get_parameter('max_history').get_parameter_value().integer_value

        # Robot pose (NED)
        self.robot_pos = np.array([0.0, 0.0, 0.0])
        self.robot_roll = 0.0   # degrees
        self.robot_pitch = 0.0  # degrees
        self.robot_yaw = 0.0    # degrees
        self._pose_received = False

        # Ray history: (class_id, camera_pair) -> list of rays
        self.ray_history: dict[tuple, list] = {}

        # Object memory: (class_id, camera_pair) -> position info
        self.object_memory: dict[tuple, dict] = {}

        # Debug counters
        self._det_msg_count: dict[str, int] = {}
        self._ray_skipped_baseline: int = 0
        self._ray_added_total: int = 0
        self._intersect_attempts: int = 0
        self._intersect_fails: int = 0
        self._debug_timer = None  # set after first detection

        # Precompute focal lengths and centers per camera
        self._cam_params = {}
        for name, p in CAM_PARAMS.items():
            fx = p['width'] / (2.0 * math.tan(math.radians(p['hfov']) / 2.0))
            self._cam_params[name] = {
                'fx': fx, 'fy': fx,  # square pixels
                'cx': p['width'] / 2.0, 'cy': p['height'] / 2.0,
                'offset': p['offset'],
                'optical_to_body': p['optical_to_body'],
            }

        # Subscribers
        self.create_subscription(DetectionArray, '/perception/detection/front_left', self._front_left_cb, 10)
        self.create_subscription(DetectionArray, '/perception/detection/front_right', self._front_right_cb, 10)
        self.create_subscription(DetectionArray, '/perception/detection/down_left', self._down_left_cb, 10)
        self.create_subscription(DetectionArray, '/perception/detection/down_right', self._down_right_cb, 10)
        self.create_subscription(PoseInfo, '/basic_motion/pose_info', self._pose_cb, 10)

        # Publisher
        self.pub_objects = self.create_publisher(ObjectPositionArray, '/perception/objects', 10)

        # Broadcast timer
        self.create_timer(0.1, self._broadcast_objects)  # 10Hz

        self.get_logger().info('Position node started')

    def _pose_cb(self, msg: PoseInfo):
        if not self._pose_received:
            self._pose_received = True
            self.get_logger().info(
                f'First pose received: robot=({msg.robot_x:.2f}, {msg.robot_y:.2f}, '
                f'{msg.robot_z:.2f}), roll={msg.robot_roll:.1f}°, pitch={msg.robot_pitch:.1f}°, yaw={msg.robot_yaw:.1f}°')
        self.robot_pos = np.array([msg.robot_x, msg.robot_y, msg.robot_z])
        self.robot_roll = msg.robot_roll
        self.robot_pitch = msg.robot_pitch
        self.robot_yaw = msg.robot_yaw

    def _front_left_cb(self, msg: DetectionArray):
        self._on_detection(msg, 'front_left')

    def _front_right_cb(self, msg: DetectionArray):
        self._on_detection(msg, 'front_right')

    def _down_left_cb(self, msg: DetectionArray):
        self._on_detection(msg, 'down_left')

    def _down_right_cb(self, msg: DetectionArray):
        self._on_detection(msg, 'down_right')

    def _on_detection(self, msg: DetectionArray, camera: str):
        # Track arrival count per camera
        n = self._det_msg_count.get(camera, 0)
        if n == 0:
            self.get_logger().info(
                f'First detection msg from {camera}: '
                f'{len(msg.detections)} detections')
            # Start debug summary timer on first detection
            if self._debug_timer is None:
                self._debug_timer = self.create_timer(3.0, self._debug_summary)
        self._det_msg_count[camera] = n + 1

        self._process_detection(msg, camera)

    def _process_detection(self, msg: DetectionArray, camera: str):
        """Process detection results: generate rays and intersect.

        camera: one of 'front_left', 'front_right', 'down_left', 'down_right'.
        Rays are grouped by (class_id, camera_pair) where camera_pair is 'front' or 'down'
        so left and right channels of the same stereo pair contribute to the same queue.
        """
        cp = self._cam_params[camera]
        fx, fy = cp['fx'], cp['fy']
        cx, cy = cp['cx'], cp['cy']
        offset = cp['offset']
        optical_to_body = cp['optical_to_body']

        # camera_pair for ray grouping: 'front' or 'down'
        camera_pair = 'front' if camera.startswith('front') else 'down'

        # Robot rotation matrix (world NED) — full 6-DOF attitude
        R_robot = _euler_to_rotation_matrix(self.robot_roll, self.robot_pitch, self.robot_yaw)

        for det in msg.detections:
            # Pixel to camera ray
            px, py = det.pixel_x, det.pixel_y
            v_cam = np.array([(px - cx) / fx, (py - cy) / fy, 1.0])
            v_cam = v_cam / np.linalg.norm(v_cam)

            # Camera to body NED
            v_body = optical_to_body @ v_cam

            # Body to world NED
            v_world = R_robot @ v_body
            v_world = v_world / np.linalg.norm(v_world)

            # Ray origin: robot position + camera offset in world frame
            offset_world = R_robot @ offset
            ray_origin = self.robot_pos + offset_world

            # Manage ray history — keyed by (class_id, camera_pair)
            class_id = det.class_id
            key = (class_id, camera_pair)
            history = self.ray_history.setdefault(key, [])

            # Check baseline distance
            if history:
                last_origin = history[-1]['origin']
                dist = np.linalg.norm(ray_origin - last_origin)
                if dist < MIN_BASELINE:
                    self._ray_skipped_baseline += 1
                    continue  # Too close to last ray

            # Add ray
            self._ray_added_total += 1
            history.append({
                'origin': ray_origin,
                'direction': v_world,
            })

            # Prune if too many
            if len(history) > self._max_history:
                self._prune_rays(history)

            # Try intersection
            if len(history) >= MIN_RAYS_FOR_INTERSECTION:
                self._intersect_attempts += 1
                position = self._intersect_rays_pairwise(history)
                if position is None:
                    self._intersect_fails += 1
                else:
                    # Validate distance
                    dist = np.linalg.norm(position - self.robot_pos)
                    if dist < MAX_DISTANCE:
                        is_new = (key not in self.object_memory)
                        self.object_memory[key] = {
                            'position': position,
                            'confidence': det.confidence,
                            'observations': len(history),
                        }
                        if is_new:
                            cls_name = CLASS_NAMES.get(class_id, str(class_id))
                            self.get_logger().info(
                                f'New object: {cls_name} (id={class_id}) '
                                f'via {camera_pair}, '
                                f'pos=({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}), '
                                f'dist={dist:.2f}m, rays={len(history)}')

    def _prune_rays(self, history: list):
        """Prune redundant rays using pairwise scoring."""
        if len(history) <= self._max_history:
            return

        # Score each ray by redundancy with others
        scores = []
        for i, ri in enumerate(history):
            score = 0.0
            for j, rj in enumerate(history):
                if i == j:
                    continue
                cos_sim = abs(np.dot(ri['direction'], rj['direction']))
                dist = np.linalg.norm(ri['origin'] - rj['origin'])
                if dist > 0:
                    score += cos_sim / dist
            scores.append(score)

        # Remove highest redundancy ray
        worst_idx = np.argmax(scores)
        history.pop(worst_idx)

    def _intersect_rays_pairwise(self, rays: list) -> np.ndarray | None:
        """Pairwise ray intersection with median filtering."""
        intersections = []

        for i in range(len(rays)):
            for j in range(i + 1, len(rays)):
                p1, d1 = rays[i]['origin'], rays[i]['direction']
                p2, d2 = rays[j]['origin'], rays[j]['direction']

                # Check parallel
                cos_angle = abs(np.dot(d1, d2))
                if cos_angle > 0.9995:
                    continue

                # Common perpendicular
                w = p1 - p2
                a = np.dot(d1, d1)
                b = np.dot(d1, d2)
                c = np.dot(d2, d2)
                d = np.dot(d1, w)
                e = np.dot(d2, w)

                denom = a * c - b * b
                if abs(denom) < 1e-10:
                    continue

                t1 = (b * e - c * d) / denom
                t2 = (a * e - b * d) / denom

                # Both must be forward
                if t1 <= 0 or t2 <= 0:
                    continue

                # Midpoint
                mid1 = p1 + t1 * d1
                mid2 = p2 + t2 * d2
                mid = (mid1 + mid2) / 2.0

                # Check perpendicular distance
                perp_dist = np.linalg.norm(mid1 - mid2)
                if perp_dist > 5.0:
                    continue

                intersections.append(mid)

        if len(intersections) == 0:
            return self._intersect_rays_3d(rays)

        # Median filter
        intersections = np.array(intersections)
        return np.median(intersections, axis=0)

    def _intersect_rays_3d(self, rays: list) -> np.ndarray | None:
        """Least-squares 3D ray intersection fallback."""
        if len(rays) < 2:
            return None

        # Build system: (I - d_i * d_i^T) * p = (I - d_i * d_i^T) * o_i
        A = np.zeros((3, 3))
        b = np.zeros(3)

        for ray in rays:
            o = ray['origin']
            d = ray['direction']
            d = d / np.linalg.norm(d)
            P = np.eye(3) - np.outer(d, d)
            A += P
            b += P @ o

        try:
            cond = np.linalg.cond(A)
            if cond > 1e5:
                return None
            return np.linalg.solve(A, b)
        except np.linalg.LinAlgError:
            return None

    def _debug_summary(self):
        """Periodic debug: log detection rates, ray history, object count."""
        det_info = ', '.join(
            f'{cam}={n}' for cam, n in sorted(self._det_msg_count.items()))
        ray_info = ', '.join(
            f'{CLASS_NAMES.get(cid, cid)}({cid})/{cp}={len(h)}'
            for (cid, cp), h in sorted(self.ray_history.items()))
        total_rays = sum(len(h) for h in self.ray_history.values())
        self.get_logger().info(
            f'[DEBUG] pose_ok={self._pose_received}, '
            f'robot_pos=({self.robot_pos[0]:.1f}, {self.robot_pos[1]:.1f}, {self.robot_pos[2]:.1f}) '
            f'rpy=({self.robot_roll:.1f}°, {self.robot_pitch:.1f}°, {self.robot_yaw:.1f}°), '
            f'objects={len(self.object_memory)}, '
            f'total_rays={total_rays} '
            f'(added={self._ray_added_total}, '
            f'skipped_base={self._ray_skipped_baseline}), '
            f'intersect: attempts={self._intersect_attempts} '
            f'fails={self._intersect_fails}, '
            f'history_keys={len(self.ray_history)}, '
            f'det_msgs=[{det_info}], '
            f'rays=[{ray_info or "(none)"}]')

    def _broadcast_objects(self):
        """Publish tracked object positions at 10Hz."""
        msg = ObjectPositionArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        for (class_id, camera_pair), data in self.object_memory.items():
            obj = ObjectPosition()
            obj.class_id = class_id
            obj.class_name = CLASS_NAMES.get(class_id, str(class_id))
            obj.world_x = float(data['position'][0])
            obj.world_y = float(data['position'][1])
            obj.world_z = float(data['position'][2])
            obj.confidence = float(data['confidence'])
            obj.num_observations = int(data['observations'])
            msg.objects.append(obj)

        self.pub_objects.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PositionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
