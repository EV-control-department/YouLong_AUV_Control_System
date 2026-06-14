"""Position node: monocular ray intersection for 3D object positioning.

Subscribes to YOLO detection results + robot pose.
Implements ray accumulation, RANSAC filtering, and pairwise intersection.
Tracks and remembers object world positions.
"""

import math
import numpy as np
from collections import defaultdict

import rclpy
from rclpy.node import Node

from uv_msgs.msg import Detection, DetectionArray, ObjectPosition, ObjectPositionArray, AuvState


# Camera parameters (from xunyun.scn)
FRONT_CAM = {
    'width': 1280, 'height': 960,
    'hfov': 34.19,  # degrees
    'offset': np.array([0.25, -0.02825, 0.25]),  # body NED (x, y, z)
    'optical_to_body': np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]]),  # cam->body NED
}

DOWN_CAM = {
    'width': 1280, 'height': 960,
    'hfov': 32.18,
    'offset': np.array([0.0, 0.03765, 0.21]),
    'optical_to_body': np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]),  # cam->body NED
}

MAX_HISTORY = 20
MIN_BASELINE = 0.1  # minimum baseline between ray origins (meters)
MAX_DISTANCE = 50.0  # maximum object distance (meters)
RANSAC_INLIER_DIST = 1.5  # meters
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


class PositionNode(Node):
    """Position node: 3D object localization via monocular ray intersection."""

    def __init__(self):
        super().__init__('position')

        # Robot pose (NED)
        self.robot_pos = np.array([0.0, 0.0, 0.0])
        self.robot_yaw = 0.0  # degrees

        # Ray history: class_id -> list of {'origin': np.array, 'direction': np.array}
        self.ray_history = defaultdict(list)

        # Object memory: class_id -> {'position': np.array, 'confidence': float, 'observations': int}
        self.object_memory = {}

        # Camera focal lengths
        self.front_fx = FRONT_CAM['width'] / (2.0 * math.tan(math.radians(FRONT_CAM['hfov']) / 2.0))
        self.front_fy = self.front_fx  # assume square pixels
        self.front_cx = FRONT_CAM['width'] / 2.0
        self.front_cy = FRONT_CAM['height'] / 2.0

        self.down_fx = DOWN_CAM['width'] / (2.0 * math.tan(math.radians(DOWN_CAM['hfov']) / 2.0))
        self.down_fy = self.down_fx
        self.down_cx = DOWN_CAM['width'] / 2.0
        self.down_cy = DOWN_CAM['height'] / 2.0

        # Subscribers
        self.create_subscription(DetectionArray, '/perception/detection/front', self._front_det_cb, 10)
        self.create_subscription(DetectionArray, '/perception/detection/down', self._down_det_cb, 10)
        self.create_subscription(AuvState, '/auv/state', self._state_cb, 10)

        # Publisher
        self.pub_objects = self.create_publisher(ObjectPositionArray, '/perception/objects', 10)

        # Broadcast timer
        self.create_timer(0.1, self._broadcast_objects)  # 10Hz

        self.get_logger().info('Position node started')

    def _state_cb(self, msg: AuvState):
        self.robot_pos = np.array([msg.pos_x, msg.pos_y, msg.pos_z])
        self.robot_yaw = math.degrees(msg.yaw)

    def _front_det_cb(self, msg: DetectionArray):
        self._process_detection(msg, 'front')

    def _down_det_cb(self, msg: DetectionArray):
        self._process_detection(msg, 'down')

    def _process_detection(self, msg: DetectionArray, camera: str):
        """Process detection results: generate rays and intersect."""
        cam = FRONT_CAM if camera == 'front' else DOWN_CAM
        fx = self.front_fx if camera == 'front' else self.down_fx
        fy = self.front_fy if camera == 'front' else self.down_fy
        cx = self.front_cx if camera == 'front' else self.down_cx
        cy = self.front_cy if camera == 'front' else self.down_cy

        # Robot rotation matrix (world NED)
        R_robot = _euler_to_rotation_matrix(0, 0, self.robot_yaw)

        for det in msg.detections:
            # Pixel to camera ray
            px, py = det.pixel_x, det.pixel_y
            v_cam = np.array([(px - cx) / fx, (py - cy) / fy, 1.0])
            v_cam = v_cam / np.linalg.norm(v_cam)

            # Camera to body NED
            v_body = cam['optical_to_body'] @ v_cam

            # Body to world NED
            v_world = R_robot @ v_body
            v_world = v_world / np.linalg.norm(v_world)

            # Ray origin: robot position + camera offset in world frame
            offset_world = R_robot @ cam['offset']
            ray_origin = self.robot_pos + offset_world

            # Manage ray history
            class_id = det.class_id
            history = self.ray_history[class_id]

            # Check baseline distance
            if history:
                last_origin = history[-1]['origin']
                dist = np.linalg.norm(ray_origin - last_origin)
                if dist < MIN_BASELINE:
                    continue  # Too close to last ray

            # Add ray
            history.append({
                'origin': ray_origin,
                'direction': v_world,
            })

            # Prune if too many
            if len(history) > MAX_HISTORY:
                self._prune_rays(class_id, history)

            # Try intersection
            if len(history) >= MIN_RAYS_FOR_INTERSECTION:
                position = self._intersect_rays_pairwise(history)
                if position is not None:
                    # Validate distance
                    dist = np.linalg.norm(position - self.robot_pos)
                    if dist < MAX_DISTANCE:
                        self.object_memory[class_id] = {
                            'position': position,
                            'confidence': det.confidence,
                            'observations': len(history),
                        }

    def _prune_rays(self, class_id: int, history: list):
        """Prune redundant rays using pairwise scoring."""
        if len(history) <= MAX_HISTORY:
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

    def _broadcast_objects(self):
        """Publish tracked object positions at 10Hz."""
        msg = ObjectPositionArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        for class_id, data in self.object_memory.items():
            obj = ObjectPosition()
            obj.class_id = class_id
            obj.class_name = str(class_id)
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
