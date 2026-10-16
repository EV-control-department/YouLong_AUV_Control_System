"""Best-centred down-view world points used by later rack/frame approaches."""
import math
import time
from types import SimpleNamespace

import cv2
import numpy as np

from uv_task.down_camera_servo import best_detection
from uv_task.pickup_alignment import target_world_xy
from uv_task.ring_observation import bbox_center, frame_key
from uv_task.task_state import TaskState


TARGET_CLASSES = {'frame': 'collection_frame_down', 'rack': 'target_rack_down'}
MEMORY_TASKS = {'26rb_grab_ball_ring', '26rb_grab_golf', '26rb_drop_ball_target_rack',
                '26rb_find_collection_frame'}


def task_state(node):
    state = getattr(node, 'state', None)
    if state is None:
        state = TaskState()
        node.state = state
    return state


def _image_size(node, camera, message):
    geometry = getattr(node, '_detection_geometry', None)
    info = None if geometry is None else geometry.cache.resolve(message)
    if info is not None:
        return float(info.width), float(info.height)
    config = node.camera_configs[camera.split('_')[0]]
    return float(config.width), float(config.height)


def _center_candidate(message, class_id, width, height, min_confidence):
    candidates = []
    for detection in getattr(message, 'detections', ()):
        if best_detection(SimpleNamespace(detections=[detection]), class_id) is None:
            continue
        if float(detection.confidence) < min_confidence:
            continue
        x, y = bbox_center(detection)
        if not 0 <= x < width or not 0 <= y < height:
            continue
        distance = math.hypot(x-width/2.0, y-height/2.0)/math.hypot(width, height)
        candidates.append((distance, -float(detection.confidence), x, y, detection))
    return min(candidates, key=lambda item: item[:4]) if candidates else None


def observe_down_targets(node, received, message):
    """Process each source frame once, freezing measured pose and calibrated ray.

    Empty, stale, invalid or less-centred sightings never erase a good point.
    The metric uses the bbox's distance from the actual image centre; the
    projection still uses the matched image calibration and camera/body TF.
    """
    camera = str(getattr(message, 'camera_name', '')).strip().lower()
    if camera not in ('down_left', 'down_right'):
        return 0
    state = task_state(node)
    now = time.monotonic()
    if not math.isfinite(received) or received > now:
        return 0
    stamp = getattr(getattr(message, 'header', None), 'stamp', None)
    source_stamp = float(getattr(stamp, 'sec', 0))+float(getattr(stamp, 'nanosec', 0))*1e-9
    source_age = None
    if source_stamp:
        try:
            source_age = float(node.get_clock().now().nanoseconds)*1e-9-source_stamp
        except (AttributeError, TypeError, ValueError):
            return 0
    observed_at = received if source_age is None else now-source_age
    epoch = state.down_target_epoch
    active_task = getattr(node, '_current_task_name', None)
    observing = active_task is None or active_task in MEMORY_TASKS
    key = frame_key(message, received)
    pose = None
    updates = 0
    for kind, class_name in TARGET_CLASSES.items():
        depth, freshness, min_confidence = state.down_target_settings(kind)
        if not 0 <= now-received <= freshness:
            continue
        if source_age is not None and not 0 <= source_age <= freshness:
            continue
        if not state.claim_down_target_frame(kind, camera, key, received, observed_at, epoch):
            continue
        if not observing:
            continue
        try:
            if not getattr(node, '_down_target_pose_valid', True):
                continue
            if pose is None:
                pose = tuple(float(v) for v in node._latest_robot_pose(require_measured=True, max_age=1.0))
                if len(pose) != 6 or not all(math.isfinite(v) for v in pose):
                    continue
            class_id = node._model_mapping.model_class_id(class_name, required=False)
            if class_id is None:
                continue
            width, height = _image_size(node, camera, message)
            if not all(math.isfinite(v) and v > 0 for v in (width, height)):
                continue
            candidate = _center_candidate(message, class_id, width, height, min_confidence)
            if candidate is None:
                continue
            distance, _, x, y, reference = candidate
            point = SimpleNamespace(pixel_x=x, pixel_y=y)
            world = target_world_xy(node, camera, point, pose, depth, reference=reference)
            if state.remember_down_target(kind, world, distance, observed_at, camera, epoch=epoch):
                updates += 1
        except (AttributeError, KeyError, TypeError, ValueError, RuntimeError, cv2.error, np.linalg.LinAlgError):
            # Observation memory is opportunistic. Missing geometry/nav data
            # keeps the configured preset and existing motion failure policy.
            continue
    return updates


def poll_down_targets(node):
    """Compatibility for task callers that directly feed the down-view cache."""
    lock = getattr(node, '_perception_lock', None)
    if lock is None:
        return
    with lock:
        entries = tuple(getattr(node, '_down_detections', {}).items())
    for camera, (received, message) in entries:
        if not hasattr(message, 'camera_name'):
            # Legacy in-process callers store the camera beside the array.
            message = SimpleNamespace(**vars(message), camera_name=camera)
        observe_down_targets(node, received, message)
