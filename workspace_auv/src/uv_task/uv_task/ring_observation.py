"""Task-local ring association and stable counts; no persistent object IDs."""
import math

from uv_task.down_camera_servo import best_detection


EYES = ('down_left', 'down_right')


def frame_key(message, received):
    """Prefer source frame identity; legacy publishers can use receive time."""
    capture = int(getattr(message, 'capture_id', 0) or 0)
    if capture > 0:
        return ('capture', capture)
    stamp = getattr(getattr(message, 'header', None), 'stamp', None)
    value = (int(getattr(stamp, 'sec', 0)), int(getattr(stamp, 'nanosec', 0)))
    return ('stamp', value) if any(value) else ('received', float(received))


def ring_detections(message, class_id):
    # Keep the shared finite-value rules without changing other tasks' selector.
    from types import SimpleNamespace
    return [det for det in getattr(message, 'detections', ())
            if best_detection(SimpleNamespace(detections=[det]), class_id) is not None]


def bbox_center(detection):
    try:
        x1, y1, x2, y2 = (float(getattr(detection, key))
                          for key in ('bbox_x1', 'bbox_y1', 'bbox_x2', 'bbox_y2'))
        if all(math.isfinite(v) for v in (x1, y1, x2, y2)) and x2 > x1 and y2 > y1:
            return ((x1+x2)/2.0, (y1+y2)/2.0)
    except (AttributeError, TypeError, ValueError):
        pass
    return (float(detection.pixel_x), float(detection.pixel_y))


class RingTargetLock:
    """Keep the active eye until two seconds after its last unique ring frame."""
    def __init__(self, class_id, timeout, make_observation):
        self.class_id = class_id
        self.timeout = timeout
        self.make_observation = make_observation
        self.active = None
        self.generation = 0
        self.after = float('-inf')
        self._keys = {}
        self._eyes = {}
        self.new_observations = []

    def update(self, entries, now):
        self.new_observations = []
        for camera in EYES:
            entry = entries.get(camera)
            if entry is None:
                continue
            received, message = entry
            if received <= self.after or not 0 <= now-received <= self.timeout:
                continue
            key = frame_key(message, received)
            previous = self._keys.get(camera)
            if previous is not None and key[0] == previous[0] and key[1] <= previous[1]:
                continue
            self._keys[camera] = key
            eye = self._eyes.setdefault(camera, {'reference': None, 'visible': None,
                                                'last_valid': float('-inf'), 'axis': None})
            candidates = ring_detections(message, self.class_id)
            if not candidates:
                eye['visible'] = None
                continue
            observations = [self.make_observation(camera, received, det, message)
                            for det in candidates]
            reference = eye['reference']
            def score(obs):
                xy = bbox_center(obs.detection)
                if reference is None:
                    distance = math.hypot(*obs.error)
                else:
                    distance = math.hypot(xy[0]-reference[0], xy[1]-reference[1])
                return (distance, -float(obs.detection.confidence), xy)
            observation = min(observations, key=score)
            eye['reference'] = bbox_center(observation.detection)
            eye['last_valid'] = received
            if observation.axis is not None:
                eye['axis'] = (received, observation.axis)
            observation.axis_sample = eye['axis']
            eye['visible'] = observation
            self.new_observations.append(observation)
        if self.active is not None:
            last = self._eyes[self.active]['last_valid']
            if now-last >= self.timeout-1e-9:
                # Inspect newly received active-eye data before expiring: a
                # blocking SET/count window is not a camera outage. Inactive
                # cached detections still cannot acquire at the release time.
                self.active = None
                self.after = now
                self._eyes.clear()
                self.new_observations = []
        if self.active is None:
            available = [(camera, eye['visible']) for camera, eye in self._eyes.items()
                         if eye['visible'] is not None
                         and 0 <= now-eye['last_valid'] < self.timeout]
            if available:
                self.active = min(available, key=lambda pair:
                                  (math.hypot(*pair[1].error), -pair[1].received, pair[0]))[0]
                self.generation += 1
        if self.active is None:
            return None
        eye = self._eyes[self.active]
        if not 0 <= now-eye['last_valid'] < self.timeout:
            return None
        return eye['visible']


class StableRingCounts:
    """Two independent image counts. Duplicate frames never create stability."""
    def __init__(self, hold_seconds, freshness):
        self.hold_seconds = hold_seconds
        self.freshness = freshness
        self._keys = {}
        self._eyes = {}

    def observe(self, camera, received, message, value):
        key = frame_key(message, received)
        previous_key = self._keys.get(camera)
        if (previous_key is not None and key[0] == previous_key[0]
                and key[1] <= previous_key[1]):
            return
        self._keys[camera] = key
        previous = self._eyes.get(camera)
        since = received
        if (previous is not None and value is not None and previous[0] == value
                and 0 <= received-previous[2] <= self.freshness):
            since = previous[1]
        self._eyes[camera] = (value, since, received)

    def result(self, now):
        result = {}
        for camera in EYES:
            entry = self._eyes.get(camera)
            if entry is None:
                return None
            value, since, received = entry
            if (value is None or not 0 <= now-received <= self.freshness
                    or received-since < self.hold_seconds-1e-9):
                return None
            result[camera] = value
        return result


def compare_ring_counts(before, after):
    """One removal confirms success; conflicting views must not reopen a claw."""
    if before is None or after is None:
        return 'unknown'
    differences = [before[eye]-after[eye] for eye in EYES]
    visible = [delta for eye, delta in zip(EYES, differences) if before[eye] > 0]
    if any(delta not in (0, 1) for delta in differences):
        return 'unknown'
    if visible and all(delta == 1 for delta in visible):
        return 'success'
    if visible and all(delta == 0 for delta in differences):
        return 'retry'
    return 'unknown'
