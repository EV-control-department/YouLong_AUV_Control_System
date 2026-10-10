"""Detection-only gate identities; no ROS state or motion commands.

The bank survives a complete search. A locked identity never changes to a
newly created track. Ambiguous observations are deliberately left unmatched.
"""
from collections import deque
from dataclasses import dataclass, field
import math

import numpy as np
from uv_perception.localization.geometry import minimum_cost_assignment


def box_state(box, width, height):
    x1, y1, x2, y2 = box
    return np.array([(x1+x2)/(2*width), (y1+y2)/(2*height),
                     math.log(max(1e-6, (x2-x1)/width)),
                     math.log(max(1e-6, (y2-y1)/height))])


def state_box(state, width, height):
    cx, cy = state[:2]*[width, height]
    w, h = np.exp(np.clip(state[2:], -15, 3))*[width, height]
    return np.array([cx-w/2, cy-h/2, cx+w/2, cy+h/2])


def clip_box(box, width, height):
    return np.clip(box, [0, 0, 0, 0], [width, height, width, height])


def association_cost(predicted, measured, width, height, center_gate, size_gate):
    a, b = clip_box(predicted, width, height), clip_box(measured, width, height)
    aw, ah = a[2:]-a[:2]
    bw, bh = b[2:]-b[:2]
    if min(aw, ah, bw, bh) <= 0:
        return None
    center = float(np.linalg.norm(((a[:2]+a[2:]-b[:2]-b[2:])/2)/[width, height]))
    height_ratio = max(ah/bh, bh/ah)
    aspect_ratio = max((aw/ah)/(bw/bh), (bw/bh)/(aw/ah))
    if center > center_gate or height_ratio > size_gate or aspect_ratio > size_gate:
        return None
    intersect = np.maximum(0, np.minimum(a[2:], b[2:])-np.maximum(a[:2], b[:2]))
    overlap = float(np.prod(intersect))
    iou = overlap/(aw*ah+bw*bh-overlap)
    return (.4*center/center_gate + .3*math.log(height_ratio)/math.log(size_gate)
            + .2*(1-iou) + .1*math.log(aspect_ratio)/math.log(size_gate))


@dataclass
class GateTrack:
    id: int
    eye: str
    frame: object
    state: np.ndarray
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(4))
    streak: int = 1
    heights: deque = field(default_factory=lambda: deque(maxlen=3))
    peak_height: float = 0.0
    confirmed: bool = False


class DetectionTracks:
    def __init__(self, width, height, params, project, measurement):
        self.width, self.height, self.p = width, height, params
        self.project, self.measurement = project, measurement
        self.tracks = {}
        self.next_id = 1
        self.locked = set()
        self.ambiguous = set()

    def cost(self, predicted, measured):
        return association_cost(predicted, measured, self.width, self.height,
                                self.p['tracking_max_center_distance_fraction'],
                                self.p['tracking_max_height_ratio'])

    def prediction(self, track, pose, eye, now):
        rotated = self.project(track.frame, eye, pose, state_box(track.state, self.width, self.height))
        if rotated is None:
            return None
        gap = now-track.frame.received
        dt = 0.0 if gap > self.p['search_detection_timeout'] else min(1.0, max(0.0, gap))
        predicted = box_state(rotated, self.width, self.height)+track.velocity*dt
        return state_box(predicted, self.width, self.height)

    def update(self, eye, frames, pose, now):
        tracks = [t for t in self.tracks.values() if t.eye == eye]
        predictions = [self.prediction(t, pose, eye, now) for t in tracks]
        measurements = [self.measurement(f) for f in frames]
        costs = [[None if box is None else self.cost(box, measured)
                  for measured in measurements] for box in predictions]
        margin = self.p['tracking_ambiguity_margin']
        blocked = set()
        for i, row in enumerate(costs):
            ranked = sorted(c for c in row if c is not None)
            if len(ranked) > 1 and ranked[1]-ranked[0] < margin:
                blocked.add(i)
        # Competing tracks must not exchange identities at a crossing either.
        for j in range(len(frames)):
            ranked = sorted((row[j], i) for i, row in enumerate(costs) if row[j] is not None)
            if len(ranked) > 1 and ranked[1][0]-ranked[0][0] < margin:
                blocked.update(i for c, i in ranked if c-ranked[0][0] < margin)
        self.ambiguous.difference_update(t.id for t in tracks)
        self.ambiguous.update(tracks[i].id for i in blocked)
        matrix = [[None]*len(frames) if i in blocked else row for i, row in enumerate(costs)]
        for i, track in enumerate(tracks):
            if track.id in self.locked:
                best = min((c for c in matrix[i] if c is not None), default=None)
                matrix[i] = [c if c == best else None for c in matrix[i]]
        matched, used = {}, set()
        for i, j, _ in minimum_cost_assignment(matrix):
            track, frame = tracks[i], frames[j]
            measured = measurements[j].copy()
            predicted = predictions[i]
            # Invisible edges carry no information about full gate size.
            clipped = np.asarray(frame.bbox)
            for edge, border in enumerate((0, 0, self.width, self.height)):
                if abs(clipped[edge]-border) < 1:
                    measured[edge] = predicted[edge]
            observed = box_state(measured, self.width, self.height)
            estimate = box_state(predicted, self.width, self.height)
            innovation = observed-estimate
            dt = max(.02, now-track.frame.received)
            track.state = estimate+.6*innovation
            track.velocity = np.clip(track.velocity+.2*innovation/dt, -1, 1)
            if now-track.frame.received > self.p['search_detection_timeout']:
                track.streak = 0
                track.heights.clear()
                track.velocity[:] = 0
            track.frame = frame
            track.streak += 1
            track.heights.append(frame.height_percent)
            track.confirmed = track.confirmed or track.streak >= self.p['tracking_confirm_frames']
            if track.streak >= self.p['tracking_confirm_frames']:
                track.peak_height = max(track.peak_height, float(np.median(track.heights)))
            matched[track.id] = frame
            used.add(j)
        for track in tracks:
            if track.id not in matched:
                track.streak = 0
                track.heights.clear()
        # Do not seed duplicates from ambiguous measurements; they could later
        # compete against the true identity even after the crossing clears.
        disputed = {j for i in blocked for j, c in enumerate(costs[i]) if c is not None}
        for j, frame in enumerate(frames):
            if j in used or j in disputed:
                continue
            track = GateTrack(self.next_id, eye, frame, box_state(measurements[j], self.width, self.height))
            track.heights.append(frame.height_percent)
            if self.p['tracking_confirm_frames'] == 1:
                track.confirmed = True
                track.peak_height = frame.height_percent
            self.tracks[track.id] = track
            matched[track.id] = frame
            self.next_id += 1
        for key, track in list(self.tracks.items()):
            if key not in self.locked and now-track.frame.received > 10 and (self.locked or not track.confirmed):
                del self.tracks[key]
        return matched

    def highest(self, now, fresh_only):
        candidates = [t for t in self.tracks.values() if t.confirmed and t.peak_height > 0
                      and (not fresh_only or
                           (t.streak >= self.p['tracking_confirm_frames']
                            and now-t.frame.received <= self.p['search_detection_timeout']))]
        if not candidates:
            return None
        return max(candidates, key=lambda t: (float(np.median(t.heights)) if fresh_only
                                             else t.peak_height, -t.id))
