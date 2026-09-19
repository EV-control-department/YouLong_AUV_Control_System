"""Pure metric helpers used by the ROS evaluation node and offline tools."""

from __future__ import annotations

import math


def translation_error(estimate, truth) -> float:
    return math.sqrt(sum((float(a) - float(b)) ** 2
                         for a, b in zip(estimate[:3], truth[:3])))


def ate_rmse(estimates, truths) -> float:
    pairs = list(zip(estimates, truths))
    if not pairs:
        return 0.0
    squared = [translation_error(estimate, truth) ** 2
               for estimate, truth in pairs]
    return math.sqrt(sum(squared) / len(squared))


def rpe_translation(estimates, truths) -> float:
    pairs = list(zip(estimates, truths))
    if len(pairs) < 2:
        return 0.0
    errors = []
    for (estimate_a, truth_a), (estimate_b, truth_b) in zip(pairs, pairs[1:]):
        estimate_delta = [float(b) - float(a)
                          for a, b in zip(estimate_a[:3], estimate_b[:3])]
        truth_delta = [float(b) - float(a)
                       for a, b in zip(truth_a[:3], truth_b[:3])]
        errors.append(translation_error(estimate_delta, truth_delta))
    return math.sqrt(sum(error ** 2 for error in errors) / len(errors))


def rpe_rotation(estimates, truths) -> float:
    """Return RMS relative yaw error in radians for four-element poses."""
    pairs = list(zip(estimates, truths))
    if len(pairs) < 2:
        return 0.0
    errors = []
    for (estimate_a, truth_a), (estimate_b, truth_b) in zip(
            pairs, pairs[1:]):
        estimate_delta = float(estimate_b[3]) - float(estimate_a[3])
        truth_delta = float(truth_b[3]) - float(truth_a[3])
        error = estimate_delta - truth_delta
        error = math.atan2(math.sin(error), math.cos(error))
        errors.append(error)
    return math.sqrt(sum(error ** 2 for error in errors) / len(errors))
