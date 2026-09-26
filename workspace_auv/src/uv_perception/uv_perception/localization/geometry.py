"""Small, dependency-light geometry guards and assignment helpers."""

from __future__ import annotations

import math


def bbox_is_localizable(detection, width, height, margin_px=8.0,
                        margin_ratio=0.02):
    """Return false for malformed, clipped, or safety-margin bbox geometry."""
    try:
        width, height = int(width), int(height)
        x1, y1 = float(detection.bbox_x1), float(detection.bbox_y1)
        x2, y2 = float(detection.bbox_x2), float(detection.bbox_y2)
    except (AttributeError, TypeError, ValueError, OverflowError):
        return False
    if width <= 0 or height <= 0:
        return False
    if not all(math.isfinite(value) for value in (x1, y1, x2, y2)):
        return False
    if x2 <= x1 or y2 <= y1:
        return False
    margin = max(float(margin_px), float(margin_ratio) * min(width, height))
    return (x1 > margin and y1 > margin and
            x2 < width - margin and y2 < height - margin)


def minimum_cost_assignment(costs, unmatched_cost=1.0):
    """Solve a rectangular one-to-one assignment with explicit unmatched rows.

    ``None``/non-finite costs are forbidden.  The Hungarian algorithm runs in
    cubic time and adds one dummy column per row, so any row can remain
    unmatched without consuming a real detection/track.
    """
    rows = len(costs)
    if rows == 0:
        return []
    columns = max((len(row) for row in costs), default=0)
    total_columns = columns + rows
    forbidden = max(1e9, float(unmatched_cost) * 1e6)
    matrix = []
    for row in costs:
        values = []
        for index in range(columns):
            value = row[index] if index < len(row) else None
            try:
                value = float(value)
            except (TypeError, ValueError):
                value = forbidden
            values.append(value if math.isfinite(value) else forbidden)
        values.extend([float(unmatched_cost)] * rows)
        matrix.append(values)

    # 1-indexed shortest augmenting path form of the Hungarian algorithm.
    u = [0.0] * (rows + 1)
    v = [0.0] * (total_columns + 1)
    p = [0] * (total_columns + 1)
    way = [0] * (total_columns + 1)
    for i in range(1, rows + 1):
        p[0] = i
        j0 = 0
        minv = [float('inf')] * (total_columns + 1)
        used = [False] * (total_columns + 1)
        while True:
            used[j0] = True
            i0 = p[j0]
            delta = float('inf')
            j1 = 0
            for j in range(1, total_columns + 1):
                if used[j]:
                    continue
                current = matrix[i0 - 1][j - 1] - u[i0] - v[j]
                if current < minv[j]:
                    minv[j] = current
                    way[j] = j0
                if minv[j] < delta:
                    delta = minv[j]
                    j1 = j
            for j in range(total_columns + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while True:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
            if j0 == 0:
                break

    assigned = [-1] * rows
    for j in range(1, total_columns + 1):
        if p[j] != 0:
            assigned[p[j] - 1] = j - 1
    result = []
    for row, column in enumerate(assigned):
        if column < 0 or column >= columns:
            continue
        value = matrix[row][column]
        if value < unmatched_cost and value < forbidden:
            result.append((row, column, value))
    return result
