"""2.5-D seabed reconstruction by dead reckoning.  numpy only, no ROS, no SLAM.

The whole reconstruction is one line of maths::

    p_map = T_map_body(t_strip) . T_body_laser . p_laser

Everything else in this module exists to answer *how good* that line is, because
the answer is not obvious and got measured:

* **The laser is not the problem.**  Reconstructing a canyon survey with the
  simulator's exact pose puts 1 570 strips through overlapping 5 cm cells and
  they agree to a median of 0.34 cm.  Extrinsics, cloud decoding and the stripe
  geometry are all correct to well below the sensor noise.
* **The pose is the problem, and it is amplified by motion.**  Feeding the same
  strips the *realistic* pose stream degraded the agreement to 1.11 cm (3.3x);
  an identical survey flown at constant velocity instead of stop-and-go degraded
  to only 0.46 cm (1.3x).  Smooth motion does not make the map more accurate by
  itself -- it stops the pose error from being projected into the map, because a
  moving vehicle changes attitude between strips and attitude error is multiplied
  by range.
* **No registration is needed for a single pass.**  Scan matching / ICP buys
  loop closure over long missions, not first-order accuracy.  Adding it before
  the pose chain is understood would hide the dominant error term.

The self-consistency metric below is what makes those numbers possible without
ground truth: for each fine cell, the per-strip medians are compared, and if the
pose were exact and the surface static they would be identical.  Terrain slope
inside a cell contaminates it, so the metric is split into **flat** and **rough**
cells by the local gradient -- on flat ground a purely horizontal pose error
produces *no* height error at all, which is why the flat number mostly measures
attitude and why a rotation-free survey is so insensitive to pose gaps.
"""

from __future__ import annotations

from collections import deque

import numpy as np

from uv_mapping.extrinsics import laser_to_body

#: Fine grid: the map's own resolution.
DEFAULT_LEAF = 0.05
#: Coarse grid: used only to decide whether a fine cell sits on flat ground.
DEFAULT_COARSE_LEAF = 0.25
#: |dz/dxy| under which a cell counts as flat.  0.05 = 5 % slope.
DEFAULT_SLOPE_MAX = 0.05


def cell_keys(xy: np.ndarray, leaf: float) -> np.ndarray:
    """Integer ``(i, j)`` cell of every xy point, floor-based.

    Floor (not ``astype(int)``) so negative coordinates -- which NED makes
    ordinary, a survey west of the origin has x < 0 -- do not collapse into a
    single cell straddling zero.
    """
    return np.floor(np.asarray(xy, dtype=float) / float(leaf)).astype(np.int64)


def _group_median(values: np.ndarray, group: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Median of ``values`` per group id, plus the group sizes.

    ``np.unique(..., return_inverse=True)`` does **not** return sorted inverse
    indices, so grouping needs an explicit lexsort (whose LAST key is the
    primary one) rather than ``searchsorted`` on the raw inverse.  Getting this
    wrong produces a quietly wrong map rather than an exception.
    """
    values = np.asarray(values, dtype=float)
    group = np.asarray(group)
    if group.size == 0:
        return np.zeros(0), np.zeros(0, dtype=np.int64)
    order = np.lexsort((values, group))
    grouped_values = values[order]
    grouped = group[order]
    count = int(grouped.max()) + 1
    starts = np.searchsorted(grouped, np.arange(count))
    ends = np.searchsorted(grouped, np.arange(count), side="right")
    sizes = ends - starts
    out = np.full(count, np.nan)
    nonempty = sizes > 0
    middle = starts[nonempty] + (sizes[nonempty] - 1) // 2
    out[nonempty] = grouped_values[middle]
    return out, sizes


def _fill_nan_grid(grid: np.ndarray) -> np.ndarray:
    """Nearest-neighbour fill of a sparse grid, for a defensible gradient.

    A cell with no data has no slope; treating it as 0 would mark the survey
    boundary as "flat" and let edge artefacts into the flat metric.  Filling
    from the nearest measured cell keeps the boundary's real steepness.
    """
    if not np.isnan(grid).any():
        return grid
    try:
        from scipy import ndimage

        indices = ndimage.distance_transform_edt(
            np.isnan(grid), return_distances=False, return_indices=True)
        return grid[tuple(indices)]
    except ImportError:  # scipy is optional; fall back to a zero fill
        return np.nan_to_num(grid)


def _coarse_slope(grid: np.ndarray, spacing: float) -> tuple[np.ndarray, np.ndarray]:
    """``(dz/dx, dz/dy)`` of a coarse depth grid, tolerant of degenerate shapes.

    A survey can legitimately be one cell wide in a direction -- a single straight
    stripe does exactly that -- and ``np.gradient`` raises
    ``ValueError: Shape of array too small`` for a length-1 axis.  The gradient of
    a constant is 0, so the axis is simply skipped.  This was a real crash on the
    first single-strip map, not a hypothetical: the same call is what every
    "flat ground" test goes through.
    """
    slope_x = np.zeros_like(grid)
    slope_y = np.zeros_like(grid)
    if grid.shape[0] >= 2:
        slope_x = np.gradient(grid, spacing, axis=0)
    if grid.shape[1] >= 2:
        slope_y = np.gradient(grid, spacing, axis=1)
    return slope_x, slope_y


class BathymetryMap:
    """Accumulates dead-reckoned strips and derives a 2.5-D depth map.

    Strips are stored in the **map/NED** frame as they arrive; the grid is
    computed on demand so that changing ``leaf`` never means re-reading a bag.
    """

    def __init__(self, leaf: float = DEFAULT_LEAF,
                 coarse_leaf: float = DEFAULT_COARSE_LEAF,
                 slope_max: float = DEFAULT_SLOPE_MAX,
                 max_strips: int | None = None,
                 decimate: bool = False) -> None:
        self.leaf = float(leaf)
        self.coarse_leaf = float(coarse_leaf)
        self.slope_max = float(slope_max)
        # ``max_strips``/``decimate`` exist for the *live* node, not for offline
        # replay.  Rebuilding the grid is O(points accumulated so far), so a live
        # publisher that recomputes it periodically gets slower and slower -- and
        # a monitoring node that steals CPU from the pipeline it monitors is
        # worse than no node.  Measured: without these bounds the live publisher
        # dragged the laser stream from 30 Hz to 24 Hz and its pose gap to 0.94 s.
        self.max_strips = max_strips
        self.decimate = bool(decimate)
        self.strips_seen = 0
        self._chunks: deque = deque()
        self._strip_ids: deque = deque()
        self._next_strip = 0

    # ── accumulation ────────────────────────────────────────────────────────
    def __len__(self) -> int:
        """Strips currently **retained** (with ``max_strips`` this is a window)."""
        return len(self._strip_ids)

    @property
    def point_count(self) -> int:
        return int(sum(chunk.shape[0] for chunk in self._chunks))

    def add_world_points(self, points_world: np.ndarray,
                         strip_id: int | None = None) -> int:
        """Add already-transformed map-frame points; returns the strip id used."""
        pts = np.asarray(points_world, dtype=float).reshape(-1, 3)
        if strip_id is None:
            strip_id = self._next_strip
            self._next_strip += 1
            self.strips_seen += 1
        if pts.shape[0] > 1 and self.decimate:
            # One point per cell per strip.  The spread metric only needs a
            # per-strip representative per cell, so this changes nothing it
            # measures while cutting memory by the intra-cell point count.
            unique = np.unique(cell_keys(pts[:, :2], self.leaf), axis=0,
                               return_index=True)[1]
            pts = pts[np.sort(unique)]
        # An empty strip still gets a chunk: one entry per strip keeps ``len()``
        # counting *frames* (a whole stripe that missed the seabed is a frame the
        # dropout rate has to see) and keeps the window eviction aligned with the
        # strip ids.
        self._chunks.append(pts)
        self._strip_ids.append(np.full(pts.shape[0], strip_id, dtype=np.int64))
        self._evict_old_strips()
        return strip_id

    def _evict_old_strips(self) -> None:
        if self.max_strips is None:
            return
        oldest_kept = self._next_strip - self.max_strips
        while self._strip_ids and int(self._strip_ids[0][0]) < oldest_kept:
            self._chunks.popleft()
            self._strip_ids.popleft()

    def add_strip(self, points_plane: np.ndarray, T_map_body: np.ndarray,
                  T_b_laser: np.ndarray | None = None,
                  strip_id: int | None = None) -> int:
        """Add one laser stripe: laser-plane points + the body pose at its stamp."""
        body = laser_to_body(points_plane, T_b_laser)
        if body.size == 0:
            return self.add_world_points(body, strip_id)
        world = body @ T_map_body[:3, :3].T + T_map_body[:3, 3]
        return self.add_world_points(world, strip_id)

    def points_world(self) -> np.ndarray:
        if not self._chunks:
            return np.zeros((0, 3), dtype=float)
        return np.concatenate(self._chunks)

    def strip_ids(self) -> np.ndarray:
        if not self._strip_ids:
            return np.zeros(0, dtype=np.int64)
        return np.concatenate(self._strip_ids)

    # ── grid ────────────────────────────────────────────────────────────────
    def grid(self) -> dict:
        """Per-cell depth map, point count, flat flag and cross-strip spread.

        Keys: ``keys`` (M,2) int64, ``xy`` (M,2) float centre, ``depth`` (M,)
        median z [m, NED down-positive], ``count`` (M,) points, ``spread`` (M,)
        max-min of the per-strip medians (NaN when fewer than two strips saw the
        cell), ``flat`` (M,) bool.  All arrays share the row order of ``keys``.
        """
        points = self.points_world()
        if points.shape[0] == 0:
            empty = np.zeros(0)
            return dict(keys=np.zeros((0, 2), dtype=np.int64),
                        xy=np.zeros((0, 2)), depth=empty, count=empty,
                        spread=empty, flat=np.zeros(0, dtype=bool))

        strip_ids = self.strip_ids()
        keys, inverse = np.unique(cell_keys(points[:, :2], self.leaf),
                                  axis=0, return_inverse=True)
        n_cells = keys.shape[0]

        depth, _ = _group_median(points[:, 2], inverse)

        # per (cell, strip) median -> spread across strips
        pair = inverse.astype(np.int64) * (int(strip_ids.max()) + 1) + strip_ids
        unique_pair, pair_inverse = np.unique(pair, return_inverse=True)
        pair_depth, _ = _group_median(points[:, 2], pair_inverse)
        cell_of_pair = unique_pair // (int(strip_ids.max()) + 1)

        z_max = np.full(n_cells, -np.inf)
        z_min = np.full(n_cells, np.inf)
        np.maximum.at(z_max, cell_of_pair, pair_depth)
        np.minimum.at(z_min, cell_of_pair, pair_depth)
        strips_per_cell = np.bincount(cell_of_pair, minlength=n_cells)
        spread = np.where(strips_per_cell >= 2, z_max - z_min, np.nan)

        return dict(keys=keys,
                    xy=(keys + 0.5) * self.leaf,
                    depth=depth,
                    count=np.bincount(inverse, minlength=n_cells).astype(float),
                    spread=spread,
                    flat=self.flat_mask(points, keys))

    def flat_mask(self, points: np.ndarray, keys: np.ndarray) -> np.ndarray:
        """Boolean per fine cell: is the ground around it locally flat?

        Computed on a coarse grid (default 0.25 m) because a 5 cm gradient from
        5 cm cells is mostly sensor noise; the coarse cell's gradient is a real
        slope.  Nearest-neighbour fill keeps the survey boundary honest.
        """
        coarse = np.unique(cell_keys(points[:, :2], self.coarse_leaf),
                           axis=0, return_inverse=True)
        coarse_keys, coarse_inverse = coarse
        sums = np.bincount(coarse_inverse, weights=points[:, 2])
        counts = np.bincount(coarse_inverse)
        coarse_depth = sums / np.maximum(counts, 1)

        origin = coarse_keys.min(axis=0)
        shape = tuple((coarse_keys.max(axis=0) - origin + 1).tolist())
        grid = np.full(shape, np.nan)
        grid[coarse_keys[:, 0] - origin[0], coarse_keys[:, 1] - origin[1]] = coarse_depth
        filled = _fill_nan_grid(grid)
        slope_x, slope_y = _coarse_slope(filled, self.coarse_leaf)
        slope = np.hypot(slope_x, slope_y)

        ratio = int(round(self.coarse_leaf / self.leaf))
        row = keys[:, 0] // ratio - origin[0]
        col = keys[:, 1] // ratio - origin[1]
        inside = ((row >= 0) & (row < shape[0]) & (col >= 0) & (col < shape[1]))
        flat = np.zeros(keys.shape[0], dtype=bool)
        flat[inside] = slope[row[inside], col[inside]] < self.slope_max
        return flat

    # ── metrics ─────────────────────────────────────────────────────────────
    def metrics(self) -> dict:
        """The numbers worth printing: coverage, depth range, self-consistency.

        Percentiles are in **centimetres** for the spread because the useful
        values are sub-centimetre; metres here would print as 0.003.
        """
        grid = self.grid()
        spread = grid['spread']
        flat = grid['flat']
        depth = grid['depth']

        def pct(values, q):
            values = np.asarray(values, dtype=float)
            values = values[~np.isnan(values)]
            return float(np.percentile(values, q)) if values.size else float('nan')

        valid = ~np.isnan(spread)
        flat_valid = valid & flat
        rough_valid = valid & ~flat
        return dict(
            points=self.point_count,
            strips=len(self),
            strips_seen=self.strips_seen,
            cells=int(grid['keys'].shape[0]),
            coverage_m2=float(grid['keys'].shape[0]) * self.leaf * self.leaf,
            depth_min=float(np.nanmin(depth)) if depth.size else float('nan'),
            depth_max=float(np.nanmax(depth)) if depth.size else float('nan'),
            extent_x=(float(grid['xy'][:, 0].min()), float(grid['xy'][:, 0].max()))
            if depth.size else (float('nan'), float('nan')),
            extent_y=(float(grid['xy'][:, 1].min()), float(grid['xy'][:, 1].max()))
            if depth.size else (float('nan'), float('nan')),
            multi_strip_cells=int(valid.sum()),
            spread_p50_cm=100.0 * pct(spread, 50),
            spread_p95_cm=100.0 * pct(spread, 95),
            flat_spread_p50_cm=100.0 * pct(spread[flat_valid], 50),
            flat_spread_p95_cm=100.0 * pct(spread[flat_valid], 95),
            rough_spread_p50_cm=100.0 * pct(spread[rough_valid], 50),
            rough_spread_p95_cm=100.0 * pct(spread[rough_valid], 95),
        )


def dead_reckon(strips, poses: dict, T_b_laser: np.ndarray,
                leaf: float = DEFAULT_LEAF) -> BathymetryMap:
    """Build a map from ``(points_plane, strip_stamp)`` pairs and a pose table.

    ``poses`` is ``{'stamps': array, 'matrices': (N,4,4)}`` already in the map
    frame.  Interpolation is the caller's choice (and is the difference between
    a clean map and a comb -- see ``geometry.interpolate_poses``), so this
    helper takes whatever table it is handed and snaps to the nearest sample.
    """
    from uv_mapping.geometry import nearest_index

    pose_stamps = np.asarray(poses['stamps'], dtype=float)
    pose_matrices = np.asarray(poses['matrices'], dtype=float)
    result = BathymetryMap(leaf=leaf)
    for points_plane, stamp in strips:
        index = int(nearest_index(pose_stamps, np.array([stamp]))[0])
        result.add_strip(points_plane, pose_matrices[index], T_b_laser)
    return result
