"""建图核心测试：死推地形、自洽指标、误差预算的机制。

这些测试把离线实测的误差预算写成可执行的形式。实测结论（详细推导见
``workspace_auv/docs/laser_mapping.md``）：

* **激光链本身是准的**：真值位姿 + 2 mm 量测噪声，重叠条纹在 5 cm 格里的一致度
  是 0.34 cm —— 这恰好是"5 条条纹各取 1 个点、σ=2 mm"的极差期望 ``2.33σ``，
  也就是说底下没有别的误差了。
* **误差是垂直位置误差**：实机位姿源 ``/zit6/state/pos`` **没有 header**，只能按接收
  时刻打戳（±半个周期 ≈ 17 ms）。分解显示平移项贡献 1.57 mm rms、姿态项仅 0.30 mm，
  即 ``z 误差 ≈ dz/dt × δt``。所以**头部那个坑不是姿态，是时间戳**。
* **地形误差的机制**（下面逐个用解析可算的合成数据钉住）：
  垂直位置误差 1:1 进深度；水平位置误差在平地上**不进深度**；横滚抖动按 ``横向偏移``
  成正比进深度（近 nadir 处为零）；俯仰抖动因为力臂只有 0.05 m 而基本无害；
  偏航对平地深度**完全无影响**（绕竖直轴转不改变 z）。
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from uv_mapping.bathymetry import BathymetryMap, cell_keys, dead_reckon
from uv_mapping.geometry import rpy_to_matrix, se3

D = math.radians
#: 与 config/laser_extrinsics.yaml 一致的安装（旋转 + 平移）
T_B_LASER = se3(np.array([[0.0, 0.0, -1.0],
                          [0.0, 1.0, 0.0],
                          [1.0, 0.0, 0.0]]), [0.05, 0.0, 0.19])
LEAF = 0.05


def stripe(across: np.ndarray, down) -> np.ndarray:
    """激光平面内的一条条纹：沿光束向下 ``down`` 米、横向 ``across`` 米。

    ``down`` 可以是标量（每条条纹一样）或与条纹一一对应的数组。
    """
    across = np.atleast_1d(np.asarray(across, dtype=float))
    return np.column_stack([np.full(across.size, float(down)), across,
                            np.zeros(across.size)])


def run(poses, across, down, noise: float = 0.0, seed: int = 3) -> BathymetryMap:
    """把每条条纹放到对应位姿下并累积成地图。

    ``down`` 是**测量本身**（光束到海床的距离），由调用方给定，**不**随位姿变化 ——
    这样"量测不变、位姿有误差"才是它该有的样子。让 ``down`` 跟着位姿走会把要测的
    误差自动补偿掉（第一版就是这么写的，于是"垂直误差 1:1"那条测试永远通过不了）。
    """
    rng = np.random.default_rng(seed)
    down_values = np.broadcast_to(np.asarray(down, dtype=float), (len(poses),))
    mapper = BathymetryMap(leaf=LEAF)
    for pose, down_i in zip(poses, down_values):
        points = stripe(across, float(down_i))
        if noise:
            points = points + rng.normal(0.0, noise, points.shape)
        mapper.add_strip(points, pose, T_B_LASER)
    return mapper


#: 名义量测：机身水平、海床在下方 5 m（激光器比机身低 0.19 m）
NOMINAL_DOWN = 5.0 - 0.19


def across_grid() -> np.ndarray:
    """每格恰好 1 个点的横向采样（叶宽 = 0.05 m）。"""
    return np.arange(-20, 21) * LEAF


def straight_strips(n: int = 200, spacing: float = 0.01, z: float = 0.0):
    """沿 +x 匀速直线、姿态水平的位姿序列（spacing < leaf ⇒ 共享网格）。"""
    return [se3(np.eye(3), [i * spacing, 0.0, z]) for i in range(n)]


# ── 正确性：地形本身 ────────────────────────────────────────────────────────
def test_flat_seabed_reconstructs_to_the_right_depth():
    metrics = run(straight_strips(), across_grid(), NOMINAL_DOWN).metrics()
    assert metrics['depth_min'] == pytest.approx(5.0, abs=1e-9)
    assert metrics['depth_max'] == pytest.approx(5.0, abs=1e-9)


def test_extrinsics_translation_is_actually_applied():
    """把 19 cm 平移丢掉，整张图会整体浅 19 cm。"""
    rotation_only = np.eye(4)
    rotation_only[:3, :3] = T_B_LASER[:3, :3]
    mapper = BathymetryMap(leaf=LEAF)
    for pose in straight_strips(20):
        mapper.add_strip(stripe(across_grid(), NOMINAL_DOWN), pose, rotation_only)
    assert np.allclose(mapper.grid()['depth'], 5.0 - 0.19, atol=1e-9)


def test_overlapping_strips_agree_exactly_without_noise():
    metrics = run(straight_strips(spacing=0.01), across_grid(),
                  NOMINAL_DOWN).metrics()
    assert metrics['multi_strip_cells'] > 100
    assert metrics['spread_p50_cm'] < 1e-9


def test_noise_floor_matches_the_predicted_extreme_spread():
    """σ=2 mm、每格 5 条条纹各 1 个点 ⇒ 极差期望 ``d2(5)·σ = 2.33·2 mm ≈ 4.7 mm``。

    这一条把"0.34 cm 的地板"从口号变成可验证的：如果它偏大，说明还有别的误差
    （外参、插值、网格），那才是要看的地方。
    """
    metrics = run(straight_strips(n=200, spacing=0.01), across_grid(),
                  NOMINAL_DOWN, noise=0.002).metrics()
    predicted_cm = 2.326 * 0.002 * 100.0
    assert metrics['spread_p50_cm'] == pytest.approx(predicted_cm, rel=0.35), (
        f"measured {metrics['spread_p50_cm']:.3f} cm, predicted {predicted_cm:.3f} cm")


def test_tilted_seabed_reconstructs_the_right_slope():
    slope = 0.10
    poses = straight_strips(n=300, spacing=0.01)
    downs = [NOMINAL_DOWN + slope * pose[0, 3] for pose in poses]
    grid = run(poses, across_grid(), downs).grid()
    order = np.argsort(grid['xy'][:, 0])
    fit = np.polyfit(grid['xy'][order, 0], grid['depth'][order], 1)
    assert fit[0] == pytest.approx(slope, abs=0.01)
    assert not grid['flat'].all(), '10% 坡度不该被判成平地'


def test_gentle_slope_counts_as_flat_and_steep_does_not():
    def flat_fraction(slope: float) -> float:
        poses = straight_strips(n=400, spacing=0.01)
        downs = [NOMINAL_DOWN + slope * pose[0, 3] for pose in poses]
        return float(run(poses, across_grid(), downs).grid()['flat'].mean())

    assert flat_fraction(0.01) > 0.8
    assert flat_fraction(0.20) < 0.2


def test_step_terrain_is_split_into_flat_and_rough():
    poses = straight_strips(n=400, spacing=0.01)
    # 台阶位置**按条纹序号**给，不按 x 阈值：x = i*0.01 正好落在格边界上时，
    # "}> 2.0{" 的浮点比较会让整格落在同一侧，于是没有格子跨台阶（第一版就是这样，
    # 断言拿到 0 个跨台阶格子）。5 条条纹一格（0.05/0.01），序号 203 一定在格中间。
    step_index = 203
    downs = [NOMINAL_DOWN + 1.0 if i >= step_index else NOMINAL_DOWN
             for i in range(len(poses))]
    mapper = run(poses, across_grid(), downs)
    grid = mapper.grid()
    assert grid['flat'].mean() > 0.5
    metrics = mapper.metrics()
    # 台阶只在跨越它的那一列格子里造成分歧，所以看分布而不是中位数：
    # 出现米级分歧的格子必须**全部**落在"陡"那一组里。
    grid = mapper.grid()
    spread = grid['spread']
    large = ~np.isnan(spread) & (spread > 0.5)   # 台阶高 1.0 m
    assert large.sum() > 0, '台阶没有被任何格子看到'
    assert not (large & grid['flat']).any(), '台阶被判成了平地'
    assert metrics['flat_spread_p95_cm'] < 0.1


# ── 误差机制 ────────────────────────────────────────────────────────────────
def test_vertical_position_error_enters_the_depth_one_to_one():
    """这是实测里的主导项：``z 误差 ≈ dz/dt × δt``（位姿时间戳不确定性）。"""
    base = run(straight_strips(), across_grid(), NOMINAL_DOWN).grid()['depth']
    shifted = run(straight_strips(z=0.05), across_grid(),
                  NOMINAL_DOWN).grid()['depth']
    assert np.allclose(shifted - base, 0.05, atol=1e-9)
    assert np.allclose(shifted, 5.05, atol=1e-9)


def test_horizontal_position_error_does_not_corrupt_depths_on_flat_ground():
    """**反直觉的一条**：平地上纯水平位移不改变任何点的高度。

    地图会被整体平移（位置错了），但**深度是对的**。这就是为什么：(a) 平地上那个
    自洽指标量的是垂直误差而不是定位精度；(b) 一个没有时间戳的位姿在平地上
    代价很小 —— 只有车在上下动的时候它才变成深度误差。
    """
    truth = run(straight_strips(), across_grid(), NOMINAL_DOWN)
    biased_poses = [se3(np.eye(3), [pose[0, 3] - 0.3, 0.25, 0.0])
                    for pose in straight_strips()]
    biased = run(biased_poses, across_grid(), NOMINAL_DOWN)

    assert np.allclose(biased.grid()['depth'], 5.0, atol=1e-9)
    assert biased.grid()['xy'][:, 0].min() == pytest.approx(
        truth.grid()['xy'][:, 0].min() - 0.3, abs=LEAF)
    assert biased.metrics()['spread_p50_cm'] < 1e-9


def test_roll_jitter_depth_error_is_proportional_to_the_across_offset():
    """横滚 δ 把点的高度改成 ``across·sin δ``：近 nadir 处为零、边缘最大。

    这条解释了为什么自洽指标在条纹上是**有形状的**（梳齿从中间向两边张开），
    也解释了为什么它正比于射程。
    """
    # 抖动幅度取小值（±4e-4 rad）是有意的：横滚同时把点**横向**推 5.0·δ ≈ ±2 mm，
    # 幅度大了点会跨出 5 cm 格子，每格条纹数就不一样了，比值被搅浑（第一版
    # 用 ±5e-3 时 y 要飘 ±25 mm，结果 near:edge 只有 21 而不是 60）。取小值后
    # 所有点留在同一格，比值回到解析值 60。
    rng = np.random.default_rng(11)
    sigma_roll = 4e-4 / math.sqrt(3.0)
    poses = [se3(rpy_to_matrix(r, 0.0, 0.0), [i * 0.01, 0.0, 0.0])
             for i, r in enumerate(rng.uniform(-4e-4, 4e-4, size=400))]

    near_nadir = run(poses, np.array([0.025]), NOMINAL_DOWN).metrics()['spread_p50_cm']
    at_edge = run(poses, np.array([1.475]), NOMINAL_DOWN).metrics()['spread_p50_cm']
    assert near_nadir < 0.05 * at_edge, (near_nadir, at_edge)
    expected_edge = 1.475 * 2.326 * sigma_roll * 100.0
    assert at_edge == pytest.approx(expected_edge, rel=0.35), (at_edge, expected_edge)


def test_pitch_jitter_is_negligible_next_to_roll_jitter():
    """几何不对称：横滚的力臂是横向偏移（±2 m），俯仰的力臂是 0.05 m 的安装前移。

    所以"姿态误差"在实践中基本等于"横滚误差"。
    """
    rng = np.random.default_rng(12)
    jitter = rng.uniform(-4e-4, 4e-4, size=400)

    def spread(axis: str) -> float:
        poses = [se3(rpy_to_matrix(a, 0.0, 0.0) if axis == 'roll'
                     else rpy_to_matrix(0.0, a, 0.0), [i * 0.01, 0.0, 0.0])
                 for i, a in enumerate(jitter)]
        return run(poses, np.array([-1.475]), NOMINAL_DOWN).metrics()['spread_p50_cm']

    # 力臂比 1.475 : 0.05 ≈ 30
    assert spread('pitch') < 0.1 * spread('roll'), (spread('pitch'), spread('roll'))


def test_yaw_jitter_has_no_effect_on_flat_ground_at_all():
    """绕竖直轴旋转不改变任何点的 z —— 所以偏航误差不该被算进深度误差。

    把它写成测试是为了防止有人用"偏航越大越糟"去解释实测数据（作者一开始就是
    这么猜的，分解结果证明不是）。
    """
    rng = np.random.default_rng(13)
    poses = [se3(rpy_to_matrix(0.0, 0.0, y), [i * 0.01, 0.0, 0.0])
             for i, y in enumerate(rng.uniform(-0.05, 0.05, size=200))]
    metrics = run(poses, across_grid(), NOMINAL_DOWN).metrics()
    assert metrics['spread_p50_cm'] < 1e-9
    assert metrics['depth_min'] == pytest.approx(5.0, abs=1e-9)


# ── 网格、边界与契约 ────────────────────────────────────────────────────────
def test_cell_keys_floor_negative_coordinates():
    """NED 下负坐标很常见（原点以西的航迹）；astype(int) 会把 -0.5 和 0.5 并成一格。"""
    keys = cell_keys(np.array([[-0.51, 0.0], [-0.49, 0.0], [0.49, 0.0]]), LEAF)
    assert keys[0, 0] == -11 and keys[1, 0] == -10 and keys[2, 0] == 9


def test_single_strip_cells_have_no_spread():
    mapper = run(straight_strips(n=1, spacing=0.01), across_grid(), NOMINAL_DOWN)
    assert np.all(np.isnan(mapper.grid()['spread']))
    assert mapper.metrics()['multi_strip_cells'] == 0


def test_empty_map_is_empty_not_an_exception():
    mapper = BathymetryMap()
    assert mapper.point_count == 0
    assert mapper.grid()['keys'].shape == (0, 2)
    metrics = mapper.metrics()
    assert metrics['cells'] == 0 and math.isnan(metrics['depth_min'])


def test_zero_point_strip_still_advances_the_strip_counter():
    """空条纹（整条射线都没命中）要算一帧，否则 QA 的丢帧率会看不见它。"""
    mapper = BathymetryMap()
    mapper.add_strip(np.zeros((0, 3)), np.eye(4), T_B_LASER)
    mapper.add_strip(stripe(np.array([0.0]), 5.0), np.eye(4), T_B_LASER)
    assert len(mapper) == 2
    assert mapper.point_count == 1


def test_finer_leaf_gives_more_cells_for_the_same_ground():
    coarse = run(straight_strips(), across_grid(), NOMINAL_DOWN).metrics()
    fine_poses = straight_strips(spacing=0.01)
    fine = run(fine_poses, np.arange(-40, 41) * 0.01, NOMINAL_DOWN)
    fine.leaf = 0.02
    assert fine.metrics()['cells'] > coarse['cells']


def test_dead_reckon_helper_snaps_to_the_nearest_pose():
    from uv_mapping.geometry import interpolate_poses

    stamps = np.array([0.0, 1.0])
    poses = np.array([se3(np.eye(3), [0.0, 0.0, 0.0]),
                      se3(np.eye(3), [1.0, 0.0, 0.0])])
    strips = [(stripe(np.array([0.0]), NOMINAL_DOWN), 0.1),
              (stripe(np.array([0.0]), NOMINAL_DOWN), 0.9)]
    mapper = dead_reckon(strips, {'stamps': stamps, 'matrices': poses}, T_B_LASER)
    assert len(mapper) == 2
    x = np.sort(np.unique(mapper.grid()['xy'][:, 0]))
    assert len(x) == 2, 'two strips far apart in time must land in different cells'
    # 插值路径存在且可用（replay 默认走它）
    assert interpolate_poses(stamps, poses, np.array([0.1])).shape == (1, 4, 4)


def test_max_strips_bounds_the_map_and_keeps_the_newest():
    """live 模式的滚动窗口：重建网格是 O(已累积点数)，不封顶会越跑越慢。"""
    mapper = BathymetryMap(leaf=LEAF, max_strips=5)
    for i in range(20):
        mapper.add_world_points(np.array([[i * 0.5, 0.0, 5.0]]))
    assert mapper.strips_seen == 20
    assert len(mapper) == 5
    assert mapper.point_count == 5
    # 留下的是最新的 5 条（x = 7.5 .. 9.5）
    assert mapper.points_world()[:, 0].min() == pytest.approx(7.5)


def test_decimation_keeps_one_point_per_cell_per_strip():
    mapper = BathymetryMap(leaf=LEAF, decimate=True)
    across = np.arange(-1.0, 1.0 + 1e-9, 0.001)      # 同一格里有 50 个点
    mapper.add_strip(stripe(across, NOMINAL_DOWN), np.eye(4), T_B_LASER)
    assert mapper.point_count == len(np.unique(np.floor(across / LEAF)))


def test_decimation_off_keeps_every_point():
    mapper = BathymetryMap(leaf=LEAF)
    across = np.arange(-1.0, 1.0 + 1e-9, 0.001)
    mapper.add_strip(stripe(across, NOMINAL_DOWN), np.eye(4), T_B_LASER)
    assert mapper.point_count == across.size


def test_metrics_contract_keys():
    """``map_replay`` 的 metrics.json 消费这些键；少一个就是运行期 KeyError。"""
    metrics = run(straight_strips(), across_grid(), NOMINAL_DOWN).metrics()
    for key in ('points', 'strips', 'strips_seen', 'cells', 'coverage_m2',
                'depth_min', 'depth_max',
                'extent_x', 'extent_y', 'multi_strip_cells', 'spread_p50_cm',
                'spread_p95_cm', 'flat_spread_p50_cm', 'flat_spread_p95_cm',
                'rough_spread_p50_cm', 'rough_spread_p95_cm'):
        assert key in metrics, key
