"""线激光外参测试：数值、平移项、以及"与文档不会漂移"。

外参是**静默错误**的高发区：漏掉 19 cm 平移就是固定深度偏置，看起来完全正常。
所以这里既钉数值，也钉**那份文档**（``docs/frames/extrinsics_reference.md`` 的
机器可读块），后者是派生外参的唯一事实源。
"""

from __future__ import annotations

import re
from pathlib import Path

import numpy as np
import pytest
import yaml

from uv_mapping.extrinsics import (
    KEY,
    body_to_world,
    laser_to_body,
    load_extrinsics,
    resolve_extrinsics_path,
)

#: 场景 `xunyun_fixed.scn` 的 `line_laser`：<origin rpy="0 -1.5708 0" xyz="0.05 0 0.19"/>
EXPECTED_TRANSLATION = np.array([0.05, 0.0, 0.19])


def _repo_root() -> Path | None:
    for parent in Path(__file__).resolve().parents:
        if (parent / '.git').exists():
            return parent
    return None


def test_default_config_is_found_and_well_formed():
    path = resolve_extrinsics_path()
    assert path is not None and path.is_file(), (
        'config/laser_extrinsics.yaml not resolvable from the source tree -- '
        'did the package layout change?')
    matrix, meta = load_extrinsics()
    assert matrix.shape == (4, 4)
    assert np.allclose(matrix[3], [0, 0, 0, 1])
    assert np.allclose(matrix[:3, :3] @ matrix[:3, :3].T, np.eye(3), atol=1e-4), (
        'generated mount carries a 3.673e-06 yaw residual; see the config comment')
    assert meta['sensor'] == 'line_laser'


def test_translation_is_exactly_the_mount_offset():
    matrix, _ = load_extrinsics()
    assert np.allclose(matrix[:3, 3], EXPECTED_TRANSLATION, atol=1e-9)


def test_stripe_point_maps_to_expected_body_point():
    """激光平面内 (x_s, y_s, 0) -> 机体系 (0.05, y_s, 0.19 + x_s)。

    x_s 是沿光束向下的距离、y_s 是扇形横向、z_s≡0（条纹本来就在平面内）。
    """
    matrix, _ = load_extrinsics()
    points = np.array([[1.0, 0.0, 0.0],      # 正下方 1 m
                       [0.0, 2.0, 0.0],      # 横向 2 m
                       [3.0, -1.5, 0.0]])
    out = laser_to_body(points, matrix)
    expected = np.array([[0.05, 0.0, 1.19],
                         [0.05, 2.0, 0.19],
                         [0.05, -1.5, 3.19]])
    # 5e-5 而不是 1e-9：生成的外参带着引擎自己的航向残差 3.673e-06，它随射程线性
    # 进入 x 分量（3 m 处 ≈ 1.1e-5）。抹掉它就会拒绝真实值。
    assert np.allclose(out, expected, atol=5e-5)


def test_dropping_the_translation_shifts_the_whole_map_by_the_mount_offset():
    """把 GUI 那份"只转不平移"的外参拿来用，误差恰好是安装偏移。

    ``visualization/auv_visualizer._LASER_R_BS`` 只做 ``pts @ R.T``：画剖面没问题，
    做地图就是 19 cm 固定深度偏置（6 m 量程上 3%）。这个测试把"别复制那段代码"
    写成可执行的。
    """
    matrix, _ = load_extrinsics()
    points = np.array([[2.0, 0.5, 0.0]])
    correct = laser_to_body(points, matrix)
    rotation_only = points @ matrix[:3, :3].T
    assert np.allclose(correct - rotation_only, EXPECTED_TRANSLATION, atol=1e-9)


def test_body_to_world_uses_the_full_transform():
    matrix, _ = load_extrinsics()
    body = laser_to_body(np.array([[1.0, 0.0, 0.0]]), matrix)
    pose = np.eye(4)
    pose[:3, 3] = [10.0, 20.0, 30.0]
    assert np.allclose(body_to_world(body, pose),
                       [[10.05, 20.0, 31.19]], atol=1e-9)


def test_explicit_path_that_does_not_exist_reports_loudly():
    """外参缺失必须报错：内置默认值会让整张地图带着错误安装量继续跑。"""
    with pytest.raises(FileNotFoundError):
        load_extrinsics(explicit='/nonexistent/laser_extrinsics.yaml')


def test_non_orthonormal_rotation_is_rejected(tmp_path):
    bad = tmp_path / 'bad.yaml'
    bad.write_text(yaml.safe_dump({KEY: [[2.0, 0.0, 0.0, 0.0],
                                         [0.0, 1.0, 0.0, 0.0],
                                         [0.0, 0.0, 1.0, 0.0],
                                         [0.0, 0.0, 0.0, 1.0]]}),
                   encoding='utf-8')
    with pytest.raises(ValueError, match='orthonormal'):
        load_extrinsics(path=bad)


def test_wrong_last_row_is_rejected(tmp_path):
    bad = tmp_path / 'bad.yaml'
    bad.write_text(yaml.safe_dump({KEY: [[1.0, 0.0, 0.0, 0.0],
                                         [0.0, 1.0, 0.0, 0.0],
                                         [0.0, 0.0, 1.0, 0.0],
                                         [0.0, 0.0, 1.0, 1.0]]}),
                   encoding='utf-8')
    with pytest.raises(ValueError, match='last row'):
        load_extrinsics(path=bad)


def test_missing_key_is_rejected(tmp_path):
    bad = tmp_path / 'bad.yaml'
    bad.write_text(yaml.safe_dump({'something_else': []}), encoding='utf-8')
    with pytest.raises(ValueError, match=KEY):
        load_extrinsics(path=bad)


def _documented_block(root: Path) -> dict | None:
    """The fenced ``yaml`` block in the extrinsics reference that carries the laser."""
    doc = root / 'docs' / 'frames' / 'extrinsics_reference.md'
    if not doc.is_file():
        return None
    text = doc.read_text(encoding='utf-8')
    for block in re.findall(r'```yaml\n(.*?)```', text, re.S):
        try:
            data = yaml.safe_load(block)
        except yaml.YAMLError:
            continue
        if not isinstance(data, dict):
            continue
        if KEY in data or KEY in (data.get('transforms') or {}):
            return data
    return None


def test_config_matches_the_documentation_block():
    """与 ``docs/frames/extrinsics_reference.md`` 的机器可读块逐元素对拍。

    文档里是 ``transforms: {T_b_line_laser: {matrix: [...]}}``（派生结果），
    所以两种形状都接受。找不到仓库（只装了包）时跳过——不假装通过。
    """
    root = _repo_root()
    if root is None:
        pytest.skip('repository root not found (installed-only layout)')
    data = _documented_block(root)
    if data is None:
        pytest.skip('extrinsics reference has no fenced yaml block with ' + KEY)

    entry = data.get(KEY) or data['transforms'][KEY]
    documented = np.asarray(entry['matrix'] if isinstance(entry, dict) else entry,
                            dtype=float)
    matrix, _ = load_extrinsics()
    assert matrix.shape == documented.shape
    assert np.allclose(matrix, documented, atol=1e-12), (
        'config/laser_extrinsics.yaml drifted from docs/frames/extrinsics_reference.md; '
        'the doc is the value authority (regenerate with derive_extrinsics.py --check)')


def test_config_matches_the_documented_mount_and_rpy():
    """第二次独立对拍：从文档里的 ``xyz``/``rpy_rad`` 重新推一遍。

    与上一个测试不同，这里不依赖文档里那份派生矩阵，所以任何一处改错都会被抓到。
    容差 1e-5 是因为场景写的是 ``-1.5708`` 而不是精确 π/2（残差 3.7e-6）。
    """
    from uv_mapping.geometry import rpy_to_matrix, se3

    root = _repo_root()
    if root is None:
        pytest.skip('repository root not found (installed-only layout)')
    data = _documented_block(root)
    if data is None:
        pytest.skip('extrinsics reference has no fenced yaml block with ' + KEY)

    sensors = data.get('sensors') or {}
    if 'line_laser' not in sensors:
        pytest.skip('documented block has no line_laser sensor entry')
    entry = sensors['line_laser']
    expected = se3(rpy_to_matrix(*entry['rpy_rad']), entry['xyz'])

    matrix, _ = load_extrinsics()
    assert np.allclose(matrix, expected, atol=1e-5), (
        f'configured mount {matrix[:3, 3]} / rpy differs from the documented '
        f"xyz {entry['xyz']} / rpy_rad {entry['rpy_rad']}")
