"""Real down-stereo calibration shared by detection and mapping."""

import json
from pathlib import Path

import numpy as np


def real_down_calibration_path():
    """Prefer the supplied workspace calibration; use the packaged copy on deploy."""
    source = Path(__file__).resolve().parents[3] / 'docs/stereo_parameters.json'
    packaged = Path(__file__).resolve().parents[1] / 'config/down_real.json'
    candidates = [source, packaged]
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(Path(get_package_share_directory('uv_camera')) /
                          'config/down_real.json')
    except Exception:
        pass
    for path in candidates:
        if path.is_file():
            return str(path)
    raise FileNotFoundError('未找到下视双目标定 stereo_parameters.json/down_real.json')


def load_real_down_json(path):
    """Return 640x480 per-eye K/D and stereo R/T; source JSON is 1280x960."""
    with Path(path).open(encoding='utf-8') as stream:
        document = json.load(stream)
    if document.get('Calibration', {}).get('WorldUnits') not in \
            ('毫米', 'mm', 'millimeters'):
        raise ValueError('双目标定平移单位必须明确为毫米')

    cameras = []
    for name in ('Camera1', 'Camera2'):
        camera = document[name]
        height, width = map(int, camera['ImageSize'])
        if (width, height) != (1280, 960):
            raise ValueError(f'{name} 标定尺寸不是 1280x960: {width}x{height}')
        k = np.asarray(camera['K'], dtype=np.float64)
        radial = np.asarray(camera['RadialDistortion'], dtype=np.float64)
        tangential = np.asarray(camera['TangentialDistortion'], dtype=np.float64)
        if k.shape != (3, 3) or radial.shape != (3,) or tangential.shape != (2,):
            raise ValueError(f'{name} 内参或畸变系数尺寸错误')
        d = np.array([radial[0], radial[1], tangential[0],
                      tangential[1], radial[2]], dtype=np.float64)
        if not np.all(np.isfinite(k)) or not np.all(np.isfinite(d)) \
                or k[0, 0] <= 0 or k[1, 1] <= 0:
            raise ValueError(f'{name} 内参或畸变系数无效')
        # Whole down stream is 1280x480: two 640x480 eyes.  Both axes
        # halve, so all non-homogeneous K entries (including skew) halve.
        k[0, :] *= 0.5
        k[1, :] *= 0.5
        cameras.append((k, d))
    rotation = np.asarray(document['Stereo']['R'], dtype=np.float64)
    translation = np.asarray(document['Stereo']['T'], dtype=np.float64).reshape(-1)
    if rotation.shape != (3, 3) or translation.shape != (3,) \
            or not np.all(np.isfinite(rotation)) \
            or not np.all(np.isfinite(translation)):
        raise ValueError('双目外参 R/T 无效')
    translation /= 1000.0
    if not 0.02 <= np.linalg.norm(translation) <= 0.3:
        raise ValueError('双目基线不在合理范围内，请核查毫米到米的转换')
    return width // 2, height // 2, *cameras[0], *cameras[1], rotation, translation
