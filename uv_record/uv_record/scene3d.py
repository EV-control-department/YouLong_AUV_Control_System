"""Recorded NED coordinates and source-labelled meshes for an offline 3D viewer."""

import copy
import json
import math
from pathlib import Path


def xyz(value):
    return (list(value[:3]) if isinstance(value, (list, tuple)) and len(value) >= 3
            and all(isinstance(v, (float, int)) and math.isfinite(v) for v in value[:3]) else None)


def marker_pattern(tag_id, dictionary):
    """Generate the actual encoded marker, never a decorative random code."""
    if not dictionary or not isinstance(tag_id, int) or tag_id < 0:
        return None
    try:
        import cv2
        aruco = cv2.aruco
        codebook = aruco.getPredefinedDictionary(getattr(aruco, dictionary))
        if tag_id >= len(codebook.bytesList):
            return None
        size = codebook.markerSize + 2
        marker = (aruco.generateImageMarker(codebook, tag_id, size)
                  if hasattr(aruco, 'generateImageMarker') else
                  aruco.drawMarker(codebook, tag_id, size))
        return (marker > 127).astype(int).tolist()
    except (ImportError, AttributeError, ValueError, RuntimeError):
        return None


def scene_from_map(mapping):
    from .analyze import cells_of
    config = mapping.get('submitted_config') or {}
    grid = copy.deepcopy(mapping.get('grid') or {})
    cells = {str(cell.get('id', key)): copy.deepcopy(cell) for key, cell in cells_of(mapping)}
    for key, cell in cells.items():
        cell.setdefault('id', int(key) if key.isdigit() else key)
    notes = []
    center = xyz(grid.get('center'))
    if center is None and all(key in config for key in ('grid_center_x', 'grid_center_y', 'floor_z')):
        center = xyz([config['grid_center_x'], config['grid_center_y'], config['floor_z']])
        notes.append('九宫格中心取记录中的 submitted_config。')
    if center is None:
        centers = [xyz(cell.get('center')) for cell in cells.values()]
        centers = [point for point in centers if point]
        if len(centers) == 9:
            center = [sum(point[axis] for point in centers)/9 for axis in range(3)]
            notes.append('九宫格中心由九个记录格点中心求均值。')
    side = grid.get('side_m', config.get('grid_side_m'))
    side = float(side) if isinstance(side, (int, float)) and math.isfinite(side) and side > 0 else None
    yaw = grid.get('yaw_deg', config.get('grid_yaw_deg', 0))
    yaw = float(yaw) if isinstance(yaw, (int, float)) and math.isfinite(yaw) else 0.
    floor = grid.get('floor_z', config.get('floor_z', center[2] if center else None))
    if not isinstance(floor, (int, float)) or not math.isfinite(floor):
        floor = None
    if center and side:
        rotation = math.radians(yaw)
        for index in range(9):
            cell = cells.setdefault(str(index), {'id': index})
            if xyz(cell.get('center')) is None:
                x, y = (1-index//3)*side/3, (index%3-1)*side/3
                cell['center'] = [center[0]+x*math.cos(rotation)-y*math.sin(rotation),
                                  center[1]+x*math.sin(rotation)+y*math.cos(rotation),
                                  floor if floor is not None else center[2]]
                cell['center_source'] = 'recorded_grid_geometry'
    if not center or not side or floor is None:
        notes.append('缺少完整九宫格几何参数；仅展示已有坐标，不补造九宫格或目标位置。')
    notes.append('NED 世界坐标：+Z 向下；仅显示时转换为向上高度，CSV/JSON 坐标不变。')
    notes.append('锥桶采用仓库比赛模型尺寸：圆形 Ø0.28×0.50 m；方形 0.35×0.35×0.50 m。尺寸为展示模型，非录像测量。')
    notes.append('视觉 position 按锥桶几何中心定位；fallback 理论格点按底面定位。未记录标记朝向，贴片朝向采用九宫格 yaw；标记边长展示为 0.30 m。')
    assignment = mapping.get('final_assignment')
    if isinstance(assignment, dict):
        for key in assignment:
            cells.setdefault(str(key), {'id': int(key) if str(key).isdigit() else key})
    visual = []
    for key, cell in cells.items():
        kind = (assignment.get(key, assignment.get(int(key) if key.isdigit() else key)) if isinstance(assignment, dict)
                else cell.get('class_id', {'square_cone': 0, 'round_cone': 1}.get(cell.get('label'))))
        if kind not in (0, 1):
            continue
        position = xyz(cell.get('position'))
        source = cell.get('source', 'unknown')
        if not position and source == 'fallback':
            position = xyz(cell.get('center'))
        visual.append({'cell': key, 'kind': kind, 'position': position,
                       'source': source, 'anchor': 'base' if source == 'fallback' else 'center',
                       'accepted': cell.get('accepted_observations'), 'votes': cell.get('votes'),
                       'covariance': cell.get('covariance'), 'measurements': cell.get('measurements', [])})
    fallback_assignment = {**dict.fromkeys(map(str, config.get('fallback_square_cells', [])), 0),
                           **dict.fromkeys(map(str, config.get('fallback_round_cells', [])), 1)}
    conclusion = mapping.get('conclusion')
    submitted = None
    if isinstance(conclusion, dict):
        submitted = conclusion.get('assignment')
        if submitted is None and ('square_cells' in conclusion or 'round_cells' in conclusion):
            submitted = {**dict.fromkeys(map(str, conclusion.get('square_cells', [])), 0),
                         **dict.fromkeys(map(str, conclusion.get('round_cells', [])), 1)}
    submitted_cells = {str(cell.get('id', index)): cell
                       for index, cell in cells_of(conclusion or {})}

    def configured_objects(chosen, actual=False):
        objects = []
        for key, kind in (chosen or {}).items():
            if kind not in (0, 1):
                continue
            recorded = submitted_cells.get(str(key), {}) if actual else {}
            position = xyz(recorded.get('position')) or xyz(cells.get(str(key), {}).get('center'))
            objects.append({'cell': str(key), 'kind': kind, 'position': position,
                            'source': recorded.get('source', (conclusion or {}).get('source', 'unknown')) if actual else 'configured_fallback',
                            'anchor': 'base', 'measurements': []})
        return objects

    dictionary = config.get('tag_dictionary')

    def tag_view(value):
        if not isinstance(value, dict):
            return None
        tag = copy.deepcopy(value)
        tag['position'] = xyz(tag.get('position'))
        tag['dictionary'] = dictionary
        tag['pattern'] = marker_pattern(tag.get('id'), dictionary)
        return tag

    fallback_tag = None
    if all(key in config for key in ('fallback_tag_id', 'tag_x', 'tag_y', 'floor_z')):
        fallback_tag = tag_view({'id': config['fallback_tag_id'], 'source': 'configured_fallback',
                                 'position': [config['tag_x'], config['tag_y'], config['floor_z']]})
    return {'state': mapping.get('state'), 'time_ns': mapping.get('receive_time_unix_ns', mapping.get('stamp_ns')),
            'frame_id': mapping.get('frame_id', 'recorded_world'), 'verified': mapping.get('verified_complete', False),
            'grid': {'center': center, 'side': side, 'yaw': yaw, 'floor': floor},
            'cells': list(cells.values()), 'notes': notes,
            'modes': {'visual': {'objects': visual, 'tag': tag_view(mapping.get('tag')), 'available': True},
                      'fallback': {'objects': configured_objects(fallback_assignment), 'tag': fallback_tag,
                                   'available': bool(config)},
                      'submission': {'objects': configured_objects(submitted, True),
                                     'tag': tag_view((conclusion or {}).get('tag')),
                                     'available': conclusion is not None, 'evidence': conclusion}}}


def export_scene3d(output, maps, poses):
    from .analyze import write_json
    scenes = [scene_from_map(mapping) for mapping in maps]
    trajectory = [dict(position=[row['pose'][axis] for axis in ('x', 'y', 'z')])
                  for row in poses[::max(1, len(poses)//2000)]]
    data = {'scenes': scenes, 'trajectory': trajectory}
    write_json(output / 'map3d.json', data)
    template = Path(__file__).with_name('map3d_view.html').read_text(encoding='utf-8')
    safe_data = json.dumps(data, ensure_ascii=False, allow_nan=False).replace('<', '\\u003c')
    (output / 'map3d.html').write_text(template.replace('__SCENE_DATA__', safe_data), encoding='utf-8')
    return ('<section id="map3d"><h2>三维九宫格 · 录制数据还原</h2>'
            '<iframe title="三维建图" src="map3d.html" style="width:100%;height:840px;border:0;border-radius:12px"></iframe>'
            '<p><a href="map3d.html">独立打开三维场景</a> · '
            '<a href="map3d.json">三维场景坐标与来源 JSON</a></p></section>')
