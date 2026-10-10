"""Offline session report: maps, trajectory, videos, task clocks and AprilTags.

Run without ROS: python3 -m uv_record.analyze SESSION --output REPORT.
The optional bag fallback only imports ROS when readable JSONL indexes are absent.
"""

from __future__ import annotations

import argparse
import ast
import copy
import csv
import html
import json
import math
import os
import re
import shutil
import sqlite3
import subprocess
import tempfile
from datetime import datetime, timedelta, timezone
from pathlib import Path


TASK_START = re.compile(r'\[(\d+)/(\d+)\]\s+(\S+)\s+参数=')
TASK_FAILURE = re.compile(r'\[(\d+)/(\d+)\]\s+(\S+)\s+(执行失败|发生异常)')
PHASE = re.compile(r'抓前计数|抓后计数|下压|XY锚点|XY恢复|DVL|置物框|释放|放置|成功|失败|超时')


def read_jsonl(path, warnings=None):
    values = []
    if not Path(path).is_file():
        return values
    with Path(path).open(encoding='utf-8', errors='replace') as stream:
        for number, line in enumerate(stream, 1):
            try:
                value = json.loads(line)
                if not isinstance(value, dict):
                    raise ValueError('expected an object')
                values.append(value)
            except (ValueError, TypeError):
                if warnings is not None:
                    warnings.append(f'{path.name}:{number} 无效/截断 JSON，已跳过')
    return values


def timestamp(value):
    # Pose/map source clocks can restart or freeze. Plot on receive clocks, and
    # retain all original clocks in the exported records.
    return int(value.get('receive_time_unix_ns') or value.get('stamp_ns')
               or value.get('timeline_ns') or float(value.get('stamp', 0))*1e9)


def clock_text(ns, offset=8):
    if not ns:
        return ''
    return datetime.fromtimestamp(ns/1e9, timezone(timedelta(hours=offset))).isoformat(
        timespec='milliseconds')


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2,
                                     allow_nan=False), encoding='utf-8')


def write_csv(path, rows, fields):
    with Path(path).open('w', encoding='utf-8-sig', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        for row in rows:
            writer.writerow({key: json.dumps(value, ensure_ascii=False)
                             if isinstance(value, (dict, list)) else value
                             for key, value in row.items()})


def bag_fallback(root, missing, warnings):
    """Read a private SQLite backup, never touch the recording's WAL/SHM.

    Requires the recording's generated ROS message packages. No DDS context or
    publishers are created. MCAP-only sessions need their JSONL indexes.
    """
    result = {kind: [] for kind in missing}
    databases = sorted((root/'bag').rglob('*.db3'))
    if not databases or not missing:
        return result
    try:
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        from rosidl_runtime_py.convert import message_to_ordereddict
    except ImportError:
        warnings.append('部分 JSONL 缺失；未加载 ROS 消息包，无法从 SQLite bag 补读')
        return result
    topics = {'/rosout': 'logs', '/basic_motion/pose_info': 'poses',
              '/auv/basic_motion/pose_info': 'poses', '/task/mapping/map': 'maps',
              '/perception/mapping/observations': 'observations', '/task/status': 'statuses'}
    with tempfile.TemporaryDirectory(prefix='uv-record-analyze-') as temp:
        for number, database in enumerate(databases):
            copied = Path(temp)/f'{number}.db3'
            shutil.copy2(database, copied)
            for suffix in ('-wal', '-shm'):
                sidecar = Path(str(database)+suffix)
                if sidecar.is_file():
                    shutil.copy2(sidecar, str(copied)+suffix)
            connection = sqlite3.connect(str(copied))
            try:
                types = {}
                for topic_id, name, type_name in connection.execute('SELECT id,name,type FROM topics'):
                    kind = topics.get(name)
                    if kind in missing:
                        try:
                            types[topic_id] = (kind, get_message(type_name))
                        except (ImportError, AttributeError, ValueError) as error:
                            warnings.append(f'无法解码 {name}: {error}')
                if not types:
                    continue
                placeholders = ','.join('?' for _ in types)
                query = f'SELECT topic_id,timestamp,data FROM messages WHERE topic_id IN ({placeholders}) ORDER BY timestamp'
                for topic_id, ns, payload in connection.execute(query, tuple(types)):
                    kind, cls = types[topic_id]
                    try:
                        msg = deserialize_message(payload, cls)
                        if kind == 'logs':
                            value = {'stamp_ns': msg.stamp.sec*10**9+msg.stamp.nanosec,
                                     'name': msg.name, 'level': msg.level, 'message': msg.msg}
                        elif kind == 'poses':
                            value = {'stamp_ns': msg.stamp.sec*10**9+msg.stamp.nanosec,
                                     'pose': {axis: float(getattr(msg, 'robot_'+axis))
                                              for axis in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')}}
                        elif kind == 'maps':
                            value = json.loads(msg.data)
                        else:
                            value = dict(message_to_ordereddict(msg))
                        value['receive_time_unix_ns'] = ns
                        value['clock_source'] = 'bag_receive_timestamp'
                        result[kind].append(value)
                    except (ValueError, TypeError, RuntimeError) as error:
                        warnings.append(f'{database.name}: 无法解码消息: {error}')
            except sqlite3.DatabaseError as error:
                warnings.append(f'{database.name}: bag 读取失败: {error}')
            finally:
                connection.close()
    return result


def task_timeline(logs, session_end):
    tasks, phases, chains = [], [], []
    active = None
    run = 0

    def finish(ns, status, inferred=False):
        nonlocal active
        if active is not None:
            active.update(end_ns=ns, duration_s=max(0., (ns-active['start_ns'])/1e9),
                          status=status, end_inferred=inferred)
            tasks.append(active)
            active = None

    for log in sorted(logs, key=timestamp):
        message, ns = log.get('message', ''), timestamp(log)
        event_task = active['name'] if active else ''
        if '任务列表开始执行' in message:
            finish(ns, 'interrupted_by_restart')
            run += 1
            chains.append({'run': run, 'start_ns': ns, 'status': 'incomplete'})
        start = TASK_START.search(message)
        failed = TASK_FAILURE.search(message)
        if start:
            # The runner logs failures explicitly, but successful tasks have no
            # end event. The next start provides an upper bound, not exact end.
            finish(ns, 'completed_inferred', inferred=True)
            active = {'run': max(1, run), 'index': int(start[1]), 'total': int(start[2]),
                      'name': start[3], 'start_ns': ns, 'start_message': message}
            event_task = start[3]
        elif failed and active is not None and int(failed[1]) == active['index']:
            finish(ns, 'failed')
        elif any(word in message for word in ('任务列表执行完成', '任务列表执行结束', '任务列表已停止')):
            stopped = '已停止' in message
            finish(ns, 'stopped' if stopped else 'completed_inferred', inferred=not stopped)
            if chains:
                chains[-1].update(end_ns=ns, status=('stopped' if stopped else
                                                    'failed' if '个任务失败' in message else 'completed'))
        if ('task' in log.get('name', '') or
                (PHASE.search(message) and 'motion' in log.get('name', ''))):
            phases.append({'timestamp_ns': ns, 'source_stamp_ns': log.get('stamp_ns'),
                           'run': run, 'task': event_task,
                           'message': message})
    finish(session_end, 'incomplete_at_recording_end')
    return tasks, phases, chains


def apriltag_results(maps, observations, logs):
    results = []
    for log in logs:
        message = log.get('message', '')
        if 'AprilTag诊断' not in message:
            continue
        def ids(pattern):
            match = re.search(pattern+r'=\[([^\]]*)\]', message)
            return [int(x) for x in re.findall(r'-?\d+', match[1])] if match else []
        def count(pattern):
            match = re.search(pattern+r'=(\d+)', message)
            return int(match[1]) if match else None
        decoded, excluded = ids('解码ID'), ids('被ID条件排除')
        dictionary = re.search(r'字典=([^，；\s]+)', message)
        results.append({'timestamp_ns': timestamp(log), 'source': 'online_diagnostic',
                        'decoded_ids': decoded, 'excluded_ids': excluded,
                        'undecoded_candidates': count('未解码候选数'),
                        'depth_rejected': count('深度失败'),
                        'dictionary': dictionary[1] if dictionary else '',
                        'status': 'decoded' if decoded else 'no_decode'})
    for frame in observations:
        for observation in frame.get('observations', []):
            if observation.get('kind') != 1:
                continue
            results.append({'timestamp_ns': timestamp(frame), 'source': 'online_world_observation',
                            'decoded_ids': [observation['tag_id']], 'status': 'world_observed',
                            'position': [observation.get('world_'+axis) for axis in ('x', 'y', 'z')],
                            'depth_m': observation.get('depth_m'),
                            'pose_age_s': observation.get('pose_age_s')})
    previous = None
    for mapping in maps:
        tag = mapping.get('tag')
        if not tag or tag == previous:
            continue
        previous = tag
        results.append({'timestamp_ns': timestamp(mapping), 'source': 'fused_map',
                        'decoded_ids': [tag.get('id')], 'position': tag.get('position'),
                        'observations': tag.get('observations'),
                        'status': 'fallback' if tag.get('source') == 'fallback' else 'fused_world_position'})
    return sorted(results, key=lambda x: x['timestamp_ns'])


def map_runs(maps, logs=()):
    """Keep the last available map of each reset, even if no final map exists."""
    runs, current, previous_count = [], None, 0
    for mapping in maps:
        count = int(mapping.get('measurement_count') or 0)
        reset = current is not None and (count < previous_count or
                 (mapping.get('state') == 'travel_to_tag' and current.get('state') != 'travel_to_tag'))
        if reset:
            runs.append(current)
        current, previous_count = mapping, count
    if current is not None:
        runs.append(current)
    # Older 1 Hz indexes dropped the final map; the task's constraint result
    # still exists in rosout. Recover only the category decision, never invent
    # final positions, covariance or a successful verification flag.
    runs = copy.deepcopy(runs)
    for log in logs:
        match = re.search(r'建图最终结果[^：]*：\s*(\{[^}]*\})', log.get('message', ''))
        if not match or not runs:
            continue
        try:
            assignment = ast.literal_eval(match[1])
        except (ValueError, SyntaxError):
            continue
        if not isinstance(assignment, dict) or not all(
                isinstance(key, int) and value in (0, 1) for key, value in assignment.items()):
            continue
        mapping = min(runs, key=lambda item: abs(timestamp(item)-timestamp(log)))
        assignment = {str(key): value for key, value in assignment.items()}
        mapping['logged_final_assignment'] = assignment
        mapping['final_assignment_time_ns'] = timestamp(log)
        if mapping.get('final_assignment') is not None:
            mapping['final_assignment_source'] = 'map_snapshot'
            continue
        mapping['final_assignment'] = assignment
        mapping['final_assignment_source'] = 'task_result_log'
        for key, cell in cells_of(mapping):
            cell['final_selected'] = str(key) in assignment
            if str(key) in assignment:
                cell['snapshot_label'] = cell.get('label')
                cell['snapshot_class_id'] = cell.get('class_id')
                cell['class_id'] = assignment[str(key)]
                cell['label'] = 'square_cone' if assignment[str(key)] == 0 else 'round_cone'
                cell['label_source'] = 'task_result_log'
    return runs


def cells_of(mapping):
    cells = mapping.get('cells', {})
    return cells.items() if isinstance(cells, dict) else enumerate(cells)


def spatial_svg(poses, mapping=None, title='轨迹', tags=()):
    mapping = mapping or {}
    points = [[row['pose']['x'], row['pose']['y']] for row in poses]
    for _, cell in cells_of(mapping):
        for key in ('center', 'position'):
            if cell.get(key):
                points.append(cell[key][:2])
    for tag in tags:
        if tag.get('position'):
            points.append(tag['position'][:2])
    points = [p for p in points if len(p) >= 2 and all(isinstance(v, (int, float)) and math.isfinite(v) for v in p[:2])]
    if not points:
        points = [[0., 0.], [1., 1.]]
    lower = [min(p[i] for p in points)-.4 for i in range(2)]
    upper = [max(p[i] for p in points)+.4 for i in range(2)]
    scale = min(820/max(upper[0]-lower[0], 1.), 440/max(upper[1]-lower[1], 1.))
    def pixel(p):
        return (70+(p[0]-lower[0])*scale, 520-(p[1]-lower[1])*scale)
    pieces = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 960 580" role="img">',
              '<rect width="960" height="580" fill="#101b2a"/>',
              f'<text x="30" y="30" fill="white">{html.escape(title)} · X/Y 米，NED；Z 向下</text>']
    for i, axis in enumerate(('X', 'Y')):
        pieces.append(f'<text x="{870 if i == 0 else 20}" y="{555 if i == 0 else 70}" fill="#bbb">{axis} (m)</text>')
    for x in [lower[0]+(upper[0]-lower[0])*i/8 for i in range(9)]:
        px, _ = pixel([x, lower[1]])
        pieces.append(f'<path d="M{px:.2f} 65V520" stroke="#243449"/><text x="{px:.2f}" y="542" fill="#aab">{x:.2f}</text>')
    for y in [lower[1]+(upper[1]-lower[1])*i/8 for i in range(9)]:
        _, py = pixel([lower[0], y])
        pieces.append(f'<path d="M70 {py:.2f}H900" stroke="#243449"/><text x="35" y="{py:.2f}" fill="#aab">{y:.2f}</text>')
    if poses:
        line = ' '.join(f'{pixel([p["pose"]["x"],p["pose"]["y"]])[0]:.2f},{pixel([p["pose"]["x"],p["pose"]["y"]])[1]:.2f}' for p in poses)
        pieces.append(f'<polyline points="{line}" fill="none" stroke="#38bdf8" stroke-width="2"/>')
        for row, label, color in ((poses[0], '起点', '#4ade80'), (poses[-1], '终点', '#fb7185')):
            x, y = pixel([row['pose']['x'], row['pose']['y']])
            pieces.append(f'<circle cx="{x}" cy="{y}" r="5" fill="{color}"/><text x="{x+8}" y="{y-8}" fill="{color}">{label}</text>')
        pieces.append('<circle id="cursor" r="7" fill="#facc15" visibility="hidden"/>')
    side = float(mapping.get('grid', {}).get('side_m') or 0)/3*scale
    yaw = -float(mapping.get('grid', {}).get('yaw_deg') or 0)
    for key, cell in cells_of(mapping):
        if cell.get('center') and side:
            x, y = pixel(cell['center'])
            pieces.append(f'<rect x="{x-side/2}" y="{y-side/2}" width="{side}" height="{side}" transform="rotate({yaw} {x} {y})" fill="none" stroke="#64748b"/>')
        position = cell.get('position') or cell.get('center')
        if not position:
            continue
        x, y = pixel(position)
        color = '#94a3b8' if cell.get('source') != 'vision' or cell.get('final_selected') is False else '#fb923c' if cell.get('label') == 'square_cone' else '#2dd4bf'
        shape = (f'<rect x="{x-7}" y="{y-7}" width="14" height="14" fill="{color}"/>'
                 if cell.get('label') == 'square_cone' else f'<circle cx="{x}" cy="{y}" r="7" fill="{color}"/>')
        label = f"格{key} {cell.get('label') or '未知'} ({cell.get('source', 'none')})"
        if cell.get('final_selected') is False:
            label += ' 未选中'
        pieces.append(f'<g><title>{html.escape(label)}</title>{shape}<text x="{x+10}" y="{y}" fill="white" font-size="12">{html.escape(label)}</text></g>')
    latest_tags = {str(tag.get('decoded_ids', [tag.get('id')])): tag
                   for tag in tags if tag.get('position')}
    for tag in tags:
        if not tag.get('position') or len(tag['position']) < 2:
            continue
        x, y = pixel(tag['position'])
        ids = tag.get('decoded_ids', [tag.get('id')])
        if tag is latest_tags.get(str(ids)):
            pieces.append(f'<path d="M{x-8} {y}h16M{x} {y-8}v16" stroke="#facc15" stroke-width="3"/><text x="{x+10}" y="{y-10}" fill="#facc15">Tag {html.escape(str(ids))}</text>')
        else:
            pieces.append(f'<circle cx="{x}" cy="{y}" r="2" fill="#facc15" opacity=".5"><title>{html.escape(str(tag))}</title></circle>')
    pieces.append('</svg>')
    return ''.join(pieces), {'lower': lower, 'scale': scale}


def timeline_svg(rows, start, end, tags=False):
    width = 1000
    lanes = {}
    if tags:
        for row in rows:
            key = str(row.get('decoded_ids', []))+' '+row.get('source', '')+' '+row.get('status', '')
            if key not in lanes:
                lanes[key] = len(lanes)
    height = max(150, 75+(len(lanes) if tags else len(rows))*32)
    span = max(1, end-start)
    pieces = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}">',
              f'<rect width="{width}" height="{height}" fill="#101b2a"/>']
    for tick in range(6):
        x = 260+tick*140
        pieces.append(f'<path d="M{x} 40V{height-20}" stroke="#243449"/><text x="{x}" y="25" fill="white">{(end-start)/1e9*tick/5:.1f}s</text>')
    emitted = set()
    for i, row in enumerate(rows):
        key = str(row.get('decoded_ids', []))+' '+row.get('source', '')+' '+row.get('status', '')
        y = 60+(lanes[key] if tags else i)*32
        a = row.get('timestamp_ns') if tags else row['start_ns']
        b = a if tags else row['end_ns']
        x = 260+(a-start)/span*700
        w = max(4, (b-a)/span*700)
        label = key if tags else f"{row['run']}.{row['index']} {row['name']}"
        color = '#fb7185' if 'fail' in row.get('status', '') else '#94a3b8' if ('incomplete' in row.get('status', '') or row.get('status') == 'no_decode') else '#2dd4bf'
        if not tags or key not in emitted:
            pieces.append(f'<text x="10" y="{y+6}" fill="white" font-size="11">{html.escape(label)}</text>')
            emitted.add(key)
        pieces.append(f'<rect x="{x}" y="{y-8}" width="{w}" height="16" fill="{color}"><title>{html.escape(str(row))}</title></rect>')
    pieces.append('</svg>')
    return ''.join(pieces)


def video_records(root, output, ffmpeg, warnings):
    """Remux HLS parts to browser-playable MP4; never merge restart gaps."""
    records = []
    directories = sorted(p for p in (root/'video').glob('*') if p.is_dir())
    for directory in directories:
        alignment = read_jsonl(directory/'frame_alignment.jsonl', warnings)
        status_path = directory/'status.json'
        try:
            status = json.loads(status_path.read_text())
        except (OSError, ValueError):
            status = {}
        files = [p for p in directory.glob('*') if p.suffix in ('.ts', '.mjpg') and p.stat().st_size > 0]
        record = {'camera': directory.name, 'files': [str(p) for p in sorted(files)],
                  'bytes': sum(p.stat().st_size for p in files), 'saved_files': len(files),
                  'alignment_entries': len(alignment), 'status': status,
                  'alignment': 'aligned' if alignment and all(x.get('timestamp_aligned') for x in alignment) else 'degraded',
                  'playbacks': [], 'incomplete_tails': [str(p) for p in directory.glob('*.ts.tmp')]}
        receive_times = [timestamp(entry) for entry in alignment if timestamp(entry)]
        if receive_times:
            record.update(receive_start_ns=min(receive_times), receive_end_ns=max(receive_times))
        if not files:
            warnings.append(f'{directory.name}: 没有实际视频/图像文件；解码索引不能还原像素')
        if files and not ffmpeg:
            warnings.append(f'{directory.name}: 未找到 FFmpeg，报告保留源视频链接但不生成 MP4')
        playlists = sorted(directory.glob('*.m3u8'))
        referenced = set()
        for number, playlist in enumerate(playlists):
            try:
                names = [line.strip() for line in playlist.read_text().splitlines()
                         if line.strip() and not line.startswith('#')]
            except OSError:
                continue
            source_files = [directory/name for name in names]
            if not source_files or any(not p.is_file() for p in source_files):
                warnings.append(f'{playlist.name}: 空或缺失分段，未生成播放副本')
                continue
            referenced.update(p.resolve() for p in source_files)
            target = output/'videos'/f'{directory.name}_{number:03d}.mp4'
            if ffmpeg:
                command = [ffmpeg, '-hide_banner', '-loglevel', 'error', '-nostdin', '-y',
                           '-i', str(playlist), '-c', 'copy', '-movflags', '+faststart', str(target)]
                try:
                    result = subprocess.run(command, capture_output=True, timeout=120)
                    if result.returncode or not target.is_file() or not target.stat().st_size:
                        raise RuntimeError(result.stderr.decode(errors='replace')[-1000:])
                except (OSError, subprocess.SubprocessError, RuntimeError) as error:
                    warnings.append(f'{playlist.name}: MP4 导出失败: {error}')
                    target.unlink(missing_ok=True)
            part = {'source': str(playlist), 'source_files': [str(p) for p in source_files],
                    'mp4': str(target.relative_to(output)) if target.is_file() else None,
                    'clock': 'source_capture' if record['alignment'] == 'aligned' else 'receive_anchored_pts_approximate'}
            # A decoder index resets to zero at each restart. Match attempt order
            # to playlists instead of assigning all parts the session start.
            attempts = []
            for entry in alignment:
                if not attempts or entry.get('decoder_frame_index') == 0:
                    attempts.append([])
                attempts[-1].append(entry)
            entries = [entry for entry in alignment if entry.get('video_playlist') == playlist.name]
            if not entries and number < len(attempts) and not any(
                    entry.get('video_playlist') for entry in alignment):
                entries = attempts[number]
                part['clock'] += '_legacy_attempt_order_inferred'
                warnings.append(f'{playlist.name}: 旧索引未关联 playlist，视频起点按重启顺序推断')
            if entries:
                part['start_ns'] = entries[0].get('source_timestamp_ns') or entries[0].get('replay_timestamp_ns') or timestamp(entries[0])
                part['end_ns'] = entries[-1].get('source_timestamp_ns') or entries[-1].get('replay_timestamp_ns') or timestamp(entries[-1])
            record['playbacks'].append(part)
        orphaned = [p for p in files if p.suffix == '.ts' and p.resolve() not in referenced]
        if orphaned:
            warnings.append(f'{directory.name}: {len(orphaned)} 个孤立 TS 分段，仅提供原文件；时间轴未知')
        jpeg_files = [p for p in files if p.suffix == '.mjpg']
        if jpeg_files:
            from .jpeg_archive import archive_entries, read_payload
            target = output/'videos'/f'{directory.name}_jpeg.mp4'
            entries = list(archive_entries(directory))
            # Render a real-time CFR review copy; archival capture stamps remain
            # authoritative. Duplicate the previous frame over capture gaps.
            if ffmpeg and entries:
                fps = 8.0
                log = output/'videos'/f'{directory.name}_encode.log'
                try:
                    with log.open('wb') as stderr:
                        process = subprocess.Popen([ffmpeg, '-hide_banner', '-loglevel', 'error',
                            '-nostdin', '-y', '-f', 'image2pipe', '-framerate', str(fps),
                            '-vcodec', 'mjpeg', '-i', 'pipe:0', '-an', '-c:v', 'libx264',
                            '-preset', 'ultrafast', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(target)],
                            stdin=subprocess.PIPE, stderr=stderr)
                        def frame_ns(entry):
                            meta = entry.metadata or {}
                            return (meta.get('source_timestamp_ns') or entry.timestamp_ns
                                    or meta.get('replay_timestamp_ns') or meta.get('receive_time_unix_ns', 0))
                        first = frame_ns(entries[0])
                        frame_number, previous = 0, None
                        for entry in entries:
                            ns = frame_ns(entry) or first
                            desired = max(frame_number, int(round((ns-first)/1e9*fps)))
                            if desired-frame_number > fps*600:
                                raise ValueError('JPEG 时钟间隙超过 10 分钟，请逐段回放原归档')
                            payload = read_payload(entry)
                            while frame_number < desired and previous:
                                process.stdin.write(previous)
                                frame_number += 1
                            process.stdin.write(payload)
                            frame_number += 1
                            previous = payload
                        process.stdin.close()
                        if process.wait(timeout=120):
                            raise RuntimeError(log.read_text()[-1000:])
                    record['playbacks'].append({'source': str(directory), 'mp4': str(target.relative_to(output)),
                                                'start_ns': first, 'end_ns': frame_ns(entries[-1]),
                                                'clock': 'CFR_review_copy'})
                except (OSError, subprocess.SubprocessError, ValueError, RuntimeError) as error:
                    warnings.append(f'{directory.name}: JPEG 播放副本失败: {error}')
                    if 'process' in locals() and process.poll() is None:
                        process.kill()
                        process.wait()
                    target.unlink(missing_ok=True)
        records.append(record)
    return records


def detect_video_tags(records, output, dictionary_name, fps, max_images, warnings):
    """Offline redetection on available review video, with pixel-corner exports."""
    import cv2
    if not hasattr(cv2, 'aruco') or not hasattr(cv2.aruco, dictionary_name):
        warnings.append(f'OpenCV 缺少 aruco/{dictionary_name}，跳过离线图像 AprilTag 解码')
        return []
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    if hasattr(cv2.aruco, 'ArucoDetector'):
        detector = cv2.aruco.ArucoDetector(dictionary)
        detect = detector.detectMarkers
    else:
        detect = lambda image: cv2.aruco.detectMarkers(image, dictionary)
    results, saved = [], 0
    for record in records:
        for part in record['playbacks']:
            if not part.get('mp4'):
                continue
            capture = cv2.VideoCapture(str(output/part['mp4']))
            video_fps = capture.get(cv2.CAP_PROP_FPS)
            if not capture.isOpened() or not math.isfinite(video_fps) or video_fps <= 0:
                warnings.append(f'{part["mp4"]}: OpenCV 无法读取；视频文件仍可单独播放')
                capture.release()
                continue
            index, next_sample = 0, 0.
            while saved < max_images:
                ok, image = capture.read()
                if not ok:
                    break
                t = capture.get(cv2.CAP_PROP_POS_MSEC)/1000
                if not math.isfinite(t) or (t == 0 and index):
                    t = index/video_fps
                index += 1
                if t+1e-6 < next_sample:
                    continue
                next_sample = t+1/fps
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                corners, ids, rejected = detect(gray)
                if ids is None:
                    continue
                cv2.aruco.drawDetectedMarkers(image, corners, ids)
                ns = int(part.get('start_ns', 0)+t*1e9) if part.get('start_ns') else 0
                cv2.putText(image, f'{record["camera"]} +{t:.3f}s {dictionary_name}',
                            (12, 28), cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 255, 255), 2)
                relative = f'apriltag/{record["camera"]}_{saved:05d}.jpg'
                if not cv2.imwrite(str(output/relative), image):
                    warnings.append(f'{relative}: 无法保存可视化图片')
                    continue
                results.append({'timestamp_ns': ns, 'source': 'offline_video_redetection',
                                'decoded_ids': [int(x) for x in ids.flatten()], 'dictionary': dictionary_name,
                                'camera': record['camera'], 'video': part['mp4'], 'video_time_s': t,
                                'corners_px': [corner.reshape(-1, 2).tolist() for corner in corners],
                                'undecoded_candidates': len(rejected), 'image': relative,
                                'status': 'offline_decoded', 'clock': part['clock']})
                saved += 1
            capture.release()
    return results


def table(rows, columns):
    header = ''.join(f'<th>{html.escape(title)}</th>' for _, title in columns)
    body = ''.join('<tr>'+''.join(f'<td>{html.escape(str(row.get(key, "")))}</td>'
                                  for key, _ in columns)+'</tr>' for row in rows)
    return '<div class="scroll"><table><thead><tr>'+header+'</tr></thead><tbody>'+body+'</tbody></table></div>'


def analyze(root, output, *, ffmpeg=None, offset=8, detect_tags=True,
            dictionary='DICT_APRILTAG_16h5', tag_fps=1., max_tag_images=200,
            use_bag=True):
    root, output = Path(root).resolve(), Path(output).resolve()
    if not (root/'manifest.json').is_file():
        raise ValueError(f'不是 uv_record session: {root}')
    if output.exists() and any(output.iterdir()):
        raise ValueError(f'输出目录非空，拒绝覆盖: {output}')
    if output == root or root in output.parents:
        raise ValueError('输出目录必须位于 session 外，保持原始证据不变')
    output.mkdir(parents=True, exist_ok=True)
    for name in ('videos', 'apriltag'):
        (output/name).mkdir()
    warnings = []
    manifest = json.loads((root/'manifest.json').read_text())
    relative = {'poses': 'metadata/trajectory.jsonl', 'maps': 'metadata/mapping.jsonl',
                'logs': 'logs/rosout.jsonl', 'observations': 'metadata/mapping_observations.jsonl',
                'statuses': 'metadata/tasks.jsonl'}
    data = {kind: read_jsonl(root/path, warnings) for kind, path in relative.items()}
    missing = [kind for kind, values in data.items() if not values]
    if use_bag:
        data.update({kind: values for kind, values in bag_fallback(root, missing, warnings).items() if values})
    for values in data.values():
        values.sort(key=timestamp)
    poses = [row for row in data['poses'] if isinstance(row.get('pose'), dict)
             and all(isinstance(row['pose'].get(a), (int, float)) and math.isfinite(row['pose'][a])
                     for a in ('x', 'y', 'z'))]
    videos = video_records(root, output, ffmpeg, warnings)
    times = [timestamp(row) for rows in data.values() for row in rows if timestamp(row)]
    times += [video[key] for video in videos for key in ('receive_start_ns', 'receive_end_ns')
              if video.get(key)]
    start, end = (min(times), max(times)) if times else (0, 0)
    tasks, phases, chains = task_timeline(data['logs'], end)
    if not poses:
        warnings.append('缺少有效轨迹记录')
    if not data['maps']:
        warnings.append('没有建图快照；本 session 无法输出实际建图结果')
    maps = map_runs(data['maps'], data['logs'])
    tags = apriltag_results(data['maps'], data['observations'], data['logs'])
    if detect_tags and any(part.get('mp4') for video in videos for part in video['playbacks']):
        tags.extend(detect_video_tags(videos, output, dictionary, tag_fps, max_tag_images, warnings))
        tags.sort(key=lambda x: x['timestamp_ns'])
    distance = sum(math.hypot(b['pose']['x']-a['pose']['x'], b['pose']['y']-a['pose']['y'])
                   for a, b in zip(poses, poses[1:]))
    summary = {'session': str(root), 'recording_status': manifest.get('status'),
               'start_ns': start, 'end_ns': end, 'start_local': clock_text(start, offset),
               'end_local': clock_text(end, offset), 'timezone_offset_hours': offset,
               'duration_s': (end-start)/1e9, 'pose_samples': len(poses),
               'sampled_xy_distance_m': distance, 'map_runs': len(maps), 'tasks': len(tasks),
               'chains': chains, 'apriltag_records': len(tags), 'warnings': warnings,
               'notes': ['XY 路长按位姿采样折线累加，含里程计漂移和坐标修正，不等同真实游动距离',
                         '任务成功结束通常由下一任务开始/链结束推断；末尾未结束任务标记 incomplete',
                         'fused_map 是融合结果；offline_video_redetection 是录像重新解码，不能冒充在线识别',
                         '没有像素文件时无法还原视频或 AprilTag 图像角点；源视频链接依赖原 session 位置']}
    write_json(output/'summary.json', summary)
    write_json(output/'maps.json', maps)
    write_json(output/'videos.json', videos)
    write_json(output/'apriltag.json', tags)
    write_json(output/'tasks.json', {'tasks': tasks, 'phases': phases, 'status_events': data['statuses']})
    write_csv(output/'trajectory.csv', [dict(timestamp_ns=timestamp(row), stamp_ns=row.get('stamp_ns'),
                                            time_local=clock_text(timestamp(row), offset), **row['pose']) for row in poses],
              ['timestamp_ns', 'stamp_ns', 'time_local', 'x', 'y', 'z', 'roll', 'pitch', 'yaw'])
    task_rows = [dict(row, start_local=clock_text(row['start_ns'], offset),
                      end_local=clock_text(row['end_ns'], offset)) for row in tasks]
    write_csv(output/'tasks.csv', task_rows, ['run', 'index', 'name', 'start_ns', 'end_ns',
                                            'start_local', 'end_local', 'duration_s', 'status', 'end_inferred'])
    write_csv(output/'task_events.csv', [dict(row, time_local=clock_text(row['timestamp_ns'], offset)) for row in phases],
              ['timestamp_ns', 'source_stamp_ns', 'time_local', 'run', 'task', 'message'])
    write_csv(output/'apriltag.csv', [dict(row, time_local=clock_text(row['timestamp_ns'], offset)) for row in tags],
              ['timestamp_ns', 'time_local', 'source', 'decoded_ids', 'status', 'position', 'depth_m',
               'excluded_ids', 'undecoded_candidates', 'depth_rejected', 'dictionary', 'camera', 'image', 'corners_px'])
    tag_positions = [tag for tag in tags if tag.get('position')]
    path_svg, transform = spatial_svg(poses, title='实际记录的 XY 路径')
    (output/'trajectory.svg').write_text(path_svg, encoding='utf-8')
    map_views = []
    for i, mapping in enumerate(maps):
        svg, _ = spatial_svg([], mapping, f'建图第 {i+1} 次 · {mapping.get("state", "")}',
                             [mapping['tag']] if mapping.get('tag') else [])
        (output/f'map_{i+1:02d}.svg').write_text(svg, encoding='utf-8')
        rows = [dict(cell_id=key, **cell) for key, cell in cells_of(mapping)]
        write_csv(output/f'map_{i+1:02d}.csv', rows, ['cell_id', 'label', 'class_id', 'center', 'position',
                  'source', 'label_source', 'final_selected', 'visited', 'accepted_observations', 'covariance'])
        final_text = ''
        if mapping.get('final_assignment') is not None:
            final_text = (f'<p>最终分类（0=方形，1=圆形）：{html.escape(str(mapping["final_assignment"]))}；'
                          f'来源：{mapping.get("final_assignment_source", "map_snapshot")}；'
                          f'时间：{clock_text(mapping.get("final_assignment_time_ns", timestamp(mapping)), offset)}。</p>')
            if mapping.get('final_assignment_source') == 'task_result_log':
                final_text += '<p class="warning">最终类别从任务日志恢复；位置/协方差取最后可用过程快照，未捕获最终地图快照。灰色候选未被最终约束选中。</p>'
        map_views.append(f'<h3>建图第 {i+1} 次 · {html.escape(mapping.get("state", ""))}</h3>'
                         f'<p>快照时间：{clock_text(timestamp(mapping), offset)}；verified_complete={mapping.get("verified_complete", False)}；'
                         f'fallback_used={mapping.get("fallback_used", False)}；灰色为未知、默认配置或最终未选中。</p>'+final_text+svg+
                         table(rows, [('cell_id', '格'), ('label', '类别'), ('source', '来源'),
                                      ('position', '位置'), ('final_selected', '最终选中'),
                                      ('accepted_observations', '接受观测数')]))
    tasks_svg = timeline_svg(tasks, start, end)
    (output/'tasks.svg').write_text(tasks_svg, encoding='utf-8')
    tags_svg = timeline_svg(tags, start, end, tags=True)
    (output/'apriltag_timeline.svg').write_text(tags_svg, encoding='utf-8')
    positions_svg, _ = spatial_svg([], title='AprilTag 世界坐标融合/观测', tags=tag_positions)
    (output/'apriltag_positions.svg').write_text(positions_svg, encoding='utf-8')
    video_views = []
    for video in videos:
        video_views.append(f'<h3>{html.escape(video["camera"])}</h3><p>实际文件 {video["saved_files"]} 个 · '
                           f'{video["bytes"]/1e6:.2f} MB · 同步 {video["alignment"]}</p>')
        for part in video['playbacks']:
            if part.get('mp4'):
                video_views.append(f'<video controls preload="metadata" src="{html.escape(part["mp4"])}"></video>'
                    f'<p>开始：{clock_text(part.get("start_ns", 0), offset) or "未知"}；{html.escape(part["clock"])}</p>')
        if not video['saved_files']:
            video_views.append('<p class="warning">此摄像头没有留下像素数据，无法播放或恢复。</p>')
        links = video['files'][:100]
        video_views.append('<details><summary>原录像文件链接（全部清单见 videos.json）</summary>'+''.join(
            f'<p><a href="{html.escape(os.path.relpath(path, output))}">{html.escape(Path(path).name)}</a></p>' for path in links)+'</details>')
    tag_images = ''.join(f'<figure><a href="{row["image"]}"><img loading="lazy" src="{row["image"]}"></a>'
                         f'<figcaption>{html.escape(row["camera"])} +{row["video_time_s"]:.3f}s · '
                         f'ID {row["decoded_ids"]} · 离线重新识别</figcaption></figure>'
                         for row in tags if row.get('image'))
    decoded_ids = sorted({tag_id for row in tags if row.get('status') != 'fallback'
                          for tag_id in row.get('decoded_ids', []) if isinstance(tag_id, int)})
    report_data = {'poses': [[timestamp(row), row['pose']['x'], row['pose']['y'], row['pose']['z']]
                             for row in poses[::max(1, len(poses)//12000)]],
                   'transform': transform, 'offset': offset}
    safe_json = json.dumps(report_data).replace('<', '\\u003c')
    content = f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<title>Session {html.escape(root.name)} 解读</title>
<style>body{{margin:30px auto;max-width:1200px;background:#0b1220;color:#e2e8f0;font:16px/1.65 sans-serif}}a{{color:#7dd3fc}}svg{{width:100%;max-height:650px}}section{{padding:20px;background:#162033;margin:22px 0;border-radius:12px}}.warning{{color:#fda4af}}table{{border-collapse:collapse;width:100%;font-size:14px}}td,th{{border:1px solid #334155;padding:8px;vertical-align:top}}.scroll{{overflow:auto;max-height:520px}}video{{width:100%;max-height:600px}}input{{width:100%}}.gallery{{display:flex;flex-wrap:wrap}}figure{{width:320px;margin:8px}}img{{width:100%}}nav{{display:flex;gap:20px;flex-wrap:wrap}}p{{overflow-wrap:anywhere}}</style>
<h1>Session {html.escape(root.name)} 解读</h1><p>{summary['start_local']} → {summary['end_local']} · {summary['duration_s']:.1f}s · 位姿 {len(poses)} 条 · 建图 {len(maps)} 次 · 任务 {len(tasks)} 个</p>
<nav><a href="#map">建图</a><a href="#path">路径</a><a href="#video">视频</a><a href="#tasks">任务时间戳</a><a href="#tags">AprilTag</a><a href="summary.json">完整导出清单</a></nav>
<section><h2>数据完整性</h2><ul>{''.join('<li class="warning">'+html.escape(w)+'</li>' for w in warnings) or '<li>未发现索引/导出异常</li>'}</ul><ul>{''.join('<li>'+html.escape(n)+'</li>' for n in summary['notes'])}</ul></section>
<section id="map"><h2>建图结果</h2>{''.join(map_views) or '<p>没有建图记录。</p>'}<a href="maps.json">地图 JSON</a></section>
<section id="path"><h2>路径</h2><p>采样 XY 折线路长 {distance:.2f}m；包括漂移和坐标修正。</p><div id="path-plot">{path_svg}</div><input id="scrub" type="range" min="0" max="{max(0,len(report_data['poses'])-1)}" value="0"><p id="pose-label"></p><a href="trajectory.csv">完整轨迹 CSV</a></section>
<section id="video"><h2>视频记录</h2>{''.join(video_views) or '<p>没有视频目录。</p>'}<a href="videos.json">视频清单及状态</a></section>
<section id="tasks"><h2>各任务时间戳</h2>{tasks_svg}{table(task_rows, [('run','轮次'),('index','序号'),('name','任务'),('start_local','开始'),('end_local','结束/截断'),('duration_s','秒'),('status','状态'),('end_inferred','推断结束')])}<p><a href="tasks.csv">任务 CSV</a> · <a href="task_events.csv">全部阶段日志时间戳</a> · <a href="tasks.json">任务及状态事件 JSON</a></p></section>
<section id="tags"><h2>AprilTag 识别结果</h2><p>已记录 ID：{decoded_ids}。在线诊断、带深度观测、融合地图与录像重识别分别标明来源。</p>{tags_svg}{positions_svg}<div class="gallery">{tag_images}</div>{table([dict(row,time_local=clock_text(row['timestamp_ns'],offset)) for row in tags[-300:]], [('time_local','时间'),('source','来源'),('decoded_ids','ID'),('status','状态'),('position','世界坐标'),('excluded_ids','ID过滤'),('depth_rejected','深度拒绝')])}<p>表格展示最近 300 条；<a href="apriltag.csv">完整 AprilTag CSV</a> · <a href="apriltag.json">角点及识别 JSON</a></p></section>
<script type="application/json" id="report-data">{safe_json}</script>
<script>const d=JSON.parse(document.getElementById('report-data').textContent);function update(){{const p=d.poses[Number(document.getElementById('scrub').value)];if(!p)return;const c=document.querySelector('#path-plot #cursor');c.setAttribute('cx',70+(p[1]-d.transform.lower[0])*d.transform.scale);c.setAttribute('cy',520-(p[2]-d.transform.lower[1])*d.transform.scale);c.setAttribute('visibility','visible');const local=new Date(p[0]/1e6+d.offset*3600000).toISOString().replace('Z','');document.getElementById('pose-label').textContent=local+' UTC'+(d.offset>=0?'+':'')+d.offset+' · x='+p[1].toFixed(3)+' y='+p[2].toFixed(3)+' z='+p[3].toFixed(3)+' m';}}document.getElementById('scrub').addEventListener('input',update);update();</script></html>'''
    (output/'report.html').write_text(content, encoding='utf-8')
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--output', type=Path, help='New output directory outside session')
    parser.add_argument('--ffmpeg', default='ffmpeg')
    parser.add_argument('--timezone-offset', type=int, default=8)
    parser.add_argument('--no-bag', action='store_true', help='Only use readable indexes; no ROS imports')
    parser.add_argument('--no-detect-tags', action='store_true')
    parser.add_argument('--tag-dictionary', default='DICT_APRILTAG_16h5')
    parser.add_argument('--tag-sample-fps', type=float, default=1.)
    parser.add_argument('--max-tag-images', type=int, default=200)
    args = parser.parse_args(argv)
    if not math.isfinite(args.tag_sample_fps) or args.tag_sample_fps <= 0 or args.max_tag_images < 0:
        parser.error('AprilTag 采样帧率必须为正数，图片数量不得为负数')
    if not -23 <= args.timezone_offset <= 23:
        parser.error('timezone offset 必须在 -23 到 23 之间')
    output = args.output or args.session.resolve().parent/(args.session.name+'_report')
    try:
        summary = analyze(args.session, output, ffmpeg=shutil.which(args.ffmpeg),
                          offset=args.timezone_offset, detect_tags=not args.no_detect_tags,
                          dictionary=args.tag_dictionary, tag_fps=args.tag_sample_fps,
                          max_tag_images=args.max_tag_images, use_bag=not args.no_bag)
    except (ValueError, OSError) as error:
        parser.exit(1, str(error)+'\n')
    print(f'报告: {output.resolve()/"report.html"}')
    print(f'轨迹 {summary["pose_samples"]} 条，地图 {summary["map_runs"]} 次，任务 {summary["tasks"]} 个，AprilTag {summary["apriltag_records"]} 条')
    for warning in summary['warnings']:
        print('注意: '+warning)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
