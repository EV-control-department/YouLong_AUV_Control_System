"""Complete, source-labelled evidence exports for offline review."""

import html
import json


def export_evidence(root, output, data, maps, warnings):
    from .analyze import (cells_of, fused_grid_svg, read_jsonl,
                          table, timestamp, write_csv, write_json)

    evidence = output / 'evidence'
    evidence.mkdir()
    inventory = []
    # Retain every readable index, including unknown/new recorder indexes.
    for path in sorted(root.rglob('*.jsonl')):
        relative = path.relative_to(root)
        rows = read_jsonl(path, warnings)
        target = evidence / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(path.read_bytes())
        inventory.append({'source': str(relative), 'records': len(rows),
                          'file': target.relative_to(output).as_posix()})
    write_json(evidence / 'manifest.json', json.loads((root / 'manifest.json').read_text()))
    write_json(output / 'mapping_snapshots.json', data['maps'])
    write_json(output / 'mapping_observations.json', data['observations'])
    write_json(output / 'rosout.json', data['logs'])
    write_csv(output / 'rosout.csv', data['logs'],
              ['receive_time_unix_ns', 'stamp_ns', 'name', 'level', 'message'])
    observations = [dict(frame_time_ns=timestamp(frame), **observation)
                    for frame in data['observations']
                    for observation in frame.get('observations', [])]
    fields = sorted({key for row in observations for key in row})
    write_csv(output / 'mapping_observations.csv', observations, fields or ['frame_time_ns'])
    comparisons, views = [], []
    for index, mapping in enumerate(maps, 1):
        conclusion = mapping.get('conclusion')
        config = mapping.get('submitted_config') or {}
        fallback = {**dict.fromkeys(map(str, config.get('fallback_square_cells', [])), 0),
                    **dict.fromkeys(map(str, config.get('fallback_round_cells', [])), 1)}
        submission = None
        if isinstance(conclusion, dict):
            submission = conclusion.get('assignment')
            if submission is None and ('square_cells' in conclusion or 'round_cells' in conclusion):
                submission = {**dict.fromkeys(map(str, conclusion.get('square_cells', [])), 0),
                              **dict.fromkeys(map(str, conclusion.get('round_cells', [])), 1)}
        visual = mapping.get('final_assignment')
        row = {'run': index, 'timestamp_ns': timestamp(mapping),
               'visual_assignment': visual, 'configured_fallback': fallback or None,
               'submitted_assignment': submission, 'conclusion': conclusion,
               'submitted_config': config, 'verified_complete': mapping.get('verified_complete'),
               'fallback_used': mapping.get('fallback_used'),
               'config_fallback_mode': mapping.get('config_fallback_mode'),
               'fallback_reason': mapping.get('fallback_reason'),
               'travel_failures': mapping.get('travel_failures', []),
               'observation_failures': mapping.get('observation_failures', [])}
        comparisons.append(row)
        if mapping.get('state') == 'initializing' and not visual and not conclusion and not config:
            continue
        panels = []
        for name, assignment, prefix in (
                ('视觉融合结果', visual, 'visual'),
                ('实际提交接口结果', submission, 'submission')):
            if assignment is None:
                panels.append(f'<div><h4>{name}</h4><p>未记录该结果，不能推断。</p></div>')
                continue
            svg = fused_grid_svg({'final_assignment': assignment, 'cells': {}})
            svg = svg.replace('最终融合九宫格', name).replace('融合结果 ·', name+' ·')
            svg = svg.replace('仅展示融合分类，不替换提交结论或原始坐标', '来源见下方证据；格点编号不变')
            filename = f'map_{index:02d}_{prefix}.svg'
            (output / filename).write_text(svg, encoding='utf-8')
            panels.append(f'<div><h4>{name}</h4>{svg}<a href="{filename}">导出 SVG</a></div>')
        raw = [dict(cell_id=key, **cell) for key, cell in cells_of(mapping)]
        details = html.escape(json.dumps(row, ensure_ascii=False, indent=2))
        views.insert(0, f'<h3>第 {index} 轮结果对照</h3>'
                     '<p>视觉融合表示算法记录的结果；是否正确需结合录像与实物核对。'
                     '提交接口的 ready/valid/verified 及来源见证据。</p>'
                     '<div class="result-comparison">'+''.join(panels)+'</div>'
                     +table(raw, [('cell_id', '格'), ('label', '快照类别'), ('source', '来源'),
                                  ('position', '坐标'), ('accepted_observations', '观测数')])
                     +f'<details><summary>提交、参数、fallback 原因及失败证据</summary><pre>{details}</pre></details>')
    write_json(output / 'result_comparison.json', comparisons)
    write_csv(output / 'result_comparison.csv', comparisons,
              list(comparisons[0]) if comparisons else ['run'])
    write_json(output / 'evidence_inventory.json', inventory)
    links = ''.join(f'<li><a href="{html.escape(item["file"])}">'
                    f'{html.escape(item["source"])}</a> · {item["records"]} 条</li>' for item in inventory)
    logs = table(data['logs'][-500:], [('name', '节点'), ('level', '级别'), ('message', '日志')])
    observation_table = table(observations[-300:], [(key, key) for key in fields])
    return ('<section id="comparison"><h2>视觉 / 实际提交对照</h2>'
            + ''.join(views) + '<p><a href="result_comparison.csv">对照 CSV</a> · '
            '<a href="result_comparison.json">完整对照 JSON</a></p></section>'
            '<section id="evidence"><h2>完整文档与原始证据</h2><p>'
            '<a href="mapping_snapshots.json">全部建图快照</a> · '
            '<a href="mapping_observations.csv">全部视觉观测 CSV</a> · '
            '<a href="mapping_observations.json">视觉观测 JSON（含拒绝信息）</a> · '
            '<a href="rosout.csv">完整 ROS 日志 CSV</a> · '
            '<a href="evidence/manifest.json">录制 manifest</a> · '
            '<a href="report.md">文字报告 Markdown</a> · '
            '<a href="exports.html">全部导出文件清单</a></p><ul>'+links+'</ul>'
            '<h3>近期视觉观测（最近 300 条，包括接受和拒绝）</h3>'+observation_table+
            '<h3>近期 ROS 日志（最近 500 条，完整记录见导出）</h3>'+logs+'</section>')


def write_documents(output, summary):
    from .analyze import write_json
    comparisons = json.loads((output / 'result_comparison.json').read_text())
    lines = ['# AUV 录制复盘报告', '', f"会话：{summary['session']}", '',
             f"时间：{summary['start_local']} → {summary['end_local']}", '',
             f"时长：{summary['duration_s']:.1f}s；位姿：{summary['pose_samples']} 条；"
             f"建图：{summary['map_runs']} 轮；任务：{summary['tasks']} 个；"
             f"AprilTag：{summary['apriltag_records']} 条。", '',
             '## 每轮视觉与实际提交', '']
    for row in reversed(comparisons):
        lines.extend([f"### 第 {row['run']} 轮", '',
                      '```json', json.dumps(row, ensure_ascii=False, indent=2), '```', ''])
    lines.extend(['## 数据完整性与解释', ''])
    lines.extend('- '+note for note in summary['warnings'] + summary['notes'])
    lines.extend(['', '## 附件', '', '[交互报告](report.html) · [导出清单](exports.html) · '
                  '[三维建图场景](map3d.html) · '
                  '[全部快照](mapping_snapshots.json) · [完整观测](mapping_observations.csv) · '
                  '[完整日志](rosout.csv)', ''])
    (output / 'report.md').write_text('\n'.join(lines), encoding='utf-8')
    files = [{'file': path.relative_to(output).as_posix(), 'bytes': path.stat().st_size}
             for path in sorted(output.rglob('*')) if path.is_file()]
    write_json(output / 'exports.json', files)
    links = [dict(row, link=f'<a href="{html.escape(row["file"], quote=True)}">'
                   f'{html.escape(row["file"])}</a>') for row in files]
    # All paths are generated locally and escaped; the reusable table keeps text safe.
    content = '<!doctype html><meta charset="utf-8"><title>导出文件清单</title>'
    content += '<style>body{font:16px sans-serif;margin:30px}td,th{padding:8px;text-align:left}</style>'
    content += '<h1>全部导出文件</h1><p><a href="report.html">返回报告</a> · '
    content += '<a href="exports.json">机器可读清单</a></p><table><tr><th>文件</th><th>字节</th></tr>'
    content += ''.join(f'<tr><td>{row["link"]}</td><td>{row["bytes"]}</td></tr>' for row in links)
    (output / 'exports.html').write_text(content+'</table>', encoding='utf-8')
