import json

from uv_record.analyze import analyze
from uv_record.review import discover_sessions


def test_discovery_accepts_scp_root_and_single_session(tmp_path):
    session = tmp_path / 'record' / 'sessions' / 'trip'
    (session / 'metadata').mkdir(parents=True)
    (session / 'manifest.json').write_text('{}')
    report = tmp_path / 'record' / 'reports' / 'evidence'
    report.mkdir(parents=True)
    (report / 'manifest.json').write_text('{}')
    assert discover_sessions(tmp_path / 'record') == [session]
    assert discover_sessions(session) == [session]


def test_report_distinguishes_visual_fallback_submission_and_preserves_all_evidence(tmp_path):
    session = tmp_path / 'session'
    (session / 'metadata').mkdir(parents=True)
    (session / 'manifest.json').write_text('{"status":"STOPPED"}')
    snapshot = {
        'state': 'complete_with_failures', 'receive_time_unix_ns': 100,
        'final_assignment': {'0': 0, '1': 0, '2': 1, '3': 1},
        'cells': {}, 'verified_complete': False,
        'submitted_config': {'fallback_square_cells': [4, 5], 'fallback_round_cells': [6, 7]},
        'conclusion': {'source': 'fallback', 'ready': True, 'valid': True,
                       'verified': False, 'assignment': {'4': 0, '5': 0, '6': 1, '7': 1}},
        'travel_failures': [{'cell': 8, 'reason': '<timeout>'}]}
    source = session / 'metadata' / 'mapping.jsonl'
    source.write_text(json.dumps(snapshot)+'\n')
    original = source.read_bytes()
    observations = session / 'metadata' / 'mapping_observations.jsonl'
    observations.write_text(json.dumps({'receive_time_unix_ns': 100, 'observations': [
        {'kind': 0, 'accepted': False, 'reject_reason': 'depth', 'world_x': 1}]})+'\n')
    extra = session / 'metadata' / 'performance.jsonl'
    extra.write_text('{"free_bytes":123}\n{"torn":')
    output = tmp_path / 'report'
    summary = analyze(session, output, detect_tags=False, use_bag=False)
    comparison = json.loads((output / 'result_comparison.json').read_text())[0]
    assert comparison['visual_assignment'] == snapshot['final_assignment']
    assert comparison['submitted_assignment'] == snapshot['conclusion']['assignment']
    assert comparison['configured_fallback'] == snapshot['conclusion']['assignment']
    assert comparison['travel_failures'] == snapshot['travel_failures']
    assert source.read_bytes() == original == (output / 'evidence/metadata/mapping.jsonl').read_bytes()
    assert extra.read_bytes() == (output / 'evidence/metadata/performance.jsonl').read_bytes()
    assert any('截断' in warning for warning in summary['warnings'])
    report = (output / 'report.html').read_text()
    assert '实际提交接口结果' in report and '&lt;timeout&gt;' in report
    assert '配置的 fallback 结果' not in report
    assert 'data-mode="fallback"' not in (output / 'map3d.html').read_text()
    assert not (output / 'map_01_fallback.svg').exists()
    assert 'fallback' in (output / 'report.md').read_text()
    exports = json.loads((output / 'exports.json').read_text())
    assert any(row['file'] == 'mapping_observations.csv' for row in exports)
    assert 'reject_reason' in (output / 'mapping_observations.csv').read_text('utf-8-sig')
    assert 'data-cell="0" data-class="0"' in (output / 'map_01_visual.svg').read_text()
    assert 'data-cell="0" data-class="None"' in (output / 'map_01_submission.svg').read_text()


def test_missing_submission_is_not_inferred_from_fallback(tmp_path):
    session = tmp_path / 'session'
    (session / 'metadata').mkdir(parents=True)
    (session / 'manifest.json').write_text('{}')
    (session / 'metadata/mapping.jsonl').write_text(json.dumps({
        'state': 'complete', 'submitted_config': {'fallback_square_cells': [0, 1]},
        'final_assignment': {'2': 1}, 'cells': {}})+'\n')
    output = tmp_path / 'report'
    analyze(session, output, detect_tags=False, use_bag=False)
    result = json.loads((output / 'result_comparison.json').read_text())[0]
    assert result['submitted_assignment'] is None
    assert not (output / 'map_01_submission.svg').exists()
