"""Reports preserve evidence and distinguish absence, fallback and completion."""

import json
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from uv_record.analyze import analyze, map_runs, task_timeline
from uv_record.recorder import ChildSupervisor, Recorder, _parse_args
from uv_record.telemetry import TelemetryRecorder


def jsonl(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(''.join(json.dumps(row, ensure_ascii=False)+'\n' for row in rows))


def test_report_exports_maps_task_bounds_and_missing_video(tmp_path):
    session = tmp_path/'session'
    session.mkdir()
    (session/'manifest.json').write_text('{"status":"RUNNING"}')
    ns = 1_790_000_000_000_000_000
    jsonl(session/'metadata/trajectory.jsonl', [
        {'receive_time_unix_ns': ns+i*10**9, 'stamp_ns': ns,
         'pose': {'x': i, 'y': 0, 'z': .3, 'yaw': 0, 'roll': 0, 'pitch': 0}}
        for i in range(3)])
    jsonl(session/'metadata/mapping.jsonl', [{
        'receive_time_unix_ns': ns, 'state': 'complete', 'verified_complete': False,
        'fallback_used': True, 'tag': {'id': 18, 'source': 'vision', 'position': [1, 2, 3]},
        'cells': {'0': {'label': 'round_cone', 'source': 'fallback', 'position': [1, 2, 3]}}}])
    jsonl(session/'logs/rosout.jsonl', [
        {'receive_time_unix_ns': ns, 'message': '=== 任务列表开始执行（共 2 个任务）==='},
        {'receive_time_unix_ns': ns, 'message': '[1/2] start 参数={}'},
        {'receive_time_unix_ns': ns+10**9, 'message': '[2/2] mapping 参数={}'},
        {'receive_time_unix_ns': ns+10**9, 'message': 'AprilTag诊断：解码ID=[18]，字典=DICT_APRILTAG_16h5，被ID条件排除=[9]，本帧多预处理未解码候选数=2，深度失败=1'}])
    with (session/'metadata/trajectory.jsonl').open('a') as stream:
        stream.write('{"torn":')
    jsonl(session/'video/down/frame_alignment.jsonl', [{'decoder_frame_index': 0}])
    output = tmp_path/'report'
    result = analyze(session, output, detect_tags=False, use_bag=False)
    assert result['pose_samples'] == 3 and result['sampled_xy_distance_m'] == 2
    tasks = json.loads((output/'tasks.json').read_text())['tasks']
    assert tasks[0]['status'] == 'completed_inferred' and tasks[0]['end_inferred']
    assert tasks[1]['status'] == 'incomplete_at_recording_end'
    tags = json.loads((output/'apriltag.json').read_text())
    diagnostic = next(row for row in tags if row['source'] == 'online_diagnostic')
    assert diagnostic['decoded_ids'] == [18] and diagnostic['excluded_ids'] == [9]
    assert diagnostic['undecoded_candidates'] == 2 and diagnostic['depth_rejected'] == 1
    assert any('没有实际视频' in w for w in result['warnings'])
    assert any('截断' in w for w in result['warnings'])
    report = (output/'report.html').read_text()
    assert '无法播放或恢复' in report and 'fallback' in report and 'scrub' in report
    for name in ('map_01.svg', 'map_01.csv', 'trajectory.csv', 'tasks.svg',
                 'apriltag_timeline.svg', 'apriltag_positions.svg', 'videos.json'):
        assert (output/name).is_file()
    with pytest.raises(ValueError, match='拒绝覆盖'):
        analyze(session, output, use_bag=False)
    with pytest.raises(ValueError, match='session 外'):
        analyze(session, session/'report', use_bag=False)


def test_task_restart_does_not_claim_success():
    logs = [dict(receive_time_unix_ns=i*10**9, message=msg) for i, msg in enumerate([
        '=== 任务列表开始执行 ===', '[1/2] grab 参数={}',
        '=== 任务列表开始执行 ===', '[1/2] grab 参数={}', '[1/2] grab 执行失败'])]
    tasks, _, chains = task_timeline(logs, 8*10**9)
    assert [task['status'] for task in tasks] == ['interrupted_by_restart', 'failed']
    assert len(chains) == 2


def test_final_map_categories_recovered_without_inventing_final_pose():
    snapshot = {'receive_time_unix_ns': 100, 'state': 'observe_cell',
                'final_assignment': None, 'verified_complete': False, 'cells': {
                    '1': {'label': 'round_cone', 'position': [1, 2, 3], 'source': 'vision'},
                    '2': {'label': 'square_cone', 'position': [3, 2, 1], 'source': 'vision'}}}
    logs = [{'receive_time_unix_ns': 110,
             'message': '建图最终结果（两方两圆约束）：{1: 0}；类别0=方形，类别1=圆形'}]
    result = map_runs([snapshot], logs)[0]
    assert result['final_assignment_source'] == 'task_result_log'
    assert result['final_assignment'] == {'1': 0}
    assert result['cells']['1']['label'] == 'square_cone'
    assert result['cells']['1']['snapshot_label'] == 'round_cone'
    assert result['cells']['1']['position'] == [1, 2, 3]
    assert result['cells']['2']['final_selected'] is False
    assert not result['verified_complete'] and result['state'] == 'observe_cell'
    assert snapshot['final_assignment'] is None  # Original evidence remains intact.


def test_competition_includes_analysis_evidence(monkeypatch):
    import re
    monkeypatch.setattr(sys, 'argv', ['record', '--profile', 'competion'])
    args = _parse_args()
    assert args.profile == 'competition' and args.segment_duration == 5
    matcher = re.compile(args.topic_regex)
    for name in ('/perception/mapping/observations',
                 '/perception/aruco/ids', '/perception/target_observations', '/task/status',
                 '/zit6/state/odom', '/zit6/cmd/servo', '/zit6/cmd/setpoint'):
        assert matcher.match(name), name
    assert not matcher.match('/camera/down/image_raw')
    assert not matcher.match('/perception/detection/down_left')  # large segmentation masks


def test_final_map_transition_bypasses_snapshot_rate_limit(tmp_path):
    recorder = TelemetryRecorder(tmp_path)
    calls = []
    recorder._write = lambda kind, value, rate: calls.append((value['state'], rate))
    for state in ('observe_cell', 'observe_cell', 'complete'):
        recorder._map(SimpleNamespace(data=json.dumps({'state': state, 'cells': {}})))
    assert calls == [('observe_cell', 0), ('observe_cell', 1), ('complete', 0)]


def test_readable_indexes_accept_actual_ros_observation_and_task_schema(tmp_path):
    messages = pytest.importorskip('uv_msgs.msg')
    pytest.importorskip('rosidl_runtime_py.convert')
    recorder = TelemetryRecorder(tmp_path)
    written = []
    recorder._write = lambda kind, value, rate=0: written.append((kind, value))
    frame = messages.MappingObservationArray()
    frame.processed = True
    frame.tag_candidates = 1
    tag = messages.MappingObservation()
    tag.kind = tag.TAG
    tag.tag_id = 18
    tag.world_x, tag.world_y, tag.world_z = 1., 2., .8
    frame.observations = [tag]
    recorder._observations(frame)
    assert written[0][0] == 'observations'
    assert written[0][1]['observations'][0]['tag_id'] == 18
    assert written[0][1]['observations'][0]['world_z'] == pytest.approx(.8)
    status = messages.TaskStatus()
    status.current_task_name = 'mapping_grid'
    status.status = status.STATUS_RUNNING
    recorder._task(status)
    recorder._task(status)
    status.status = status.STATUS_DONE
    recorder._task(status)
    assert [value['status'] for kind, value in written if kind == 'task'] == [1, 3]


def test_decoded_frame_is_not_saved_video(tmp_path, monkeypatch):
    directory = tmp_path/'down'
    directory.mkdir()
    (directory/'status.json').write_text(json.dumps({
        'frames': 1, 'status': 'recording', 'last_frame_received_unix_ns': time.time_ns()}))
    recorder = Recorder.__new__(Recorder)
    recorder.args = SimpleNamespace(video_format='ts', segment_duration=0)
    recorder.video_directories = [directory]
    times = iter([0, 0, 100])
    monkeypatch.setattr('uv_record.recorder.time.monotonic', lambda: next(times))
    monkeypatch.setattr('uv_record.recorder.time.sleep', lambda _: None)
    with pytest.raises(RuntimeError, match='complete video segment'):
        recorder._wait_for_video_frames(timeout=1)


def test_failed_child_keeps_retrying_with_working_supervisor(tmp_path):
    child = ChildSupervisor('failed_encoder', lambda: [sys.executable, '-c', 'raise SystemExit(1)'],
                            tmp_path/'encoder.log')
    child.start()
    try:
        deadline = time.monotonic()+5
        while child.restarts < 2 and child.is_alive() and time.monotonic() < deadline:
            time.sleep(.05)
        assert child.restarts >= 2 and child.is_alive()
        assert child.last_exit == 1
    finally:
        child.stop()
        child.join(timeout=1)
    assert not child.is_alive()


def test_restart_preserves_interrupted_ts_tail(tmp_path):
    from uv_record.recorder import _next_segment_number
    (tmp_path/'000006.ts').write_bytes(b'segment')
    (tmp_path/'000007.ts.tmp').write_bytes(b'interrupted tail')
    assert _next_segment_number(tmp_path) == 8


def test_encoder_error_written_to_status_and_log(tmp_path):
    fake_ffmpeg = tmp_path/'ffmpeg'
    fake_ffmpeg.write_text('#!/bin/sh\necho "Unknown encoder: broken_codec" >&2\nexit 1\n')
    fake_ffmpeg.chmod(0o755)
    process = launch_proxy(tmp_path/'video', str(fake_ffmpeg), 'unused', 'libx264', False)
    _, stderr = process.communicate(timeout=5)
    assert process.returncode == 1 and b'Unknown encoder' in stderr
    status = json.loads((tmp_path/'video/status.json').read_text())
    assert status['status'] == 'failed' and status['bytes'] == 0
    assert 'Unknown encoder' in status['error']


def launch_proxy(output, ffmpeg, url, codec, fallback):
    # The real recording/FFmpeg paths run, while source timestamp lookup is
    # disabled so these offline tests never initialize DDS or contact an AUV.
    code = '''
from types import SimpleNamespace
from uv_record import mjpeg_proxy as proxy
proxy.FrameMappingSubscriber = lambda camera, mode: SimpleNamespace(camera=camera, stream_mode=mode, match=lambda pts: None, close=lambda: None)
raise SystemExit(proxy.main())
'''
    command = [sys.executable, '-c', code, '--output-dir', str(output),
               '--playlist', str(output/'index_000000.m3u8'), '--url', url,
               '--camera', output.name if output.name in ('front', 'down') else 'front',
               '--output-format', 'ts', '--video-codec', codec, '--ffmpeg', ffmpeg,
               '--segment-duration', '1', '--fps', '8', '--max-width', '640', '--wallclock-input']
    if fallback:
        command.append('--fallback-jpeg')
    return subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)


@pytest.mark.parametrize('camera,codec,fallback', [
    ('front', 'libx264', False), ('down', 'libx264', False), ('front', 'missing_encoder', True)])
def test_live_mjpeg_to_report_and_apriltag(tmp_path, camera, codec, fallback):
    """Actual encoder, HLS/JPEG persistence, MP4 and visible decoded corners."""
    import cv2
    import numpy as np
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
    ffmpeg = os.environ.get('UV_RECORD_TEST_FFMPEG') or shutil.which('ffmpeg')
    if not ffmpeg:
        pytest.skip('set UV_RECORD_TEST_FFMPEG to run real video integration')
    if not hasattr(cv2, 'aruco'):
        pytest.skip('OpenCV aruco required for the pixel-corner integration test')
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5)
    marker = (cv2.aruco.generateImageMarker(dictionary, 18, 128)
              if hasattr(cv2.aruco, 'generateImageMarker')
              else cv2.aruco.drawMarker(dictionary, 18, 128))
    image = np.full((240, 640, 3), 255, np.uint8)
    image[56:184, 96:224] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    jpeg = cv2.imencode('.jpg', image)[1].tobytes()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while True:
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                     +str(len(jpeg)).encode()+b'\r\n\r\n'+jpeg+b'\r\n')
                    self.wfile.flush()
                    time.sleep(.05)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *args):
            pass

    try:
        server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    except PermissionError:
        pytest.skip('loopback socket permission required for live MJPEG integration')
    threading.Thread(target=server.serve_forever, daemon=True).start()
    session = tmp_path/'session'
    session.mkdir()
    (session/'manifest.json').write_text('{"status":"STOPPED"}')
    directory = session/'video'/camera
    process = launch_proxy(directory, ffmpeg, f'http://127.0.0.1:{server.server_port}/{camera}', codec, fallback)
    try:
        deadline = time.monotonic()+18
        ready = False
        while time.monotonic() < deadline:
            if process.poll() is not None:
                break
            try:
                status = json.loads((directory/'status.json').read_text())
                ready = status.get('status') == 'recording' and status.get('bytes', 0) > 0
                if ready:
                    break
            except (OSError, ValueError):
                pass
            time.sleep(.1)
        process.send_signal(signal.SIGINT) if process.poll() is None else None
        _, stderr = process.communicate(timeout=15)
        assert ready, stderr.decode(errors='replace')[-2000:]
        assert process.returncode == 0, stderr.decode(errors='replace')[-2000:]
        status = json.loads((directory/'status.json').read_text())
        assert status['status'] == 'stopped' and status['bytes'] > 0
        if fallback:
            assert status['output_format'] == 'jpeg' and 'missing_encoder' in status['fallback_reason']
            assert list(directory.glob('chunk_*.mjpg'))
        else:
            assert status['complete_segments'] >= 1 and list(directory.glob('*.ts'))
        result = analyze(session, tmp_path/'report', ffmpeg=ffmpeg, use_bag=False)
        assert result['apriltag_records'] > 0
        tags = json.loads((tmp_path/'report/apriltag.json').read_text())
        assert any(18 in row['decoded_ids'] and row['corners_px'] for row in tags)
        assert list((tmp_path/'report/videos').glob('*.mp4'))
        assert list((tmp_path/'report/apriltag').glob('*.jpg'))
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        server.shutdown()
        server.server_close()
