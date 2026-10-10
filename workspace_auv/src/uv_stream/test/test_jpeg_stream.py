"""JPEG consumers preserve H.264 geometry and source-frame metadata."""

import io
from collections import OrderedDict
from sensor_msgs.msg import CameraInfo
from uv_camera.image_geometry import EyeUndistorter, CalibrationCache
import itertools
import json
import shutil
import subprocess
from types import SimpleNamespace as NS

import cv2
import numpy as np
import pytest

from uv_image_transport import ENCODING_JPEG, FrameHeader, FramePacket, InvalidFrameError
from uv_stream import camera_streamer as stream


@pytest.mark.parametrize('mode', ('raw', 'annotated'))
def test_jpeg_to_h264_stream_and_mapping(tmp_path, monkeypatch, mode):
    if not shutil.which('ffmpeg') or not shutil.which('ffprobe'):
        pytest.skip('ffmpeg and ffprobe are required for the stream integration test')
    image = np.zeros((960, 2560, 3), np.uint8)
    image[:, :1280] = (200, 20, 10)
    image[:, 1280:] = (10, 20, 200)
    payload = cv2.imencode('.jpg', image)[1].tobytes()
    packets = [FramePacket(FrameHeader(index + 1, 1_000_000_000 + index * 100_000_000,
                                       index + 1, 1, 2560, 960, 0,
                                       encoding=ENCODING_JPEG, camera_info_version=1), payload)
               for index in range(24)]
    values = iter([InvalidFrameError('bad JPEG'), *packets, None])

    def read():
        value = next(values)
        if isinstance(value, Exception):
            raise value
        return value

    monkeypatch.setattr(stream, 'Iceoryx2Reader', lambda _: NS(
        read=read, close=lambda: None, service='jpeg_test_camera'))
    ticks = itertools.count(0, 100_000_000)
    monkeypatch.setattr(stream.time, 'monotonic_ns', lambda: next(ticks))
    mappings = []
    cache = stream.DetectionCache('front')
    bridge = stream.StreamMetadataBridge.__new__(stream.StreamMetadataBridge)
    bridge.camera='front';bridge.mode=mode;bridge.calibrations=CalibrationCache()
    bridge._corrections=OrderedDict();bridge._last_calibration_warning=float('-inf')
    calibration_ids={}
    if mode=='annotated':
        for side in ('left','right'):
            correction=EyeUndistorter(CameraInfo(width=1280,height=960,
                k=[1100.,.6,640.,0.,1100.,480.,0.,0.,1.],d=[-.3,.1,.001,0.,0.]))
            metadata=correction.message('front_'+side)
            bridge.calibrations.add(metadata);calibration_ids[side]=metadata.calibration_id
    for packet in packets:
        cache.add(NS(header=NS(stamp=NS(sec=packet.header.timestamp_ns // 1_000_000_000,
                                        nanosec=packet.header.timestamp_ns % 1_000_000_000)),
                     camera_name='front_left', capture_id=packet.header.capture_id,
                     image_space=1 if mode=='annotated' else 0,
                     calibration_id=calibration_ids.get('left',0), camera_info_version=1,
                     stereo_pair_id=packet.header.stereo_pair_id,
                     detections=(NS(class_id=1, confidence=0.9, bbox_x1=100,
                                    bbox_y1=100, bbox_x2=300, bbox_y2=300),)))
    bridge.detection_cache=cache;bridge.close=lambda:None
    bridge.publish=lambda frame, sequence, fps: mappings.append((frame,sequence,fps))
    monkeypatch.setattr(stream, 'StreamMetadataBridge', lambda *_: bridge)
    original_decode = cv2.imdecode
    decoded = []

    def decode(*args):
        decoded.append(True)
        return original_decode(*args)

    monkeypatch.setattr(cv2, 'imdecode', decode)
    output = io.BytesIO()
    assert stream.run('front', mode, 10, video_output=output) == 0
    video = tmp_path / 'output.ts'
    video.write_bytes(output.getvalue())
    report = json.loads(subprocess.check_output([
        'ffprobe', '-v', 'error', '-count_frames', '-select_streams', 'v:0',
        '-show_entries', 'stream=codec_name,width,height,nb_read_frames',
        '-of', 'json', str(video)]))['streams'][0]
    assert (report['codec_name'], report['width'], report['height']) == ('h264', 1280, 480)
    assert len(mappings) == int(report['nb_read_frames']) >= 10
    assert len(decoded) == 18  # seven sample reads, then only the selected frames
    assert [sequence for _, sequence, _ in mappings] == list(range(len(mappings)))
    assert all(fps == 10 for _, _, fps in mappings)
    assert mappings[0][0].image_space == (1 if mode=='annotated' else 0)
    assert mappings[0][0].capture_id == 7
    assert mappings[0][0].timestamp_ns == 1_600_000_000
    cap = cv2.VideoCapture(str(video))
    ok, frame = cap.read()
    cap.release()
    assert ok
    assert frame[240, 320, 0] > frame[240, 960, 0] + 150
    if mode == 'annotated':
        assert frame[50, 100, 1] > 100
