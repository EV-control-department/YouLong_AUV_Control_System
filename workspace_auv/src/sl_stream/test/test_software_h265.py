import json
import shutil
import subprocess

import pytest


@pytest.mark.skipif(
    not shutil.which('gst-launch-1.0') or not shutil.which('ffprobe'),
    reason='GStreamer and ffprobe are required for the software encoder integration test',
)
@pytest.mark.parametrize(('width', 'height'), [(1280, 720), (2560, 960)])
def test_software_encoder_emits_full_resolution_hevc_mpegts(tmp_path, width, height):
    output = tmp_path / f'camera-{width}x{height}.ts'
    pipeline = [
        'gst-launch-1.0', '-e', '-q',
        'videotestsrc', 'num-buffers=90', 'pattern=ball',
        '!', f'video/x-raw,width={width},height={height},framerate=30/1',
        '!', 'videoconvert', '!', 'video/x-raw,format=I420',
        '!', 'x265enc', 'bitrate=2000', 'key-int-max=30',
        'speed-preset=ultrafast', 'tune=zerolatency',
        'option-string=vbv-maxrate=3000:vbv-bufsize=3000:repeat-headers=1:bframes=0:hrd=1',
        '!', 'h265parse', 'config-interval=-1', '!', 'mpegtsmux',
        '!', 'filesink', f'location={output}',
    ]
    subprocess.run(pipeline, check=True, capture_output=True, timeout=30)
    report = json.loads(subprocess.check_output([
        'ffprobe', '-v', 'error', '-select_streams', 'v:0',
        '-show_entries', 'stream=codec_name,width,height,avg_frame_rate',
        '-of', 'json', str(output),
    ]))['streams'][0]

    assert (report['codec_name'], report['width'], report['height']) == (
        'hevc', width, height)
    assert report['avg_frame_rate'] == '30/1'
