"""Optional real DDS/Iceoryx2/bag integration using synthetic capture devices."""

from dataclasses import replace
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from types import SimpleNamespace
import uuid

import cv2
import numpy as np
import pytest


def test_native_discovery_late_start_restart_and_mcap(tmp_path, monkeypatch):
    pytest.importorskip('iceoryx2')
    rosbag2_py = pytest.importorskip('rosbag2_py')
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage, Image
    from std_msgs.msg import String
    from sl_camera.catalog import catalog_qos
    from sl_camera.capture import MjpegCapture
    from sl_camera.driver import SlCamera
    from sl_camera.profile import load_profile
    from sl_msgs.msg import CameraCatalog
    from uv_record.recorder import _mcap_storage_available

    if not _mcap_storage_available():
        pytest.skip('MCAP storage is optional')
    token = uuid.uuid4().hex
    monkeypatch.setenv('ROS_DOMAIN_ID', str(180 + os.getpid() % 40))
    base = load_profile()
    topic = '/auv/sl_test/catalog_' + token
    specs, payloads, available = [], {}, {}
    for index, width in enumerate((64, 128, 96)):
        spec = replace(
            base.cameras[0], id=10 + index, name=f'generic_{index}',
            device=f'/dev/synthetic{index}', width=width, height=32,
            ice_service=f'youlong/test/{token}/{index}', frame_id=f'camera_{index}',
            startup_timeout_sec=0.2, reconnect_interval_sec=0.02)
        image = np.zeros((32, width, 3), np.uint8)
        image[:, :width // 2] = (20, 60, 190 + index)
        payloads[spec.device] = cv2.imencode('.jpg', image)[1].tobytes()
        available[spec.device] = index != 1
        specs.append(spec)

    def fake_open(capture):
        def read():
            time.sleep(1 / 30)
            if not available[capture.config.device]:
                return False, None
            return True, np.frombuffer(payloads[capture.config.device], np.uint8).reshape(1, -1)
        capture.on_fps(30.0)
        return SimpleNamespace(read=read, release=lambda: None)

    monkeypatch.setattr(MjpegCapture, '_open_capture', fake_open)
    # Recording must work even when the settings file has no camera definitions.
    settings = tmp_path / 'record-only.yaml'
    settings.write_text(
        'schema_version: 2\ncatalog_topic: ' + topic
        + '\ncameras: ignored-by-recorder\nrecording: {}\n')
    session = tmp_path / 'session'
    log = (tmp_path / 'recorder.log').open('w')
    process = subprocess.Popen([
        sys.executable, '-c', 'from sl_record.recorder import main; raise SystemExit(main())',
        '--profile', str(settings), '--session-dir', str(session)],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    context = Context()
    rclpy.init(args=[], context=context)
    executor = SingleThreadedExecutor(context=context)
    observer = Node('sl_test_observer', context=context)
    executor.add_node(observer)
    telemetry = observer.create_publisher(String, '/auv/sl_test/telemetry_' + token, 10)
    images = observer.create_publisher(Image, '/auv/sl_test/image_' + token, 10)
    camera = None
    catalogs = []

    def rows(spec):
        path = session / 'camera/raw' / f'camera_{spec.id}_{spec.name}' / 'frames.jsonl'
        return [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []

    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            assert process.poll() is None, (tmp_path / 'recorder.log').read_text()
            message = String()
            message.data = 'telemetry'
            telemetry.publish(message)
            image = Image()
            image.width, image.height, image.step, image.encoding = 1, 1, 3, 'bgr8'
            image.data = b'\x00\x00\x00'
            images.publish(image)
            executor.spin_once(timeout_sec=0.02)

    try:
        pump(0.6)  # Recorder precedes the camera publisher.
        camera = SlCamera(
            profile=replace(base, cameras=tuple(specs[:2]), catalog_topic=topic), context=context)
        first_instance = camera.instance_id
        executor.add_node(camera)
        pump(0.5)
        camera._timer.cancel()
        observer.create_subscription(CameraCatalog, topic, catalogs.append, catalog_qos())
        pump(0.7)  # A late reader receives the durable catalog with periodic publication paused.
        assert catalogs and catalogs[-1].configured_count == 2
        assert all(not item.calibrated and not item.calibrations for item in catalogs[-1].cameras)
        camera._timer.reset()
        pump(1.0)
        assert len(rows(specs[0])) > 5
        assert rows(specs[1]) == []
        available[specs[1].device] = True
        pump(1.2)
        assert len(rows(specs[1])) > 5
        executor.remove_node(camera)
        camera.destroy_node()
        camera = SlCamera(
            profile=replace(base, cameras=tuple(specs[1:]), catalog_topic=topic), context=context)
        second_instance = camera.instance_id
        executor.add_node(camera)
        pump(1.8)
        assert len(rows(specs[2])) > 5
        assert {row['source_instance_id'] for row in rows(specs[1])} == {
            first_instance, second_instance}
        os.killpg(process.pid, signal.SIGINT)
        assert process.wait(timeout=30) == 0, (tmp_path / 'recorder.log').read_text()
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        if camera is not None:
            executor.remove_node(camera)
            camera.destroy_node()
        executor.remove_node(observer)
        observer.destroy_node()
        executor.shutdown()
        if rclpy.ok(context=context):
            rclpy.shutdown(context=context)
        log.close()

    manifest = json.loads((session / 'manifest.json').read_text())
    assert manifest['status'] == 'STOPPED'
    assert not (session / 'session.lock').exists()
    histories = [json.loads(line) for line in
                 (session / 'metadata/camera_catalogs.jsonl').read_text().splitlines()]
    history_by_sequence = {item['catalog_sequence']: item for item in histories}
    for spec in specs:
        entries = rows(spec)
        assert [entry['frame_sequence'] for entry in entries] == list(range(len(entries)))
        for row in entries:
            catalog = history_by_sequence[row['catalog_sequence']]['catalog']
            assert row['source_instance_id'] == catalog['source_instance_id']
            assert row['config_version'] == catalog['config_version']
            assert any(item['id'] == spec.id for item in catalog['cameras'])
            path = session / 'camera/raw' / f'camera_{spec.id}_{spec.name}' / row['path']
            assert path.read_bytes() == payloads[spec.device]
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(session / 'session.mcap'), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    topics = {item.name: item.type for item in reader.get_all_topics_and_types()}
    assert topics[topic] == 'sl_msgs/msg/CameraCatalog'
    assert 'sensor_msgs/msg/Image' not in topics.values()
    saved, instances = {}, set()
    try:
        while reader.has_next():
            name, data, timestamp = reader.read_next()
            if name.endswith('/compressed'):
                image = deserialize_message(data, CompressedImage)
                key = name.split('/')[-2]
                saved.setdefault(key, []).append((timestamp, bytes(image.data)))
                assert image.header.stamp.sec * 1_000_000_000 + image.header.stamp.nanosec == timestamp
            elif name == topic:
                instances.add(deserialize_message(data, CameraCatalog).source_instance_id)
    finally:
        reader.close()
    assert instances == {first_instance, second_instance}
    assert saved == {
        f'camera_{spec.id}_{spec.name}': [
            (row['timestamp_ns'], payloads[spec.device]) for row in rows(spec)] for spec in specs}
