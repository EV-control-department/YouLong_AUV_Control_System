"""Build one session MCAP from ROS bag parts and recorded camera frames."""

from __future__ import annotations

import heapq
import json
import os
from pathlib import Path
import re
import shutil
import tempfile
import time


_IMAGE_TYPE = 'sensor_msgs/msg/CompressedImage'
_STRING_TYPE = 'std_msgs/msg/String'


def _read_json_lines(path: Path) -> list[dict]:
    records = []
    try:
        with path.open(encoding='utf-8') as stream:
            for line in stream:
                try:
                    value = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if isinstance(value, dict):
                    records.append(value)
    except OSError:
        pass
    return records


def _sort_ts(path: Path):
    match = re.search(r'(\d+)$', path.stem)
    return (int(match.group(1)) if match else -1, path.name)


def _camera_sources(session_root: Path, record_mode: str) -> list[dict]:
    sources = []
    if record_mode == 'raw':
        raw_root = session_root / 'camera' / 'raw'
        for directory in sorted(path for path in raw_root.glob('*') if path.is_dir()):
            records = _read_json_lines(directory / 'frames.jsonl')
            if records:
                sources.append({
                    'name': directory.name,
                    'kind': 'raw',
                    'directory': directory,
                    'records': records,
                })
        return sources

    from .jpeg_archive import archive_entries

    video_root = session_root / 'video'
    for directory in sorted(path for path in video_root.glob('*') if path.is_dir()):
        refs = archive_entries(directory)
        if refs:
            records = [{
                'timestamp_ns': ref.timestamp_ns,
                'source_timestamp_ns': ref.timestamp_ns,
                'sequence': ref.sequence,
                'timestamp_aligned': bool(ref.timestamp_ns > 0),
                **(ref.metadata or {}),
            } for ref in refs]
            sources.append({
                'name': directory.name,
                'kind': 'jpeg',
                'directory': directory,
                'records': records,
                'refs': refs,
            })
        segments = sorted(
            (path for path in directory.glob('*.ts')
             if path.is_file() and path.stat().st_size > 0),
            key=_sort_ts)
        alignment = _read_json_lines(directory / 'frame_alignment.jsonl')
        if segments:
            sources.append({
                'name': directory.name,
                'kind': 'ts',
                'directory': directory,
                'records': alignment,
                'segments': segments,
            })
    return sources


def _camera_topics(source: dict) -> tuple[str, str, str]:
    name = re.sub(r'[^A-Za-z0-9_]+', '_', source['name']).strip('_') or 'camera'
    base = '/auv/record/camera/{}'.format(name)
    if source['kind'] == 'ts':
        return ('', base + '/frame_metadata', base + '/video_segment')
    return base + '/compressed', base + '/frame_metadata', ''


def _open_reader(rosbag2_py, path: Path):
    reader = rosbag2_py.SequentialReader()
    storage_id = 'sqlite3' if path.suffix.lower() == '.db3' else 'mcap'
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id=storage_id),
        rosbag2_py.ConverterOptions('', ''),
    )
    return reader


def _bag_topics(rosbag2_py, paths: list[Path]) -> dict:
    topics = {}
    for path in paths:
        reader = _open_reader(rosbag2_py, path)
        try:
            for item in reader.get_all_topics_and_types():
                name = str(item.name)
                topic_type = str(item.type)
                existing = topics.get(name)
                if existing is not None and existing['type'] != topic_type:
                    raise ValueError(
                        'ROS topic {!r} changed type from {} to {}'.format(
                            name, existing['type'], topic_type))
                if existing is None:
                    topics[name] = {
                        'type': topic_type,
                        'qos': list(getattr(item, 'offered_qos_profiles', [])),
                    }
        finally:
            reader.close()
    return topics


def _bag_records(rosbag2_py, paths: list[Path], topic_specs=None, allow_legacy=True):
    last_timestamp = 0
    timestamp_offset = 0
    for path in paths:
        reader = _open_reader(rosbag2_py, path)
        try:
            while reader.has_next():
                topic, serialized, timestamp = reader.read_next()
                if topic_specs and topic_specs.get(str(topic), {}).get('type') == 'uv_msgs/msg/DetectionArray':
                    from rclpy.serialization import deserialize_message, serialize_message
                    from .detection_compat import deserialize_detection
                    serialized = serialize_message(deserialize_detection(serialized, deserialize_message, allow_legacy))
                timestamp = int(timestamp) + timestamp_offset
                if timestamp < last_timestamp:
                    timestamp_offset += last_timestamp + 1 - timestamp
                    timestamp = last_timestamp + 1
                last_timestamp = timestamp
                yield timestamp, str(topic), serialized
        finally:
            reader.close()


def _safe_timestamp(value, previous: int) -> int:
    value = int(value or 0)
    if value <= 0:
        value = time.time_ns()
    return max(value, previous)


def _camera_records(
        source: dict, fps: float, segment_duration: float, serialize_message):
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String

    from .player import _timeline_times

    records = source['records']
    times = _timeline_times(records, fps)
    image_topic, metadata_topic, segment_topic = _camera_topics(source)
    previous_timestamp = 0

    def emit(timestamp, image_data, image_format, metadata):
        nonlocal previous_timestamp
        timestamp = _safe_timestamp(timestamp, previous_timestamp)
        previous_timestamp = timestamp
        message = CompressedImage()
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(
            timestamp, 1_000_000_000)
        message.header.frame_id = source['name']
        message.format = image_format
        message.data = image_data
        image_serialized = serialize_message(message)
        metadata_message = String()
        metadata_message.data = json.dumps(metadata, ensure_ascii=False)
        metadata_serialized = serialize_message(metadata_message)
        yield timestamp, image_topic, image_serialized
        yield timestamp, metadata_topic, metadata_serialized

    kind = source['kind']
    if kind == 'raw':
        for index, record in enumerate(records):
            filename = Path(str(record.get('path', '')))
            if filename.name != str(record.get('path', '')):
                continue
            try:
                image_data = (source['directory'] / filename).read_bytes()
            except OSError:
                continue
            metadata = dict(record)
            image_format = 'jpeg' if filename.suffix.lower() in ('.jpg', '.jpeg') else 'png'
            metadata['format'] = image_format
            timestamp = times[index] if index < len(times) else 0
            yield from emit(timestamp, image_data, image_format, metadata)
        return

    if kind == 'jpeg':
        from .jpeg_archive import read_payload

        for index, (ref, record) in enumerate(zip(source['refs'], records)):
            image_data = read_payload(ref)
            if image_data is None:
                continue
            metadata = dict(record)
            metadata['archive_chunk'] = ref.chunk.name
            metadata['format'] = 'jpeg'
            timestamp = times[index] if index < len(times) else 0
            yield from emit(timestamp, image_data, 'jpeg', metadata)
        return

    if kind == 'ts':
        from std_msgs.msg import MultiArrayDimension, UInt8MultiArray

        # Keep the original H.264 segments compressed in the MCAP. The
        # frame-alignment records are written separately with source stamps.
        estimated_frames_per_segment = max(
            1, int(round(max(1.0, fps) * max(0.5, segment_duration))))
        period_ns = max(1, int(round(1_000_000_000 / max(1.0, fps))))

        def segment_records():
            last_timestamp = 0
            segment_frame_cursor = 0
            for segment in source['segments']:
                binary = UInt8MultiArray()
                payload = segment.read_bytes()
                dimension = MultiArrayDimension()
                dimension.label = segment.name
                dimension.size = len(payload)
                dimension.stride = len(payload)
                binary.layout.dim = [dimension]
                binary.layout.data_offset = 0
                segment_record_index = min(
                    segment_frame_cursor, max(0, len(records) - 1))
                timestamp = (
                    times[segment_record_index] if records and
                    segment_record_index < len(times) else
                    (last_timestamp + period_ns if last_timestamp
                     else time.time_ns()))
                timestamp = _safe_timestamp(timestamp, last_timestamp)
                last_timestamp = timestamp
                yield (timestamp, 0, segment_topic,
                       serialize_message(binary))
                segment_frame_cursor += estimated_frames_per_segment

        def alignment_records():
            last_timestamp = 0
            for index, record in enumerate(records):
                timestamp = times[index] if index < len(times) else 0
                timestamp = _safe_timestamp(timestamp, last_timestamp)
                last_timestamp = timestamp
                metadata = dict(record)
                metadata['record_type'] = 'frame_alignment'
                message = String()
                message.data = json.dumps(metadata, ensure_ascii=False)
                yield (timestamp, 1, metadata_topic,
                       serialize_message(message))

        yield from (
            (timestamp, topic, serialized)
            for timestamp, _order, topic, serialized in heapq.merge(
                segment_records(), alignment_records(),
                key=lambda item: (item[0], item[1]))
        )


def _topic_metadata(rosbag2_py, topic_id: int, name: str, topic_type: str,
                    qos=None):
    qos = list(qos or [])
    try:
        return rosbag2_py.TopicMetadata(
            topic_id, name, topic_type, 'cdr', qos)
    except TypeError:
        # Older rosbag2_py releases do not expose the topic ID argument.
        return rosbag2_py.TopicMetadata(
            name=name, type=topic_type, serialization_format='cdr',
            offered_qos_profiles=qos)


def export_session_mcap(
        session_root: str | Path,
        record_mode: str,
        fps: float = 10.0,
        segment_duration: float = 2.0,
) -> dict:
    """Merge recorded ROS topics and camera frames into session.mcap."""
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from std_msgs.msg import String

    from .player import _bag_part_paths

    session_root = Path(session_root).resolve()
    output_path = session_root / 'session.mcap'
    if output_path.exists():
        raise FileExistsError('refusing to overwrite {}'.format(output_path))
    bag_paths = _bag_part_paths(session_root / 'bag')
    try:
        manifest = json.loads((session_root / 'manifest.json').read_text())
    except (OSError, ValueError):
        manifest = {}
    allow_legacy = manifest.get('perception', {}).get('detection_schema_version', 1) < 2
    sources = _camera_sources(session_root, record_mode)
    bag_topics = _bag_topics(rosbag2_py, bag_paths) if bag_paths else {}
    topic_specs = dict(bag_topics)
    marker_topic = '/auv/record/session_metadata'
    topic_specs.setdefault(marker_topic, {'type': _STRING_TYPE, 'qos': []})
    source_topics = []
    for source in sources:
        image_topic, metadata_topic, segment_topic = _camera_topics(source)
        topic_specs.setdefault(metadata_topic, {'type': _STRING_TYPE, 'qos': []})
        if image_topic:
            topic_specs.setdefault(image_topic, {'type': _IMAGE_TYPE, 'qos': []})
        if segment_topic:
            topic_specs.setdefault(
                segment_topic,
                {'type': 'std_msgs/msg/UInt8MultiArray', 'qos': []})
        source_topics.append((source, image_topic, metadata_topic,
                              segment_topic))

    temporary_root = Path(tempfile.mkdtemp(
        prefix='.mcap_export_', dir=str(session_root)))
    writer = None
    writer_open = False
    try:
        writer = rosbag2_py.SequentialWriter()
        writer.open(
            rosbag2_py.StorageOptions(
                uri=str(temporary_root / 'combined'),
                storage_id='mcap',
                max_cache_size=0,
            ),
            rosbag2_py.ConverterOptions('', ''),
        )
        writer_open = True
        for topic_id, (name, spec) in enumerate(topic_specs.items()):
            writer.create_topic(_topic_metadata(
                rosbag2_py, topic_id, name, spec['type'], spec.get('qos')))

        streams = []
        if bag_paths:
            streams.append(_bag_records(rosbag2_py, bag_paths, bag_topics, allow_legacy))
        for source, _image_topic, _metadata_topic, _segment_topic in source_topics:
            streams.append(_camera_records(
                source, fps, segment_duration, serialize_message))
        marker = String()
        marker.data = json.dumps({
            'status': 'complete',
            'record_mode': record_mode,
            'bag_parts': len(bag_paths),
            'camera_streams': [source['name'] for source in sources],
        }, ensure_ascii=False)
        marker_serialized = serialize_message(marker)

        # Every input stream is ordered by its own message timestamps. Merge
        # them lazily so large camera payloads are never held in memory together.
        def tag_stream(stream, stream_index):
            for timestamp, topic, serialized in stream:
                yield timestamp, stream_index, topic, serialized

        tagged = [
            tag_stream(stream, stream_index)
            for stream_index, stream in enumerate(streams)
        ]
        merged = heapq.merge(
            *tagged, key=lambda record: (record[0], record[1]))
        message_count = 0
        camera_frame_count = 0
        last_timestamp = 0
        ts_metadata_topics = {
            _camera_topics(source)[1]
            for source in sources if source['kind'] == 'ts'
        }
        for timestamp, _stream_index, topic, serialized in merged:
            writer.write(topic, serialized, int(timestamp))
            last_timestamp = int(timestamp)
            message_count += 1
            if topic.endswith('/compressed') or topic in ts_metadata_topics:
                camera_frame_count += 1
            if message_count % 10000 == 0:
                print(
                    'uv_record: MCAP export wrote {:,} messages '
                    '({:,} camera frames)'.format(
                        message_count, camera_frame_count),
                    flush=True)
        # Put the completion marker at the last recorded timestamp, so it
        # cannot extend the bag duration when ROS and camera clocks differ.
        writer.write(marker_topic, marker_serialized, last_timestamp)
        message_count += 1
        writer.close()
        writer_open = False

        mcap_files = sorted(temporary_root.rglob('*.mcap'))
        if len(mcap_files) != 1:
            raise RuntimeError(
                'rosbag2 MCAP writer produced {} MCAP files'.format(
                    len(mcap_files)))
        os.replace(mcap_files[0], output_path)
        try:
            fd = os.open(output_path, os.O_RDONLY)
            try:
                os.fsync(fd)
            finally:
                os.close(fd)
        except OSError:
            pass
        shutil.rmtree(temporary_root, ignore_errors=True)
        return {
            'path': output_path.name,
            'storage': 'mcap',
            'messages': message_count,
            'camera_frames': camera_frame_count,
            'bag_parts': len(bag_paths),
            'camera_streams': [source['name'] for source in sources],
        }
    except Exception:
        if writer is not None and writer_open:
            try:
                writer.close()
            except Exception:
                pass
        shutil.rmtree(temporary_root, ignore_errors=True)
        raise
