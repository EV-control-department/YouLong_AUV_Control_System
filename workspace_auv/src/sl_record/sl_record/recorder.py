"""DDS-discovered camera recording using read-only session/rosbag utilities."""

import argparse
import json
import os
from pathlib import Path
import re
import signal
import threading
import time

import rclpy

from sl_camera.profile import load_settings, positive
from uv_record.performance import ProcessSampler
from uv_record.recorder import (
    ChildSupervisor, METADATA_TOPIC_REGEX, Recorder, SegmentSyncer,
    _rosbag_record_help, _select_bag_storage, _session_mcap_unavailable_reason,
)
from uv_record.session import (
    append_event, create_session, finish_session, initialise_session,
    project_root, update_heartbeat, update_manifest,
)

from .discovery import DiscoveryThread
from .frames import DiscoveredFrames


def default_output_root():
    return project_root() / 'records/sl_sessions'


def _parse_args(argv=None):
    selector = argparse.ArgumentParser(add_help=False)
    selector.add_argument('--profile', default='default')
    selected, _ = selector.parse_known_args(argv)
    path, topic, settings, _ = load_settings(selected.profile)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', default=str(path))
    parser.add_argument('--session-dir')
    parser.add_argument('--output-root', default=settings['output_root'] or str(default_output_root()))
    parser.add_argument('--catalog-topic', default=topic)
    parser.add_argument('--video-fps', type=float, default=float(settings['video_fps']))
    parser.add_argument('--bag-duration', type=float, default=float(settings['bag_duration']))
    parser.add_argument('--bag-storage', choices=('auto', 'sqlite3', 'mcap'),
                        default=settings['bag_storage'])
    parser.add_argument('--topic-regex', default=METADATA_TOPIC_REGEX)
    parser.add_argument('--use-sim-time', default='false')
    args, _ = parser.parse_known_args(argv)
    for key in ('video_fps', 'bag_duration'):
        try:
            positive(getattr(args, key), key)
        except ValueError as error:
            parser.error(str(error))
    if not args.catalog_topic.startswith('/'):
        parser.error('--catalog-topic must be absolute')
    args.profile = str(path)
    return args


class BagCommand:
    """Use the existing rosbag command builder without changing the UV recorder."""

    def __init__(self, paths, args):
        self.paths, self.args = paths, args
        self.bag_storage = _select_bag_storage(args.bag_storage)
        self._bag_record_help = _rosbag_record_help()

    def _bag_topic_regex(self):
        if re.search(self.args.topic_regex, self.args.catalog_topic):
            return self.args.topic_regex
        return f'(?:{self.args.topic_regex})|(?:^{re.escape(self.args.catalog_topic)}$)'

    def command(self, part):
        return Recorder._bag_command(self, part)


class SlRecorder:
    def __init__(self, paths, args):
        self.paths, self.args = paths, args
        self.frames = DiscoveredFrames(paths.root, fps=args.video_fps)
        self.bag = BagCommand(paths, args)
        self.discovery = None
        self.child = None
        self.syncer = None
        self.closed = False
        self.sampler = ProcessSampler(os.getpid(), storage_path=paths.root)
        self.next_performance = 0.0

    def start(self):
        update_manifest(
            self.paths.root, sl_schema_version=1,
            recorder={'profile': self.args.profile, 'video_fps': self.args.video_fps,
                      'topic_regex': self.bag._bag_topic_regex(), 'image_topics_excluded': True},
            bag={'directory': 'bag', 'storage': self.bag.bag_storage,
                 'segment_seconds': self.args.bag_duration},
            camera_raw={'directory': 'camera/raw', 'format': 'jpeg', 'frame_index': 'frames.jsonl'},
            camera_catalog={'topic': self.args.catalog_topic, 'type': 'sl_msgs/msg/CameraCatalog',
                            'snapshot': 'metadata/camera_catalog.json',
                            'history': 'metadata/camera_catalogs.jsonl'})
        self.discovery = DiscoveryThread(self.frames, self.args.catalog_topic)
        self.discovery.start()
        parts = [int(path.name[5:]) for path in self.paths.bag.glob('part_*')
                 if path.name[5:].isdigit()]
        next_part = max(parts, default=-1) + 1

        def command():
            nonlocal next_part
            result = self.bag.command(next_part)
            next_part += 1
            return result

        self.child = ChildSupervisor(
            'rosbag2', command, self.paths.logs / 'nodes/rosbag2.log',
            stop_signal=signal.SIGINT)
        self.child.start()
        self.syncer = SegmentSyncer(
            [self.paths.bag], patterns=('*.mcap', '*.db3', 'metadata.yaml'))
        self.syncer.start()
        append_event(self.paths.root, {'event': 'sl_recorder_started'})
        print(f'sl_record: session {self.paths.root}; waiting for {self.args.catalog_topic}', flush=True)

    def tick(self):
        status = self.frames.snapshot()
        if self.frames.error is not None:
            raise RuntimeError(f'camera archive failed: {self.frames.error}')
        child = self.child.snapshot() if self.child is not None else {}
        update_heartbeat(self.paths.root, camera_raw=status, rosbag=child)
        if time.monotonic() >= self.next_performance:
            self.next_performance = time.monotonic() + 5
            sample = self.sampler.sample()
            sample['camera_raw'] = status
            sample['rosbag'] = child
            with (self.paths.metadata / 'performance.jsonl').open('a') as stream:
                stream.write(json.dumps(sample, ensure_ascii=False) + '\n')
                stream.flush()

    def stop(self):
        if self.closed:
            return
        self.closed = True
        errors = []
        if self.child is not None:
            self.child.stop()
        if not self.frames.stop():
            errors.append('camera reader threads did not stop')
        if self.discovery is not None:
            try:
                self.discovery.stop()
            except Exception as error:
                errors.append(str(error))
        if self.child is not None:
            self.child.join(timeout=20)
            if self.child.is_alive():
                errors.append('rosbag writer did not stop')
        if self.syncer is not None:
            self.syncer.stop()
            self.syncer.join(timeout=5)
            if self.syncer.is_alive():
                errors.append('bag syncer did not stop')
            self.syncer.sync_all()
        status = self.frames.snapshot()
        if self.frames.error is not None:
            errors.append(str(self.frames.error))
        update_manifest(
            self.paths.root,
            timestamp_alignment={'status': status['alignment_status'], 'mode': 'raw', **status},
            recording_health={'shutdown_errors': errors},
            camera_catalog={
                'topic': self.args.catalog_topic, 'type': 'sl_msgs/msg/CameraCatalog',
                'snapshot': 'metadata/camera_catalog.json',
                'history': 'metadata/camera_catalogs.jsonl',
                'last_catalog_sequence': status['catalog_sequence']})
        if errors:
            raise RuntimeError('; '.join(errors))
        reason = _session_mcap_unavailable_reason()
        if reason is None:
            from uv_record.mcap_export import export_session_mcap
            final = export_session_mcap(self.paths.root, 'raw', fps=self.args.video_fps)
        else:
            final = {'status': 'unavailable', 'reason': reason, 'path': None}
        update_manifest(self.paths.root, final_mcap=final)
        append_event(self.paths.root, {'event': 'sl_recorder_stopped'})


def run_recorder(args):
    stop_requested = threading.Event()
    paths = None
    recorder = None
    rclpy.init(args=[])
    old_handlers = {}

    def request_stop(_signum, _frame):
        stop_requested.set()

    for number in (signal.SIGINT, signal.SIGTERM):
        old_handlers[number] = signal.signal(number, request_stop)
    failure = None
    try:
        paths = (initialise_session(args.session_dir) if args.session_dir
                 else create_session(args.output_root))
        recorder = SlRecorder(paths, args)
        recorder.start()
        while not stop_requested.wait(1):
            recorder.tick()
    except (KeyboardInterrupt,):
        stop_requested.set()
    except Exception as error:
        failure = error
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            if recorder is not None:
                try:
                    recorder.stop()
                except Exception as error:
                    failure = failure or error
            if paths is not None:
                finish_session(paths.root, 'FAILED' if failure else 'STOPPED',
                               error=str(failure) if failure else None)
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            for number, handler in old_handlers.items():
                signal.signal(number, handler)
    if failure:
        print(f'sl_record: failed: {failure}', flush=True)
        return 1
    print('sl_record: stopped cleanly', flush=True)
    return 0


def main():
    return run_recorder(_parse_args())
