"""Capture launch lifecycle and non-ROS output inside the recording session."""

import re

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessIO, OnProcessStart
from uv_log.session import append_event, update_manifest, write_json_atomic


def session_log_handlers(paths, arguments):
    write_json_atomic(paths.metadata / 'launch_arguments.json', arguments)
    update_manifest(paths.root, logs={
        'ros': 'logs/ros', 'process_output': 'logs/nodes',
        'lifecycle': 'events.jsonl', 'performance': 'metadata/performance.jsonl',
        'launch_arguments': 'metadata/launch_arguments.json',
    })

    def lifecycle(event, context):
        record = {
            'event': 'process_exit' if hasattr(event, 'returncode') else 'process_start',
            'name': event.process_name, 'pid': event.pid,
        }
        if hasattr(event, 'returncode'):
            record['returncode'] = event.returncode
        else:
            record['command'] = event.cmd
        append_event(paths.root, record)
        return []

    def output(event):
        name = re.sub(r'[^A-Za-z0-9_.-]', '_', event.process_name)
        path = paths.logs / 'nodes' / f'{name}_{event.pid}.log'
        with path.open('ab') as handle:
            handle.write(event.text)

    return [
        RegisterEventHandler(OnProcessStart(on_start=lifecycle)),
        RegisterEventHandler(OnProcessExit(on_exit=lifecycle)),
        RegisterEventHandler(OnProcessIO(on_stdout=output, on_stderr=output)),
    ]
