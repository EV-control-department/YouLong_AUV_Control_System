#!/usr/bin/env python3
"""Save every received front/down MJPEG frame from uv_camera to a dataset.

Simulation and real cameras expose the same endpoints. Run beside uv_camera:

    python3 scripts/record_camera_streams.py
    python3 scripts/record_camera_streams.py --base-url http://AUV_IP:8090

The saved JPEGs are exactly the preview stream frames; they are not the
lossless, unannotated sensor images written by uv_camera's built-in recorder.
"""

import argparse
import json
import os
import threading
import time
import urllib.request
from datetime import datetime
from pathlib import Path


MAX_FRAME_BYTES = 32 * 1024 * 1024


def read_frames(response):
    """Yield (JPEG bytes, MIME part headers) from uv_camera's MJPEG response."""
    content_type = response.headers.get('Content-Type', '')
    if 'multipart/x-mixed-replace' not in content_type.lower():
        raise ValueError(f'not an MJPEG stream: {content_type!r}')
    boundary = None
    for field in content_type.split(';')[1:]:
        key, separator, value = field.strip().partition('=')
        if separator and key.lower() == 'boundary':
            boundary = value.strip().strip('"').encode('ascii')
            break
    if not boundary:
        raise ValueError('MJPEG boundary missing')
    marker = boundary if boundary.startswith(b'--') else b'--' + boundary

    while True:
        line = response.readline(2048)
        if not line:
            return
        if line.strip() == marker + b'--':
            return
        if line.strip() != marker:
            continue
        headers = {}
        for _ in range(16):
            line = response.readline(2048)
            if not line:
                raise EOFError('MJPEG part headers truncated')
            if line in (b'\r\n', b'\n'):
                break
            key, separator, value = line.partition(b':')
            if not separator:
                raise ValueError('invalid MJPEG part header')
            headers[key.decode('ascii').lower()] = value.strip().decode('ascii')
        else:
            raise ValueError('too many MJPEG part headers')
        length = int(headers.get('content-length', '0'))
        if not 4 <= length <= MAX_FRAME_BYTES:
            raise ValueError(f'invalid JPEG size: {length}')
        jpeg = response.read(length)
        if len(jpeg) != length:
            raise EOFError('JPEG truncated')
        if not (jpeg.startswith(b'\xff\xd8') and jpeg.endswith(b'\xff\xd9')):
            raise ValueError('invalid JPEG markers')
        yield jpeg, headers


def optional_int(value):
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


class Session:
    def __init__(self, output, base_url):
        output.mkdir(parents=True, exist_ok=True)
        base_name = datetime.now().strftime('stream_%Y%m%d_%H%M%S')
        for suffix in range(1000):
            directory = output / f'{base_name}_{os.getpid()}_{suffix}'
            try:
                directory.mkdir()
                break
            except FileExistsError:
                continue
        else:
            raise RuntimeError('could not create a unique recording directory')
        self.directory = directory
        self.lock = threading.Lock()
        self.counts = {'front': 0, 'down': 0}
        self.errors = []
        for stream in self.counts:
            (directory / stream).mkdir()
        self.manifest = (directory / 'frames.jsonl').open('a', encoding='utf-8')
        (directory / 'session.json').write_text(json.dumps({
            'source': base_url, 'format': 'uv_camera_mjpeg_v1',
            'created_unix_ns': time.time_ns(),
        }, indent=2) + '\n', encoding='utf-8')

    def save(self, stream, jpeg, headers):
        with self.lock:
            index = self.counts[stream]
            relative = Path(stream) / f'{index:08d}.jpg'
            target = self.directory / relative
            temporary = target.with_suffix('.jpg.tmp')
            with temporary.open('wb') as output:
                output.write(jpeg)
            os.replace(temporary, target)
            self.manifest.write(json.dumps({
                'stream': stream,
                'index': index,
                'path': str(relative),
                'capture_stamp_ns': optional_int(headers.get('x-frame-stamp-ns')),
                'source_sequence': optional_int(headers.get('x-frame-sequence')),
                'received_unix_ns': time.time_ns(),
            }) + '\n')
            self.manifest.flush()
            self.counts[stream] += 1

    def close(self, state):
        self.manifest.close()
        (self.directory / 'status.json').write_text(json.dumps({
            'state': state, 'frames': self.counts, 'errors': self.errors,
        }, indent=2) + '\n', encoding='utf-8')


def record_stream(session, stream, base_url, max_fps, max_frames,
                  retries, timeout, stop):
    url = f'{base_url}/{stream}'
    failures = 0
    last_saved = float('-inf')
    while not stop.is_set() and (not max_frames or session.counts[stream] < max_frames):
        try:
            with urllib.request.urlopen(url, timeout=timeout) as response:
                last_sequence = None
                for jpeg, headers in read_frames(response):
                    if stop.is_set():
                        return
                    failures = 0
                    sequence = headers.get('x-frame-sequence')
                    if sequence is not None and sequence == last_sequence:
                        continue
                    last_sequence = sequence
                    now = time.monotonic()
                    if max_fps and now - last_saved < 1.0 / max_fps:
                        continue
                    session.save(stream, jpeg, headers)
                    last_saved = now
                    if max_frames and session.counts[stream] >= max_frames:
                        return
            raise EOFError('stream closed')
        except (OSError, EOFError, ValueError) as error:
            if stop.is_set():
                return
            failures += 1
            print(f'{stream}: {error}; reconnect {failures}/{retries}', flush=True)
            if failures > retries:
                with session.lock:
                    session.errors.append(f'{stream}: {error}')
                stop.set()
                return
            stop.wait(1.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base-url', default='http://127.0.0.1:8090',
                        help='uv_camera MJPEG address; use AUV IP for real cameras')
    parser.add_argument('--output', type=Path, default=Path('records/datasets'))
    parser.add_argument('--fps', type=float, default=0,
                        help='maximum stored FPS per stream; 0 stores every frame')
    parser.add_argument('--frames', type=int, default=0,
                        help='frames per stream, 0 means record until Ctrl-C')
    parser.add_argument('--retries', type=int, default=5)
    parser.add_argument('--timeout', type=float, default=10.0,
                        help='HTTP idle timeout in seconds')
    args = parser.parse_args()
    if args.fps < 0 or args.frames < 0 or args.retries < 0 or args.timeout <= 0:
        parser.error('fps, frames and retries must be nonnegative; timeout must be positive')
    base_url = args.base_url.rstrip('/')
    if not base_url.startswith(('http://', 'https://')):
        parser.error('base-url must start with http:// or https://')

    session = Session(args.output, base_url)
    print(f'Recording front and down to {session.directory}', flush=True)
    stop = threading.Event()
    workers = [threading.Thread(
        target=record_stream,
        args=(session, stream, base_url, args.fps, args.frames,
              args.retries, args.timeout, stop), daemon=True)
        for stream in ('front', 'down')]
    interrupted = False
    try:
        for worker in workers:
            worker.start()
        while any(worker.is_alive() for worker in workers):
            for worker in workers:
                worker.join(timeout=0.2)
    except KeyboardInterrupt:
        interrupted = True
        stop.set()
    finally:
        stop.set()
        for worker in workers:
            worker.join(timeout=args.timeout + 1.0)
        state = 'failed' if session.errors else ('interrupted' if interrupted else 'completed')
        session.close(state)
        print(f'{state}: {session.counts}; {session.directory}', flush=True)
    return 1 if session.errors else 0


if __name__ == '__main__':
    raise SystemExit(main())
