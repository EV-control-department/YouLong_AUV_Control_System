"""Record the algorithm-master image stream, independent of video preview."""

from __future__ import annotations

from pathlib import Path
import argparse
import json
import signal
import time

import cv2

from auv_protocol.topics import ICEORYX_CAMERA_DOWN, ICEORYX_CAMERA_FRONT
from uv_perception.transport.iceoryx2 import Iceoryx2Reader


def _record_service(service: str, camera: str, output: Path, stop) -> None:
    frame_dir = output / 'frames' / camera
    frame_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = output / 'manifest.jsonl'
    reader = Iceoryx2Reader(service)
    try:
        with manifest_path.open('a', encoding='utf-8') as manifest:
            while not stop.is_set():
                packet = reader.read()
                if packet is None:
                    return
                image = packet.bgr()
                filename = f'{packet.header.capture_id:020d}.png'
                relative = Path('frames') / camera / filename
                if not cv2.imwrite(str(output / relative), image,
                                   [cv2.IMWRITE_PNG_COMPRESSION, 1]):
                    raise RuntimeError(f'failed to write {output / relative}')
                manifest.write(json.dumps({
                    'camera_group': camera,
                    'service': service,
                    'capture_id': packet.header.capture_id,
                    'stereo_pair_id': packet.header.stereo_pair_id,
                    'timestamp_ns': packet.header.timestamp_ns,
                    'camera_info_version': packet.header.camera_info_version,
                    'width': packet.header.width,
                    'height': packet.header.height,
                    'stride': packet.header.stride,
                    'encoding': packet.header.encoding,
                    'path': str(relative),
                }, ensure_ascii=False) + '\n')
                manifest.flush()
    finally:
        reader.close()


def record(output: Path, cameras: tuple[str, ...]) -> None:
    output.mkdir(parents=True, exist_ok=True)
    stop = __import__('threading').Event()
    signal.signal(signal.SIGINT, lambda *_args: stop.set())
    signal.signal(signal.SIGTERM, lambda *_args: stop.set())
    # Each service gets its own reader.  Threads prevent a quiet down camera
    # from delaying front data and preserve the original timestamps.
    import threading
    services = {
        'front': ICEORYX_CAMERA_FRONT,
        'down': ICEORYX_CAMERA_DOWN,
    }
    threads = [threading.Thread(
        target=_record_service,
        args=(services[camera], camera, output, stop),
        name=f'dataset-{camera}', daemon=True)
        for camera in cameras]
    for thread in threads:
        thread.start()
    try:
        while any(thread.is_alive() for thread in threads):
            time.sleep(0.2)
    except KeyboardInterrupt:
        stop.set()
    finally:
        stop.set()
        for thread in threads:
            thread.join(timeout=2.0)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--camera', choices=('front', 'down'), action='append')
    args = parser.parse_args(argv)
    record(args.output, tuple(args.camera or ('front', 'down')))


if __name__ == '__main__':  # pragma: no cover
    main()
