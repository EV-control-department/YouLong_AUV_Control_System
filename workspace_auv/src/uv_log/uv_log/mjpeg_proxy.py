"""Record an MJPEG stream as JPEG archives or crash-recoverable TS.

The JPEG archive path stores the already-compressed source frames and defers
decoding until playback.  The legacy TS path keeps the old constant-rate
H.264 encoder for compatibility.
"""

from __future__ import annotations

import argparse
import re
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from urllib.request import Request, urlopen

from .jpeg_archive import JpegArchiveWriter
from .session import write_json_atomic


class LatestJpeg:
    """Thread-safe latest-frame cache used by the downloader and pacer."""

    def __init__(self):
        self._condition = threading.Condition()
        self._frame: tuple[bytes, int, int] | None = None

    def put(self, frame: bytes, sequence: int, timestamp_ns: int) -> None:
        with self._condition:
            self._frame = (frame, sequence, timestamp_ns)
            self._condition.notify_all()

    def get(self) -> tuple[bytes, int, int] | None:
        with self._condition:
            return self._frame


def _mjpeg_reader(
    url: str,
    cache: LatestJpeg | None,
    stop_event: threading.Event,
    on_frame=None,
) -> None:
    """Reconnect to MJPEG and deliver complete JPEGs with source metadata."""
    fallback_sequence = 0
    while not stop_event.is_set():
        try:
            request = Request(url, headers={'User-Agent': 'uv-log-recorder'})
            with urlopen(request, timeout=3.0) as response:
                buffer = b''
                while not stop_event.is_set():
                    # Return available bytes immediately; read(n) can wait for
                    # the next low-rate frame just to fill its 64 KiB buffer.
                    chunk = response.read1(65536)
                    if not chunk:
                        break
                    buffer += chunk
                    while True:
                        start = buffer.find(b'\xff\xd8')
                        if start < 0:
                            boundary = buffer.rfind(b'--frame')
                            buffer = buffer[boundary:] if boundary >= 0 else buffer[-1:]
                            break
                        end = buffer.find(b'\xff\xd9', start + 2)
                        if end < 0:
                            if len(buffer) > 16 * 1024 * 1024:
                                buffer = buffer[start:]
                            break
                        payload = buffer[start:end + 2]
                        part_start = buffer.rfind(b'--frame', 0, start)
                        headers = buffer[part_start:start] if part_start >= 0 else b''
                        sequence_match = re.search(
                            rb'(?im)^X-Frame-Sequence:\s*(\d+)', headers)
                        stamp_match = re.search(
                            rb'(?im)^X-Frame-Stamp-Ns:\s*(\d+)', headers)
                        sequence = int(sequence_match.group(1)) \
                            if sequence_match else fallback_sequence
                        timestamp_ns = int(stamp_match.group(1)) \
                            if stamp_match else time.time_ns()
                        if on_frame is not None:
                            on_frame(payload, sequence, timestamp_ns)
                        elif cache is not None:
                            cache.put(payload, sequence, timestamp_ns)
                        fallback_sequence += 1
                        buffer = buffer[end + 2:]
        except (OSError, ValueError):
            pass
        stop_event.wait(0.25)


def _rate_text(value: float) -> str:
    return str(int(value)) if value.is_integer() else f'{value:g}'


def _ffmpeg_command(args) -> list[str]:
    output_dir = Path(args.output_dir)
    output = str(output_dir / '%06d.ts')
    duration = max(0.5, float(args.segment_duration))
    fps = max(1.0, float(args.fps))
    codec_options = ['-c:v', args.video_codec]
    if args.video_codec == 'libx264':
        codec_options += ['-preset', 'ultrafast', '-tune', 'zerolatency']
    elif args.video_codec == 'h264_nvenc':
        codec_options += ['-preset', 'p1', '-tune', 'll']
    return [
        args.ffmpeg,
        '-hide_banner', '-nostdin', '-loglevel', 'warning',
        # The proxy supplies one complete JPEG per input frame at a known
        # cadence.  image2pipe does not infer a fake 25 Hz clock from network
        # packet arrival times.
        '-f', 'image2pipe', '-framerate', _rate_text(fps),
        '-vcodec', 'mjpeg', '-i', 'pipe:0',
        '-map', '0:v:0', '-an',
        *codec_options,
        '-pix_fmt', 'yuv420p',
        # Keep one output frame per input tick.  The input cadence is set to
        # the camera topic rate by the recorder, not the old fake 25 FPS.
        '-fps_mode', 'cfr',
        '-force_key_frames', f'expr:gte(t,n_forced*{duration})',
        '-flush_packets', '1',
        '-f', 'segment',
        '-segment_time', str(duration),
        '-segment_format', 'mpegts',
        '-reset_timestamps', '1',
        '-segment_start_number', str(args.start_number),
        '-segment_list', args.playlist,
        '-segment_list_type', 'm3u8',
        '-segment_list_size', '0',
        output,
    ]


def _parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--playlist')
    parser.add_argument('--start-number', type=int, default=0)
    parser.add_argument('--segment-duration', type=float, default=2.0)
    parser.add_argument('--fps', type=float, default=10.0)
    parser.add_argument(
        '--output-format', choices=('jpeg', 'ts'), default='ts',
        help='Store source JPEG chunks or transcode to MPEG-TS/H.264')
    parser.add_argument('--video-codec', default='libx264')
    parser.add_argument('--ffmpeg', default='ffmpeg')
    args, _ros_args = parser.parse_known_args()
    return args


def main():
    args = _parse_args()
    if args.output_format == 'jpeg':
        return _run_jpeg_archive(args)
    if not args.playlist:
        raise SystemExit('--playlist is required for --output-format ts')
    stop_event = threading.Event()

    def request_stop(_signum, _frame):
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    cache = LatestJpeg()
    reader = threading.Thread(
        target=_mjpeg_reader,
        args=(args.url, cache, stop_event),
        name='uv-log-mjpeg-reader',
        daemon=True,
    )
    reader.start()

    process = None
    try:
        Path(args.output_dir).mkdir(parents=True, exist_ok=True)
        process = subprocess.Popen(
            _ffmpeg_command(args),
            stdin=subprocess.PIPE,
            stdout=None,
            stderr=None,
        )

        while not stop_event.is_set():
            if cache.get() is not None:
                break
            if process.poll() is not None:
                return process.returncode or 1
            stop_event.wait(0.05)
        if stop_event.is_set():
            return 0

        frame_period = 1.0 / max(1.0, float(args.fps))
        next_tick = time.monotonic()
        while not stop_event.is_set():
            sample = cache.get()
            if sample is not None and process.poll() is None:
                try:
                    process.stdin.write(sample[0])
                    process.stdin.flush()
                except (BrokenPipeError, OSError):
                    break

            next_tick += frame_period
            delay = next_tick - time.monotonic()
            if delay > 0.0:
                stop_event.wait(delay)
            else:
                # Encoding fell behind; do not burst-write old frames and
                # create another artificial speed-up.
                next_tick = time.monotonic()

        if process.stdin is not None:
            process.stdin.close()
        return process.wait(timeout=5.0)
    except (OSError, subprocess.SubprocessError) as error:
        print(f'uv_log mjpeg proxy failed: {error}', file=sys.stderr)
        return 1
    finally:
        stop_event.set()
        reader.join(timeout=4.0)
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


def _run_jpeg_archive(args) -> int:
    """Store source JPEGs directly; no decoder or video encoder is started."""
    stop_event = threading.Event()
    write_error = []
    progress = {'status': 'waiting_for_video', 'frames': 0, 'bytes': 0,
                'last_frame_received_unix_ns': 0}
    writer = JpegArchiveWriter(
        args.output_dir,
        start_number=args.start_number,
        fps=args.fps,
        chunk_seconds=args.segment_duration,
    )

    def request_stop(_signum, _frame):
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    def save_frame(payload, sequence, timestamp_ns):
        try:
            if writer.write(payload, timestamp_ns=timestamp_ns, sequence=sequence):
                progress.update(status='recording', frames=progress['frames'] + 1,
                                bytes=progress['bytes'] + len(payload),
                                last_frame_received_unix_ns=time.time_ns())
        except (OSError, ValueError) as error:
            write_error.append(error)
            print(f'uv_log mjpeg archive failed: {error}', file=sys.stderr)
            stop_event.set()

    reader = threading.Thread(
        target=_mjpeg_reader,
        args=(args.url, None, stop_event),
        kwargs={'on_frame': save_frame},
        name='uv-log-mjpeg-archive-reader',
        daemon=True,
    )
    reader.start()
    try:
        while not stop_event.is_set():
            write_json_atomic(Path(args.output_dir) / 'status.json', dict(progress))
            stop_event.wait(5.0)
    finally:
        stop_event.set()
        reader.join(timeout=4.0)
        writer.close()
        progress['status'] = 'failed' if write_error else 'stopped'
        write_json_atomic(Path(args.output_dir) / 'status.json', dict(progress))
    return 1 if write_error else 0


if __name__ == '__main__':
    raise SystemExit(main())
