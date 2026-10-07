"""Record go2rtc HTTP video frames with source-frame timestamp metadata."""

from __future__ import annotations

import argparse
import json
import os
import queue
import re
import select
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

from .frame_mapping import FrameMappingSubscriber
from .jpeg_archive import JpegArchiveWriter, archive_entries
from .session import write_json_atomic

PTS_RE = re.compile(rb'pts_time:(-?\d+(?:\.\d+)?|N/A)')
SHOWINFO_N_RE = re.compile(rb'\bn:\s*(\d+)')


def _install_signals(stop_event: threading.Event) -> None:
    def request_stop(_signum, _frame):
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)


def _video_filter(args):
    # Select source frames without synthesizing new timestamps or duplicate frames.
    fps = max(0.1, float(args.fps))
    filters = [f"select='isnan(prev_selected_t)+gte(t-prev_selected_t,{1.0/fps})'"]
    width = int(getattr(args, 'max_width', 0))
    if width > 0:
        filters.append(f"scale='trunc(min(iw,{width})/2)*2':-2")
    return ','.join(filters + ['showinfo'])


def _input_options(args):
    # HTTP MJPEG has no reliable frame cadence; FFmpeg otherwise guesses 25Hz
    # and the recording plays fast/slow. Keep receive-time spacing, zero origin.
    return (['-use_wallclock_as_timestamps', '1', '-start_at_zero']
            if getattr(args, 'wallclock_input', False) else [])


def _jpeg_command(args) -> list[str]:
    return [
        args.ffmpeg,
        '-hide_banner', '-nostdin', '-loglevel', 'info', '-copyts',
        '-rw_timeout', '15000000',
        *_input_options(args),
        '-i', args.url,
        '-map', '0:v:0', '-an', '-vf', _video_filter(args),
        '-vsync', '0', '-threads', '2', '-q:v', '3',
        '-f', 'image2pipe', '-vcodec', 'mjpeg', 'pipe:1',
    ]


def _ts_command(args) -> list[str]:
    output_dir = Path(args.output_dir)
    output = str(output_dir / '%06d.ts')
    duration = max(0.5, float(args.segment_duration))
    codec_options = ['-c:v', args.video_codec]
    if args.video_codec == 'libx264':
        codec_options += ['-preset', 'ultrafast', '-tune', 'zerolatency']
    elif args.video_codec == 'h264_nvenc':
        codec_options += ['-preset', 'p1', '-tune', 'll']
    bitrate = int(getattr(args, 'bitrate_kbps', 0))
    if bitrate > 0:
        codec_options += ['-b:v', f'{bitrate}k', '-maxrate', f'{bitrate}k',
                          '-bufsize', f'{bitrate*2}k']
    return [
        args.ffmpeg,
        '-hide_banner', '-nostdin', '-loglevel', 'info', '-copyts',
        '-rw_timeout', '15000000',
        *_input_options(args),
        '-i', args.url,
        '-map', '0:v:0', '-an', '-vf', _video_filter(args),
        *codec_options,
        '-threads', '2', '-vsync', '0',
        '-pix_fmt', 'yuv420p',
        '-force_key_frames', f'expr:gte(t,n_forced*{duration})',
        '-flush_packets', '1',
        '-hls_time', str(duration),
        '-hls_playlist_type', 'event',
        '-hls_list_size', '0',
        '-hls_flags', 'independent_segments+temp_file',
        '-hls_segment_filename', output,
        '-start_number', str(args.start_number),
        '-f', 'hls', args.playlist,
    ]


def _parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--playlist')
    parser.add_argument('--camera', choices=('front', 'down'), required=True)
    parser.add_argument(
        '--stream-mode', choices=('unannotated', 'annotated'),
        default='unannotated')
    parser.add_argument('--start-number', type=int, default=0)
    parser.add_argument('--segment-duration', type=float, default=2.0)
    parser.add_argument('--fps', type=float, default=10.0)
    parser.add_argument('--max-width', type=int, default=0)
    parser.add_argument('--bitrate-kbps', type=int, default=0)
    parser.add_argument('--wallclock-input', action='store_true')
    parser.add_argument(
        '--output-format', choices=('jpeg', 'ts'), default='jpeg',
        help='Store recoverable JPEG chunks or HLS MPEG-TS segments')
    parser.add_argument('--video-codec', default='libx264')
    parser.add_argument('--ffmpeg', default='ffmpeg')
    args, _ros_args = parser.parse_known_args()
    return args


def _parse_pts(line: bytes) -> int | None:
    match = PTS_RE.search(line)
    if not match or match.group(1) == b'N/A':
        return None
    try:
        return int(round(float(match.group(1)) * 1_000_000_000))
    except ValueError:
        return None


def _parse_showinfo(line: bytes) -> tuple[int | None, int | None]:
    frame_match = SHOWINFO_N_RE.search(line)
    return (int(frame_match.group(1)) if frame_match else None,
            _parse_pts(line))


def _pts_for_frame(pts_queue, pending, frame_index: int, timeout: float):
    """Take only the PTS logged for this exact decoded frame number."""
    deadline = time.monotonic() + max(0.0, timeout)
    while True:
        if frame_index in pending:
            return pending.pop(frame_index)
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return None
        try:
            logged_index, pts_ns = pts_queue.get(timeout=remaining)
        except queue.Empty:
            return None
        if logged_index is None or logged_index < frame_index:
            continue
        pending[logged_index] = pts_ns
        if len(pending) > 64:
            pending.pop(min(pending))


def _alignment_record(mapping: FrameMappingSubscriber, sequence: int,
                      pts_ns: int | None, decoder_frame_index: int | None = None) -> dict:
    receive_ns = time.time_ns()
    source = mapping.match(pts_ns) if pts_ns is not None else None
    replay_ns = receive_ns
    if pts_ns is not None:
        previous = getattr(mapping, '_replay_last_pts', None)
        if previous is None or pts_ns < previous:
            mapping._replay_anchor = (pts_ns, receive_ns)
        mapping._replay_last_pts = pts_ns
        anchor_pts, anchor_receive = mapping._replay_anchor
        replay_ns = anchor_receive + pts_ns - anchor_pts
    return {
        'sequence': int(sequence),
        'decoder_frame_index': decoder_frame_index,
        'presentation_timestamp_ns': pts_ns,
        'source_timestamp_ns': (
            source['source_timestamp_ns'] if source else None),
        'timestamp_ns': source['source_timestamp_ns'] if source else 0,
        'timestamp_aligned': bool(source and source['source_timestamp_ns'] > 0),
        # Approximate playback clock only, NEVER pretend this is a capture stamp.
        'replay_timestamp_ns': replay_ns,
        'replay_clock': 'receive_anchored_pts',
        'receive_time_unix_ns': receive_ns,
        'stream_instance_id': source['stream_instance_id'] if source else None,
        'camera_name': mapping.camera,
        'stream_mode': mapping.stream_mode,
        'frame_sequence': source['frame_sequence'] if source else None,
        'capture_id': source['capture_id'] if source else None,
        'stereo_pair_id': source['stereo_pair_id'] if source else None,
        'timestamp_epoch': source['timestamp_epoch'] if source else None,
    }


def _read_alignment_lines(stream, mapping, output_dir, progress, lock):
    """Drain FFmpeg showinfo and persist one source map entry per output frame."""
    index_path = Path(output_dir) / 'frame_alignment.jsonl'
    sequence = 0
    try:
        with index_path.open(encoding='utf-8') as existing:
            for old_line in existing:
                try:
                    sequence = max(sequence, int(json.loads(old_line).get('sequence', -1)) + 1)
                except (ValueError, json.JSONDecodeError):
                    continue
    except OSError:
        pass
    try:
        with index_path.open('a', encoding='utf-8') as index:
            for line in iter(stream.readline, b''):
                if b'pts_time:' not in line:
                    continue
                decoder_frame_index, pts_ns = _parse_showinfo(line)
                record = _alignment_record(
                    mapping, sequence, pts_ns, decoder_frame_index)
                index.write(json.dumps(record, ensure_ascii=False) + '\n')
                index.flush()
                with lock:
                    progress['frames'] += 1
                    progress['unaligned_frames'] += int(not record['timestamp_aligned'])
                    progress['last_frame_received_unix_ns'] = record[
                        'receive_time_unix_ns']
                    if record['timestamp_aligned']:
                        progress['last_source_timestamp_ns'] = record[
                            'source_timestamp_ns']
                sequence += 1
    except OSError as error:
        with lock:
            progress['error'] = str(error)


def _run_ts(args) -> int:
    if not args.playlist:
        raise SystemExit('--playlist is required for --output-format ts')
    stop_event = threading.Event()
    _install_signals(stop_event)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    progress = {
        'status': 'waiting_for_video', 'frames': 0, 'unaligned_frames': 0,
        'last_frame_received_unix_ns': 0, 'alignment_status': 'unknown',
    }
    lock = threading.Lock()
    mapping = FrameMappingSubscriber(args.camera, args.stream_mode)
    process = None
    alignment_thread = None
    try:
        process = subprocess.Popen(
            _ts_command(args), stderr=subprocess.PIPE, start_new_session=True)
        alignment_thread = threading.Thread(
            target=_read_alignment_lines,
            args=(process.stderr, mapping, output_dir, progress, lock),
            name='uv-record-ts-frame-index', daemon=True)
        alignment_thread.start()
        while process.poll() is None and not stop_event.wait(0.2):
            with lock:
                progress_copy = dict(progress)
            progress_copy['status'] = 'recording'
            write_json_atomic(output_dir / 'status.json', progress_copy)
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        result = process.wait(timeout=10.0)
        if alignment_thread.is_alive():
            alignment_thread.join(timeout=3.0)
        segments = sorted(output_dir.glob('*.ts'))
        for path in output_dir.glob('*.ts.tmp'):
            path.unlink(missing_ok=True)
        segments = [path for path in segments if path.is_file() and path.stat().st_size > 0]
        playlist = Path(args.playlist)
        has_entries = False
        try:
            has_entries = any(
                line.startswith('#EXTINF:')
                for line in playlist.read_text(encoding='utf-8').splitlines())
        except OSError:
            pass
        if not segments or not has_entries:
            print('uv_record HTTP TS recorder produced no complete HLS segment.',
                  file=sys.stderr)
            return result or 1
        with lock:
            progress_copy = dict(progress)
        progress_copy.update({
            'status': 'stopped' if stop_event.is_set() else 'failed',
            'alignment_status': (
                'aligned' if progress_copy['frames'] > 0
                and progress_copy['unaligned_frames'] == 0 else 'degraded'),
        })
        write_json_atomic(output_dir / 'status.json', progress_copy)
        return 0 if stop_event.is_set() else result
    except (OSError, subprocess.SubprocessError) as error:
        print(f'uv_record HTTP video recorder failed: {error}', file=sys.stderr)
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        return 1
    finally:
        mapping.close()


def _run_jpeg_archive(args) -> int:
    stop_event = threading.Event()
    _install_signals(stop_event)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    progress = {
        'status': 'waiting_for_video', 'frames': 0, 'bytes': 0,
        'unaligned_frames': 0, 'last_frame_received_unix_ns': 0,
        'alignment_status': 'unknown',
    }
    progress_lock = threading.Lock()
    mapping = FrameMappingSubscriber(args.camera, args.stream_mode)
    writer = JpegArchiveWriter(
        output_dir, start_number=args.start_number, fps=args.fps,
        chunk_seconds=args.segment_duration)
    process = None
    alignment_thread = None
    alignment_index = None
    write_error = None
    sequence = 0
    buffer = bytearray()
    pts_queue: queue.Queue[tuple[int | None, int | None]] = queue.Queue(maxsize=64)
    pending_pts = {}
    decoded_frame_index = 0
    sequence = max((entry.sequence for entry in archive_entries(output_dir)),
                   default=-1) + 1
    last_index_sync = time.monotonic()
    alignment_path = output_dir / 'frame_alignment.jsonl'
    try:
        with alignment_path.open(encoding='utf-8') as existing:
            for old_line in existing:
                try:
                    sequence = max(sequence, int(json.loads(old_line).get('sequence', -1)) + 1)
                except (ValueError, json.JSONDecodeError):
                    continue
    except OSError:
        pass
    exit_code = 0
    next_status = time.monotonic()

    def read_pts(stream):
        for line in iter(stream.readline, b''):
            if b'pts_time:' in line:
                try:
                    pts_queue.put(_parse_showinfo(line), timeout=0.5)
                except queue.Full:
                    pass

    def consume_jpegs(chunk):
        nonlocal sequence, decoded_frame_index, write_error, last_index_sync
        buffer.extend(chunk)
        while True:
            start = buffer.find(b'\xff\xd8')
            if start < 0:
                buffer[:] = buffer[-1:]
                break
            end = buffer.find(b'\xff\xd9', start + 2)
            if end < 0:
                if len(buffer) > 16 * 1024 * 1024:
                    del buffer[:start]
                break
            payload = bytes(buffer[start:end + 2])
            del buffer[:end + 2]
            pts_ns = _pts_for_frame(
                pts_queue, pending_pts, decoded_frame_index, timeout=1.0)
            metadata = _alignment_record(
                mapping, sequence, pts_ns, decoded_frame_index)
            decoded_frame_index += 1
            alignment_index.write(
                json.dumps(metadata, ensure_ascii=False) + '\n')
            alignment_index.flush()
            now = time.monotonic()
            if now - last_index_sync >= 1.0:
                os.fsync(alignment_index.fileno())
                last_index_sync = now
            source_ns = metadata['source_timestamp_ns']
            try:
                if writer.write(
                        payload,
                        timestamp_ns=source_ns if source_ns is not None else 0,
                        sequence=sequence,
                        metadata=metadata):
                    with progress_lock:
                        progress.update(
                            status='recording',
                            frames=progress['frames'] + 1,
                            bytes=progress['bytes'] + len(payload),
                            unaligned_frames=progress['unaligned_frames']
                            + int(not metadata['timestamp_aligned']),
                            last_frame_received_unix_ns=metadata[
                                'receive_time_unix_ns'],
                        )
                        if metadata['timestamp_aligned']:
                            progress['last_source_timestamp_ns'] = source_ns
                sequence += 1
            except (OSError, ValueError) as error:
                write_error = error
                print(f'uv_record JPEG archive failed: {error}', file=sys.stderr)
                stop_event.set()
                break

    try:
        alignment_index = alignment_path.open('a', encoding='utf-8', buffering=1)
        process = subprocess.Popen(
            _jpeg_command(args), stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            bufsize=0, start_new_session=True)
        assert process.stdout is not None and process.stderr is not None
        alignment_thread = threading.Thread(
            target=read_pts, args=(process.stderr,),
            name='uv-record-jpeg-pts', daemon=True)
        alignment_thread.start()
        while not stop_event.is_set():
            ready, _, _ = select.select([process.stdout], [], [], 0.25)
            now = time.monotonic()
            if now >= next_status:
                with progress_lock:
                    progress_copy = dict(progress)
                write_json_atomic(output_dir / 'status.json', progress_copy)
                next_status = now + 5.0
            if not ready:
                if process.poll() is not None:
                    exit_code = process.returncode or 1
                    break
                continue
            chunk = os.read(process.stdout.fileno(), 65536)
            if not chunk:
                if process.poll() is not None:
                    exit_code = process.returncode or 1
                break
            consume_jpegs(chunk)

        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            drain_deadline = time.monotonic() + 5.0
            while process.poll() is None and time.monotonic() < drain_deadline:
                ready, _, _ = select.select(
                    [process.stdout], [], [], min(0.1, drain_deadline - time.monotonic()))
                if not ready:
                    continue
                chunk = os.read(process.stdout.fileno(), 65536)
                if not chunk:
                    break
                consume_jpegs(chunk)
        try:
            process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
    except (OSError, subprocess.SubprocessError) as error:
        write_error = error
        print(f'uv_record HTTP video decoder failed: {error}', file=sys.stderr)
        exit_code = 1
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        if alignment_thread is not None and alignment_thread.is_alive():
            alignment_thread.join(timeout=1.0)
        if alignment_index is not None:
            alignment_index.flush()
            os.fsync(alignment_index.fileno())
            alignment_index.close()
        writer.close()
        with progress_lock:
            progress_copy = dict(progress)
        progress_copy.update({
            'status': (
                'stopped' if stop_event.is_set() and not write_error
                and progress_copy['frames'] > 0 else 'failed'),
            'alignment_status': (
                'aligned' if progress_copy['frames'] > 0
                and progress_copy['unaligned_frames'] == 0 else 'degraded'),
        })
        write_json_atomic(output_dir / 'status.json', progress_copy)
        mapping.close()

    if write_error:
        return 1
    if progress['frames'] == 0:
        return exit_code or 1
    return 0 if stop_event.is_set() else (exit_code or 1)


def main():
    args = _parse_args()
    if args.output_format == 'jpeg':
        return _run_jpeg_archive(args)
    return _run_ts(args)


if __name__ == '__main__':
    raise SystemExit(main())
