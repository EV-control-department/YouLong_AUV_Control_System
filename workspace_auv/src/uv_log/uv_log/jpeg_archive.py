"""Crash-recoverable JPEG frame archives used by :mod:`uv_log`.

The camera HTTP endpoint already carries complete JPEG frames.  Re-encoding
those frames to H.264 while recording costs CPU and makes the encoder the
critical path.  This module stores each JPEG in a short, self-describing
chunk, with a small JSON-lines index beside it.  The frame header is the
authoritative metadata, so an index can always be rebuilt after a crash.
"""

from __future__ import annotations

import json
import os
import re
import struct
import time
import zlib
from dataclasses import dataclass
from pathlib import Path


MAGIC = b'UVJPG01\0'
FRAME_HEADER = struct.Struct('<8sQQII')
MAX_FRAME_BYTES = 128 * 1024 * 1024
CHUNK_PATTERN = re.compile(r'chunk_(\d+)\.mjpg$')


@dataclass(frozen=True)
class JpegFrameRef:
    """Location and metadata for one JPEG payload inside a chunk."""

    chunk: Path
    timestamp_ns: int
    sequence: int
    offset: int
    size: int
    crc32: int


def _chunk_number(path: Path) -> int:
    match = CHUNK_PATTERN.fullmatch(path.name)
    return int(match.group(1)) if match else -1


def next_chunk_number(directory: Path) -> int:
    numbers = [
        _chunk_number(path) for path in directory.glob('chunk_*.mjpg')
    ]
    return max(numbers, default=-1) + 1


def _fsync_directory(directory: Path) -> None:
    try:
        fd = os.open(directory, os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0))
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
    except OSError:
        pass


def _is_jpeg(payload: bytes) -> bool:
    return len(payload) >= 4 and payload[:2] == b'\xff\xd8' and payload[-2:] == b'\xff\xd9'


def _index_path(chunk: Path) -> Path:
    return chunk.with_suffix('.jsonl')


def _entry_json(ref: JpegFrameRef) -> str:
    return json.dumps({
        'chunk': ref.chunk.name,
        'timestamp_ns': ref.timestamp_ns,
        'sequence': ref.sequence,
        'offset': ref.offset,
        'size': ref.size,
        'crc32': ref.crc32,
    }, ensure_ascii=False, separators=(',', ':')) + '\n'


class JpegArchiveWriter:
    """Append JPEG frames to short, independently recoverable chunks."""

    def __init__(
        self,
        output_dir: str | Path,
        start_number: int = 0,
        fps: float = 10.0,
        chunk_seconds: float = 2.0,
    ):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.max_frames = max(1, int(round(max(1.0, float(fps)) *
                                          max(0.5, float(chunk_seconds)))))
        self.chunk_number = int(start_number)
        self.data = None
        self.index = None
        self.data_path: Path | None = None
        self.index_path: Path | None = None
        self.frames_in_chunk = 0
        self._open_chunk()

    def _open_chunk(self) -> None:
        self.data_path = self.output_dir / f'chunk_{self.chunk_number:06d}.mjpg'
        self.index_path = _index_path(self.data_path)
        # A restarted child always receives a fresh chunk number.  Append is
        # still safe if a caller intentionally resumes the same chunk.
        self.data = self.data_path.open('ab', buffering=0)
        self.index = self.index_path.open('a', encoding='utf-8', buffering=1)
        self.frames_in_chunk = 0

    @staticmethod
    def _sync_handle(handle) -> None:
        handle.flush()
        try:
            os.fsync(handle.fileno())
        except OSError:
            pass

    def _close_chunk(self) -> None:
        if self.data is None or self.index is None:
            return
        # Sync payload before its index.  If power is lost earlier, recovery
        # validates the self-describing records and rebuilds the index.
        self._sync_handle(self.data)
        self._sync_handle(self.index)
        data_path = self.data_path
        index_path = self.index_path
        self.data.close()
        self.index.close()
        self.data = None
        self.index = None
        if data_path is not None:
            _fsync_directory(data_path.parent)
        if (
            index_path is not None
            and index_path.parent != (data_path.parent if data_path else None)
        ):
            _fsync_directory(index_path.parent)

    def write(
        self,
        payload: bytes,
        timestamp_ns: int | None = None,
        sequence: int | None = None,
    ) -> bool:
        """Write one complete JPEG and return whether it was accepted."""
        if not _is_jpeg(payload) or len(payload) > MAX_FRAME_BYTES:
            return False
        if self.data is None or self.index is None:
            self._open_chunk()
        if self.frames_in_chunk >= self.max_frames:
            self._close_chunk()
            self.chunk_number += 1
            self._open_chunk()

        timestamp = int(timestamp_ns if timestamp_ns is not None else time.time_ns())
        frame_sequence = int(sequence if sequence is not None else self.frames_in_chunk)
        crc32 = zlib.crc32(payload) & 0xffffffff
        header = FRAME_HEADER.pack(
            MAGIC, timestamp, frame_sequence, len(payload), crc32)
        payload_offset = self.data.tell() + FRAME_HEADER.size
        self.data.write(header)
        self.data.write(payload)
        ref = JpegFrameRef(
            chunk=self.data_path,
            timestamp_ns=timestamp,
            sequence=frame_sequence,
            offset=payload_offset,
            size=len(payload),
            crc32=crc32,
        )
        self.index.write(_entry_json(ref))
        self.index.flush()
        self.frames_in_chunk += 1
        return True

    def close(self) -> None:
        self._close_chunk()


def scan_chunk(
    path: str | Path,
    start_offset: int = 0,
    verify_crc: bool = True,
) -> tuple[list[JpegFrameRef], int]:
    """Scan valid records and return ``(entries, valid_end_offset)``.

    A partial header, partial JPEG, bad marker, or bad CRC terminates the
    scan.  This is intentional: the damaged tail is the only part that can
    be incomplete after a sudden power loss.
    """
    path = Path(path)
    entries: list[JpegFrameRef] = []
    valid_end = int(start_offset)
    try:
        with path.open('rb') as handle:
            handle.seek(max(0, start_offset))
            while True:
                record_start = handle.tell()
                raw_header = handle.read(FRAME_HEADER.size)
                if not raw_header:
                    valid_end = record_start
                    break
                if len(raw_header) != FRAME_HEADER.size:
                    break
                magic, timestamp, sequence, size, crc32 = FRAME_HEADER.unpack(raw_header)
                if magic != MAGIC or size <= 0 or size > MAX_FRAME_BYTES:
                    break
                payload_offset = handle.tell()
                payload = handle.read(size)
                if len(payload) != size or not _is_jpeg(payload):
                    break
                if verify_crc and (zlib.crc32(payload) & 0xffffffff) != crc32:
                    break
                entries.append(JpegFrameRef(
                    chunk=path,
                    timestamp_ns=int(timestamp),
                    sequence=int(sequence),
                    offset=payload_offset,
                    size=int(size),
                    crc32=int(crc32),
                ))
                valid_end = handle.tell()
    except OSError:
        return [], 0
    return entries, valid_end


def _read_index(path: Path, chunk: Path) -> list[JpegFrameRef]:
    entries: list[JpegFrameRef] = []
    try:
        with path.open(encoding='utf-8') as handle:
            for line in handle:
                try:
                    item = json.loads(line)
                    offset = int(item['offset'])
                    size = int(item['size'])
                    timestamp = int(item['timestamp_ns'])
                    sequence = int(item['sequence'])
                    crc32 = int(item['crc32'])
                except (TypeError, ValueError, KeyError, json.JSONDecodeError):
                    continue
                if offset < FRAME_HEADER.size or size <= 0:
                    continue
                entries.append(JpegFrameRef(
                    chunk=chunk,
                    timestamp_ns=timestamp,
                    sequence=sequence,
                    offset=offset,
                    size=size,
                    crc32=crc32,
                ))
    except OSError:
        return []
    return entries


def load_chunk_entries(path: str | Path) -> list[JpegFrameRef]:
    """Load an index and recover any valid records missing from its tail."""
    chunk = Path(path)
    try:
        file_size = chunk.stat().st_size
    except OSError:
        return []
    entries = _read_index(_index_path(chunk), chunk)
    entries = [
        entry for entry in entries
        if entry.offset + entry.size <= file_size
    ]
    entries.sort(key=lambda entry: entry.offset)
    indexed_end = 0
    if entries:
        if entries[0].offset != FRAME_HEADER.size:
            entries = []
            indexed_end = 0
        else:
            indexed_end = entries[-1].offset + entries[-1].size
            indexed_end += FRAME_HEADER.size
        # An index pointing into an invalid or overlapping file is not usable.
        if any(
            current.offset < FRAME_HEADER.size
            or (previous is not None and current.offset < previous.offset + previous.size)
            for previous, current in zip([None] + entries[:-1], entries)
        ):
            entries = []
            indexed_end = 0
    if not entries or indexed_end > file_size:
        entries, _ = scan_chunk(chunk, verify_crc=True)
        return entries
    if indexed_end < file_size:
        tail, _ = scan_chunk(chunk, start_offset=indexed_end, verify_crc=True)
        entries.extend(tail)
    return entries


def archive_entries(directory: str | Path) -> list[JpegFrameRef]:
    directory = Path(directory)
    chunks = sorted(directory.glob('chunk_*.mjpg'), key=_chunk_number)
    entries: list[JpegFrameRef] = []
    for chunk in chunks:
        entries.extend(load_chunk_entries(chunk))
    return entries


def read_payload(ref: JpegFrameRef) -> bytes | None:
    try:
        with ref.chunk.open('rb') as handle:
            handle.seek(ref.offset)
            payload = handle.read(ref.size)
    except OSError:
        return None
    if len(payload) != ref.size or not _is_jpeg(payload):
        return None
    if (zlib.crc32(payload) & 0xffffffff) != ref.crc32:
        return None
    return payload


def _write_index(path: Path, entries: list[JpegFrameRef]) -> None:
    temporary = path.with_name(f'.{path.name}.{os.getpid()}.tmp')
    try:
        with temporary.open('w', encoding='utf-8') as handle:
            for entry in entries:
                handle.write(_entry_json(entry))
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        _fsync_directory(path.parent)
    except BaseException:
        try:
            temporary.unlink()
        except OSError:
            pass
        raise


def recover_directory(directory: str | Path) -> int:
    """Truncate damaged tails and rebuild indexes for a JPEG stream."""
    directory = Path(directory)
    recovered = 0
    for chunk in sorted(directory.glob('chunk_*.mjpg'), key=_chunk_number):
        entries, valid_end = scan_chunk(chunk, verify_crc=True)
        try:
            file_size = chunk.stat().st_size
        except OSError:
            continue
        if valid_end < file_size:
            try:
                with chunk.open('r+b') as handle:
                    handle.truncate(valid_end)
                    handle.flush()
                    os.fsync(handle.fileno())
            except OSError:
                continue
        _write_index(_index_path(chunk), entries)
        recovered += 1
    if recovered:
        _fsync_directory(directory)
    return recovered
