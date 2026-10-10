"""Inspect a JPEG header without decoding its pixels."""

from __future__ import annotations


_SOF_MARKERS = frozenset((
    0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7,
    0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF,
))


def jpeg_dimensions(payload: bytes) -> tuple[int, int]:
    """Return (width, height), rejecting incomplete or malformed headers."""
    if len(payload) < 4 or payload[:2] != b'\xff\xd8' or payload[-2:] != b'\xff\xd9':
        raise ValueError('incomplete JPEG: expected SOI and EOI markers')
    position = 2
    dimensions = None
    while position < len(payload) - 2:
        if payload[position] != 0xFF:
            raise ValueError('invalid JPEG marker')
        while position < len(payload) and payload[position] == 0xFF:
            position += 1
        if position >= len(payload):
            break
        marker = payload[position]
        position += 1
        if marker in (0x00, 0xD8, 0xD9) or 0xD0 <= marker <= 0xD7:
            raise ValueError('unexpected JPEG marker before scan')
        if marker == 0x01:
            continue
        if position + 2 > len(payload) - 2:
            raise ValueError('truncated JPEG segment length')
        size = int.from_bytes(payload[position:position + 2], 'big')
        end = position + size
        if size < 2 or end > len(payload) - 2:
            raise ValueError('truncated JPEG segment')
        if marker in _SOF_MARKERS:
            if size < 8:
                raise ValueError('short JPEG frame header')
            height = int.from_bytes(payload[position + 3:position + 5], 'big')
            width = int.from_bytes(payload[position + 5:position + 7], 'big')
            components = payload[position + 7]
            if width <= 0 or height <= 0 or components == 0 or size != 8 + 3 * components:
                raise ValueError('invalid JPEG frame dimensions or components')
            dimensions = width, height
        if marker == 0xDA:
            if dimensions is None or size < 6 or end >= len(payload) - 2:
                raise ValueError('invalid JPEG scan header')
            return dimensions
        position = end
    raise ValueError('JPEG has no complete frame and scan headers')
