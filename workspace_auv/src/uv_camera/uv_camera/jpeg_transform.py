"""Lossless JPEG rotation through the stable TurboJPEG 2.x C API."""

import ctypes as ct
from ctypes.util import find_library


class _Region(ct.Structure):
    _fields_ = [(name, ct.c_int) for name in ('x', 'y', 'w', 'h')]


class _Transform(ct.Structure):
    _fields_ = [
        ('r', _Region), ('op', ct.c_int), ('options', ct.c_int),
        ('data', ct.c_void_p), ('custom_filter', ct.c_void_p),
    ]


class JpegRotator:
    """One transform handle, owned and reused by one camera worker."""

    def __init__(self):
        try:
            self._lib = ct.CDLL(find_library('turbojpeg') or 'libturbojpeg.so.0')
        except OSError as error:
            raise RuntimeError(
                'JPEG rotation requires libturbojpeg; install libturbojpeg0-dev') from error
        byte_pointer = ct.POINTER(ct.c_ubyte)
        self._lib.tjInitTransform.argtypes = []
        self._lib.tjInitTransform.restype = ct.c_void_p
        self._lib.tjTransform.argtypes = [
            ct.c_void_p, byte_pointer, ct.c_ulong, ct.c_int,
            ct.POINTER(byte_pointer), ct.POINTER(ct.c_ulong),
            ct.POINTER(_Transform), ct.c_int,
        ]
        self._lib.tjTransform.restype = ct.c_int
        self._lib.tjGetErrorStr2.argtypes = [ct.c_void_p]
        self._lib.tjGetErrorStr2.restype = ct.c_char_p
        self._lib.tjFree.argtypes = [ct.c_void_p]
        self._lib.tjFree.restype = None
        self._lib.tjDestroy.argtypes = [ct.c_void_p]
        self._lib.tjDestroy.restype = ct.c_int
        self._handle = self._lib.tjInitTransform()
        if not self._handle:
            raise RuntimeError('could not create TurboJPEG transform handle')

    def rotate_180(self, payload: bytes) -> bytes:
        """Rotate DCT coefficients; reject partial MCUs rather than crop."""
        if not self._handle:
            raise RuntimeError('JPEG rotator is closed')
        source = (ct.c_ubyte * len(payload)).from_buffer_copy(payload)
        destination = ct.POINTER(ct.c_ubyte)()
        size = ct.c_ulong(0)
        # ROT180=6, PERFECT=1, COPYNONE=64. Drop EXIF orientation markers so
        # independent viewers cannot apply an additional orientation change.
        transform = _Transform(op=6, options=1 | 64)
        try:
            result = self._lib.tjTransform(
                self._handle, source, len(payload), 1,
                ct.byref(destination), ct.byref(size), ct.byref(transform),
                8192)  # TJFLAG_STOPONWARNING
            if result != 0:
                detail = self._lib.tjGetErrorStr2(self._handle)
                raise ValueError('lossless JPEG rotation failed: {}'.format(
                    detail.decode('utf-8', errors='replace') if detail else 'unknown error'))
            return ct.string_at(destination, size.value)
        finally:
            if destination:
                self._lib.tjFree(destination)

    def close(self):
        if self._handle:
            self._lib.tjDestroy(self._handle)
            self._handle = None

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        self.close()
