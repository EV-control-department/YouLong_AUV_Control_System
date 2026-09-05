"""Display front/down annotated MJPEG streams in positioned OpenCV windows."""

import argparse
import ctypes
import ctypes.util
import os
import re
import signal
import subprocess
import threading
import time
from queue import Empty, Full, Queue
from urllib.request import Request, urlopen

try:
    import cv2
    import numpy as np
except ImportError:  # pragma: no cover - only used on incomplete deployments
    cv2 = None
    np = None


class _XClientMessageData(ctypes.Union):
    """The data union used by an X11 ClientMessage event."""

    _fields_ = [
        ('b', ctypes.c_char * 20),
        ('s', ctypes.c_short * 10),
        ('l', ctypes.c_long * 5),
    ]


class _XClientMessageEvent(ctypes.Structure):
    """Subset of XClientMessageEvent needed for EWMH window placement."""

    _fields_ = [
        ('type', ctypes.c_int),
        ('serial', ctypes.c_ulong),
        ('send_event', ctypes.c_int),
        ('display', ctypes.c_void_p),
        ('window', ctypes.c_ulong),
        ('message_type', ctypes.c_ulong),
        ('format', ctypes.c_int),
        ('data', _XClientMessageData),
    ]


class _XEvent(ctypes.Union):
    """Storage large enough for an XEvent passed to XSendEvent."""

    _fields_ = [
        ('type', ctypes.c_int),
        ('xclient', _XClientMessageEvent),
        ('padding', ctypes.c_long * 24),
    ]


class _MjpegReader(threading.Thread):
    """Read one MJPEG stream and retain only its newest decoded frame."""

    def __init__(self, url: str):
        super().__init__(daemon=True)
        self.url = url
        self.stop_event = threading.Event()
        self.frames = Queue(maxsize=1)
        self.dimensions = None

    def _put_latest(self, frame):
        try:
            self.frames.get_nowait()
        except Empty:
            pass
        try:
            self.frames.put_nowait(frame)
        except Full:
            pass

    def run(self):
        while not self.stop_event.is_set():
            try:
                request = Request(
                    self.url,
                    headers={'User-Agent': 'uv-annotated-preview'},
                )
                with urlopen(request, timeout=3.0) as response:
                    buffer = b''
                    while not self.stop_event.is_set():
                        chunk = response.read(65536)
                        if not chunk:
                            break
                        buffer += chunk
                        while True:
                            start = buffer.find(b'\xff\xd8')
                            if start < 0:
                                buffer = buffer[-1:]
                                break
                            end = buffer.find(b'\xff\xd9', start + 2)
                            if end < 0:
                                if len(buffer) > 8 * 1024 * 1024:
                                    buffer = buffer[start:]
                                break
                            jpeg = buffer[start:end + 2]
                            buffer = buffer[end + 2:]
                            frame = cv2.imdecode(
                                np.frombuffer(jpeg, dtype=np.uint8),
                                cv2.IMREAD_COLOR,
                            )
                            if frame is not None and frame.size:
                                self.dimensions = (frame.shape[1], frame.shape[0])
                                self._put_latest(frame)
            except (OSError, ValueError):
                pass
            self.stop_event.wait(0.25)

    def latest(self):
        """Return the newest frame, or ``None`` when no new frame is ready."""
        frame = None
        while True:
            try:
                frame = self.frames.get_nowait()
            except Empty:
                return frame

    def stop(self):
        self.stop_event.set()


class _X11WindowController:
    """Move top-level X11 windows without requiring xdotool or wmctrl."""

    def __init__(self):
        self._lib = None
        self._display = None
        library = ctypes.util.find_library('X11') or 'libX11.so.6'
        try:
            self._lib = ctypes.CDLL(library)
            self._lib.XOpenDisplay.argtypes = [ctypes.c_char_p]
            self._lib.XOpenDisplay.restype = ctypes.c_void_p
            self._lib.XMoveWindow.argtypes = [
                ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_int]
            self._lib.XMoveResizeWindow.argtypes = [
                ctypes.c_void_p,
                ctypes.c_ulong,
                ctypes.c_int,
                ctypes.c_int,
                ctypes.c_uint,
                ctypes.c_uint,
            ]
            self._lib.XDefaultScreen.argtypes = [ctypes.c_void_p]
            self._lib.XDefaultScreen.restype = ctypes.c_int
            self._lib.XRootWindow.argtypes = [ctypes.c_void_p, ctypes.c_int]
            self._lib.XRootWindow.restype = ctypes.c_ulong
            self._lib.XInternAtom.argtypes = [
                ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
            self._lib.XInternAtom.restype = ctypes.c_ulong
            self._lib.XSendEvent.argtypes = [
                ctypes.c_void_p,
                ctypes.c_ulong,
                ctypes.c_int,
                ctypes.c_long,
                ctypes.POINTER(_XEvent),
            ]
            self._lib.XSendEvent.restype = ctypes.c_int
            self._lib.XFlush.argtypes = [ctypes.c_void_p]
            self._lib.XCloseDisplay.argtypes = [ctypes.c_void_p]
            self._display = self._lib.XOpenDisplay(
                os.environ.get('DISPLAY', '').encode() or None)
        except (OSError, AttributeError):
            self.close()

    def close(self):
        if self._lib is not None and self._display:
            self._lib.XCloseDisplay(self._display)
        self._display = None

    @staticmethod
    def _find_window(title):
        try:
            result = subprocess.run(
                ['xwininfo', '-root', '-tree'],
                check=True,
                capture_output=True,
                text=True,
                timeout=2.0,
            )
        except (OSError, subprocess.SubprocessError):
            return None
        for line in result.stdout.splitlines():
            match = re.match(r'\s*(0x[0-9a-fA-F]+)\s+"([^"]*)"', line)
            if match and match.group(2) == title:
                return int(match.group(1), 16)
        return None

    def _ewmh_move_resize(self, window, x, y, width=None, height=None):
        """Ask the window manager to place a managed top-level window."""
        if self._lib is None or not self._display:
            return

        atom = self._lib.XInternAtom(
            self._display, b'_NET_MOVERESIZE_WINDOW', 0)
        root = self._lib.XRootWindow(
            self._display, self._lib.XDefaultScreen(self._display))
        value_mask = 1 | 2  # x and y
        values = [int(x), int(y), 0, 0]
        if width is not None:
            value_mask |= 4
            values[2] = max(1, int(width))
        if height is not None:
            value_mask |= 8
            values[3] = max(1, int(height))

        event = _XEvent()
        event.xclient.type = 33  # ClientMessage
        event.xclient.send_event = 1
        event.xclient.display = self._display
        event.xclient.window = ctypes.c_ulong(window)
        event.xclient.message_type = atom
        event.xclient.format = 32
        event.xclient.data.l[:] = [value_mask << 8, *values]
        # SubstructureRedirectMask | SubstructureNotifyMask.
        self._lib.XSendEvent(
            self._display, root, 0, (1 << 20) | (1 << 19),
            ctypes.byref(event))

    def move(self, title, x, y):
        """Move a titled window and report whether it was found."""
        if self._lib is None or not self._display:
            return False
        window = self._find_window(title)
        if window is None:
            return False
        self._ewmh_move_resize(window, x, y)
        self._lib.XMoveWindow(
            self._display, ctypes.c_ulong(window), int(x), int(y))
        self._lib.XFlush(self._display)
        return True

    def move_resize(self, title, x, y, width, height):
        """Move and resize a titled window and report whether it was found."""
        if self._lib is None or not self._display:
            return False
        window = self._find_window(title)
        if window is None:
            return False
        self._ewmh_move_resize(window, x, y, width, height)
        self._lib.XMoveResizeWindow(
            self._display,
            ctypes.c_ulong(window),
            int(x),
            int(y),
            max(1, int(width)),
            max(1, int(height)),
        )
        self._lib.XFlush(self._display)
        return True


def _window_size(frame_size, args):
    """Calculate a right-column size that exactly matches the frame ratio."""
    frame_width, frame_height = frame_size
    ratio = frame_width / max(1, frame_height)
    right_column_width = max(1, args.monitor_width - args.sim_width)
    max_height = min(args.height, max(1, args.monitor_height // 2))
    width = min(args.width, right_column_width, args.monitor_width)
    width = min(width, max(1, int(max_height * ratio)))
    width = max(1, int(width))
    height = max(1, int(round(width / ratio)))
    return width, height


def _fit_frame(frame, size):
    """Return a frame rendered at the exact pixel size of its window."""
    target_width, target_height = size
    if frame.shape[1] == target_width and frame.shape[0] == target_height:
        return frame
    return cv2.resize(
        frame, (target_width, target_height), interpolation=cv2.INTER_AREA)


def _parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8090)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=360)
    parser.add_argument('--sim-width', type=int, default=1280)
    parser.add_argument('--monitor-x', type=int, default=0)
    parser.add_argument('--monitor-y', type=int, default=0)
    parser.add_argument('--monitor-width', type=int, default=1920)
    parser.add_argument('--monitor-height', type=int, default=1080)
    parser.add_argument('--sim-title', default='Stonefish Simulator')
    parser.add_argument('--wait-timeout', type=float, default=60.0)
    # launch_ros can append ROS arguments to a Node action. They are not
    # relevant to this standalone helper, so accept and ignore unknown args.
    return parser.parse_known_args()[0]


def main():
    """Position the simulator and display both annotated camera streams."""
    args = _parse_args()
    if cv2 is None or np is None:
        print('annotated_preview: OpenCV is not available', flush=True)
        return 0

    front_url = f'http://{args.host}:{args.port}/front_annotated'
    down_url = f'http://{args.host}:{args.port}/down_annotated'
    readers = [_MjpegReader(front_url), _MjpegReader(down_url)]
    controller = _X11WindowController()
    stop_requested = threading.Event()

    def request_stop(_signum, _frame):
        stop_requested.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        for reader in readers:
            reader.start()

        deadline = time.monotonic() + max(0.0, args.wait_timeout)
        while not stop_requested.is_set():
            # Stonefish may recreate/reconfigure its SDL window while the
            # scene is loading.  Re-issue the placement request until both
            # camera streams are ready instead of assuming the first request
            # was the final one.
            controller.move(args.sim_title, args.monitor_x, args.monitor_y)
            if all(reader.dimensions is not None for reader in readers):
                break
            if time.monotonic() >= deadline:
                print(
                    f'annotated_preview: timed out waiting for {front_url} '
                    f'and {down_url}',
                    flush=True,
                )
                return 0
            time.sleep(0.2)

        front_size = _window_size(readers[0].dimensions, args)
        down_size = _window_size(readers[1].dimensions, args)
        front_x = args.monitor_x + args.monitor_width - front_size[0]
        down_x = args.monitor_x + args.monitor_width - down_size[0]
        front_y = args.monitor_y
        down_y = front_y + front_size[1]

        flags = cv2.WINDOW_NORMAL
        flags |= getattr(cv2, 'WINDOW_FREERATIO', 0)
        cv2.namedWindow('front_annotated', flags)
        cv2.namedWindow('down_annotated', flags)
        cv2.resizeWindow('front_annotated', *front_size)
        cv2.resizeWindow('down_annotated', *down_size)
        cv2.moveWindow('front_annotated', front_x, front_y)
        cv2.moveWindow('down_annotated', down_x, down_y)
        controller.move_resize(
            'front_annotated', front_x, front_y, *front_size)
        controller.move_resize(
            'down_annotated', down_x, down_y, *down_size)
        # One final request handles the case where Stonefish applied its SDL
        # position after the first X11 request during startup.
        controller.move(args.sim_title, args.monitor_x, args.monitor_y)

        print(
            f'annotated_preview: monitor=({args.monitor_x},{args.monitor_y}) '
            f'{args.monitor_width}x{args.monitor_height}; '
            f'front={front_size[0]}x{front_size[1]} at '
            f'({front_x},{front_y}); down={down_size[0]}x{down_size[1]} '
            f'at ({down_x},{down_y})',
            flush=True,
        )

        latest = [None, None]
        while not stop_requested.is_set():
            for index, reader in enumerate(readers):
                frame = reader.latest()
                if frame is not None:
                    latest[index] = frame
            if latest[0] is not None:
                cv2.imshow('front_annotated', _fit_frame(latest[0], front_size))
            if latest[1] is not None:
                cv2.imshow('down_annotated', _fit_frame(latest[1], down_size))
            key = cv2.waitKey(20) & 0xFF
            if key in (27, ord('q')):
                stop_requested.set()
    finally:
        for reader in readers:
            reader.stop()
        for reader in readers:
            reader.join(timeout=2.0)
        if cv2 is not None:
            cv2.destroyAllWindows()
        controller.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
