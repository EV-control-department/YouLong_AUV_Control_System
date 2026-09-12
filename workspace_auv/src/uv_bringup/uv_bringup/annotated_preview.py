"""Display front/down annotated MJPEG streams in positioned OpenCV windows."""

from __future__ import annotations

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
        self.last_error = None
        self.connection_attempts = 0
        self.decoded_frames = 0

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
                self.connection_attempts += 1
                request = Request(
                    self.url,
                    headers={'User-Agent': 'uv-annotated-preview'},
                )
                with urlopen(request, timeout=3.0) as response:
                    buffer = b''
                    read_chunk = getattr(response, 'read1', response.read)
                    while not self.stop_event.is_set():
                        chunk = read_chunk(65536)
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
                                self.decoded_frames += 1
                                self.last_error = None
                                self._put_latest(frame)
            except (OSError, ValueError, AttributeError, cv2.error) as error:
                self.last_error = str(error)
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


# The simulator publishes a horizontally stitched stereo image.  This is
# only a bootstrap ratio; the first decoded frame replaces it with the exact
# source dimensions.  Keeping a fallback lets both GUI windows exist even
# when one HTTP stream is a little slower during startup.
DEFAULT_STITCHED_FRAME_SIZE = (1280, 480)


def _initial_window_size(args):
    """Return a usable size before either MJPEG stream has produced a frame."""
    right_column_width = max(1, args.monitor_width - args.sim_width)
    return (
        max(1, min(args.width, right_column_width, args.monitor_width)),
        max(1, min(args.height, max(1, args.monitor_height // 2))),
    )


def _waiting_frame(title, size):
    """Create a visible placeholder so a delayed camera has a window too."""
    width, height = size
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    cv2.putText(
        frame, title, (24, max(36, height // 2 - 8)),
        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (220, 220, 220), 2, cv2.LINE_AA)
    cv2.putText(
        frame, 'waiting for annotated stream...',
        (24, max(70, height // 2 + 32)),
        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (150, 150, 150), 1, cv2.LINE_AA)
    return frame


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

    cv2.setNumThreads(1)

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

        flags = cv2.WINDOW_NORMAL
        flags |= getattr(cv2, 'WINDOW_FREERATIO', 0)
        windows = {}
        latest = [None, None]
        placeholders = [None, None]
        dirty = [True, True]
        initial_size = _initial_window_size(args)
        gui_error_reported = set()

        def update_layout():
            """Create and position both windows before their first frame."""
            y = args.monitor_y
            for index, reader in enumerate(readers):
                title = ('front_annotated' if index == 0
                         else 'down_annotated')
                frame_size = reader.dimensions or DEFAULT_STITCHED_FRAME_SIZE
                size = (_window_size(frame_size, args)
                        if reader.dimensions is not None else initial_size)
                x = args.monitor_x + args.monitor_width - size[0]
                if title not in windows:
                    try:
                        cv2.namedWindow(title, flags)
                    except cv2.error as error:
                        # A failure in one HighGUI window must not prevent the
                        # other camera from being displayed. Retry this one on
                        # the next layout pass.
                        if title not in gui_error_reported:
                            print(
                                f'annotated_preview: cannot create {title}: '
                                f'{error}', flush=True)
                            gui_error_reported.add(title)
                        continue
                    windows[title] = (size, x, y)
                    print(
                        f'annotated_preview: {title}={size[0]}x{size[1]} '
                        f'at ({x},{y})', flush=True)
                else:
                    windows[title] = (size, x, y)
                try:
                    cv2.resizeWindow(title, *size)
                    cv2.moveWindow(title, x, y)
                    controller.move_resize(title, x, y, *size)
                except cv2.error as error:
                    if title not in gui_error_reported:
                        print(
                            f'annotated_preview: cannot position {title}: '
                            f'{error}', flush=True)
                        gui_error_reported.add(title)
                y += size[1]
            # One final request handles the case where Stonefish applied its
            # SDL position after the first X11 request during startup.
            controller.move(args.sim_title, args.monitor_x, args.monitor_y)

        # Do not wait for a camera frame before creating the windows.  The
        # camera readers reconnect independently while the GUI stays alive.
        update_layout()
        deadline = time.monotonic() + max(0.0, args.wait_timeout)
        timeout_reported = False
        layout_dimensions = tuple(reader.dimensions for reader in readers)
        next_sim_move = 0.0
        while not stop_requested.is_set():
            # Stonefish may recreate/reconfigure its SDL window while the
            # scene is loading. Re-issue the placement request until the
            # simulator and both preview windows settle.
            now = time.monotonic()
            if now >= next_sim_move:
                controller.move(args.sim_title, args.monitor_x, args.monitor_y)
                next_sim_move = now + 0.5
            # A stream may become available after the other one. Create its
            # window then and compact the layout so the two remain stacked.
            current_dimensions = tuple(reader.dimensions for reader in readers)
            if current_dimensions != layout_dimensions or len(windows) < len(readers):
                update_layout()
                layout_dimensions = current_dimensions
                dirty = [True, True]
            for index, reader in enumerate(readers):
                frame = reader.latest()
                if frame is not None:
                    latest[index] = frame
                    dirty[index] = True
            if not timeout_reported and time.monotonic() >= deadline:
                missing = [
                    ('front_annotated' if index == 0 else 'down_annotated')
                    for index, reader in enumerate(readers)
                    if reader.dimensions is None
                ]
                if missing:
                    status = [
                        {
                            'attempts': reader.connection_attempts,
                            'frames': reader.decoded_frames,
                            'error': reader.last_error,
                        }
                        for reader in readers
                    ]
                    print(
                        f'annotated_preview: still waiting for {missing}; '
                        f'status={status}',
                        flush=True,
                    )
                timeout_reported = True
            for index, reader in enumerate(readers):
                title = ('front_annotated' if index == 0
                         else 'down_annotated')
                if title not in windows:
                    continue
                size = windows[title][0]
                frame = latest[index]
                if frame is None:
                    if placeholders[index] is None or (
                            placeholders[index].shape[1],
                            placeholders[index].shape[0]) != size:
                        placeholders[index] = _waiting_frame(title, size)
                    frame = placeholders[index]
                if not dirty[index]:
                    continue
                try:
                    cv2.imshow(title, _fit_frame(frame, size))
                    dirty[index] = False
                except cv2.error as error:
                    if title not in gui_error_reported:
                        print(
                            f'annotated_preview: cannot update {title}: '
                            f'{error}', flush=True)
                        gui_error_reported.add(title)
            try:
                key = cv2.waitKey(50) & 0xFF
            except cv2.error as error:
                print(f'annotated_preview: HighGUI event loop failed: {error}',
                      flush=True)
                break
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
