"""Composed uv_camera node: uv_sensor + uv_ai in ONE process (A3).

One rclpy Node hosts both:
  * uv_sensor (sensor.Sensor) — selects the frame source (Stonefish sim via ROS
    stitched topics, or real V4L2 camera), updates the raw MJPEG go2rtc preview,
    and hands each BGR frame to uv_ai through an in-memory FrameGate.
  * uv_ai (ai.Ai) — YOLO detection + draw + publish /perception/detection/*,
    /perception/line/*, /perception/aruco/ids, and the annotated MJPEG cache.

No ROS sensor_msgs/Image crosses sensor->ai, and no JPEG is encoded between
them (A3): frames are BGR numpy arrays handed over the FrameGate. go2rtc is
used only for the outward preview (:1984). position runs as a separate
executable in this package and consumes /perception/detection/*.
"""

import os
import socket
import subprocess
import threading
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray, Header

from . import ai as ai_mod
from . import sensor as sensor_mod
from .common import (
    ENABLE_GORTC,
    GORTC_EXECUTABLE,
    GORTC_HTTP_PORT,
    STREAM_ANNOTATED,
    VISION_MJPEG_PORT,
    _MjpegHandler,
    _MjpegServer,
    FrameGate,
    image_msg_to_bgr,
)


def _as_bool(value) -> bool:
    """Parse launch-provided booleans without treating 'false' as true."""
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


class CameraAiNode(Node):
    """uv_camera node: composes uv_sensor (source+preview) + uv_ai (detection)."""

    def __init__(self):
        super().__init__('uv_camera')
        self._declare_params()
        params = self._read_params()

        # stream caches (go2rtc preview: raw + annotated)
        self._preview_enabled = params['enable_gortc']
        self._stream_annotated = (
            params['stream_annotated'] and self._preview_enabled)
        self._mjpeg_port = params['mjpeg_port']
        self._stream_lock = threading.Lock()
        self._stream_clients = {}
        self._annotated_max_width = params['annotated_max_width']
        self._stream_jpegs = {'front': None, 'down': None}
        self._stream_annotated_jpegs = {'front': None, 'down': None}
        self._stream_sequence = {'front': 0, 'down': 0}
        self._stream_annotated_sequence = {'front': 0, 'down': 0}
        self._stream_stamps = {'front': 0, 'down': 0}
        self._stream_annotated_stamps = {'front': 0, 'down': 0}
        self._stream_condition = threading.Condition(self._stream_lock)
        self._stream_stop = threading.Event()
        self._mjpeg_server = None
        self._gortc_process = None
        self._gortc_port = None
        self._gortc_config = None

        # JPEG/undistort work is small and frequent; prevent OpenCV from
        # creating another large worker pool alongside PyTorch and Stonefish.
        try:
            cv2.setNumThreads(1)
        except (AttributeError, cv2.error):
            pass

        # Latest ZIT6 pose used for the small telemetry overlay on all streams.
        # /zit6/state/pos has no header, so the newest received sample is the
        # closest available pose for the frame being encoded.
        self._pose_lock = threading.Lock()
        self._pose_overlay = None
        self._pose_sub = self.create_subscription(
            Float32MultiArray, '/zit6/state/pos', self._pose_cb, 10)

        # determine active cameras
        active = []
        if params['enable_front']:
            active.append('front')
        if params['enable_down']:
            active.append('down')

        # uv_ai (must be built before gate so gate.consumer is ready)
        self.ai = ai_mod.Ai(
            self, self.update_annotated_stream, cameras=active,
            inference_fps=params['inference_fps'],
            inference_threads=params['inference_threads'],
            gate_feature_mode=params['gate_feature_mode'],
            confidence=params['confidence'])
        self._gate = FrameGate(self.ai.process, cameras=active,
                               max_workers=2, log_warn=self._warn)
        self.ai.load_model(params['model_path'])

        # uv_sensor
        self.sensor = sensor_mod.Sensor(
            self, self._gate, params['sim_mode'],
            params['enable_front'], params['enable_down'])
        self.sensor.start()

        # MJPEG server + go2rtc (preview only)
        gortc_started = False
        if self._preview_enabled:
            if self._start_mjpeg_server():
                gortc_started = self._start_gortc(params['gortc_port'])

        preview_text = (
            f'preview enabled on {self._gortc_port}'
            if gortc_started else (
                f'MJPEG source on {params["mjpeg_port"]}; '
                'go2rtc unavailable' if self._preview_enabled
                else 'preview disabled'))
        self.get_logger().info(
            f'uv_camera started: sensor(source={params["sim_mode"] and "sim" or "v4l"})'
            f' + ai, {preview_text}')

    # ── params ──────────────────────────────────────────────────────────
    def _declare_params(self):
        self.declare_parameter('sim_mode', False)
        self.declare_parameter('inference_fps', 5.0)
        self.declare_parameter('inference_threads', 2)
        self.declare_parameter('confidence', 0.8)
        self.declare_parameter('gate_feature_mode', 'auto')
        self.declare_parameter('enable_gortc', ENABLE_GORTC)
        self.declare_parameter('gortc_executable', GORTC_EXECUTABLE)
        self.declare_parameter('gortc_http_port', GORTC_HTTP_PORT)
        self.declare_parameter('mjpeg_port', VISION_MJPEG_PORT)
        self.declare_parameter('stream_annotated', STREAM_ANNOTATED)
        self.declare_parameter('annotated_max_width', 0)
        self.declare_parameter('enable_front_camera', True)
        self.declare_parameter('enable_down_camera', True)
        self.declare_parameter('front_cam_path', '/dev/video2')
        self.declare_parameter('down_cam_path', '/dev/video0')
        self.declare_parameter('line_contour_min_area', 200)
        self.declare_parameter('line_filter_process_noise', 1.0)
        self.declare_parameter('line_filter_measurement_noise', 3.0)
        self.declare_parameter('save_dataset', False)
        self.declare_parameter('model_path', '')
        # calibration (defaults come from common constants)
        from .common import (FRONT_CAMERA_MATRIX, FRONT_DIST_COEFFS,
                             DOWN_CAMERA_MATRIX, DOWN_DIST_COEFFS)
        self.declare_parameter('front_camera_matrix', list(FRONT_CAMERA_MATRIX))
        self.declare_parameter('front_dist_coeffs', list(FRONT_DIST_COEFFS))
        self.declare_parameter('down_camera_matrix', list(DOWN_CAMERA_MATRIX))
        self.declare_parameter('down_dist_coeffs', list(DOWN_DIST_COEFFS))

    def _read_params(self):
        g = self.get_parameter
        return {
            'sim_mode': g('sim_mode').value,
            'inference_fps': max(0.0, float(g('inference_fps').value)),
            'inference_threads': max(1, int(g('inference_threads').value)),
            'confidence': min(1.0, max(0.05, float(g('confidence').value))),
            'gate_feature_mode': str(g('gate_feature_mode').value).strip().lower(),
            'enable_gortc': _as_bool(g('enable_gortc').value),
            'gortc_port': g('gortc_http_port').value,
            'stream_annotated': _as_bool(g('stream_annotated').value),
            'annotated_max_width': max(0, int(g('annotated_max_width').value)),
            'mjpeg_port': g('mjpeg_port').value,
            'enable_front': g('enable_front_camera').value,
            'enable_down': g('enable_down_camera').value,
            'model_path': g('model_path').value,
        }

    def _warn(self, msg):
        self.get_logger().warn(msg)

    def _rclpy_ok(self):
        return rclpy.ok()

    def _pose_cb(self, msg):
        """Cache the latest ZIT6 position for video telemetry.

        Supported formats are the current 4-element [x, y, z, yaw_rad]
        format and the 6-element [x, y, z, roll_rad, pitch_rad, yaw_rad]
        format used by newer ZIT6 firmware.
        """
        if len(msg.data) >= 6:
            values = (msg.data[0], msg.data[1], msg.data[2], msg.data[5])
        elif len(msg.data) >= 4:
            values = (msg.data[0], msg.data[1], msg.data[2], msg.data[3])
        else:
            return
        if not all(math.isfinite(float(value)) for value in values):
            return
        with self._pose_lock:
            self._pose_overlay = tuple(float(value) for value in values)

    def _draw_pose_overlay(self, frame):
        """Render the latest x/y/z/yaw in the lower-right corner."""
        overlay = frame.copy()
        with self._pose_lock:
            pose = self._pose_overlay

        if pose is None:
            text = 'POS unavailable'
        else:
            x, y, z, yaw_rad = pose
            text = (f'x={x:.2f} y={y:.2f} z={z:.2f} '
                    f'yaw={math.degrees(yaw_rad):.1f} deg')

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.45
        thickness = 1
        margin = 12
        (text_width, text_height), baseline = cv2.getTextSize(
            text, font, font_scale, thickness)
        x0 = max(margin, overlay.shape[1] - text_width - margin)
        y0 = overlay.shape[0] - margin

        # A compact black backing keeps the small white text readable over any
        # underwater image without changing the rest of the video.
        cv2.rectangle(
            overlay,
            (x0 - 6, y0 - text_height - baseline - 6),
            (min(overlay.shape[1] - 1, x0 + text_width + 6), y0 + 4),
            (0, 0, 0), -1)
        cv2.putText(overlay, text, (x0, y0), font, font_scale,
                    (255, 255, 255), thickness, cv2.LINE_AA)
        return overlay

    # ── sensor connector: ROS->BGR gate submission + raw preview ────────
    def submit_image(self, camera, msg, right_stamp=None, stereo_pair_id=0):
        """Called from uv_sensor when a ROS Image arrives (sim mode).

        Decode to BGR, update the raw preview cache (same as the V4L2 path),
        then hand the frame to uv_ai through the in-memory gate.  Simulator
        stereo metadata keeps the two eye detections tied to their original
        capture stamps without changing the stitched image transport.
        """
        try:
            cv_img = image_msg_to_bgr(msg)
        except Exception as e:
            self.get_logger().warn(f'Image conversion failed ({camera}): {e}')
            return
        # raw preview updated at arrival (independent of YOLO speed)
        self.update_raw_preview(camera, cv_img, msg.header.stamp)
        # Keep the annotated endpoint usable while the first YOLO inference is
        # still warming up. It starts as a pose-overlay-only frame and is
        # replaced by the real annotated frame as soon as AI finishes. This
        # prevents the preview window from waiting on model startup forever.
        if self.stream_requested(camera, True):
            with self._stream_lock:
                annotated_ready = (
                    self._stream_annotated_jpegs[camera] is not None)
            if not annotated_ready:
                self.update_annotated_stream(
                    camera, cv_img, msg.header.stamp)
        self._gate.submit(
            camera,
            ('opencv', cv_img, msg.header.stamp, False,
             right_stamp, int(stereo_pair_id or 0)),
        )

    def submit_frame(self, camera, frame, stamp=None):
        """Called from uv_sensor when a V4L2 frame arrives (real mode)."""
        stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
        if self.stream_requested(camera, True):
            with self._stream_lock:
                annotated_ready = (
                    self._stream_annotated_jpegs[camera] is not None)
            if not annotated_ready:
                self.update_annotated_stream(camera, frame, stamp)
        self._gate.submit(camera, ('opencv', frame, stamp, True, None, 0))

    @staticmethod
    def _stamp_to_ns(stamp):
        if stamp is None:
            return time.time_ns()
        try:
            return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        except (AttributeError, TypeError, ValueError):
            return time.time_ns()

    def stream_requested(self, camera, annotated=False):
        """Only encode streams consumed by a preview or recorder connection."""
        if not self._preview_enabled or (annotated and not self._stream_annotated):
            return False
        with self._stream_lock:
            return self._stream_clients.get((camera, annotated), 0) > 0

    def update_raw_preview(self, camera, frame, stamp=None):
        if not self.stream_requested(camera):
            return
        frame = self._draw_pose_overlay(frame)
        ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            return
        with self._stream_condition:
            self._stream_jpegs[camera] = encoded.tobytes()
            self._stream_sequence[camera] += 1
            self._stream_stamps[camera] = self._stamp_to_ns(stamp)
            self._stream_condition.notify_all()

    def update_annotated_stream(self, camera, frame, stamp=None):
        if not self.stream_requested(camera, True):
            return
        width = self._annotated_max_width
        if width and frame.shape[1] > width:
            height = max(1, round(frame.shape[0] * width / frame.shape[1]))
            frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
        frame = self._draw_pose_overlay(frame)
        ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            return
        with self._stream_condition:
            self._stream_annotated_jpegs[camera] = encoded.tobytes()
            self._stream_annotated_sequence[camera] += 1
            self._stream_annotated_stamps[camera] = self._stamp_to_ns(stamp)
            self._stream_condition.notify_all()

    def _wait_for_stream_frame(self, camera, annotated, last_sequence):
        seq_cache = (self._stream_annotated_sequence if annotated
                     else self._stream_sequence)
        jpeg_cache = (self._stream_annotated_jpegs if annotated
                      else self._stream_jpegs)
        stamp_cache = (self._stream_annotated_stamps if annotated
                       else self._stream_stamps)
        with self._stream_condition:
            while not self._stream_stop.is_set():
                seq = seq_cache[camera]
                payload = jpeg_cache[camera]
                if payload is not None and seq != last_sequence:
                    return payload, seq, stamp_cache[camera]
                self._stream_condition.wait(timeout=0.5)
        return None

    # ── MJPEG server + go2rtc (preview) ─────────────────────────────────
    def _start_mjpeg_server(self):
        try:
            _MjpegHandler.node = self
            self._mjpeg_server = _MjpegServer(('0.0.0.0', self._mjpeg_port),
                                              _MjpegHandler)
            threading.Thread(target=self._mjpeg_server.serve_forever,
                             name='uv-camera-mjpeg', daemon=True).start()
            self.get_logger().info(
                f'uv_camera MJPEG server on 0.0.0.0:{self._mjpeg_port}; '
                f'raw/annotated endpoints: /front, /down, '
                f'/front_annotated, /down_annotated')
            return True
        except OSError as e:
            self._mjpeg_server = None
            _MjpegHandler.node = None
            self.get_logger().error(
                f'Cannot open MJPEG port {self._mjpeg_port}: {e}')
            return False

    def _start_gortc(self, port):
        executable = self._find_gortc()
        if executable is None:
            self.get_logger().warn(
                'go2rtc not found; local MJPEG remains available')
            return False
        try:
            requested_port = int(port)
        except (TypeError, ValueError) as e:
            self.get_logger().error(f'Invalid go2rtc HTTP port {port!r}: {e}')
            return False
        if not 1 <= requested_port <= 65535:
            self.get_logger().error(
                f'Invalid go2rtc HTTP port {requested_port}; expected 1..65535')
            return False

        # A desktop port forward (for example VS Code's forwarded-port
        # helper) may already own 1984. Keep the requested port when possible;
        # otherwise use the first nearby free port and report it clearly.
        gortc_port = requested_port
        if not self._tcp_port_available(gortc_port):
            for candidate in range(requested_port + 1,
                                   min(65535, requested_port + 101)):
                if self._tcp_port_available(candidate):
                    gortc_port = candidate
                    break
            else:
                self.get_logger().error(
                    f'No free go2rtc HTTP port near {requested_port}')
                return False
            self.get_logger().warn(
                f'go2rtc HTTP port {requested_port} is occupied; '
                f'using {gortc_port}')
        self._gortc_port = gortc_port
        self._gortc_config = os.path.join(
            '/tmp', f'uv_camera_go2rtc_{os.getpid()}.yaml')
        # Bind the HTTP API on every interface so it is reachable from the
        # host/another machine when uv_camera runs inside Docker.  The
        # container still needs to publish this port (or use host networking).
        cfg = ('api:\n'
               f'  listen: "0.0.0.0:{gortc_port}"\n'
               'streams:\n'
               f'  front: "http://127.0.0.1:{self._mjpeg_port}/front"\n'
               f'  down: "http://127.0.0.1:{self._mjpeg_port}/down"\n')
        if self._stream_annotated:
            cfg += (f'  front_annotated: "http://127.0.0.1:{self._mjpeg_port}/front_annotated"\n'
                    f'  down_annotated: "http://127.0.0.1:{self._mjpeg_port}/down_annotated"\n')
        try:
            with open(self._gortc_config, 'w', encoding='utf-8') as f:
                f.write(cfg)
            self._gortc_process = subprocess.Popen(
                [executable, '-config', self._gortc_config],
                stdin=subprocess.DEVNULL, stdout=None, stderr=None)
            # Popen succeeds even when go2rtc immediately exits because a
            # listener failed. Catch that race before claiming it started.
            time.sleep(0.25)
            returncode = self._gortc_process.poll()
            if returncode is not None:
                self.get_logger().error(
                    f'go2rtc exited during startup (code {returncode})')
                self._gortc_process = None
                return False
            self.get_logger().info(
                f'go2rtc started on {gortc_port}; streams: front, down'
                + (', front_annotated, down_annotated' if self._stream_annotated else ''))
            return True
        except (OSError, ValueError, RuntimeError) as e:
            self.get_logger().error(f'Failed to start go2rtc: {e}')
            return False

    @staticmethod
    def _tcp_port_available(port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(('0.0.0.0', int(port)))
            return True
        except OSError:
            return False
        finally:
            sock.close()

    def _find_gortc(self):
        import shutil
        configured = self.get_parameter('gortc_executable').value.strip()
        candidates = [os.environ.get('GO2RTC_BIN'), configured]
        try:
            from ament_index_python.packages import get_package_share_directory
            candidates.append(os.path.join(
                get_package_share_directory('uv_camera'), 'bin', 'go2rtc'))
        except Exception:
            pass
        # Look in the checkout as well.  The absolute path below is useful on
        # the development host, but it does not exist when this package runs
        # from the /workspace mount inside Docker.
        module_path = os.path.realpath(__file__)
        repo_paths = []
        path = module_path
        for _ in range(7):
            path = os.path.dirname(path)
            repo_paths.append(os.path.join(path, 'third_party', 'go2rtc', 'go2rtc'))
        repo_paths = [
            '/workspace/third_party/go2rtc/go2rtc',
            '/home/doc049/dev/UUV/YouLong_AUV_Control_System/third_party/go2rtc/go2rtc',
            os.path.expanduser('~/YouLong_AUV_Control_System/third_party/go2rtc/go2rtc'),
        ] + repo_paths
        repo_paths.append(os.path.join(os.getcwd(), 'third_party', 'go2rtc', 'go2rtc'))
        candidates.extend(repo_paths)
        candidates.append('go2rtc')
        for c in candidates:
            if c:
                resolved = shutil.which(c) or (c if os.path.isfile(c) else None)
                if resolved and os.access(resolved, os.X_OK):
                    return resolved
        return None

    # ── shutdown ────────────────────────────────────────────────────────
    def destroy_node(self):
        self._stream_stop.set()
        with self._stream_condition:
            self._stream_condition.notify_all()
        _MjpegHandler.node = None
        if self._mjpeg_server is not None:
            self._mjpeg_server.shutdown()
            self._mjpeg_server.server_close()
        self.sensor.shutdown()
        self.ai.shutdown()
        self._gate.shutdown()
        if self._gortc_process is not None and self._gortc_process.poll() is None:
            self._gortc_process.terminate()
            try:
                self._gortc_process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._gortc_process.kill()
        if self._gortc_config:
            try:
                os.unlink(self._gortc_config)
            except OSError:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraAiNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        # launch may already have shut down the default context while
        # delivering SIGINT.
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
