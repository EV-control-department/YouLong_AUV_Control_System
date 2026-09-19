"""Composed uv_camera node: uv_sensor + uv_ai in ONE process (A3).

One rclpy Node hosts both:
  * uv_sensor (sensor.Sensor) — selects the frame source (Stonefish sim via
    POSIX shared-memory rings, or real V4L2 camera), updates the raw MJPEG
    go2rtc preview,
    and hands each BGR frame to uv_ai through an in-memory FrameGate.
  * uv_ai (ai.Ai) — YOLO detection + draw + publish /auv/perception/* and the
    annotated MJPEG cache.

No ROS sensor_msgs/Image crosses sensor->ai, and no JPEG is encoded between
them (A3): frames are BGR numpy arrays handed over the FrameGate. go2rtc is
used only for the outward preview (:1984). position runs as a separate
executable in this package and consumes /auv/perception/detections/*.
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

from std_msgs.msg import Header
from auv_protocol.topics import STATE_ODOM

from . import ai as ai_mod
from . import sensor as sensor_mod
from .camera_config import load_camera_config, profile_for_mode
from uv_msgs.msg import PoseInfo

from .common import (
    DATASET_DIR,
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
        camera_profile = profile_for_mode(
            params['sim_mode'], params['camera_config_profile'])
        config_dir = params['camera_config_dir'] or None
        camera_configs = {
            camera: load_camera_config(camera, camera_profile, config_dir)
            for camera in ('front', 'down')
        }
        self.camera_configs = camera_configs

        # stream caches (go2rtc preview: raw + annotated)
        self._preview_enabled = params['enable_gortc']
        self._stream_annotated = (
            params['stream_annotated'] and self._preview_enabled)
        self._stream_pose_overlay = params['stream_pose_overlay']
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
        # Hardware position has no header, so the newest received sample is the
        # closest available pose for the frame being encoded.
        self._pose_lock = threading.Lock()
        self._pose_overlay = None
        self._pose_sub = self.create_subscription(
            PoseInfo, STATE_ODOM, self._pose_cb, 10)

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
            dataset_fps=params['dataset_fps'],
            dataset_debug=params['dataset_debug'],
            dataset_debug_period_s=params['dataset_debug_period_s'],
            dataset_submit_timeout_s=params['dataset_submit_timeout_s'],
            dataset_writer_workers=params['dataset_writer_workers'],
            dataset_webp_method=params['dataset_webp_method'],
            dataset_fsync_each_file=params['dataset_fsync_each_file'],
            inference_threads=params['inference_threads'],
            gate_feature_mode=params['gate_feature_mode'],
            confidence=params['confidence'], camera_configs=camera_configs)
        self._gate = FrameGate(self.ai.process, cameras=active,
                               max_workers=2, log_warn=self._warn)
        if params['enable_ai']:
            self.ai.load_model(params['model_path'])
        else:
            self.get_logger().info(
                'YOLO detection disabled; running sensor/preview/dataset '
                'recording only')

        # uv_sensor
        self.sensor = sensor_mod.Sensor(
            self, self._gate, params['sim_mode'],
            params['enable_front'], params['enable_down'],
            startup_timeout_s=params['camera_startup_timeout_s'],
            camera_configs=camera_configs)
        try:
            self.sensor.start()
        except Exception as error:
            self.get_logger().error(
                f'Camera startup failed; recording not started: {error}')
            self.sensor.shutdown()
            self.ai.fail_dataset_recording(f'camera startup failed: {error}')
            self.ai.shutdown()
            raise

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
            f' + {"ai" if params["enable_ai"] else "recording-only"}, '
            f'{preview_text}')

    # ── params ──────────────────────────────────────────────────────────
    def _declare_params(self):
        self.declare_parameter('sim_mode', False)
        self.declare_parameter('enable_ai', True)
        self.declare_parameter('inference_fps', 5.0)
        self.declare_parameter('dataset_fps', 5.0)
        self.declare_parameter('dataset_debug', False)
        self.declare_parameter('dataset_debug_period_sec', 1.0)
        self.declare_parameter('dataset_submit_timeout_sec', 1.0)
        self.declare_parameter('dataset_writer_workers', 4)
        self.declare_parameter('dataset_webp_method', 0)
        self.declare_parameter('dataset_fsync_each_file', False)
        self.declare_parameter('camera_startup_timeout_sec', 5.0)
        self.declare_parameter('inference_threads', 2)
        self.declare_parameter('confidence', 0.8)
        self.declare_parameter('gate_feature_mode', 'auto')
        self.declare_parameter('enable_gortc', ENABLE_GORTC)
        self.declare_parameter('gortc_executable', GORTC_EXECUTABLE)
        self.declare_parameter('gortc_http_port', GORTC_HTTP_PORT)
        self.declare_parameter('mjpeg_port', VISION_MJPEG_PORT)
        self.declare_parameter('stream_annotated', STREAM_ANNOTATED)
        self.declare_parameter('stream_pose_overlay', False)
        self.declare_parameter('annotated_max_width', 0)
        self.declare_parameter('enable_front_camera', True)
        self.declare_parameter('enable_down_camera', True)
        self.declare_parameter('camera_config_profile', 'auto')
        self.declare_parameter('camera_config_dir', '')
        self.declare_parameter('line_contour_min_area', 200)
        self.declare_parameter('line_filter_process_noise', 1.0)
        self.declare_parameter('line_filter_measurement_noise', 3.0)
        self.declare_parameter('save_dataset', False)
        self.declare_parameter('dataset_dir', DATASET_DIR)
        self.declare_parameter('dataset_queue_size', 32)
        self.declare_parameter('dataset_png_compression', 1)
        self.declare_parameter('dataset_format', 'png')
        self.declare_parameter('model_path', '')

    def _read_params(self):
        g = self.get_parameter
        return {
            'sim_mode': _as_bool(g('sim_mode').value),
            'enable_ai': _as_bool(g('enable_ai').value),
            'inference_fps': max(0.0, float(g('inference_fps').value)),
            'dataset_fps': max(0.0, float(g('dataset_fps').value)),
            'dataset_debug': _as_bool(g('dataset_debug').value),
            'dataset_debug_period_s': max(
                0.1, float(g('dataset_debug_period_sec').value)),
            'dataset_submit_timeout_s': max(
                0.1, float(g('dataset_submit_timeout_sec').value)),
            'dataset_writer_workers': max(
                1, min(8, int(g('dataset_writer_workers').value))),
            'dataset_webp_method': max(
                0, min(6, int(g('dataset_webp_method').value))),
            'dataset_fsync_each_file': _as_bool(
                g('dataset_fsync_each_file').value),
            'camera_startup_timeout_s': max(
                1.0, float(g('camera_startup_timeout_sec').value)),
            'inference_threads': max(1, int(g('inference_threads').value)),
            'confidence': min(1.0, max(0.05, float(g('confidence').value))),
            'gate_feature_mode': str(g('gate_feature_mode').value).strip().lower(),
            'enable_gortc': _as_bool(g('enable_gortc').value),
            'gortc_port': g('gortc_http_port').value,
            'stream_annotated': _as_bool(g('stream_annotated').value),
            'stream_pose_overlay': _as_bool(
                g('stream_pose_overlay').value),
            'annotated_max_width': max(0, int(g('annotated_max_width').value)),
            'mjpeg_port': g('mjpeg_port').value,
            'enable_front': _as_bool(g('enable_front_camera').value),
            'enable_down': _as_bool(g('enable_down_camera').value),
            'camera_config_profile': str(g('camera_config_profile').value),
            'camera_config_dir': str(g('camera_config_dir').value).strip(),
            'model_path': g('model_path').value,
        }

    def _warn(self, msg):
        self.get_logger().warn(msg)

    def report_camera_failure(self, camera, reason):
        """Report a persistent sensor failure and terminate recording safely."""
        message = f'{camera} camera failure: {reason}'
        self.get_logger().error(message)
        self.ai.fail_dataset_recording(message)

    def _rclpy_ok(self):
        return rclpy.ok()

    def _pose_cb(self, msg):
        """Cache the formal estimated pose for video telemetry."""
        values = (
            msg.robot_x, msg.robot_y, msg.robot_z,
            math.radians(msg.robot_yaw),
        )
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
        """Compatibility entry point for callers that already have an Image.

        The simulator Sensor no longer calls this method: it reads the shared
        memory rings and uses :meth:`submit_frame`, so image payloads do not
        enter the ROS graph.
        """
        try:
            cv_img = image_msg_to_bgr(msg)
        except Exception as e:
            self.get_logger().warn(f'Image conversion failed ({camera}): {e}')
            return
        # raw preview updated at arrival (independent of YOLO speed)
        self.update_raw_preview(camera, cv_img, msg.header.stamp)
        # Show live frames until the first inference finishes. If YOLO could
        # not load, keep the annotated endpoint live without detection boxes.
        if self.stream_requested(camera, True):
            with self._stream_lock:
                annotated_ready = (
                    self._stream_annotated_jpegs[camera] is not None)
            if not self.ai._model_loaded or not annotated_ready:
                self.update_annotated_stream(
                    camera, cv_img, msg.header.stamp)
        self.ai.record_capture_frame(
            camera, cv_img, msg.header, right_stamp, int(stereo_pair_id or 0))
        self._gate.submit(
            camera,
            ('opencv', cv_img, msg.header.stamp, False,
             right_stamp, int(stereo_pair_id or 0)),
        )

    def submit_frame(self, camera, frame, stamp=None, right_stamp=None,
                     stereo_pair_id=0):
        """Hand a local BGR frame to preview and AI without ROS Image."""
        stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
        # Keep the raw preview fed by both the simulator shared-memory path
        # and the real V4L2 path.  The HTTP handler registers a client before
        # waiting, so encoding happens only while a stream is requested.
        self.update_raw_preview(camera, frame, stamp)
        if self.stream_requested(camera, True):
            with self._stream_lock:
                annotated_ready = (
                    self._stream_annotated_jpegs[camera] is not None)
            if not self.ai._model_loaded or not annotated_ready:
                self.update_annotated_stream(camera, frame, stamp)
        capture_header = Header()
        capture_header.stamp = stamp
        self.ai.record_capture_frame(
            camera, frame, capture_header, right_stamp,
            int(stereo_pair_id or 0))
        self._gate.submit(
            camera, ('opencv', frame, stamp, True, right_stamp,
                     int(stereo_pair_id or 0)))

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
        if self._stream_pose_overlay:
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
        if self._stream_pose_overlay:
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
        # Stop producers before draining AI.  Otherwise FrameGate workers can
        # enqueue new frames while the recorder is already being closed.
        self.sensor.shutdown()
        self._gate.shutdown()
        self.ai.shutdown()
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
