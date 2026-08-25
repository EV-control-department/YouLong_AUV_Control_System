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
import subprocess
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Header

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
    import_cv_bridge,
)


class CameraAiNode(Node):
    """uv_camera node: composes uv_sensor (source+preview) + uv_ai (detection)."""

    def __init__(self):
        super().__init__('uv_camera')
        self._declare_params()
        params = self._read_params()

        # stream caches (go2rtc preview: raw + annotated)
        self._stream_annotated = params['stream_annotated']
        self._mjpeg_port = params['mjpeg_port']
        self._stream_lock = threading.Lock()
        self._stream_jpegs = {'front': None, 'down': None}
        self._stream_annotated_jpegs = {'front': None, 'down': None}
        self._stream_sequence = {'front': 0, 'down': 0}
        self._stream_annotated_sequence = {'front': 0, 'down': 0}
        self._stream_condition = threading.Condition(self._stream_lock)
        self._stream_stop = threading.Event()
        self._mjpeg_server = None
        self._gortc_process = None
        self._gortc_config = None

        # determine active cameras
        active = []
        if params['enable_front']:
            active.append('front')
        if params['enable_down']:
            active.append('down')

        # uv_ai (must be built before gate so gate.consumer is ready)
        self.ai = ai_mod.Ai(self, self.update_annotated_stream, cameras=active)
        self._gate = FrameGate(self.ai.process, cameras=active,
                               max_workers=2, log_warn=self._warn)
        self.ai.load_model(params['model_path'])

        # uv_sensor
        self.sensor = sensor_mod.Sensor(
            self, self._gate, params['sim_mode'],
            params['enable_front'], params['enable_down'])
        self.sensor.start()

        # cv_bridge only for sim mode (decode ROS Image -> BGR)
        self._cv_bridge_ok = False
        self.bridge = None
        if params['sim_mode']:
            try:
                self.bridge = import_cv_bridge()()
                self._cv_bridge_ok = True
            except Exception as e:
                self.get_logger().error(f'cv_bridge not available in sim mode: {e}')

        # MJPEG server + go2rtc (preview only)
        if params['enable_gortc']:
            if self._start_mjpeg_server():
                self._start_gortc(params['gortc_port'])

        self.get_logger().info(
            f'uv_camera started: sensor(source={params["sim_mode"] and "sim" or "v4l"})'
            f' + ai, go2rtc preview on {params["gortc_port"]}')

    # ── params ──────────────────────────────────────────────────────────
    def _declare_params(self):
        self.declare_parameter('sim_mode', False)
        self.declare_parameter('enable_gortc', ENABLE_GORTC)
        self.declare_parameter('gortc_executable', GORTC_EXECUTABLE)
        self.declare_parameter('gortc_http_port', GORTC_HTTP_PORT)
        self.declare_parameter('mjpeg_port', VISION_MJPEG_PORT)
        self.declare_parameter('stream_annotated', STREAM_ANNOTATED)
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
            'enable_gortc': g('enable_gortc').value,
            'gortc_port': g('gortc_http_port').value,
            'stream_annotated': g('stream_annotated').value,
            'mjpeg_port': g('mjpeg_port').value,
            'enable_front': g('enable_front_camera').value,
            'enable_down': g('enable_down_camera').value,
            'model_path': g('model_path').value,
        }

    def _warn(self, msg):
        self.get_logger().warn(msg)

    def _rclpy_ok(self):
        return rclpy.ok()

    # ── sensor connector: ROS->BGR gate submission + raw preview ────────
    def submit_image(self, camera, msg):
        """Called from uv_sensor when a ROS Image arrives (sim mode).

        Decode to BGR, update the raw preview cache (same as the V4L2 path),
        then hand the frame to uv_ai through the in-memory gate.
        """
        if not self._cv_bridge_ok:
            return
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'Image conversion failed ({camera}): {e}')
            return
        # raw preview updated at arrival (independent of YOLO speed)
        self.update_raw_preview(camera, cv_img)
        self._gate.submit(camera, ('opencv', cv_img, msg.header.stamp, False))

    def submit_frame(self, camera, frame):
        """Called from uv_sensor when a V4L2 frame arrives (real mode)."""
        stamp = self.get_clock().now().to_msg()
        self._gate.submit(camera, ('opencv', frame, stamp, True))

    def update_raw_preview(self, camera, frame):
        ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            return
        with self._stream_condition:
            self._stream_jpegs[camera] = encoded.tobytes()
            self._stream_sequence[camera] += 1
            self._stream_condition.notify_all()

    def update_annotated_stream(self, camera, frame):
        if not self._stream_annotated:
            return
        ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            return
        with self._stream_condition:
            self._stream_annotated_jpegs[camera] = encoded.tobytes()
            self._stream_annotated_sequence[camera] += 1
            self._stream_condition.notify_all()

    def _wait_for_stream_frame(self, camera, annotated, last_sequence):
        seq_cache = (self._stream_annotated_sequence if annotated
                     else self._stream_sequence)
        jpeg_cache = (self._stream_annotated_jpegs if annotated
                      else self._stream_jpegs)
        with self._stream_condition:
            while not self._stream_stop.is_set():
                seq = seq_cache[camera]
                payload = jpeg_cache[camera]
                if payload is not None and seq != last_sequence:
                    return payload, seq
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
                f'uv_camera MJPEG server on port {self._mjpeg_port}')
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
            return
        self._gortc_config = os.path.join(
            '/tmp', f'uv_camera_go2rtc_{os.getpid()}.yaml')
        cfg = ('api:\n'
               f'  listen: ":{port}"\n'
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
            self.get_logger().info(
                f'go2rtc started on {port}; streams: front, down'
                + (', front_annotated, down_annotated' if self._stream_annotated else ''))
        except (OSError, ValueError, RuntimeError) as e:
            self.get_logger().error(f'Failed to start go2rtc: {e}')

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
        # repo 根下的 go2rtc(固定位置;不要用 __file__ 反推,依赖 symlink/copy 布局脆弱)
        repo_paths = [
            '/home/doc049/dev/UUV/YouLong_AUV_Control_System/third_party/go2rtc/go2rtc',
            os.path.expanduser('~/YouLong_AUV_Control_System/third_party/go2rtc/go2rtc'),
        ]
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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
