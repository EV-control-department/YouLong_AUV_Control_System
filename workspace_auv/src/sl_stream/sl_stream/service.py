"""Own go2rtc and keep its on-demand camera routes in sync with CameraCatalog."""

from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
from urllib.error import URLError
from urllib.request import urlopen

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sl_msgs.msg import CameraCatalog


CATALOG_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
CATALOG_TIMEOUT_SEC = 5.0


def stream_name(camera_id: int) -> str:
    return f'sl_{int(camera_id)}'


def _find_go2rtc() -> str:
    configured = os.environ.get('GO2RTC')
    if configured:
        return configured
    for start in (Path.cwd(), Path(__file__).resolve()):
        current = start if start.is_dir() else start.parent
        for parent in (current, *current.parents):
            candidate = parent / 'third_party' / 'go2rtc' / 'go2rtc'
            if candidate.is_file() and os.access(candidate, os.X_OK):
                return str(candidate)
    return shutil.which('go2rtc') or 'go2rtc'


def _descriptor_identity(catalog):
    """Only fields that change the frame source or encoded stream geometry."""
    cameras = {}
    for camera in catalog.cameras:
        camera_id = int(camera.id)
        if camera_id == 0 or camera_id in cameras:
            raise ValueError(f'duplicate or invalid camera ID: {camera_id}')
        if (camera.width <= 0 or camera.height <= 0 or not camera.ice_service
                or camera.encoding != 'JPEG'):
            raise ValueError(f'invalid camera descriptor: {camera.name or camera_id}')
        cameras[camera_id] = (
            str(camera.name), str(camera.ice_service), int(camera.width),
            int(camera.height), bool(camera.enabled),
        )
    if (int(catalog.configured_count) != len(cameras)
            or int(catalog.enabled_count) != sum(item[4] for item in cameras.values())
            or int(catalog.online_count) != sum(bool(camera.online)
                                                for camera in catalog.cameras)):
        raise ValueError('camera catalog counts do not match its descriptors')
    return cameras


class SlStream(Node):
    def __init__(self):
        super().__init__('sl_stream')
        self.declare_parameter('catalog_topic', '/auv/sl_camera/cameras')
        self.declare_parameter('max_fps', 30.0)
        self.declare_parameter('bitrate', 2_000_000)
        self.declare_parameter('max_bitrate', 3_000_000)
        self.declare_parameter('encoder', 'auto')
        self.declare_parameter('api_port', 1984)
        self.declare_parameter('go2rtc_executable', '')
        self.topic = str(self.get_parameter('catalog_topic').value)
        self.max_fps = min(30.0, float(self.get_parameter('max_fps').value))
        self.bitrate = int(self.get_parameter('bitrate').value)
        self.max_bitrate = int(self.get_parameter('max_bitrate').value)
        self.encoder = str(self.get_parameter('encoder').value).strip().lower()
        self.api_port = int(self.get_parameter('api_port').value)
        if self.max_fps <= 0 or self.bitrate <= 0 or self.max_bitrate < self.bitrate:
            raise ValueError('fps and bitrate must be positive; max_bitrate must be >= bitrate')
        if self.encoder not in ('auto', 'jetson', 'software'):
            raise ValueError('encoder must be auto, jetson, or software')
        if not 1 <= self.api_port <= 65535:
            raise ValueError('api_port must be between 1 and 65535')

        executable = str(self.get_parameter('go2rtc_executable').value).strip()
        self.go2rtc = executable or _find_go2rtc()
        self._catalog = None
        self._catalog_at = 0.0
        self._registered = {}
        self._lock = threading.RLock()
        self._temporary = tempfile.TemporaryDirectory(prefix='sl-stream-')
        self._config = Path(self._temporary.name) / 'go2rtc.yaml'
        self._write_runtime_config({})
        self._process = None
        self._start_go2rtc()
        self.create_subscription(
            CameraCatalog, self.topic, self._on_catalog, CATALOG_QOS)
        self._timer = self.create_timer(0.25, self._reconcile)
        self.get_logger().info(
            f'catalog={self.topic}; HTTP=:{self.api_port}; go2rtc={self.go2rtc}; '
            f'encoder={self.encoder}; max_fps={self.max_fps:g}; '
            f'bitrate={self.bitrate}/{self.max_bitrate} bps')

    def _write_runtime_config(self, routes):
        template = Path(__file__).resolve().parents[1] / 'config' / 'go2rtc.yaml'
        if not template.is_file():
            from ament_index_python.packages import get_package_share_directory
            template = Path(get_package_share_directory('sl_stream')) / 'config' / 'go2rtc.yaml'
        config = template.read_text(encoding='utf-8').replace(
            '":1984"', f'":{self.api_port}"')
        if routes:
            entries = []
            for camera_id, camera, catalog in routes:
                name = stream_name(camera_id)
                source = self._route_source(camera_id, camera, catalog)
                entries.append(f'  {json.dumps(name)}: {json.dumps(source)}')
            config = config.replace('streams: {}', 'streams:\n' + '\n'.join(entries))
        self._config.write_text(config, encoding='utf-8')

    @property
    def _api(self):
        return f'http://127.0.0.1:{self.api_port}'

    def _start_go2rtc(self):
        from ament_index_python.packages import get_package_prefix
        binary_dir = Path(get_package_prefix('sl_stream')) / 'lib' / 'sl_stream'
        environment = os.environ.copy()
        environment['PATH'] = str(binary_dir) + os.pathsep + environment.get('PATH', '')
        self._process = subprocess.Popen(
            [self.go2rtc, '-config', str(self._config)],
            stdin=subprocess.DEVNULL, stdout=sys.stderr, stderr=sys.stderr,
            env=environment, start_new_session=True, close_fds=True)
        deadline = time.monotonic() + 10.0
        settle_until = time.monotonic() + 0.2
        last_error = 'HTTP API did not become ready'
        while time.monotonic() < deadline:
            if self._process.poll() is not None:
                raise RuntimeError(f'go2rtc exited with code {self._process.returncode}')
            if time.monotonic() < settle_until:
                time.sleep(0.05)
                continue
            try:
                with urlopen(self._api + '/api/streams', timeout=0.5) as response:
                    if response.status == 200 and self._process.poll() is None:
                        return
            except (OSError, URLError, TimeoutError) as error:
                last_error = str(error)
                time.sleep(0.1)
        self._stop_go2rtc()
        raise RuntimeError(f'go2rtc API failed to start: {last_error}')

    def _on_catalog(self, message):
        with self._lock:
            self._catalog = message
            self._catalog_at = time.monotonic()

    def _route_source(self, camera_id, camera, catalog):
        service = camera[1]
        args = [
            'camera_streamer', '--camera-id', str(camera_id),
            '--catalog-topic', self.topic,
            '--source-instance-id', str(catalog.source_instance_id),
            '--config-version', str(catalog.config_version),
            '--max-fps', str(self.max_fps), '--bitrate', str(self.bitrate),
            '--max-bitrate', str(self.max_bitrate), '--encoder', self.encoder,
        ]
        args.extend(['--ice-service', service, '--width', str(camera[2]),
                     '--height', str(camera[3])])
        return 'exec:' + ' '.join(args)

    def _reconcile(self):
        if self._process is None or self._process.poll() is not None:
            if self._process is not None:
                self.get_logger().error('go2rtc exited; restarting its private instance')
                self._stop_go2rtc()
            try:
                self._start_go2rtc()
            except Exception as error:
                self.get_logger().error(f'go2rtc restart failed: {error}')
                return

        with self._lock:
            catalog = self._catalog
            catalog_age = time.monotonic() - self._catalog_at
        if catalog is None or catalog_age > CATALOG_TIMEOUT_SEC:
            if self._registered:
                self.get_logger().warning(
                    'camera catalog is stale; removing its stream routes')
                try:
                    self._write_runtime_config({})
                except OSError as error:
                    self.get_logger().error(
                        f'failed to clear stale routes from runtime config: {error}')
                    return
                self._registered.clear()
                self._restart_go2rtc()
            return
        try:
            cameras = _descriptor_identity(catalog)
        except ValueError as error:
            self.get_logger().error(f'ignoring invalid camera catalog: {error}')
            return

        desired = {camera_id: camera for camera_id, camera in cameras.items() if camera[4]}
        generation = (str(catalog.source_instance_id), int(catalog.config_version))
        desired_identities = {
            camera_id: (*generation, camera[1], camera[2], camera[3])
            for camera_id, camera in desired.items()
        }
        if desired_identities == self._registered:
            return

        routes = [(camera_id, camera, catalog)
                  for camera_id, camera in desired.items()]
        try:
            self._write_runtime_config(routes)
        except OSError as error:
            self.get_logger().error(f'failed to write go2rtc runtime config: {error}')
            return
        self._registered = desired_identities
        self._restart_go2rtc()
        for camera_id, camera in desired.items():
            self.get_logger().info(
                f'registered {stream_name(camera_id)}: {camera[0]} '
                f'({camera[2]}x{camera[3]})')

    def _restart_go2rtc(self):
        self._stop_go2rtc()
        try:
            self._start_go2rtc()
        except Exception as error:
            self.get_logger().error(f'go2rtc restart failed: {error}')

    def _stop_go2rtc(self):
        process = self._process
        self._process = None
        if process is None:
            return
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            pass
        try:
            os.killpg(process.pid, 0)
        except ProcessLookupError:
            return
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                break
            time.sleep(0.05)
        else:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        process.wait(timeout=2.0)

    def destroy_node(self):
        self._stop_go2rtc()
        self._temporary.cleanup()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SlStream()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':  # pragma: no cover
    main()
