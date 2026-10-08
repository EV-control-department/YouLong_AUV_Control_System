"""Profile-aware, staged real-vehicle startup coordinator.

Child ROS launches are isolated into process groups and their output is sent
to files.  This process owns only children it created; discovered nodes are
never terminated during cleanup.
"""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

from auv_protocol.topics import (
    CAMERA_HEALTH,
    DOWN_LEFT_INFO, DOWN_RIGHT_INFO,
    FRONT_LEFT_INFO, FRONT_RIGHT_INFO,
    MODEL_CLASS_MAPPING, PERCEPTION_DETECTIONS, PERCEPTION_HEALTH,
    PLANNING_STATUS, STATE_HEALTH,
    STATE_ODOM, TRACKS, ZIT6_HEARTBEAT_STATE, ZIT6_STATUS,
)
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data,
    QoSProfile, ReliabilityPolicy,
)
try:
    from rclpy.signals import SignalHandlerOptions
except ImportError:  # Foxy predates SignalHandlerOptions.
    SignalHandlerOptions = None
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import UInt32
from uv_msgs.msg import (
    DetectionArray, ModelClassMapping, ObjectTrackArray, PoseInfo,
    SensorHealth,
)
from zit6_interfaces.msg import ZitStatus


class StartupBlocked(RuntimeError):
    """A safety/readiness gate failed; already-started processes are retained."""


class ShutdownRequested(Exception):
    """Ctrl-C/SIGTERM requested an orderly shutdown."""


CAMERA_INFO_TOPICS = {
    'front_left': FRONT_LEFT_INFO,
    'front_right': FRONT_RIGHT_INFO,
    'down_left': DOWN_LEFT_INFO,
    'down_right': DOWN_RIGHT_INFO,
}
CAMERA_SIDES = {
    'front': ('front_left', 'front_right'),
    'down': ('down_left', 'down_right'),
}


def _as_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


class RealStartupManager(Node):
    """Launch and gate real vehicle components without duplicating ROS nodes."""

    def __init__(self, args):
        super().__init__('real_startup_manager')
        self.args = args
        self._shutdown_requested = False
        self.timeout = float(args.ready_timeout)
        self.max_age = float(args.max_age)
        if self.timeout <= 0 or self.max_age <= 0:
            raise ValueError('ready-timeout and max-age must be positive')
        if args.startup_mode not in ('auto', 'adopt', 'managed'):
            raise ValueError('startup-mode must be auto, adopt, or managed')
        if not args.check_backend_health:
            self.get_logger().warning(
                'Backend-health startup gate is disabled; '
                'runtime health will still be displayed for observation')

        log_root = Path(os.environ.get('ROS_LOG_DIR', Path.home() / '.ros' / 'log'))
        self.log_dir = log_root / f'real_startup_{datetime.now():%Y%m%d_%H%M%S}'
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.children = []
        self.component_state = {}
        self._observed_phases = {}
        self.startup_blocked = False
        self._last_dashboard = 0.0
        self._last_monitor = 0.0
        self._last_odom = 0.0
        self._odom_samples = 0
        self._localization_ok = False
        self._localization_at = 0.0
        self._mcu_status = None
        self._mcu_status_at = 0.0
        self._heartbeat = None
        self._heartbeat_at = 0.0
        self._heartbeat_samples = 0
        self._camera = {
            name: {'available': False, 'health_at': 0.0, 'detail': ''}
            for name in CAMERA_SIDES
        }
        self._calibration = {
            name: False for sides in CAMERA_SIDES.values() for name in sides
        }
        self._mapping_ready = False
        self._detections = Counter()
        self._detection_at = {}
        self._detector_health = {}
        self._tracks = 0
        self._tracks_at = 0.0
        self._nav_health = None
        self._nav_health_at = 0.0

        self.create_subscription(PoseInfo, STATE_ODOM, self._odom_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(SensorHealth, STATE_HEALTH, self._localization_cb, 10)
        self.create_subscription(ZitStatus, ZIT6_STATUS, self._mcu_cb, 10)
        self.create_subscription(UInt32, ZIT6_HEARTBEAT_STATE,
                                 self._heartbeat_cb, 10)
        mapping_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            ModelClassMapping, MODEL_CLASS_MAPPING,
            self._mapping_cb, mapping_qos)
        camera_info_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for camera, info_topic in CAMERA_INFO_TOPICS.items():
            self.create_subscription(
                CameraInfo, info_topic,
                lambda msg, key=camera: self._camera_info_cb(key, msg),
                camera_info_qos)
        self.create_subscription(
            SensorHealth, CAMERA_HEALTH, self._camera_health_cb, 10)
        self.create_subscription(
            DetectionArray, PERCEPTION_DETECTIONS, self._detection_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            SensorHealth, PERCEPTION_HEALTH, self._detector_health_cb, 10)
        self.create_subscription(
            ObjectTrackArray, TRACKS, self._tracks_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            SensorHealth, PLANNING_STATUS, self._nav_cb, 10)
        self.get_logger().info(
            f'profile={args.profile} mode={args.startup_mode} '
            f'logs={self.log_dir}; task lifecycle is external to real bringup')

    def _odom_cb(self, msg):
        values = (msg.robot_x, msg.robot_y, msg.robot_z, msg.robot_yaw)
        if all(math.isfinite(float(value)) for value in values):
            self._odom_samples += 1
            self._last_odom = time.monotonic()

    def _localization_cb(self, msg):
        if msg.sensor_name == 'localization':
            self._localization_ok = bool(msg.available)
            self._localization_at = time.monotonic()

    def _mcu_cb(self, msg):
        self._mcu_status = msg
        self._mcu_status_at = time.monotonic()

    def _heartbeat_cb(self, msg):
        if self._heartbeat is None or int(msg.data) != self._heartbeat:
            self._heartbeat_samples += 1
        self._heartbeat = int(msg.data)
        self._heartbeat_at = time.monotonic()

    def _camera_info_cb(self, camera, msg):
        self._calibration[camera] = (
            int(msg.width) > 0 and int(msg.height) > 0
            and len(msg.k) >= 9 and float(msg.k[0]) > 0.0
            and float(msg.k[4]) > 0.0)

    def _camera_health_cb(self, msg):
        name = str(msg.sensor_name).strip().lower()
        if name.startswith('camera/'):
            name = name[len('camera/'):]
        if name in self._camera:
            state = self._camera[name]
            available = bool(msg.available)
            detail = str(msg.detail)
            changed = (state['available'] != available
                       or (not available and detail.startswith('error:')
                           and state['detail'] != detail))
            state['available'] = available
            state['health_at'] = time.monotonic()
            state['detail'] = detail
            self.component_state[f'camera/{name}'] = (
                'READY' if available else f'DEGRADED ({detail})')
            if changed:
                message = (f'camera/{name}: '
                           f'{"healthy" if available else "unavailable"}: {detail}')
                if available:
                    self.get_logger().info(message)
                else:
                    self.get_logger().warning(message)

    def _detection_cb(self, msg):
        camera = str(msg.camera_name).strip().lower()
        if camera in self._calibration:
            self._detections[camera] += 1
            self._detection_at[camera] = time.monotonic()

    def _detector_health_cb(self, msg):
        name = str(msg.sensor_name).strip().lower()
        if name in ('detector/front', 'detector/down'):
            self._detector_health[name] = (
                bool(msg.available), time.monotonic(), str(msg.detail))

    def _tracks_cb(self, _msg):
        self._tracks += 1
        self._tracks_at = time.monotonic()

    def _nav_cb(self, msg):
        if msg.sensor_name == 'navigator':
            self._nav_health = bool(msg.available)
            self._nav_health_at = time.monotonic()

    def _mapping_cb(self, _msg):
        self._mapping_ready = True

    def _node_names(self):
        names = Counter(
            f'{namespace.rstrip("/")}/{name}'.replace('//', '/')
            for name, namespace in self.get_node_names_and_namespaces())
        return names

    def _matching_nodes(self, expected_name):
        short_name = expected_name.rsplit('/', 1)[-1]
        names = self._node_names()
        return [name for name, count in names.items()
                if name.rsplit('/', 1)[-1] == short_name for _ in range(count)]

    def _has_node(self, expected_name):
        return bool(self._matching_nodes(expected_name))

    def _parameters_match(self, node_name, expected):
        if not expected:
            return True, 'node discovered; no component-specific parameters'
        service_name = node_name.rstrip('/') + '/get_parameters'
        client = self.create_client(GetParameters, service_name)
        try:
            if not client.wait_for_service(timeout_sec=min(3.0, self.timeout)):
                return False, f'{node_name} parameter service unavailable'
            request = GetParameters.Request()
            request.names = list(expected)
            future = client.call_async(request)
            # A node can appear in the ROS graph before its executor is
            # spinning. Give slow initializers (for example YOLO model load)
            # the configured startup readiness window to answer.
            response_timeout = max(0.1, float(self.timeout))
            self._wait_for(
                future.done,
                f'{node_name} parameter response',
                duration=response_timeout)
            if not future.done():
                return False, (
                    f'{node_name} parameter response timed out after '
                    f'{response_timeout:.1f}s')
            service_error = future.exception()
            if service_error is not None:
                return False, (
                    f'{node_name} parameter request failed: {service_error}')
            response = future.result()
            for index, (name, wanted) in enumerate(expected.items()):
                try:
                    parameter_msg = ParameterMsg()
                    parameter_msg.name = name
                    parameter_msg.value = response.values[index]
                    value = Parameter.from_parameter_msg(parameter_msg).value
                except (AttributeError, IndexError, TypeError):
                    return False, f'{node_name} does not expose parameter {name}'
                if isinstance(wanted, float):
                    matches = isinstance(value, (int, float)) and abs(value - wanted) < 1e-6
                else:
                    matches = value == wanted
                if not matches:
                    return False, f'{node_name}.{name}={value!r}, expected {wanted!r}'
            return True, 'effective parameters match'
        finally:
            self.destroy_client(client)

    def _launch(self, component, command, expected_nodes=(), params=None):
        matches = {name: self._matching_nodes(name) for name in expected_nodes}
        found = [name for name, instances in matches.items() if instances]
        if any(len(instances) > 1 for instances in matches.values()):
            raise StartupBlocked(
                f'{component}: duplicate ROS node names detected: {matches}')
        if self.args.startup_mode == 'managed' and found:
            raise StartupBlocked(
                f'{component}: managed mode found existing nodes {found}; '
                'refusing to create duplicates')
        if found:
            if len(found) != len(expected_nodes):
                raise StartupBlocked(
                    f'{component}: only part of its node group exists {found}; '
                    'cannot safely adopt or launch the missing members')
            compatibility = params or {}
            for name in expected_nodes:
                actual_name = matches[name][0]
                compatible, detail = self._parameters_match(
                    actual_name, compatibility.get(name, {}))
                if not compatible:
                    raise StartupBlocked(f'{component}: incompatible node: {detail}')
            self.component_state[component] = 'REUSED'
            self._dashboard()
            return False
        if self.args.startup_mode == 'adopt':
            self.component_state[component] = 'MISSING'
            self._dashboard()
            return False

        log_path = self.log_dir / f'{component}.log'
        log_file = log_path.open('ab', buffering=0)
        try:
            process = subprocess.Popen(
                command, stdout=log_file, stderr=subprocess.STDOUT,
                start_new_session=True, env=os.environ.copy())
        except Exception:
            log_file.close()
            raise
        self.children.append((component, process, log_file))
        self.component_state[component] = 'STARTING'
        self.get_logger().info(
            f'{component}: started by this launch; output -> {log_path}')
        self._dashboard()
        if expected_nodes:
            self._wait_for(
                lambda: all(len(self._matching_nodes(name)) == 1
                            for name in expected_nodes),
                f'{component} ROS nodes {expected_nodes}')
            for name in expected_nodes:
                actual_name = self._matching_nodes(name)[0]
                compatible, detail = self._parameters_match(
                    actual_name, (params or {}).get(name, {}))
                if not compatible:
                    raise StartupBlocked(
                        f'{component}: started node failed compatibility check: {detail}')
        else:
            self._wait_for(lambda: process.poll() is None,
                           f'{component} process remains alive',
                           duration=min(1.0, self.timeout))
        self.component_state[component] = 'READY'
        self._dashboard()
        return True

    def _wait_for(self, predicate, description, duration=None):
        deadline = time.monotonic() + (self.timeout if duration is None else duration)
        while (rclpy.ok() and not self._shutdown_requested
               and time.monotonic() < deadline):
            self._spin()
            self._check_children()
            if predicate():
                return
        if self._shutdown_requested:
            raise ShutdownRequested('shutdown requested')
        raise StartupBlocked(f'timed out waiting for {description}')

    def _spin(self):
        rclpy.spin_once(self, timeout_sec=0.1)
        self._dashboard()
        self.monitor()

    def _dashboard(self):
        now = time.monotonic()
        if now - self._last_dashboard < 1.0:
            return
        self._last_dashboard = now
        color_enabled = (
            'NO_COLOR' not in os.environ
            and (sys.stdout.isatty()
                 or bool(os.environ.get('FORCE_COLOR'))
                 or os.environ.get('TERM', '') not in ('', 'dumb')))

        def colored_row(name, state):
            normalized = str(state).upper()
            if any(token in normalized for token in (
                    'BLOCKED', 'ERROR', 'FAILED', 'EXITED', 'CONFLICT',
                    'DEGRADED', 'UNAVAILABLE', 'STALE', 'FAULT', 'LATCHED',
                    'UNVERIFIED')):
                color = '\033[31m'  # red: failed or unsafe
            elif any(token in normalized for token in (
                    'STARTING', 'WAITING', 'DEFERRED', 'PAUSED', 'RETRY')):
                color = '\033[33m'  # yellow: startup/readiness in progress
            elif any(token in normalized for token in (
                    'READY', 'REUSED', 'COMPLETE', 'TRIGGERED',
                    'ATTACHED', 'RUNNING', 'DONE', 'HEALTHY', 'FRESH')):
                color = '\033[32m'  # green: active and healthy
            else:
                color = '\033[90m'  # gray: disabled, missing, or not started
            row = f'{name:18} {state}'
            return f'{color}{row}\033[0m' if color_enabled else row

        rows = [colored_row(name, state)
                for name, state in sorted(self.component_state.items())]

        status = self._mcu_status
        if status is None:
            mcu_state = 'NO DATA'
        elif now - self._mcu_status_at > self.max_age:
            mcu_state = 'STALE'
        elif int(status.error_flags) != 0:
            mcu_state = f'ERROR flags=0x{int(status.error_flags):x}'
        else:
            mcu_state = (
                f'HEALTHY armed={bool(status.is_armed)} '
                f'navigation={bool(status.navigation_ready)}')
        rows.append(colored_row('MCU', mcu_state))

        if self._heartbeat is None:
            heartbeat_state = 'NO DATA'
        elif now - self._heartbeat_at > max(3.0, self.max_age):
            heartbeat_state = f'STALE counter={self._heartbeat}'
        elif self._heartbeat_samples < 2:
            heartbeat_state = f'WAITING counter change={self._heartbeat}'
        else:
            heartbeat_state = f'FRESH counter={self._heartbeat}'
        rows.append(colored_row('ARM heartbeat', heartbeat_state))

        if self._localization_at <= 0:
            localization_state = 'NO DATA'
        elif now - self._localization_at > self.max_age:
            localization_state = 'STALE'
        else:
            localization_state = (
                'HEALTHY' if self._localization_ok else 'UNAVAILABLE')
        rows.append(colored_row('localization', localization_state))

        if self._odom_samples < 2:
            odom_state = f'WAITING samples={self._odom_samples}'
        elif now - self._last_odom > self.max_age:
            odom_state = f'STALE samples={self._odom_samples}'
        else:
            odom_state = f'HEALTHY samples={self._odom_samples}'
        rows.append(colored_row('odom', odom_state))

        for camera in ('front', 'down'):
            camera_data = self._camera[camera]
            calibrated = all(
                self._calibration[side] for side in CAMERA_SIDES[camera])
            if camera_data['health_at'] <= 0:
                camera_state = 'NO DATA'
            elif now - camera_data['health_at'] > self.max_age:
                camera_state = 'STALE'
            elif not camera_data['available']:
                camera_state = f'UNAVAILABLE ({camera_data["detail"]})'
            elif not calibrated:
                camera_state = 'WAITING for calibration'
            else:
                camera_state = 'HEALTHY and calibrated'
            rows.append(colored_row(f'camera/{camera}', camera_state))

        if not self.args.enable_ai:
            perception_state = 'SKIPPED (AI disabled)'
        elif self._healthy_perception():
            perception_state = 'HEALTHY'
        elif self._detections or self._tracks:
            perception_state = 'WAITING or stale data'
        else:
            perception_state = 'WAITING for camera frames'
        rows.append(colored_row('perception', perception_state))

        if not self.args.enable_nav:
            navigation_state = 'SKIPPED (navigation disabled)'
        elif self._nav_health is None:
            navigation_state = 'NO DATA'
        elif now - self._nav_health_at > self.max_age:
            navigation_state = 'STALE'
        else:
            navigation_state = (
                'HEALTHY' if self._nav_health else 'UNAVAILABLE')
        rows.append(colored_row('navigation', navigation_state))
        discovered = ', '.join(sorted(
            name for name in self._node_names()
            if name != '/real_startup_manager')) or '(none)'
        rows.append(f'{"ROS nodes":18} {discovered}')
        title = 'REAL BRINGUP STATUS'
        if color_enabled:
            title = f'\033[1;36m{title}\033[0m'
        table = '\n'.join([title, *rows])
        if sys.stdout.isatty():
            print('\033[2J\033[H' + table, flush=True)
        else:
            self.get_logger().info(table.replace('\n', ' | '))

    def _check_children(self):
        for component, process, _log in self.children:
            code = process.poll()
            if code is not None:
                self.component_state[component] = f'EXITED({code})'
                raise StartupBlocked(
                    f'{component} launch process exited with status {code}')

    def _healthy_backend(self):
        if not self.args.check_backend_health:
            return True
        now = time.monotonic()
        status = self._mcu_status
        mcu_ok = (
            status is not None and now - self._mcu_status_at <= self.max_age
            and int(status.error_flags) == 0)
        hb_ok = (self._heartbeat is not None
                 and now - self._heartbeat_at <= max(3.0, self.max_age)
                 and self._heartbeat_samples >= 2)
        odom_ok = self._odom_samples >= 2 and now - self._last_odom <= self.max_age
        loc_ok = (self._localization_ok
                  and now - self._localization_at <= self.max_age)
        return mcu_ok and hb_ok and odom_ok and loc_ok

    def _healthy_cameras(self):
        now = time.monotonic()
        cameras = [name for name, state in self._camera.items()
                   if state['available']
                   and now - state['health_at'] <= self.max_age
                   and all(self._calibration[side]
                           for side in CAMERA_SIDES[name])]
        return cameras

    def _healthy_perception(self):
        now = time.monotonic()
        cameras = self._healthy_cameras()
        if not cameras:
            return False
        required_sides = [side for camera in cameras
                          for side in CAMERA_SIDES[camera]]
        detections_ok = all(
            self._detections[name] >= 2
            and now - self._detection_at.get(name, 0.0) <= self.max_age
            for name in required_sides)
        detector_ok = all(
            (health := self._detector_health.get(f'detector/{camera}')) is not None
            and health[0] and now - health[1] <= self.max_age
            for camera in cameras)
        tracks_ok = (self._tracks >= 2
                     and now - self._tracks_at <= self.max_age)
        return detector_ok and detections_ok and tracks_ok

    def _port_accepting(self, port):
        try:
            with socket.create_connection(('127.0.0.1', int(port)), timeout=0.3):
                return True
        except (OSError, ValueError):
            return False

    @staticmethod
    def _existing_go2rtc_matches_launch_config():
        try:
            from ament_index_python.packages import get_package_share_directory
            expected_config = (Path(get_package_share_directory('uv_stream'))
                               / 'config' / 'go2rtc.yaml').resolve()
        except Exception:
            return False
        proc_root = Path('/proc')
        if not proc_root.is_dir():
            return False
        for entry in proc_root.iterdir():
            if not entry.name.isdigit():
                continue
            try:
                argv = [part.decode('utf-8', errors='ignore') for part in
                        (entry / 'cmdline').read_bytes().split(b'\0') if part]
            except OSError:
                continue
            if not argv or 'go2rtc' not in Path(argv[0]).name.lower():
                continue
            try:
                config_index = argv.index('-config')
                actual_config = Path(argv[config_index + 1]).resolve()
            except (ValueError, IndexError):
                continue
            if actual_config == expected_config:
                return True
        return False

    def _phase(self, component, predicate, description):
        self.component_state[component] = 'WAITING'
        self._dashboard()
        self._wait_for(predicate, description)
        self.component_state[component] = 'READY'
        self._dashboard()

    def _observe_phase(self, component, predicate, description):
        """Report data readiness without blocking independent component startup."""
        self._observed_phases[component] = (predicate, description)
        self.component_state[component] = (
            'READY' if predicate() else f'WAITING (observation only: {description})')
        self._dashboard()

    def _start_core(self):
        components = [
            ('model_mapping', ['ros2', 'launch', 'uv_perception',
                               'model_mapping_launch.py'],
             ['/model_class_publisher'], {}),
            ('description', ['ros2', 'launch', 'auv_description',
                             'description.launch.py', 'use_sim_time:=false'],
             ['/robot_state_publisher'],
             {'/robot_state_publisher': {'use_sim_time': False}}),
            ('localization', ['ros2', 'launch', 'uv_localization',
                              'localization_launch.py', 'sim_mode:=false',
                              'publish_tf:=true'],
             ['/uv_localization'],
             {'/uv_localization': {'sim_mode': False, 'publish_tf': True}}),
        ]
        if self.args.enable_hardware:
            components.append((
                'hardware', ['ros2', 'launch', 'uv_hm', 'hardware_launch.py',
                             'enable_hardware:=true'], ['/hw_manager'],
                {'/hw_manager': {'watchdog_timeout': 7.0,
                                 'legacy_state_topics': True}}))
        for name, command, nodes, params in components:
            self._launch(name, command, nodes, params)
        if not self.args.enable_hardware:
            self.component_state['hardware'] = (
                'SKIPPED (external MCU status remains required)')
        if self.args.startup_mode == 'adopt':
            required = (self.component_state.get('model_mapping') == 'REUSED'
                        and self.component_state.get('description') == 'REUSED'
                        and self.component_state.get('localization') == 'REUSED'
                        and (not self.args.enable_hardware
                             or self.component_state.get('hardware') == 'REUSED'))
            if not required:
                raise StartupBlocked('adopt mode cannot start missing core nodes')
        if self.args.check_backend_health:
            self._observe_phase(
                'backend gate', self._healthy_backend,
                'fresh healthy MCU status, heartbeat, localization, and odom')
        else:
            self.component_state['backend gate'] = (
                'SKIPPED (check_backend_health=false)')
            self._dashboard()

    def _start_motion_component(self):
        if not self.args.enable_motion:
            self.component_state['basic_motion'] = 'SKIPPED'
            self._dashboard()
            return
        self._launch(
            'basic_motion',
            ['ros2', 'launch', 'uv_control', 'control_launch.py',
             'enable_motion:=true', 'sim_mode:=false'],
            ['/basic_motion'],
            {'/basic_motion': {'sim_mode': False, 'arm_mode': 1,
                               'heartbeat_rate': 15.0, 'start_timeout': 10.0,
                               'arm_confirmation_timeout': 20.0}})
        if self.args.startup_mode == 'adopt' \
                and self.component_state.get('basic_motion') != 'REUSED':
            raise StartupBlocked('adopt mode requires an existing BasicMotion node')
        motion_state = self.component_state['basic_motion']
        if motion_state == 'READY':
            motion_state = 'READY (node only; no START sent)'
        elif motion_state == 'REUSED':
            motion_state = 'REUSED (no START sent)'
        self.component_state['basic_motion'] = motion_state
        self._dashboard()

    def _start_camera_perception(self):
        if self.args.enable_camera:
            camera_dir = self.args.camera_config_dir
            self._launch(
                'camera',
                ['ros2', 'launch', 'uv_camera', 'camera_launch.py',
                 'sim_mode:=false', 'camera_mode:=real',
                 f'camera_config_dir:={camera_dir}'],
                ['/uv_camera'],
                {'/uv_camera': {
                    'sim_mode': False, 'camera_mode': 'real',
                    'enable_front': True, 'enable_down': True,
                    'camera_config_dir': camera_dir}})
            self._observe_phase('camera gate', lambda: bool(self._healthy_cameras()),
                        'at least one fresh calibrated camera view')
        else:
            self.component_state['camera gate'] = 'SKIPPED (enable_camera=false)'
            self._dashboard()

        if self.args.enable_ai:
            self._phase('model mapping gate', lambda: self._mapping_ready,
                        'latched perception model mapping')
            perception_args = [
                'confidence:=0.8', 'enable_gui:=false',
            ]
            for name, switch, node_name, params in (
                ('object_detector', 'enable_detector', '/object_detector',
                 {'confidence': 0.8}),
                ('object_localizer', 'enable_localizer', '/object_localizer',
                 {'world_frame': 'odom'}),
                ('object_estimator', 'enable_estimator', '/object_estimator',
                 {'world_frame': 'odom'}),
            ):
                self._launch(
                    name,
                    ['ros2', 'launch', 'uv_perception', 'perception_launch.py',
                     *perception_args, f'{switch}:=true',
                     *[f'{other}:=false' for other in
                       ('enable_detector', 'enable_localizer', 'enable_estimator')
                       if other != switch]],
                    [node_name], {node_name: params})
            self.component_state['perception'] = 'READY'
            if self.args.enable_perception_gui:
                self._launch(
                    'perception_gui',
                    ['ros2', 'run', 'uv_perception', 'perception_gui'],
                    ['/perception_gui'])
            if not self.args.enable_perception_gate:
                self.component_state['perception gate'] = (
                    'SKIPPED (enable_perception_gate=false)')
                self._dashboard()
            else:
                self._observe_phase(
                    'perception gate', self._healthy_perception,
                    'camera detections and odom-based tracks; waiting does not block startup')
        else:
            self.component_state['perception'] = 'SKIPPED (enable_ai=false)'
            self.component_state['perception gate'] = 'SKIPPED'
            self._dashboard()

    def _start_navigation(self):
        if not self.args.enable_nav:
            self.component_state['navigation'] = 'SKIPPED (enable_nav=false)'
            return
        self._launch(
            'navigation',
            ['ros2', 'launch', 'uv_planning', 'planning_launch.py',
             'enable_nav:=true'], ['/navigator'])
        if self.args.startup_mode == 'adopt' \
                and self.component_state.get('navigation') != 'REUSED':
            raise StartupBlocked('adopt mode requires an existing navigator')
        self._observe_phase(
            'navigation gate',
            lambda: (self._nav_health is True
                     and time.monotonic() - self._nav_health_at <= self.max_age),
            'navigator fresh odometry/tracks readiness')

    def _start_auxiliary(self):
        if self.args.enable_stream:
            if self._port_accepting(self.args.preview_port):
                compatible_stream = self._existing_go2rtc_matches_launch_config()
                if self.args.startup_mode == 'managed':
                    raise StartupBlocked(
                        f'managed mode found a listener on go2rtc port '
                        f'{self.args.preview_port}')
                if self.args.startup_mode == 'auto' and not compatible_stream:
                    raise StartupBlocked(
                        f'go2rtc port {self.args.preview_port} is already occupied; '
                        'the existing service configuration cannot be verified')
                if self.args.startup_mode == 'adopt' and not compatible_stream:
                    raise StartupBlocked(
                        f'adopt mode found a listener on go2rtc port '
                        f'{self.args.preview_port}, but its configuration cannot '
                        'be verified')
                self.component_state['stream'] = (
                    'REUSED' if compatible_stream
                    else 'REUSED')
            elif self.args.startup_mode == 'adopt':
                self.component_state['stream'] = 'MISSING (adopt mode; not started)'
            else:
                self._launch(
                    'stream', ['ros2', 'launch', 'uv_stream', 'stream_launch.py'])
        else:
            self.component_state['stream'] = 'SKIPPED'
        if self.args.record_session:
            existing_recorder = self._existing_recorder_process()
            if existing_recorder:
                raise StartupBlocked(
                    'an existing recorder process was found but its session '
                    'configuration cannot be verified')
            if self.args.startup_mode == 'adopt':
                self.component_state['recording'] = 'MISSING (adopt mode; not started)'
                return
            values = {
                'record_session': 'true', 'record_root': self.args.record_root,
                'record_mode': self.args.record_mode,
                'go2rtc_stream_mode': self.args.go2rtc_stream_mode,
                'go2rtc_video_format': self.args.go2rtc_video_format,
                'record_video_fps': self.args.record_video_fps,
                'record_video_codec': self.args.record_video_codec,
                'video_segment_seconds': self.args.video_segment_seconds,
                'bag_segment_seconds': self.args.bag_segment_seconds,
                'record_bag_storage': self.args.record_bag_storage,
                'record_use_sim_time': 'false',
                'record_image_topics': self.args.record_image_topics,
                'preview_port': self.args.preview_port,
            }
            self._launch(
                'recording',
                ['ros2', 'launch', 'uv_bringup', 'observability.launch.py']
                + [f'{key}:={value}' for key, value in values.items()])
        else:
            self.component_state['recording'] = 'SKIPPED'

    @staticmethod
    def _existing_recorder_process():
        proc_root = Path('/proc')
        if not proc_root.is_dir():
            return False
        for entry in proc_root.iterdir():
            if not entry.name.isdigit():
                continue
            try:
                command = (entry / 'cmdline').read_bytes().replace(b'\0', b' ').decode(
                    'utf-8', errors='ignore')
            except OSError:
                continue
            if '--session-dir' in command and (
                    'uv_record' in command or 'record --session-dir' in command):
                return True
        return False

    def _report_startup_plan(self):
        components = [
            ('model_mapping', True, ['/model_class_publisher']),
            ('description', True, ['/robot_state_publisher']),
            ('localization', True, ['/uv_localization']),
            ('hardware', self.args.enable_hardware, ['/hw_manager']),
            ('basic_motion', self.args.enable_motion, ['/basic_motion']),
            ('camera', self.args.enable_camera, ['/uv_camera']),
            ('object_detector', self.args.enable_ai, ['/object_detector']),
            ('object_localizer', self.args.enable_ai, ['/object_localizer']),
            ('object_estimator', self.args.enable_ai, ['/object_estimator']),
            ('perception_gui', self.args.enable_ai and
             self.args.enable_perception_gui, ['/perception_gui']),
            ('navigation', self.args.enable_nav, ['/navigator']),
        ]
        rows = []
        for component, enabled, nodes in components:
            if not enabled:
                decision = 'SKIP (disabled by profile/override)'
            else:
                present = [self._has_node(node) for node in nodes]
                if all(present):
                    decision = ('CONFLICT (managed mode)' if
                                self.args.startup_mode == 'managed' else
                                'REUSE after compatibility/readiness checks')
                elif any(present):
                    decision = 'CONFLICT (incomplete component node set)'
                elif self.args.startup_mode == 'adopt':
                    decision = 'MISSING (adopt mode will not start it)'
                else:
                    decision = 'START if later readiness gates permit'
            rows.append(f'{component:18} {decision}')

        if not self.args.enable_stream:
            stream = 'SKIP (disabled)'
        elif self._port_accepting(self.args.preview_port):
            stream = ('CONFLICT (managed mode)' if
                      self.args.startup_mode == 'managed' else
                      'REUSE only if installed go2rtc config matches')
        else:
            stream = ('MISSING (adopt mode will not start it)' if
                      self.args.startup_mode == 'adopt' else
                      'START if later readiness gates permit')
        rows.append(f'{"stream":18} {stream}')

        if not self.args.record_session:
            recording = 'SKIP (disabled)'
        elif self._existing_recorder_process():
            recording = 'CONFLICT (existing session config is unverifiable)'
        elif self.args.startup_mode == 'adopt':
            recording = 'MISSING (adopt mode will not start it)'
        else:
            recording = 'START if later readiness gates permit'
        rows.append(f'{"recording":18} {recording}')
        self.get_logger().info(
            'Initial component decisions (existing nodes still undergo '
            'compatibility/readiness checks):\n' + '\n'.join(rows))

    def _ensure_startup_mode_nodes_absent(self):
        if self.args.startup_mode != 'adopt':
            if self.args.enable_stream and self._port_accepting(self.args.preview_port):
                if (self.args.startup_mode == 'managed'
                        or not self._existing_go2rtc_matches_launch_config()):
                    raise StartupBlocked(
                        f'go2rtc port {self.args.preview_port} is occupied and its '
                        'configuration cannot be verified')
            if self.args.record_session and self._existing_recorder_process():
                raise StartupBlocked(
                    'an existing recorder process is present and its configuration '
                    'cannot be verified')
        if self.args.startup_mode == 'adopt':
            targets = ['/model_class_publisher', '/robot_state_publisher',
                       '/uv_localization']
            if self.args.enable_hardware:
                targets.append('/hw_manager')
            if self.args.enable_motion:
                targets.append('/basic_motion')
            if self.args.enable_camera:
                targets.append('/uv_camera')
            if self.args.enable_ai:
                targets.extend(['/object_detector', '/object_localizer',
                                '/object_estimator'])
                if self.args.enable_perception_gui:
                    targets.append('/perception_gui')
            if self.args.enable_nav:
                targets.append('/navigator')
            missing = [name for name in targets if not self._has_node(name)]
            if missing:
                raise StartupBlocked(f'adopt mode is missing required nodes: {missing}')
            return
        if self.args.startup_mode != 'managed':
            return
        targets = ['/model_class_publisher', '/robot_state_publisher',
                   '/uv_localization']
        if self.args.enable_hardware:
            targets.append('/hw_manager')
        if self.args.enable_motion:
            targets.append('/basic_motion')
        if self.args.enable_camera:
            targets.append('/uv_camera')
        if self.args.enable_ai:
            targets.extend(['/object_detector', '/object_localizer',
                            '/object_estimator'])
            if self.args.enable_perception_gui:
                targets.append('/perception_gui')
        if self.args.enable_nav:
            targets.append('/navigator')
        existing = [name for name in targets if self._has_node(name)]
        if existing:
            raise StartupBlocked(
                f'managed mode found pre-existing nodes {existing}; no components started')

    def run_startup(self):
        for _ in range(5):
            self._spin()
        self._report_startup_plan()
        self._ensure_startup_mode_nodes_absent()
        planned = ['model_mapping', 'description', 'localization']
        if self.args.enable_hardware:
            planned.append('hardware')
        if self.args.enable_motion:
            planned.append('BasicMotion node (no START command)')
        planned.append('camera + calibration gate' if self.args.enable_camera
                       else 'camera skipped')
        if self.args.enable_ai:
            planned.append('perception')
        if self.args.enable_nav:
            planned.append('navigation')
        self.get_logger().info(
            f'planned profile={self.args.profile}: {", ".join(planned)}; '
            f'flags: hardware={self.args.enable_hardware}, '
            f'motion={self.args.enable_motion}, camera={self.args.enable_camera}, '
            f'ai={self.args.enable_ai}, '
            f'perception_gate={self.args.enable_perception_gate}, '
            f'nav={self.args.enable_nav}, stream={self.args.enable_stream}, '
            f'record={self.args.record_session}; '
            f'detected nodes={sorted(self._node_names())}')
        self._start_core()
        # START must be available before odom-dependent perception/navigation
        # readiness. The operator manages origin initialization independently.
        self._start_motion_component()
        self._start_camera_perception()
        self._start_navigation()
        self._start_auxiliary()
        self.component_state['startup'] = 'COMPLETE'
        self._dashboard()

    def monitor(self):
        now = time.monotonic()
        if now - self._last_monitor < 0.5:
            return
        self._last_monitor = now
        for component, (predicate, description) in self._observed_phases.items():
            self.component_state[component] = (
                'READY' if predicate() else f'WAITING (observation only: {description})')
        for component, process, _log in self.children:
            code = process.poll()
            if code is not None:
                state = f'EXITED({code})'
                if self.component_state.get(component) != state:
                    self.component_state[component] = state
                    self.get_logger().warning(
                        f'{component} process exited with status {code}; '
                        'reported for observation only')
        self._dashboard()

    def cleanup_owned(self):
        self._terminate_owned_process_groups()

    @staticmethod
    def _process_group_exists(pgid):
        try:
            os.killpg(pgid, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            return True

    def _wait_for_groups(self, children, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            remaining = []
            for component, process, _log_file in children:
                process.poll()  # Reap an exited group leader when possible.
                if self._process_group_exists(process.pid):
                    remaining.append((component, process, _log_file))
            if not remaining:
                return []
            time.sleep(0.1)
        return [entry for entry in children
                if self._process_group_exists(entry[1].pid)]

    def _terminate_owned_process_groups(self):
        """Stop only dedicated sessions created by this manager.

        Each child launch is started with ``start_new_session=True``, so its
        process group cannot contain a node that was already running before
        bringup. Check the whole group, not just the ros2-launch leader: the
        leader can exit while one of its launched nodes is still alive.
        """
        children = list(reversed(self.children))
        for component, process, _log_file in children:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            except OSError as exc:
                self.get_logger().error(
                    f'{component}: SIGINT to owned process group failed: {exc}')

        remaining = self._wait_for_groups(children, timeout=8.0)
        for component, process, _log_file in remaining:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            except OSError as exc:
                self.get_logger().error(
                    f'{component}: SIGTERM to owned process group failed: {exc}')

        remaining = self._wait_for_groups(remaining, timeout=3.0)
        for component, process, _log_file in remaining:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            except OSError as exc:
                self.get_logger().error(
                    f'{component}: SIGKILL to owned process group failed: {exc}')

        still_running = self._wait_for_groups(remaining, timeout=0.5)
        for component, process, _log_file in still_running:
            self.get_logger().error(
                f'{component}: owned process group {process.pid} still exists '
                'after SIGKILL')

        for component, process, log_file in children:
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self.get_logger().error(
                    f'{component}: process leader did not exit after SIGKILL')
            try:
                log_file.close()
            except Exception:
                pass
            self.component_state[component] = 'STOPPED (owned by bringup)'
        self.children.clear()


def _parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', required=True)
    parser.add_argument('--startup-mode', default='auto')
    parser.add_argument('--ready-timeout', type=float, default=120.0)
    parser.add_argument('--max-age', type=float, default=2.0)
    for name, default in (
        ('enable_hardware', True), ('enable_motion', True),
        ('check_backend_health', False),
        ('enable_camera', False), ('enable_ai', True),
        ('enable_perception_gate', False), ('enable_nav', False),
        ('enable_stream', True),
        ('enable_perception_gui', False), ('record_session', False),
    ):
        parser.add_argument('--' + name.replace('_', '-'),
                            default=str(default).lower())
    parser.add_argument('--camera-config-dir', default='')
    parser.add_argument('--record-root', default=str(Path.home() / 'auv_recordings'))
    parser.add_argument('--record-mode', default='raw')
    parser.add_argument('--go2rtc-stream-mode', default='unannotated')
    parser.add_argument('--go2rtc-video-format', default='jpeg')
    parser.add_argument('--record-video-fps', default='5.0')
    parser.add_argument('--record-video-codec', default='libx264')
    parser.add_argument('--video-segment-seconds', default='2.0')
    parser.add_argument('--bag-segment-seconds', default='10.0')
    parser.add_argument('--record-bag-storage', default='auto')
    parser.add_argument('--record-image-topics', default='false')
    parser.add_argument('--preview-port', default='1984')
    args, ros_args = parser.parse_known_args(argv)
    for name in ('enable_hardware', 'enable_motion', 'check_backend_health',
                 'enable_camera', 'enable_ai', 'enable_perception_gate',
                 'enable_nav',
                 'enable_stream',
                 'enable_perception_gui', 'record_session'):
        setattr(args, name, _as_bool(getattr(args, name)))
    return args, ros_args


def main(argv=None):
    args, ros_args = _parse_args(argv)
    manager = None
    signal_state = {'requested': False}
    managed_signals = (signal.SIGINT, signal.SIGTERM)
    previous_handlers = {}

    def request_orderly_shutdown(signum, _frame):
        signal_state['requested'] = True
        signal_state['signal'] = signum
        if manager is not None:
            manager._shutdown_requested = True

    try:
        for signum in managed_signals:
            previous_handlers[signum] = signal.getsignal(signum)
            signal.signal(signum, request_orderly_shutdown)
        # Keep the ROS context alive for orderly shutdown and cleanup of
        # component process groups started by this bringup.
        if SignalHandlerOptions is None:
            rclpy.init(args=ros_args)
        else:
            rclpy.init(
                args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
        manager = RealStartupManager(args)
        # Foxy installs its SIGINT guard when the global executor is created.
        # Re-apply our orderly-shutdown handlers afterward so Ctrl-C leaves
        # the ROS context alive while owned component processes are cleaned up.
        rclpy.get_global_executor()
        for signum in managed_signals:
            signal.signal(signum, request_orderly_shutdown)
        if signal_state['requested']:
            manager._shutdown_requested = True
        try:
            if not manager._shutdown_requested:
                manager.run_startup()
        except ShutdownRequested:
            manager.get_logger().info('shutdown requested during startup')
        except StartupBlocked as exc:
            manager.startup_blocked = True
            manager.component_state['startup'] = 'BLOCKED'
            manager.get_logger().fatal(
                f'STARTUP BLOCKED: {exc}. Previously started components remain '
                'running; no later phase will be started.')
            manager._dashboard()
        except Exception as exc:
            manager.startup_blocked = True
            manager.component_state['startup'] = 'BLOCKED'
            manager.get_logger().fatal(
                f'STARTUP BLOCKED by an unexpected error: {exc}. '
                'Previously started components remain running.')
            manager._dashboard()
        while rclpy.ok() and not manager._shutdown_requested:
            rclpy.spin_once(manager, timeout_sec=0.2)
            manager.monitor()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if manager is not None:
            try:
                manager.cleanup_owned()
            except Exception as exc:
                manager.get_logger().error(
                    f'owned-process cleanup raised unexpectedly: {exc}')
            finally:
                try:
                    manager.destroy_node()
                except Exception:
                    pass
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
