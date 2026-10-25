"""Read one catalog-validated Iceoryx2 camera and emit H.265 MPEG-TS on stdout."""

from __future__ import annotations

import argparse
from fractions import Fraction
import os
import queue
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sl_msgs.msg import CameraCatalog

from uv_image_transport import ENCODING_JPEG, Iceoryx2Error, Iceoryx2Reader, InvalidFrameError


CATALOG_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
CATALOG_TIMEOUT_SEC = 5.0
GST_NSECOND = 1_000_000_000


def gst_pipeline_description(width, height, fps, bitrate, max_bitrate,
                             backend, output_fd):
    """Build a hardware or software GStreamer pipeline for one camera stream."""
    width, height = int(width), int(height)
    rate = Fraction(max(0.1, float(fps))).limit_denominator(1000)
    key_interval = max(1, int(round(float(fps))))
    frame_bytes = width * height * 3
    source = (
        'appsrc name=camera_source is-live=true format=time block=true '
        f'max-bytes={frame_bytes * 2} do-timestamp=false '
        f'caps="video/x-raw,format=BGR,width={width},height={height},'
        f'framerate={rate.numerator}/{rate.denominator}" '
    )
    convert = '! videoconvert ! video/x-raw,format=I420 '
    if backend == 'jetson':
        encode = (
            '! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 '
            f'! nvv4l2h265enc control-rate=0 bitrate={int(bitrate)} '
            f'peak-bitrate={int(max_bitrate)} iframeinterval={key_interval} '
            f'idrinterval={key_interval} num-B-Frames=0 insert-sps-pps=true '
        )
    else:
        target_kbps = max(1, int(round(int(bitrate) / 1000)))
        peak_kbps = max(target_kbps, int(round(int(max_bitrate) / 1000)))
        options = (
            f'vbv-maxrate={peak_kbps}:vbv-bufsize={peak_kbps}:'
            'repeat-headers=1:bframes=0:hrd=1'
        )
        encode = (
            f'! x265enc bitrate={target_kbps} key-int-max={key_interval} '
            'speed-preset=ultrafast tune=zerolatency '
            f'option-string="{options}" '
        )
    return (
        source + convert + encode
        + '! h265parse config-interval=-1 ! mpegtsmux '
        + f'! fdsink fd={int(output_fd)} sync=false'
    )


class CatalogWatch(Node):
    def __init__(self, args):
        super().__init__(f'sl_stream_camera_{int(args.camera_id)}')
        self.args = args
        self.lock = threading.RLock()
        self.message = None
        self.last_update = 0.0
        self.changed = threading.Event()
        self.reader = None
        self.subscription = self.create_subscription(
            CameraCatalog, args.catalog_topic, self._on_catalog, CATALOG_QOS)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self)
        self.thread = threading.Thread(
            target=self._spin, name=f'sl-stream-catalog-{args.camera_id}', daemon=True)
        self.thread.start()

    def _spin(self):
        try:
            while rclpy.ok(context=self.context) and self.executor:
                self.executor.spin_once(timeout_sec=0.1)
        except (ExternalShutdownException, RuntimeError):
            pass

    def _on_catalog(self, message):
        with self.lock:
            self.message = message
            self.last_update = time.monotonic()
            camera = next((item for item in message.cameras
                           if int(item.id) == int(self.args.camera_id)), None)
            valid_source = (
                camera is not None
                and bool(camera.enabled)
                and str(message.source_instance_id) == self.args.source_instance_id
                and int(message.config_version) == int(self.args.config_version)
                and str(camera.ice_service) == self.args.ice_service
                and int(camera.width) == int(self.args.width)
                and int(camera.height) == int(self.args.height)
            )
            if not valid_source or (not camera.online and self.reader is not None):
                self.changed.set()
                if self.reader is not None:
                    self.reader.request_stop()

    def current_camera(self, require_online=True):
        with self.lock:
            if self.message is None or time.monotonic() - self.last_update > CATALOG_TIMEOUT_SEC:
                return None
            camera = next((item for item in self.message.cameras
                           if int(item.id) == int(self.args.camera_id)), None)
            if camera is None:
                return None
            if (not camera.enabled
                    or str(self.message.source_instance_id) != self.args.source_instance_id
                    or int(self.message.config_version) != int(self.args.config_version)
                    or str(camera.ice_service) != self.args.ice_service
                    or int(camera.width) != int(self.args.width)
                    or int(camera.height) != int(self.args.height)):
                return None
            if require_online and not camera.online:
                return None
            return camera

    def generation_matches(self):
        return self.current_camera(require_online=False) is not None

    def catalog_is_fresh(self):
        with self.lock:
            return (self.message is not None
                    and time.monotonic() - self.last_update <= CATALOG_TIMEOUT_SEC)

    def close(self):
        with self.lock:
            reader = self.reader
            self.reader = None
        if reader is not None:
            reader.request_stop()
        self.executor.remove_node(self)
        self.executor.shutdown(timeout_sec=1.0)
        self.thread.join(timeout=2.0)
        if self.thread.is_alive():
            raise TimeoutError('camera catalog subscription did not stop')
        self.destroy_node()


def _load_gstreamer(backend=None):
    try:
        import gi
        gi.require_version('Gst', '1.0')
        from gi.repository import Gst
    except Exception as error:
        raise RuntimeError(f'GStreamer Python bindings are unavailable: {error}') from error
    Gst.init(None)
    if backend == 'jetson':
        if not Gst.ElementFactory.find('nvv4l2h265enc'):
            raise RuntimeError('Jetson H.265 encoder nvv4l2h265enc is unavailable')
        if not Gst.ElementFactory.find('nvvidconv'):
            raise RuntimeError('Jetson video converter nvvidconv is unavailable')
    elif backend == 'software' and not Gst.ElementFactory.find('x265enc'):
        raise RuntimeError('software H.265 encoder x265enc is unavailable')
    return Gst


def _make_pipeline(Gst, args, backend, output):
    description = gst_pipeline_description(
        args.width, args.height, args.fps, args.bitrate,
        args.max_bitrate, backend, output.fileno())
    try:
        pipeline = Gst.parse_launch(description)
    except Exception as error:
        raise RuntimeError(f'could not create {backend} H.265 pipeline: {error}') from error
    source = pipeline.get_by_name('camera_source')
    if source is None:
        pipeline.set_state(Gst.State.NULL)
        raise RuntimeError('GStreamer pipeline did not expose camera_source')
    bus = pipeline.get_bus()
    result = pipeline.set_state(Gst.State.PLAYING)
    if result == Gst.StateChangeReturn.FAILURE:
        pipeline.set_state(Gst.State.NULL)
        raise RuntimeError(f'{backend} H.265 pipeline failed to start')
    return pipeline, source, bus


def _open_video_output():
    """Keep native ROS/GStreamer logs out of go2rtc's encoded media pipe."""
    sys.stdout.flush()
    video_fd = os.dup(sys.stdout.fileno())
    try:
        os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
        return os.fdopen(video_fd, 'wb', buffering=0)
    except BaseException:
        os.close(video_fd)
        raise


def _select_backend(Gst, requested):
    if requested == 'jetson':
        _load_gstreamer('jetson')
        return 'jetson'
    if requested == 'software':
        _load_gstreamer('software')
        return 'software'
    if requested != 'auto':
        raise ValueError('encoder must be auto, jetson, or software')
    try:
        if (Gst.ElementFactory.find('nvv4l2h265enc')
                and Gst.ElementFactory.find('nvvidconv')):
            return 'jetson'
    except Exception:
        pass
    if Gst.ElementFactory.find('x265enc'):
        return 'software'
    raise RuntimeError('no Jetson or software H.265 encoder is available')


def _next_pts(timestamp_ns, state, fps):
    """Map source ROS timestamps to monotonic output PTS across clock resets."""
    duration = max(1, round(GST_NSECOND / max(0.1, fps)))
    timestamp_ns = int(timestamp_ns)
    if timestamp_ns <= 0:
        timestamp_ns = time.monotonic_ns()
    if state['source_base'] is None or timestamp_ns <= state['last_source']:
        state['source_base'] = timestamp_ns
        pts = 0 if state['last_pts'] < 0 else state['last_pts'] + duration
    else:
        pts = max(0, timestamp_ns - state['source_base'])
        if state['last_pts'] >= 0:
            pts = max(pts, state['last_pts'] + duration)
    state['last_source'] = timestamp_ns
    state['last_pts'] = pts
    return pts, duration


def _push_frame(Gst, source, image, pts, duration):
    if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
        raise InvalidFrameError('camera JPEG did not decode to a 3-channel BGR image')
    buffer = Gst.Buffer.new_allocate(None, int(image.nbytes), None)
    buffer.fill(0, image.tobytes())
    buffer.pts = int(pts)
    buffer.dts = Gst.CLOCK_TIME_NONE
    buffer.duration = int(duration)
    result = source.emit('push-buffer', buffer)
    if result != Gst.FlowReturn.OK:
        raise RuntimeError(f'GStreamer appsrc stopped: {result.value_nick}')


def _pipeline_error(Gst, bus):
    message = bus.pop_filtered(Gst.MessageType.ERROR)
    if message is None:
        return None
    error, debug = message.parse_error()
    return f'{error.message}' + (f' ({debug})' if debug else '')


def _wait_for_online(watch, timeout=30.0):
    started = time.monotonic()
    deadline = started + timeout
    while time.monotonic() < deadline:
        camera = watch.current_camera()
        if camera:
            return camera
        if not watch.catalog_is_fresh():
            with watch.lock:
                last_update = watch.last_update
            if (last_update and time.monotonic() - last_update > CATALOG_TIMEOUT_SEC
                    or not last_update
                    and time.monotonic() - started > CATALOG_TIMEOUT_SEC):
                raise RuntimeError('no fresh sl_camera catalog was received')
        time.sleep(0.1)
    if not watch.catalog_is_fresh():
        raise RuntimeError('no fresh sl_camera catalog was received')
    raise RuntimeError(f'camera {watch.args.camera_id} did not become online in time')


def _run(args, output):
    Gst = _load_gstreamer()
    backend = _select_backend(Gst, args.encoder)
    camera_desc = CameraDescriptorProxy(args)
    camera = None
    watch = CatalogWatch(args)
    pipeline = source = bus = reader = None
    queue_frames = queue.Queue(maxsize=1)
    pts_state = {'source_base': None, 'last_source': -1, 'last_pts': -1}
    stop_event = threading.Event()
    producer_error = []

    try:
        node_camera = _wait_for_online(watch)
        camera_desc.frame_rate = _catalog_frame_rate(node_camera, args.max_fps)
        pipeline, source, bus = _make_pipeline(Gst, args, backend, output)
        print(
            f'camera_streamer: camera {args.camera_id} using {backend} H.265 '
            f'at {camera_desc.frame_rate:.2f} fps, {args.width}x{args.height}; '
            f'{args.bitrate}/{args.max_bitrate} bps',
            file=sys.stderr, flush=True)

        reader = Iceoryx2Reader(args.ice_service)
        with watch.lock:
            watch.reader = reader

        def feed_encoder():
            try:
                while not stop_event.is_set():
                    try:
                        packet = queue_frames.get(timeout=0.1)
                    except queue.Empty:
                        if watch.changed.is_set() or not watch.catalog_is_fresh():
                            return
                        error = _pipeline_error(Gst, bus)
                        if error:
                            raise RuntimeError(error)
                        continue
                    pts, duration = _next_pts(
                        packet.header.timestamp_ns, pts_state, camera_desc.frame_rate)
                    _push_frame(Gst, source, packet.bgr(), pts, duration)
            except Exception as error:
                producer_error.append(error)
                stop_event.set()
                reader.request_stop()

        encoder_thread = threading.Thread(
            target=feed_encoder, name=f'sl-stream-encoder-{args.camera_id}', daemon=True)
        encoder_thread.start()
        interval_ns = max(1, round(GST_NSECOND / camera_desc.frame_rate))
        last_input_ts = None
        while not stop_event.is_set():
            if watch.changed.is_set() or not watch.generation_matches():
                raise RuntimeError('camera catalog changed, camera went offline, or was disabled')
            if not watch.catalog_is_fresh():
                raise RuntimeError('camera catalog stopped updating')
            error = _pipeline_error(Gst, bus)
            if error:
                raise RuntimeError(error)
            try:
                packet = reader.read()
            except InvalidFrameError:
                continue
            except Iceoryx2Error as error:
                if watch.changed.is_set():
                    raise RuntimeError('camera configuration changed') from error
                raise
            if packet is None:
                break
            packet.validate()
            header = packet.header
            if (int(header.camera_group) != int(args.camera_id)
                    or int(header.camera_info_version) != int(args.config_version)
                    or int(header.width) != int(args.width)
                    or int(header.height) != int(args.height)
                    or int(header.encoding) != ENCODING_JPEG):
                raise InvalidFrameError('discarding an Iceoryx2 frame from a different camera generation')
            timestamp = int(header.timestamp_ns) or time.monotonic_ns()
            if (last_input_ts is not None and timestamp > last_input_ts
                    and timestamp - last_input_ts < interval_ns):
                continue
            last_input_ts = timestamp
            try:
                queue_frames.put_nowait(packet)
            except queue.Full:
                try:
                    queue_frames.get_nowait()
                except queue.Empty:
                    pass
                queue_frames.put_nowait(packet)
            if producer_error:
                raise RuntimeError(f'H.265 encoder failed: {producer_error[0]}')
        stop_event.set()
        reader.request_stop()
        encoder_thread.join(timeout=2.0)
        if producer_error:
            raise RuntimeError(f'H.265 encoder failed: {producer_error[0]}')
        return 0
    finally:
        stop_event.set()
        if reader is not None:
            reader.request_stop()
        try:
            queue_frames.put_nowait(None)
        except queue.Full:
            try:
                queue_frames.get_nowait()
            except queue.Empty:
                pass
            queue_frames.put_nowait(None)
        if 'encoder_thread' in locals():
            encoder_thread.join(timeout=2.0)
        if pipeline is not None:
            source.emit('end-of-stream')
            pipeline.set_state(Gst.State.NULL)
        with watch.lock:
            watch.reader = None
        if reader is not None:
            reader.close()
        watch.close()


def _catalog_frame_rate(camera, maximum):
    available = float(camera.actual_fps or camera.negotiated_fps or camera.requested_fps)
    if not np.isfinite(available) or available <= 0:
        available = 10.0
    return max(0.1, min(30.0, float(maximum), available))


class CameraDescriptorProxy:
    """Keep only the encoder's resolved source frame rate."""

    def __init__(self, args):
        self.frame_rate = min(args.max_fps, 30.0)


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera-id', type=int, required=True)
    parser.add_argument('--ice-service', required=True)
    parser.add_argument('--width', type=int, required=True)
    parser.add_argument('--height', type=int, required=True)
    parser.add_argument('--catalog-topic', default='/auv/sl_camera/cameras')
    parser.add_argument('--source-instance-id', required=True)
    parser.add_argument('--config-version', type=int, required=True)
    parser.add_argument('--max-fps', type=float, default=30.0)
    parser.add_argument('--bitrate', type=int, default=2_000_000)
    parser.add_argument('--max-bitrate', type=int, default=3_000_000)
    parser.add_argument('--encoder', choices=('auto', 'jetson', 'software'), default='auto')
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    if (args.camera_id <= 0 or args.config_version <= 0 or args.width <= 0
            or args.height <= 0 or args.max_fps <= 0 or args.bitrate <= 0
            or args.max_bitrate < args.bitrate):
        raise SystemExit('camera dimensions, ID, frame rate, and bitrates must be valid')
    with _open_video_output() as output:
        try:
            return _run(args, output)
        except (KeyboardInterrupt, ExternalShutdownException):
            return 0
        except Exception as error:
            print(f'camera_streamer: {error}', file=sys.stderr, flush=True)
            return 1


if __name__ == '__main__':  # pragma: no cover
    raise SystemExit(main())
