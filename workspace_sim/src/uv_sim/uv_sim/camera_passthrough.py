"""Camera passthrough: republish Stonefish stereo camera images + synchronized stitch.

从原 sim_bridge 抽出,行为保持一致:
- 订阅 /sim/front_cam/{left,right}/image_color, /sim/down_cam/{left,right}/image_color
- 重发布为 /auv/front_cam/{left,right}, /auv/down_cam/{left,right}
- 仅将同一时刻的左右图拼接后发布 /auv/front_cam/stitched,
  /auv/down_cam/stitched
供 uv_camera 使用,与控制逻辑正交。
"""

import time

import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

from uv_camera.common import bgr_to_image_msg, image_msg_to_bgr

try:
    from uv_msgs.msg import StereoFrameInfo
except ImportError:  # Keep the simulator importable before uv_msgs is rebuilt.
    StereoFrameInfo = None


STEREO_STITCH_SLOP_SEC = 0.04
# Stonefish renders the two front cameras serially.  Under the current low
# render budget their header stamps are consistently one front-camera period
# apart (~100 ms), even though they are the corresponding stereo views.  Keep
# the normal synchronisation threshold for real/tightly synchronised streams,
# but allow this simulator-specific front-camera offset so the front stream is
# not silently dropped forever.
FRONT_STEREO_STITCH_SLOP_SEC = 0.12


class CameraPassthrough:
    """Owns camera subscriptions + stitched publishing. Call bind(node) to wire."""

    def __init__(self, stitch_fps=10.0, publish_raw_views=False):
        self._publish_raw_views = (
            publish_raw_views if isinstance(publish_raw_views, bool) else
            str(publish_raw_views).strip().lower()
            in ('1', 'true', 'yes', 'on'))
        # Image data is high-rate and disposable.  A deep reliable queue can
        # make the simulator or recorder process stale frames after any short
        # CPU/IO spike, which looks like video lag.  Keep only the newest one.
        self._image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        stitch_fps = float(stitch_fps)
        self._stitch_period = (
            0.0 if stitch_fps <= 0.0 else 1.0 / stitch_fps)
        self._last_stitch_time = {'Front': float('-inf'), 'Down': float('-inf')}
        self._pair_sequence = {'Front': 0, 'Down': 0}
        self.front_left_img = None
        self.front_right_img = None
        self.down_left_img = None
        self.down_right_img = None
        self._last_front_pair_key = None
        self._last_down_pair_key = None

    def bind(self, node):
        self.node = node

        # The perception path consumes only the stitched topics.  Individual
        # camera republishers duplicate several megabytes/s of ROS traffic and
        # are therefore opt-in for diagnostics/legacy consumers.
        self.front_rect_left_pub = None
        self.front_rect_right_pub = None
        self.down_rect_left_pub = None
        self.down_rect_right_pub = None
        if self._publish_raw_views:
            self.front_rect_left_pub = node.create_publisher(
                Image, "/auv/front_cam/left", self._image_qos)
            self.front_rect_right_pub = node.create_publisher(
                Image, "/auv/front_cam/right", self._image_qos)
            self.down_rect_left_pub = node.create_publisher(
                Image, "/auv/down_cam/left", self._image_qos)
            self.down_rect_right_pub = node.create_publisher(
                Image, "/auv/down_cam/right", self._image_qos)
        self.front_rect_pub = node.create_publisher(
            Image, "/auv/front_cam/stitched", self._image_qos)
        self.down_rect_pub = node.create_publisher(
            Image, "/auv/down_cam/stitched", self._image_qos)
        self.front_stereo_info_pub = None
        self.down_stereo_info_pub = None
        if StereoFrameInfo is not None:
            # The metadata is tiny, so it follows the same latest-only policy
            # as the image.  It lets uv_camera preserve the true left/right
            # capture stamps even though the image transport is stitched.
            self.front_stereo_info_pub = node.create_publisher(
                StereoFrameInfo, "/auv/front_cam/stereo_info", self._image_qos)
            self.down_stereo_info_pub = node.create_publisher(
                StereoFrameInfo, "/auv/down_cam/stereo_info", self._image_qos)

        node.create_subscription(
            Image, "/sim/front_cam/left/image_color", self._front_left_img_cb,
            self._image_qos)
        node.create_subscription(
            Image, "/sim/front_cam/right/image_color", self._front_right_img_cb,
            self._image_qos)
        node.create_subscription(
            Image, "/sim/down_cam/left/image_color", self._down_left_img_cb,
            self._image_qos)
        node.create_subscription(
            Image, "/sim/down_cam/right/image_color", self._down_right_img_cb,
            self._image_qos)

    def _front_left_img_cb(self, msg):
        if self.front_rect_left_pub is not None:
            self.front_rect_left_pub.publish(msg)
        self.front_left_img = msg
        self._publish_stitched_front()

    def _front_right_img_cb(self, msg):
        if self.front_rect_right_pub is not None:
            self.front_rect_right_pub.publish(msg)
        self.front_right_img = msg
        self._publish_stitched_front()

    def _down_left_img_cb(self, msg):
        if self.down_rect_left_pub is not None:
            self.down_rect_left_pub.publish(msg)
        self.down_left_img = msg
        self._publish_stitched_down()

    def _down_right_img_cb(self, msg):
        if self.down_rect_right_pub is not None:
            self.down_rect_right_pub.publish(msg)
        self.down_right_img = msg
        self._publish_stitched_down()

    def _publish_stitched_front(self):
        self._last_front_pair_key = self._publish_stitched(
            "Front", self.front_left_img, self.front_right_img,
            self.front_rect_pub, self._last_front_pair_key,
            slop_sec=FRONT_STEREO_STITCH_SLOP_SEC,
            info_publisher=self.front_stereo_info_pub)

    def _publish_stitched_down(self):
        self._last_down_pair_key = self._publish_stitched(
            "Down", self.down_left_img, self.down_right_img,
            self.down_rect_pub, self._last_down_pair_key,
            info_publisher=self.down_stereo_info_pub)

    @staticmethod
    def _stamp_seconds(message: Image) -> float:
        stamp = message.header.stamp
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @classmethod
    def _synchronized_pair_key(cls, left_message: Image | None,
                               right_message: Image | None,
                               slop_sec: float = STEREO_STITCH_SLOP_SEC):
        """Return a unique pair key only when the two images are contemporaneous."""
        if left_message is None or right_message is None:
            return None
        left_stamp = cls._stamp_seconds(left_message)
        right_stamp = cls._stamp_seconds(right_message)
        if abs(left_stamp - right_stamp) > max(0.0, float(slop_sec)):
            return None
        return (
            int(left_message.header.stamp.sec),
            int(left_message.header.stamp.nanosec),
            int(right_message.header.stamp.sec),
            int(right_message.header.stamp.nanosec),
        )

    def _publish_stitched(self, camera_name: str, left_message: Image | None,
                          right_message: Image | None, publisher, last_pair_key,
                          slop_sec: float = STEREO_STITCH_SLOP_SEC,
                          info_publisher=None):
        """Publish each timestamp-matched pair once; never mix adjacent frames."""
        now = time.monotonic()
        if (now - self._last_stitch_time[camera_name]
                < self._stitch_period):
            return last_pair_key
        pair_key = self._synchronized_pair_key(
            left_message, right_message, slop_sec=slop_sec)
        if pair_key is None or pair_key == last_pair_key:
            return last_pair_key
        try:
            left = image_msg_to_bgr(left_message)
            right = image_msg_to_bgr(right_message)
            stitched = np.hstack((left, right))
            out = bgr_to_image_msg(stitched, left_message.header)
            if info_publisher is not None and StereoFrameInfo is not None:
                self._pair_sequence[camera_name] += 1
                info = StereoFrameInfo()
                info.header = out.header
                info.camera_name = camera_name.lower()
                info.left_stamp = left_message.header.stamp
                info.right_stamp = right_message.header.stamp
                info.stereo_pair_id = int(self._pair_sequence[camera_name])
                # Publish metadata first.  uv_sensor can then attach the
                # right-eye timestamp before the stitched image reaches its
                # latest-only frame gate.
                info_publisher.publish(info)
            publisher.publish(out)
            self._last_stitch_time[camera_name] = now
            return pair_key
        except Exception as error:
            self.node.get_logger().error(f"{camera_name} stitch failed: {error}")
            return last_pair_key
