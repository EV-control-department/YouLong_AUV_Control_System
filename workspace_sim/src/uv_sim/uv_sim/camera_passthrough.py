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
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


STEREO_STITCH_SLOP_SEC = 0.04


class CameraPassthrough:
    """Owns camera subscriptions + stitched publishing. Call bind(node) to wire."""

    def __init__(self, stitch_fps=10.0, publish_raw_views=False):
        self.bridge = CvBridge()
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
            self.front_rect_pub, self._last_front_pair_key)

    def _publish_stitched_down(self):
        self._last_down_pair_key = self._publish_stitched(
            "Down", self.down_left_img, self.down_right_img,
            self.down_rect_pub, self._last_down_pair_key)

    @staticmethod
    def _stamp_seconds(message: Image) -> float:
        stamp = message.header.stamp
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @classmethod
    def _synchronized_pair_key(cls, left_message: Image | None,
                               right_message: Image | None):
        """Return a unique pair key only when the two images are contemporaneous."""
        if left_message is None or right_message is None:
            return None
        left_stamp = cls._stamp_seconds(left_message)
        right_stamp = cls._stamp_seconds(right_message)
        if abs(left_stamp - right_stamp) > STEREO_STITCH_SLOP_SEC:
            return None
        return (
            int(left_message.header.stamp.sec),
            int(left_message.header.stamp.nanosec),
            int(right_message.header.stamp.sec),
            int(right_message.header.stamp.nanosec),
        )

    def _publish_stitched(self, camera_name: str, left_message: Image | None,
                          right_message: Image | None, publisher, last_pair_key):
        """Publish each timestamp-matched pair once; never mix adjacent frames."""
        now = time.monotonic()
        if (now - self._last_stitch_time[camera_name]
                < self._stitch_period):
            return last_pair_key
        pair_key = self._synchronized_pair_key(left_message, right_message)
        if pair_key is None or pair_key == last_pair_key:
            return last_pair_key
        try:
            left = self.bridge.imgmsg_to_cv2(left_message, "bgr8")
            right = self.bridge.imgmsg_to_cv2(right_message, "bgr8")
            stitched = np.hstack((left, right))
            out = self.bridge.cv2_to_imgmsg(stitched, "bgr8")
            out.header = left_message.header
            publisher.publish(out)
            self._last_stitch_time[camera_name] = now
            return pair_key
        except Exception as error:
            self.node.get_logger().error(f"{camera_name} stitch failed: {error}")
            return last_pair_key
