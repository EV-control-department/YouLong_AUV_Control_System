"""Camera metadata bridge for the Stonefish shared-memory image path.

Stonefish RGB payloads are written to POSIX shared-memory rings and consumed
by ``uv_camera`` directly.  This adapter relays only the small ``CameraInfo``
messages that describe calibration.  It does not subscribe to, or publish,
any ``sensor_msgs/Image`` topic.
"""

from __future__ import annotations

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image

from auv_protocol.topics import (
    SIM_RAW_FRONT_LEFT_INFO, FRONT_LEFT_INFO,
    SIM_RAW_FRONT_RIGHT_INFO, FRONT_RIGHT_INFO,
    SIM_RAW_DOWN_LEFT_INFO, DOWN_LEFT_INFO,
    SIM_RAW_DOWN_RIGHT_INFO, DOWN_RIGHT_INFO,
)


STEREO_STITCH_SLOP_SEC = 0.04
# Stonefish renders the two front eyes serially at the current render budget.
FRONT_STEREO_STITCH_SLOP_SEC = 0.12


class CameraPassthrough:
    """Relay camera calibration metadata; image bytes stay out of DDS."""

    def __init__(self, stitch_fps=10.0, publish_raw_views=False):
        # Keep constructor arguments for launch and downstream compatibility.
        del stitch_fps
        self._publish_raw_views = (
            publish_raw_views if isinstance(publish_raw_views, bool) else
            str(publish_raw_views).strip().lower()
            in ('1', 'true', 'yes', 'on'))
        self._image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

    def bind(self, node):
        """Wire only four CameraInfo relays into the ROS graph."""
        self.node = node
        info_pairs = (
            (SIM_RAW_FRONT_LEFT_INFO, FRONT_LEFT_INFO),
            (SIM_RAW_FRONT_RIGHT_INFO, FRONT_RIGHT_INFO),
            (SIM_RAW_DOWN_LEFT_INFO, DOWN_LEFT_INFO),
            (SIM_RAW_DOWN_RIGHT_INFO, DOWN_RIGHT_INFO),
        )
        for source, target in info_pairs:
            publisher = node.create_publisher(
                CameraInfo, target, self._image_qos)
            node.create_subscription(
                CameraInfo, source, publisher.publish, self._image_qos)

    @staticmethod
    def _stamp_seconds(message: Image) -> float:
        stamp = message.header.stamp
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @classmethod
    def _synchronized_pair_key(cls, left_message: Image | None,
                               right_message: Image | None,
                               slop_sec: float = STEREO_STITCH_SLOP_SEC):
        """Return a unique timing key for compatibility checks.

        The runtime shared-memory reader performs the actual eye pairing.  This
        helper remains available to older tests and downstream code that used
        the previous bridge's timestamp policy.
        """
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
