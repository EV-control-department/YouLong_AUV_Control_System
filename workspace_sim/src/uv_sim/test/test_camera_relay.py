"""``camera_relay``: stitching correctness and the one-stitch-per-frame rule.

A5-5 is the width check (``stitched == 2 x 单目宽``).  The rate rule is the other
half: the pre-T5 monolith stitched and published inside *both* the left and the
right callback, so every frame pair produced two identical stitched images (about
19 Hz for a ~9.5 Hz per-eye stream).  The relay stitches on a timer with a dirty
flag instead, so the unique frame rate is unchanged while the topic rate and the
stitching CPU are halved.

Publishers are replaced with recorders: the assertions are about *what* would go
out and *how often*, and putting a real DDS subscriber in the loop would only make
the test slower and timing-dependent.
"""

from __future__ import annotations

import numpy as np
import pytest
import rclpy
from cv_bridge import CvBridge
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image

from uv_sim.camera_relay import CameraRelayNode


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


class _Recorder:
    def __init__(self) -> None:
        self.messages: list = []

    def publish(self, msg) -> None:
        self.messages.append(msg)


def _node(ros_context, name: str, log_rates: bool = False) -> CameraRelayNode:
    overrides = [Parameter("log_rates", value=log_rates)] if log_rates else None
    return CameraRelayNode(node_name=name, parameter_overrides=overrides)


def _image(bridge: CvBridge, width: int, height: int = 4, stamp_sec: int = 1) -> Image:
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    msg = bridge.cv2_to_imgmsg(frame, "bgr8")
    msg.header.frame_id = "front_cam_left"
    msg.header.stamp.sec = stamp_sec
    return msg


def _install_recorders(node: CameraRelayNode) -> dict[str, _Recorder]:
    recorders = {
        "front_left": _Recorder(), "front_right": _Recorder(),
        "down_left": _Recorder(), "down_right": _Recorder(),
        "front_stitched": _Recorder(), "down_stitched": _Recorder(),
    }
    node.front_rect_left_pub = recorders["front_left"]
    node.front_rect_right_pub = recorders["front_right"]
    node.down_rect_left_pub = recorders["down_left"]
    node.down_rect_right_pub = recorders["down_right"]
    node.front_rect_pub = recorders["front_stitched"]
    node.down_rect_pub = recorders["down_stitched"]
    return recorders


# --------------------------------------------------------------------------- #
# A5-5: stitching
# --------------------------------------------------------------------------- #
def test_stitched_width_is_the_sum_of_the_two_eyes(ros_context) -> None:
    node = _node(ros_context, "camera_relay_stitch")
    bridge = CvBridge()
    try:
        recorders = _install_recorders(node)
        left = _image(bridge, width=8)
        right = _image(bridge, width=8)

        node._front_left_img_cb(left)
        node._front_right_img_cb(right)
        node._relay_tick()

        stitched = recorders["front_stitched"].messages
        assert len(stitched) == 1
        assert stitched[0].width == left.width + right.width
        assert stitched[0].width == 2 * left.width
        assert stitched[0].height == left.height
        # the left eye is the reference for the header
        assert stitched[0].header.frame_id == left.header.frame_id
        assert stitched[0].header.stamp.sec == left.header.stamp.sec
    finally:
        node.destroy_node()


def test_per_eye_images_are_forwarded_unchanged(ros_context) -> None:
    node = _node(ros_context, "camera_relay_forward")
    bridge = CvBridge()
    try:
        recorders = _install_recorders(node)
        front_left = _image(bridge, width=8)
        down_right = _image(bridge, width=6)

        node._front_left_img_cb(front_left)
        node._down_right_img_cb(down_right)

        assert recorders["front_left"].messages == [front_left]
        assert recorders["down_right"].messages == [down_right]
    finally:
        node.destroy_node()


def test_no_stitch_until_both_eyes_have_arrived(ros_context) -> None:
    node = _node(ros_context, "camera_relay_wait")
    bridge = CvBridge()
    try:
        recorders = _install_recorders(node)
        node._front_left_img_cb(_image(bridge, width=8))
        node._relay_tick()
        node._relay_tick()

        assert recorders["front_stitched"].messages == []
    finally:
        node.destroy_node()


def test_one_stitch_per_frame_pair(ros_context) -> None:
    """Two ticks with no new frames must not re-publish (the pre-T5 behaviour did)."""
    node = _node(ros_context, "camera_relay_dirty")
    bridge = CvBridge()
    try:
        recorders = _install_recorders(node)
        node._front_left_img_cb(_image(bridge, width=8))
        node._front_right_img_cb(_image(bridge, width=8))

        node._relay_tick()
        node._relay_tick()
        node._relay_tick()

        assert len(recorders["front_stitched"].messages) == 1
    finally:
        node.destroy_node()


def test_down_pair_is_stitched_independently(ros_context) -> None:
    node = _node(ros_context, "camera_relay_down")
    bridge = CvBridge()
    try:
        recorders = _install_recorders(node)
        node._down_left_img_cb(_image(bridge, width=5))
        node._down_right_img_cb(_image(bridge, width=7))
        node._relay_tick()

        assert recorders["front_stitched"].messages == []
        assert len(recorders["down_stitched"].messages) == 1
        assert recorders["down_stitched"].messages[0].width == 12
    finally:
        node.destroy_node()


def test_a_mismatched_pair_logs_instead_of_crashing(ros_context) -> None:
    """Different heights make hstack raise; the node must survive and keep serving."""
    node = _node(ros_context, "camera_relay_mismatch")
    bridge = CvBridge()
    try:
        recorders = _install_recorders(node)
        node._front_left_img_cb(_image(bridge, width=8, height=4))
        node._front_right_img_cb(_image(bridge, width=8, height=6))
        node._relay_tick()

        assert recorders["front_stitched"].messages == []
    finally:
        node.destroy_node()


# --------------------------------------------------------------------------- #
# Writer-side rate instrumentation
# --------------------------------------------------------------------------- #
def test_publish_counters_track_every_topic(ros_context) -> None:
    """The counter is the second, trustworthy camera-rate measurement channel."""
    node = _node(ros_context, "camera_relay_counts", log_rates=True)
    bridge = CvBridge()
    try:
        _install_recorders(node)
        node._front_left_img_cb(_image(bridge, width=8))
        node._front_right_img_cb(_image(bridge, width=8))
        node._relay_tick()

        assert node._counts == {
            "/auv/front_cam/left": 1,
            "/auv/front_cam/right": 1,
            "/auv/front_cam/stitched": 1,
        }
        node._log_publish_rates()
        assert node._counts == {
            "/auv/front_cam/left": 0,
            "/auv/front_cam/right": 0,
            "/auv/front_cam/stitched": 0,
        }
    finally:
        node.destroy_node()
