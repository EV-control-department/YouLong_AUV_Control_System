"""Camera relay: Stonefish's four image topics -> the stack's ``/auv/*_cam/*``.

Two jobs, one node:

1. **Per-eye republish.**  ``/sim/{front,down}_cam/{left,right}/image_color`` go
   out unchanged as ``/auv/{front,down}_cam/{left,right}``.  Same message, same
   stamp: re-stamping would fake new samples (which is how the pre-T5 image-rate
   measurements became inconsistent, see ``workspace_sim/stonefish_notes.md``).
2. **Horizontal stitch.**  ``/auv/{front,down}_cam/stitched`` is the left+right
   pair hstacked (``2 x 单目宽``), which is what ``vision`` subscribes to in
   ``sim_mode``.

The stitch runs on a 100 Hz timer with a **dirty flag**, not inside the image
callbacks: before T5 both the left and the right callback stitched and published,
so every frame pair produced two identical stitched images (≈19 Hz for a ≈9.5 Hz
per-eye stream).  Now each new frame produces at most one stitch, so the *unique*
frame rate is unchanged while the stitching CPU and the topic rate are halved.
That is an intentional rate change on an existing topic -- names and types are
untouched (A5-2) -- and it is recorded in the devlog section 9.

``log_rates:=true`` prints a 1 Hz writer-side counter for all six published topics.
That is the second half of the camera-rate measurement problem: a Python
subscriber on a large RELIABLE image topic undercounts badly, so the number to
trust is the count on the publishing side (this log) or the minimum stamp interval.
"""

from __future__ import annotations

import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from uv_sim import runner

#: Stitch timer: fast enough that the camera frame rate is the limit, not this.
RELAY_PERIOD_SECONDS = 0.01
RATE_LOG_PERIOD_SECONDS = 1.0

#: The two stereo pairs and their frame keys.
PAIRS = ('front', 'down')


class CameraRelayNode(Node):
    def __init__(self, node_name: str = 'camera_relay', **kwargs) -> None:
        super().__init__(node_name, **kwargs)

        self.declare_parameter('log_rates', False)
        self._log_rates = bool(self.get_parameter('log_rates').value)

        self.bridge = CvBridge()
        self._frames: dict[str, Image | None] = {
            'front_left': None, 'front_right': None,
            'down_left': None, 'down_right': None,
        }
        self._dirty: set[str] = set()
        self._counts: dict[str, int] = {}
        self._last_rate_log = self._now_seconds()

        # Publishers: the per-eye images keep the engine's frame, the stitched one
        # carries the left image's header (left is the reference eye).
        self.front_rect_left_pub = self.create_publisher(Image, "/auv/front_cam/left", 10)
        self.front_rect_right_pub = self.create_publisher(Image, "/auv/front_cam/right", 10)
        self.down_rect_left_pub = self.create_publisher(Image, "/auv/down_cam/left", 10)
        self.down_rect_right_pub = self.create_publisher(Image, "/auv/down_cam/right", 10)
        self.front_rect_pub = self.create_publisher(Image, "/auv/front_cam/stitched", 10)
        self.down_rect_pub = self.create_publisher(Image, "/auv/down_cam/stitched", 10)

        # Subscriptions: literal topic strings on purpose (the docs contract
        # checker matches them in the AST, so an f-string would hide them).
        self.create_subscription(
            Image, "/sim/front_cam/left/image_color", self._front_left_img_cb, 10)
        self.create_subscription(
            Image, "/sim/front_cam/right/image_color", self._front_right_img_cb, 10)
        self.create_subscription(
            Image, "/sim/down_cam/left/image_color", self._down_left_img_cb, 10)
        self.create_subscription(
            Image, "/sim/down_cam/right/image_color", self._down_right_img_cb, 10)

        self.create_timer(RELAY_PERIOD_SECONDS, self._relay_tick)
        if self._log_rates:
            self.create_timer(RATE_LOG_PERIOD_SECONDS, self._log_publish_rates)

    # ── inputs ──────────────────────────────────────────────────────────────

    def _front_left_img_cb(self, msg: Image) -> None:
        self._publish_image(self.front_rect_left_pub, '/auv/front_cam/left', msg)
        self._note_frame('front_left', 'front', msg)

    def _front_right_img_cb(self, msg: Image) -> None:
        self._publish_image(self.front_rect_right_pub, '/auv/front_cam/right', msg)
        self._note_frame('front_right', 'front', msg)

    def _down_left_img_cb(self, msg: Image) -> None:
        self._publish_image(self.down_rect_left_pub, '/auv/down_cam/left', msg)
        self._note_frame('down_left', 'down', msg)

    def _down_right_img_cb(self, msg: Image) -> None:
        self._publish_image(self.down_rect_right_pub, '/auv/down_cam/right', msg)
        self._note_frame('down_right', 'down', msg)

    def _note_frame(self, key: str, pair: str, msg: Image) -> None:
        self._frames[key] = msg
        self._dirty.add(pair)

    # ── output ──────────────────────────────────────────────────────────────

    def _relay_tick(self) -> None:
        """Stitch every pair that has a new frame (at most once per period)."""
        for pair in PAIRS:
            if pair in self._dirty:
                self._dirty.discard(pair)
                self._stitch_pair(pair)

    def _stitch_pair(self, pair: str) -> None:
        left = self._frames[f'{pair}_left']
        right = self._frames[f'{pair}_right']
        if left is None or right is None:
            return  # wait for both eyes
        try:
            left_array = self.bridge.imgmsg_to_cv2(left, "bgr8")
            right_array = self.bridge.imgmsg_to_cv2(right, "bgr8")
            stitched = np.hstack((left_array, right_array))
            out = self.bridge.cv2_to_imgmsg(stitched, "bgr8")
            out.header = left.header
            self._publish_image(self._stitched_publisher(pair),
                                f'/auv/{pair}_cam/stitched', out)
        except Exception as error:
            self.get_logger().error(f"{pair.capitalize()} stitch failed: {error}")

    def _stitched_publisher(self, pair: str):
        return self.front_rect_pub if pair == 'front' else self.down_rect_pub

    def _publish_image(self, publisher, topic: str, msg: Image) -> None:
        publisher.publish(msg)
        self._counts[topic] = self._counts.get(topic, 0) + 1

    # ── optional rate instrumentation ───────────────────────────────────────

    def _log_publish_rates(self) -> None:
        now = self._now_seconds()
        elapsed = now - self._last_rate_log
        self._last_rate_log = now
        if elapsed <= 0.0:
            return
        rates = ", ".join(
            f"{topic}={count / elapsed:.2f}Hz"
            for topic, count in sorted(self._counts.items())
        )
        self.get_logger().info(f"camera relay publish rates (writer side): {rates}")
        self._counts = {topic: 0 for topic in self._counts}

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None) -> int:
    return runner.run(CameraRelayNode, args)


if __name__ == "__main__":
    raise SystemExit(main())
