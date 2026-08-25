"""Camera passthrough: republish Stonefish stereo camera images + hstack stitch.

从原 sim_bridge 抽出,行为保持一致:
- 订阅 /sim/front_cam/{left,right}/image_color, /sim/down_cam/{left,right}/image_color
- 重发布为 /auv/front_cam/{left,right}, /auv/down_cam/{left,right}
- 左右拼接后发布 /auv/front_cam/stitched, /auv/down_cam/stitched
供 uv_camera 使用,与控制逻辑正交。
"""

import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class CameraPassthrough:
    """Owns camera subscriptions + stitched publishing. Call bind(node) to wire."""

    def __init__(self):
        self.bridge = CvBridge()
        self.front_left_img = None
        self.front_right_img = None
        self.down_left_img = None
        self.down_right_img = None

    def bind(self, node):
        self.node = node

        self.front_rect_left_pub = node.create_publisher(Image, "/auv/front_cam/left", 10)
        self.front_rect_right_pub = node.create_publisher(Image, "/auv/front_cam/right", 10)
        self.down_rect_left_pub = node.create_publisher(Image, "/auv/down_cam/left", 10)
        self.down_rect_right_pub = node.create_publisher(Image, "/auv/down_cam/right", 10)
        self.front_rect_pub = node.create_publisher(Image, "/auv/front_cam/stitched", 10)
        self.down_rect_pub = node.create_publisher(Image, "/auv/down_cam/stitched", 10)

        node.create_subscription(Image, "/sim/front_cam/left/image_color", self._front_left_img_cb, 10)
        node.create_subscription(Image, "/sim/front_cam/right/image_color", self._front_right_img_cb, 10)
        node.create_subscription(Image, "/sim/down_cam/left/image_color", self._down_left_img_cb, 10)
        node.create_subscription(Image, "/sim/down_cam/right/image_color", self._down_right_img_cb, 10)

    def _front_left_img_cb(self, msg):
        self.front_rect_left_pub.publish(msg)
        self.front_left_img = msg
        self._publish_stitched_front()

    def _front_right_img_cb(self, msg):
        self.front_rect_right_pub.publish(msg)
        self.front_right_img = msg
        self._publish_stitched_front()

    def _down_left_img_cb(self, msg):
        self.down_rect_left_pub.publish(msg)
        self.down_left_img = msg
        self._publish_stitched_down()

    def _down_right_img_cb(self, msg):
        self.down_rect_right_pub.publish(msg)
        self.down_right_img = msg
        self._publish_stitched_down()

    def _publish_stitched_front(self):
        if self.front_left_img and self.front_right_img:
            try:
                left = self.bridge.imgmsg_to_cv2(self.front_left_img, "bgr8")
                right = self.bridge.imgmsg_to_cv2(self.front_right_img, "bgr8")
                stitched = np.hstack((left, right))
                out = self.bridge.cv2_to_imgmsg(stitched, "bgr8")
                out.header = self.front_left_img.header
                self.front_rect_pub.publish(out)
            except Exception as e:
                self.node.get_logger().error(f"Front stitch failed: {e}")

    def _publish_stitched_down(self):
        if self.down_left_img and self.down_right_img:
            try:
                left = self.bridge.imgmsg_to_cv2(self.down_left_img, "bgr8")
                right = self.bridge.imgmsg_to_cv2(self.down_right_img, "bgr8")
                stitched = np.hstack((left, right))
                out = self.bridge.cv2_to_imgmsg(stitched, "bgr8")
                out.header = self.down_left_img.header
                self.down_rect_pub.publish(out)
            except Exception as e:
                self.node.get_logger().error(f"Down stitch failed: {e}")
