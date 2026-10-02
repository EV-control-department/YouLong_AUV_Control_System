"""The mapping frontend keeps image processing within uv_camera."""

from types import SimpleNamespace

import numpy as np
from sensor_msgs.msg import CameraInfo
from uv_msgs.msg import Detection, DetectionArray, PoseInfo

from uv_camera.mapping_vision import MappingVision


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Node:
    def __init__(self):
        self.publisher = _Publisher()
        self.params = {
            'mapping_left_translation': [-0.13, -0.05, 0.2645],
            'mapping_right_translation': [-0.13, 0.05, 0.2645],
            'mapping_camera_rotation': [0., -1., 0., 1., 0., 0., 0., 0., 1.],
            'mapping_min_depth_m': 0.2,
            'mapping_max_depth_m': 8.0,
            'mapping_min_depth_points': 20,
            'mapping_depth_bin_m': 0.02,
            'mapping_depth_peak_ratio': 0.12,
            'mapping_min_confidence': 0.35,
            'mapping_max_pose_age_s': 1.0,
            'mapping_tag_id': -1,
            'mapping_allowed_tag_ids': [0, 1, 2, 3, 4, 5, 6],
            'mapping_tag_dictionary': 'DICT_APRILTAG_16h5',
            'mapping_sgbm_block_size': 5,
            'mapping_sgbm_num_disparities': 128,
        }

    def create_publisher(self, *_args):
        return self.publisher

    def create_subscription(self, *_args):
        return object()

    def get_parameter(self, name):
        return SimpleNamespace(value=self.params[name])

    def get_logger(self):
        return SimpleNamespace(info=lambda *_: None, warning=lambda *_: None)


def _info():
    info = CameraInfo()
    info.width, info.height = 640, 480
    info.k = [400., 0., 320., 0., 400., 240., 0., 0., 1.]
    info.d = [0., 0., 0., 0., 0.]
    return info


def test_stereo_segmentation_publishes_world_observation():
    node = _Node()
    vision = MappingVision(node, sim_mode=True)
    vision._info_cb('left', _info())
    vision._info_cb('right', _info())
    pose = PoseInfo()
    pose.stamp.sec = 10
    pose.robot_x, pose.robot_y, pose.robot_z = 0., 0., 0.
    pose.robot_roll = pose.robot_pitch = pose.robot_yaw = 0.
    vision._pose_cb(pose)

    rng = np.random.default_rng(7)
    gray = rng.integers(0, 256, (480, 640), dtype=np.uint8)
    left = np.repeat(gray[:, :, None], 3, axis=2)
    right = np.empty_like(left)
    right[:, :-8] = left[:, 8:]
    right[:, -8:] = left[:, -1:]
    frame = np.hstack((left, right))
    detection = Detection()
    detection.class_id = 0
    detection.confidence = 0.9
    detection.mask_x = [260., 360., 360., 260.]
    detection.mask_y = [190., 190., 290., 290.]
    left_detections, right_detections = DetectionArray(), DetectionArray()
    left_detections.detections = [detection]
    right_detections.detections = [detection]
    vision.process(frame, pose.stamp, left_detections, right_detections)

    result = node.publisher.messages[-1]
    assert result.processed, result.reason
    cones = [item for item in result.observations if item.kind == item.CONE]
    assert len(cones) == 1
    assert cones[0].class_id == 0
    assert cones[0].depth_samples >= 20
    assert 4.0 < cones[0].depth_m < 6.0
    assert cones[0].header.stamp.sec == 10
    for name in ('input', 'overlay', 'disparity', 'depth', 'histogram'):
        jpeg, stamp = vision.debug_jpeg(name)
        assert jpeg.startswith(b'\xff\xd8')
        assert stamp.sec == 10
