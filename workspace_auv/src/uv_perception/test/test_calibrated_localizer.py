import time
import numpy as np
import pytest
from uv_msgs.msg import DetectionArray
from uv_camera.image_geometry import EyeUndistorter, CalibrationCache, normalized_pixel
from test_localizer_transform import _localizer, _camera_info, _detection
from uv_perception.object_localizer_static import PendingPair


def test_corrected_localization_uses_matched_zero_distortion_camera():
    localizer=_localizer();localizer._calibrations=CalibrationCache()
    raw=_camera_info();raw.k[1]=1.2;raw.d=[-.3,.1,.002,0.,0.]
    transform=EyeUndistorter(raw,4);localizer._calibrations.add(transform.message('front_left'))
    message=DetectionArray(camera_name='front_left',image_space=1,
        calibration_id=transform.calibration_id,camera_info_version=4,detections=[_detection()])
    message.header.frame_id='front_left_camera_optical_frame';message.header.stamp.sec=42
    pending=PendingPair(time.monotonic_ns(),{'front_left':message},{'front_left':[1]})
    localizer._publish_pending(pending,{'front_left':raw})
    ray=localizer.publisher.messages[-1].measurements[0]
    expected=np.array([*normalized_pixel(raw.k,[],600.,400.),1.]);expected/=np.linalg.norm(expected)
    assert [ray.ray_direction_x,ray.ray_direction_y,ray.ray_direction_z]==pytest.approx(expected)
    before=len(localizer.publisher.messages);message.calibration_id+=1
    localizer._publish_pending(pending,{'front_left':raw})
    assert len(localizer.publisher.messages)==before
