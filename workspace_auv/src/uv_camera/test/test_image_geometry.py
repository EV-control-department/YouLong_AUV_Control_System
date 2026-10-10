from types import SimpleNamespace
import threading
from collections import OrderedDict

import cv2
import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo
from uv_msgs.msg import Detection, DetectionArray
from uv_camera.image_geometry import EyeUndistorter, CalibrationCache, normalized_pixel
from uv_camera.perception_geometry import TaskCameraGeometry


def info(width=80, height=60, skew=1.3, distortion=None):
    value = CameraInfo(width=width, height=height)
    value.k = [100., skew, width/2, 0., 98., height/2, 0., 0., 1.]
    value.d = [-.3, .1, .002, -.001, .03] if distortion is None else distortion
    value.distortion_model = 'plumb_bob'
    return value


def distort(x, y, k, d):
    r2=x*x+y*y
    radial=1+d[0]*r2+d[1]*r2*r2+d[4]*r2*r2*r2
    xd=x*radial+2*d[2]*x*y+d[3]*(r2+2*x*x)
    yd=y*radial+d[2]*(r2+2*y*y)+2*d[3]*x*y
    return k[0,0]*xd+k[0,1]*yd+k[0,2], k[1,1]*yd+k[1,2]


@pytest.mark.parametrize('skew', [0., 1.3, -3.])
def test_mapping_and_rays_match_independent_brown_model(skew):
    correction=EyeUndistorter(info(skew=skew), 7)
    k,d=correction.k,correction.d
    for u,v in [(5,8),(40,30),(73,52)]:
        x,y,_=np.linalg.solve(k,[u,v,1])
        expected=distort(x,y,k,d)
        assert [correction.maps[0][v,u], correction.maps[1][v,u]] == pytest.approx(expected, abs=1e-4)
        assert normalized_pixel(k,d,*expected) == pytest.approx([x,y], abs=1e-6)
        assert normalized_pixel(k,np.zeros(5),u,v) == pytest.approx([x,y], abs=1e-12)
    assert list(correction.image_info.k)==list(correction.source_info.k)
    assert not np.any(correction.image_info.d)
    assert list(correction.image_info.r)==np.eye(3).reshape(-1).tolist()


def test_zero_distortion_does_not_resample_or_drop_skew():
    transform=EyeUndistorter(info(distortion=[0.]*5))
    image=np.arange(80*60*3,dtype=np.uint8).reshape(60,80,3)
    assert transform.apply(image) is image
    assert transform.maps is None


def test_id_ignores_stamps_but_changes_with_intrinsics_and_source_version():
    source=info(); first=EyeUndistorter(source,3)
    source.header.stamp.sec=99
    assert EyeUndistorter(source,3).calibration_id==first.calibration_id
    assert EyeUndistorter(source,4).calibration_id!=first.calibration_id
    source.k[0]+=.1
    assert EyeUndistorter(source,3).calibration_id!=first.calibration_id


@pytest.mark.parametrize('bad', ['dimensions','nan','distortion','model','matrix'])
def test_reject_invalid_calibration(bad):
    source=info()
    if bad=='dimensions': source.width=0
    if bad=='nan': source.k[0]=float('nan')
    if bad=='distortion': source.d=[1.,2.]
    if bad=='model': source.distortion_model='equidistant'
    if bad=='matrix': source.k[6]=.1
    with pytest.raises(ValueError): EyeUndistorter(source)


def test_dimensions_and_invalid_edges_are_checked():
    transform=EyeUndistorter(info(distortion=[.9, .3, 0.,0.,0.]))
    with pytest.raises(ValueError): transform.apply(np.zeros((61,80,3),np.uint8))
    assert not transform.valid_point(-1,4)
    assert not transform.valid_point(float('nan'),4)
    assert not transform.valid_point(0,0)
    assert transform.valid_point(40,30)


def test_calibration_matching_and_task_binding_prevent_double_undistortion():
    transform=EyeUndistorter(info(),8); meta=transform.message('down_right')
    cache=CalibrationCache();cache.add(meta)
    array=DetectionArray(camera_name='down_right',image_space=1,
                         calibration_id=meta.calibration_id,camera_info_version=8)
    detection=Detection(pixel_x=12.,pixel_y=19.);array.detections=[detection]
    assert cache.resolve(array) is not None
    array.camera_info_version=9
    assert cache.resolve(array) is None
    array.camera_info_version=8
    geometry=TaskCameraGeometry.__new__(TaskCameraGeometry)
    geometry.cache=cache;geometry._lock=threading.RLock();geometry._detections=OrderedDict()
    assert geometry.bind(array)
    # Raw config is deliberately very different; corrected detections must use metadata.
    raw=SimpleNamespace(matrix=np.eye(3),distortion=np.zeros(5))
    node=SimpleNamespace(camera_configs={'down':SimpleNamespace(side=lambda side:raw)})
    actual=geometry.normalized(node,'down_right',detection)
    assert actual==pytest.approx(normalized_pixel(meta.image_info.k,[],12.,19.))
    assert actual!=pytest.approx(normalized_pixel(meta.source_info.k,meta.source_info.d,12.,19.))
    array.calibration_id+=1
    assert not geometry.bind(array)


def test_left_and_right_calibrations_are_independent():
    left=EyeUndistorter(info(skew=0.),4);right=EyeUndistorter(info(skew=2.1),4)
    assert left.calibration_id!=right.calibration_id
    assert not np.array_equal(left.maps[0],right.maps[0])


def test_derived_feature_points_use_their_parent_detection_calibration():
    transform=EyeUndistorter(info(),8)
    geometry=TaskCameraGeometry.__new__(TaskCameraGeometry)
    geometry.cache=CalibrationCache();geometry.cache.add(transform.message('down_right'))
    geometry._lock=threading.RLock();geometry._detections=OrderedDict()
    detection=Detection(pixel_x=12.,pixel_y=19.)
    array=DetectionArray(camera_name='down_right',image_space=1,
                         calibration_id=transform.calibration_id,camera_info_version=8,detections=[detection])
    assert geometry.bind(array)
    derived=SimpleNamespace(pixel_x=17.,pixel_y=23.)
    actual=geometry.normalized(None,'down_right',derived,reference=detection)
    assert actual==pytest.approx(normalized_pixel(transform.k,[],17.,23.))
    with pytest.raises(ValueError):geometry.normalized(None,'down_right',derived)
