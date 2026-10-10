from types import SimpleNamespace as NS
import numpy as np
from uv_msgs.msg import DetectionArray, Detection
from uv_stream import camera_streamer as stream


def batch(camera='front_left',space=1,calibration_id=15,capture_id=7):
    return DetectionArray(camera_name=camera,capture_id=capture_id,stereo_pair_id=9,
        image_space=space,calibration_id=calibration_id,
        detections=[Detection(class_id=4,confidence=.9,bbox_x1=100.,bbox_y1=100.,bbox_x2=300.,bbox_y2=300.)])


def test_overlay_rejects_other_camera_space_and_calibration(monkeypatch):
    image=np.zeros((480,1280,3),np.uint8)
    frame=stream.CachedFrame(image,7,9,0,0,'front',(15,16),1)
    cache=stream.DetectionCache()
    for value in [batch('down_left'),batch(space=0),batch(calibration_id=99),batch(),batch('front_right',calibration_id=16)]:cache.add(value)
    drawn=[]
    monkeypatch.setattr(stream,'_draw_batch',lambda image,camera,detections:drawn.append(camera) or image)
    stream._annotate_from_cache(image,frame,cache)
    assert drawn==['front_left','front_right']


def test_cache_accepts_only_its_camera():
    cache=stream.DetectionCache('front')
    cache.add(batch('down_left'));cache.add(batch('front_left'))
    assert [value.camera_name for value in cache._batches]==['front_left']


def test_overlay_chooses_only_one_nearby_batch_per_eye(monkeypatch):
    image=np.zeros((480,1280,3),np.uint8)
    frame=stream.CachedFrame(image,70,9,0,0,'front',(15,16),1)
    cache=stream.DetectionCache('front')
    for capture in range(1,5):cache.add(batch(capture_id=capture))
    drawn=[]
    monkeypatch.setattr(stream,'_draw_batch',lambda image,camera,detections:drawn.append(camera) or image)
    stream._annotate_from_cache(image,frame,cache)
    assert drawn==['front_left']


def test_reused_capture_id_does_not_match_previous_clock_epoch():
    image=np.zeros((480,1280,3),np.uint8)
    frame=stream.CachedFrame(image,7,9,2_000_000_000,0,'front',(15,16),1)
    cache=stream.DetectionCache('front');cache.add(batch())
    assert cache.for_frame(frame)==()
