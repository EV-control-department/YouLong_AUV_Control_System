from types import SimpleNamespace
from pathlib import Path
import threading
import copy
import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo
from uv_perception.object_detector import ObjectDetector
from uv_perception.detector.yolo_detector import YoloDetector
from uv_perception.model_class_publisher import mapping_message
from auv_protocol.model_mapping import ModelClassRegistry


def detector():
    obj=ObjectDetector.__new__(ObjectDetector)
    obj._mapping_registry=ModelClassRegistry.from_message(mapping_message())
    obj.confidence=.5
    return obj


@pytest.mark.parametrize('camera,allowed',[('front',{1,3,4,5,6,7,8,10,11}),('down',{0,2,4,5,6,7,8,9,11})])
def test_camera_metadata_filters_boxes_and_preserves_masks(camera,allowed):
    obj=detector()
    results=[(i,.9,(10.,10.,20.,20.)) for i in range(12)]
    polygons=[np.array([[i,0],[i,1],[i+1,1]]) for i in range(12)]
    boxes,masks=obj._filter_results(camera,np.zeros((40,40,3)),results,polygons)
    assert {box[0] for box in boxes}==allowed
    assert all(mask[0,0]==box[0] for box,mask in zip(boxes,masks))


def test_unknown_low_confidence_and_invalid_boxes_are_not_published():
    obj=detector()
    results=[(99,.9,(1,1,4,4)),(4,.1,(1,1,4,4)),(4,.9,(1,1,float('nan'),4)),
             (4,.9,(1,1,1,4)),(4,.9,(-1,1,4,4)),(4,.9,(1,1,41,4))]
    assert obj._filter_results('front',np.zeros((40,40,3)),results,[])==((),())


def test_model_mapping_checks_ids_and_names():
    obj=detector();adapter=YoloDetector.__new__(YoloDetector)
    names={entry.id:entry.name for entry in obj._mapping_registry.entries}
    adapter.model=SimpleNamespace(names=names)
    adapter.validate_mapping(obj._mapping_registry)
    adapter.model=SimpleNamespace(names={**names,4:'wrong'})
    with pytest.raises(ValueError):adapter.validate_mapping(obj._mapping_registry)


class Node:
    def __init__(self,params):self.params=params;self.sent=[]
    def declare_parameter(self,name,default):return SimpleNamespace(value=self.params.get(name,default))
    def create_subscription(self,*args):return args
    def create_publisher(self,*args):return SimpleNamespace(publish=self.sent.append)
    def create_timer(self,*args):return args
    def get_logger(self):return SimpleNamespace(info=lambda *a:None,error=lambda *a:None,warning=lambda *a:None)


def test_independent_model_instances_paths_and_single_eye_routing(tmp_path,monkeypatch):
    import uv_perception.object_detector as module
    front=tmp_path/'front.pt';down=tmp_path/'down.pt'
    front.touch();down.touch()
    registry=detector()._mapping_registry
    class Model:
        def __init__(self,path,*args):self.path=path;self.images=[]
        def validate_mapping(self,mapping):assert mapping.entries==registry.entries
        def detect_with_masks(self,image):self.images.append(image.copy());return (),()
    monkeypatch.setattr(module,'YoloDetector',Model)
    monkeypatch.setattr(module.threading.Thread,'start',lambda self:None)
    monkeypatch.setattr(ObjectDetector,'_make_aruco_detector',lambda self:None)
    node=Node({'front_model_path':str(front),'down_model_path':str(down)})
    obj=ObjectDetector(node);obj._mapping_callback(mapping_message())
    assert obj._detectors['front'] is not obj._detectors['down']
    assert obj._model_paths=={'front':str(front),'down':str(down)}
    obj._publish_aruco=lambda image:None
    published=[];obj._publish=lambda camera,side,*a:published.append((camera,side))
    calls=[]
    packet=SimpleNamespace(header=SimpleNamespace(camera_info_version=1),
                           bgr=lambda:calls.append(1) or np.zeros((6,16,3),np.uint8))
    for camera in ('front','down'):
        for side in ('left','right'):
            info=CameraInfo(width=8,height=6,k=[10.,0.,4.,0.,10.,3.,0.,0.,1.],d=[0.]*5)
            obj._camera_info(f'{camera}_{side}',info)
        packets=iter([packet,None])
        monkeypatch.setattr(module,'Iceoryx2Reader',lambda service:SimpleNamespace(read=lambda:next(packets),close=lambda:None))
        obj._read_loop(camera,camera)
    assert len(calls)==2
    assert published==[('front','left'),('front','right'),('down','left'),('down','right')]
    assert len(obj._detectors['front'].images)==len(obj._detectors['down'].images)==2
    # A new metadata timestamp must not regenerate the map.
    info=copy.deepcopy(obj._source_infos['front_left']);info.header.stamp.sec=10
    before=obj._undistorters['front_left'];obj._camera_info('front_left',info)
    assert obj._undistorters['front_left'] is before


def test_failed_front_model_does_not_disable_down(tmp_path,monkeypatch):
    import uv_perception.object_detector as module
    down=tmp_path/'down.pt';down.touch()
    class Model:
        def __init__(self,*args):pass
        def validate_mapping(self,registry):pass
    monkeypatch.setattr(module,'YoloDetector',Model)
    monkeypatch.setattr(module.threading.Thread,'start',lambda self:None)
    monkeypatch.setattr(ObjectDetector,'_make_aruco_detector',lambda self:None)
    obj=ObjectDetector(Node({'front_model_path':str(tmp_path/'missing'),'down_model_path':str(down)}))
    obj._mapping_callback(mapping_message())
    assert obj._model_valid=={'front':False,'down':True}
