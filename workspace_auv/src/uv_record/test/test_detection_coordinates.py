from types import SimpleNamespace
import sqlite3

import pytest
from rclpy.serialization import serialize_message, deserialize_message
from sensor_msgs.msg import CameraInfo
from uv_msgs.msg import Detection, DetectionArray, LegacyDetectionArray, PerceptionCameraInfo
from uv_camera.image_geometry import EyeUndistorter
from uv_record.detection_compat import deserialize_detection
from uv_record.mcap_export import _bag_records
from uv_record.player import BagPlayback


def old_array():
    return LegacyDetectionArray(camera_name='down_left',capture_id=17,stereo_pair_id=19,
        detections=[Detection(class_id=8,confidence=.9,pixel_x=14.,pixel_y=19.,
                              orientation_valid=True,orientation_axis_deg=44.)])


def test_legacy_cdr_is_explicitly_raw_and_preserves_ids_orientation():
    old=old_array();payload=serialize_message(old)
    current=deserialize_detection(payload,deserialize_message)
    assert current.image_space==DetectionArray.IMAGE_RAW
    assert current.capture_id==17 and current.stereo_pair_id==19
    assert current.detections==deserialize_message(payload,LegacyDetectionArray).detections
    assert current.calibration_id==0
    with pytest.raises(Exception):deserialize_detection(payload,deserialize_message,allow_legacy=False)


def test_new_cdr_retains_coordinate_contract_and_rejects_truncation():
    value=DetectionArray(camera_name='front_left',image_space=1,calibration_id=123,camera_info_version=4)
    payload=serialize_message(value)
    assert deserialize_detection(payload,deserialize_message,False)==value
    with pytest.raises(Exception):deserialize_detection(payload[:-3],deserialize_message,False)
    with pytest.raises(Exception):deserialize_detection(b'bad',deserialize_message)


def test_mcap_export_upgrades_old_detection_bytes_and_keeps_other_topics(monkeypatch):
    import uv_record.mcap_export as module
    payload=serialize_message(old_array())
    records=iter([('/auv/perception/detections',payload,1),('/test',b'untouched',2)])
    class Reader:
        count=0
        def has_next(self):return self.count<2
        def read_next(self):self.count+=1;return next(records)
        def close(self):pass
    monkeypatch.setattr(module,'_open_reader',lambda *args:Reader())
    result=list(_bag_records(None,[None],{'/auv/perception/detections':{'type':'uv_msgs/msg/DetectionArray'}}))
    assert deserialize_message(result[0][2],DetectionArray).image_space==0
    assert result[1][2]==b'untouched'


def test_sqlite_playback_upgrades_legacy_schema_and_retains_calibration_qos(tmp_path,monkeypatch):
    import uv_record.player as module
    from rclpy.qos import DurabilityPolicy
    from rosidl_runtime_py.utilities import get_message
    info=CameraInfo(width=80,height=60,k=[100.,0.,40.,0.,100.,30.,0.,0.,1.],d=[-.3,.1,0.,0.,0.])
    meta=EyeUndistorter(info,3).message('down_left')
    root=tmp_path/'bag';root.mkdir();path=root/'part_0.db3'
    connection=sqlite3.connect(path)
    connection.executescript('CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT,type TEXT,serialization_format TEXT,offered_qos_profiles TEXT);CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB);')
    topics=[('/auv/perception/camera/downward/left/calibration','uv_msgs/msg/PerceptionCameraInfo',meta),
            ('/auv/perception/detections','uv_msgs/msg/DetectionArray',old_array())]
    for index,(topic,typename,message) in enumerate(topics,1):
        connection.execute('INSERT INTO topics VALUES(?,?,?,"cdr","")',(index,topic,typename))
        connection.execute('INSERT INTO messages VALUES(?,?,?,?)',(index,index,index,serialize_message(message)))
    connection.commit();connection.close()
    monkeypatch.setattr(module,'_rosbag_modules',lambda:(None,None,deserialize_message,get_message))
    sent={};profiles={}
    def publisher(kind,topic,qos):
        sent[topic]=[];profiles[topic]=qos
        return SimpleNamespace(publish=sent[topic].append)
    playback=BagPlayback(root,SimpleNamespace(create_publisher=publisher))
    assert playback.publish_until(10)==2
    assert sent[topics[1][0]][0].image_space==0
    assert profiles[topics[0][0]].durability==DurabilityPolicy.TRANSIENT_LOCAL
    assert not playback.errors
    playback.seek(2)
    assert sent[topics[0][0]][-1].calibration_id==meta.calibration_id
    playback.close()
