"""DDS catalog conversion shared by acquisition and discovery."""

from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sl_msgs.msg import CameraCalibration, CameraDescriptor


def catalog_qos():
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def descriptor(spec):
    message = CameraDescriptor()
    message.id = spec.id
    for key in ('name', 'device', 'ice_service', 'frame_id', 'enabled',
                'width', 'height', 'pixel_format', 'layout'):
        setattr(message, key, getattr(spec, key))
    message.requested_fps = float(spec.fps)
    message.actual_fps = 0.0
    message.encoding = 'JPEG'
    message.eye_width = spec.width // 2 if spec.layout == 'stereo_sbs' else spec.width
    message.eye_height = spec.height
    for raw in spec.calibrations:
        if not isinstance(raw, dict) or raw.get('eye') not in ('mono', 'left', 'right'):
            raise ValueError('each calibration needs eye: mono, left, or right')
        item = CameraCalibration()
        item.eye = raw['eye']
        set_message_fields(item.camera_info, raw.get('camera_info', {}))
        message.calibrations.append(item)
    message.calibrated = bool(message.calibrations) and all(
        item.camera_info.k[0] > 0 and item.camera_info.k[4] > 0
        for item in message.calibrations)
    message.status = 'starting' if spec.enabled else 'disabled'
    return message


def as_dict(message):
    return message_to_ordereddict(message)
