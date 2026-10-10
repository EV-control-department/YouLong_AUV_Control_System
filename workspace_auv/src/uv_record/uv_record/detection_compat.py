"""Read old DetectionArray CDR without interpreting old pixels as corrected."""
def deserialize_detection(payload, deserialize_message, allow_legacy=True):
    from uv_msgs.msg import DetectionArray, LegacyDetectionArray
    try:
        return deserialize_message(payload, DetectionArray)
    except Exception:
        if not allow_legacy:
            raise
        from rclpy.serialization import serialize_message
        old = deserialize_message(payload, LegacyDetectionArray)
        if len(serialize_message(old)) != len(payload):
            raise ValueError('invalid legacy DetectionArray payload length')
        result = DetectionArray()
        result.header = old.header
        result.camera_name = old.camera_name
        result.capture_id = old.capture_id
        result.stereo_pair_id = old.stereo_pair_id
        result.detections = old.detections
        result.image_space = DetectionArray.IMAGE_RAW
        return result
