# Array of detections from one camera frame

std_msgs/Header header
string camera_name    # "front_left", "front_right", "down_left", or "down_right"
uint64 capture_id     # monotonically increasing source-frame identifier
uint64 stereo_pair_id # non-zero: the same rendered stereo pair in both eyes
Detection[] detections

# Zero/default values identify legacy detections in the original image.
uint8 IMAGE_RAW=0
uint8 IMAGE_UNDISTORTED=1
uint8 image_space
uint64 calibration_id
uint32 camera_info_version
