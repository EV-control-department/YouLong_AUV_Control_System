# Array of detections from one camera frame

std_msgs/Header header
string camera_name    # "front" or "down"
uint64 stereo_pair_id # non-zero: the same rendered stereo pair in both eyes
Detection[] detections
