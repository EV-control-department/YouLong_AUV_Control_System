uint32 id
string name
string device
string ice_service
string frame_id
bool enabled
bool online
string status
uint32 width
uint32 height
float32 requested_fps
# Zero means V4L2 did not report a negotiated frame rate.
float32 negotiated_fps
# Observed successful publication rate; zero means unknown or offline.
float32 actual_fps
string pixel_format
string encoding
# mono or stereo_sbs; the latter still publishes the complete source image.
string layout
uint32 eye_width
uint32 eye_height
bool calibrated
sl_msgs/CameraCalibration[] calibrations
builtin_interfaces/Time last_frame_stamp
uint64 frames_published
