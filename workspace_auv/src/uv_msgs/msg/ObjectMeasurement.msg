# One geometry-only observation derived from a Detection.
# ObjectMeasurementArray.header.frame_id defines the frame for position and ray
# XYZ values. The perception localizer publishes every such geometry in odom.

uint8 FORM_FRONT_STEREO=1
uint8 FORM_DOWN_STEREO=16
uint8 FORM_FRONT_BEARING=2
uint8 FORM_DOWN_DIRECT=4
uint8 FORM_DOWN_BEARING=8

uint64 observation_id
builtin_interfaces/Time observation_stamp
uint64 capture_id
uint64 stereo_pair_id

uint64[] source_detection_ids
string source_camera
uint16 class_id
string class_name
string physical_class_name
bool multi_instance
float32 confidence
uint8 measurement_form

bool has_position
float32 world_x
float32 world_y
float32 world_z
# Row-major 3x3 XYZ covariance, in square metres.
float32[9] position_covariance

bool has_ray
float32 ray_origin_x
float32 ray_origin_y
float32 ray_origin_z
float32 ray_direction_x
float32 ray_direction_y
float32 ray_direction_z
# One-sigma angular bearing uncertainty, in radians.
float32 ray_sigma_rad
