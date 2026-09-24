# One geometry-only observation derived from a Detection.
# Coordinates use the odom/NED convention when has_position is true.

uint8 FORM_FRONT_STEREO=1
uint8 FORM_FRONT_BEARING=2
uint8 FORM_DOWN_DIRECT=4

uint64 observation_id
builtin_interfaces/Time observation_stamp
uint64 capture_id
uint64 stereo_pair_id

uint64[] source_detection_ids
string source_camera
uint16 class_id
string class_name
string physical_class_name
float32 confidence
uint8 measurement_form

bool has_position
float32 world_x
float32 world_y
float32 world_z
float32[9] position_covariance

bool has_ray
float32 ray_origin_x
float32 ray_origin_y
float32 ray_origin_z
float32 ray_direction_x
float32 ray_direction_y
float32 ray_direction_z
