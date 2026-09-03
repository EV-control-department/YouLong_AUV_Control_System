# One geometrically valid observation used by the static target localizer.
# Coordinates use the odom frame with the project NED convention.

uint8 FORM_FRONT_STEREO=1
uint8 FORM_FRONT_MULTI_VIEW=2
uint8 FORM_DOWN_DIRECT=4

uint64 observation_id
builtin_interfaces/Time observation_stamp

uint16 class_id
uint32 instance_id
string class_name
# Raw class_id/class_name identifies this detector output; this field identifies
# the physical target track shared by xxx_front and xxx_down observations.
string physical_class_name
float32 confidence
uint8 observation_form

# True for a direct 3-D measurement.  FRONT_MULTI_VIEW is normally a ray
# observation, so has_position is false until it is fused with another ray.
bool has_position
float32 world_x
float32 world_y
float32 world_z
float32[9] position_covariance  # Row-major 3x3, m^2; zero for a ray.

# Ray representation for FRONT_MULTI_VIEW.  Direct measurements leave these
# fields at zero.  The direction is normalized in odom/NED.
float32 ray_origin_x
float32 ray_origin_y
float32 ray_origin_z
float32 ray_direction_x
float32 ray_direction_y
float32 ray_direction_z
