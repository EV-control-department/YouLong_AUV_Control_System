# Static target position estimate in odom/NED.

uint8 STATUS_UNINITIALIZED=0
uint8 STATUS_ESTIMATING=1
uint8 STATUS_STABLE=2
uint8 STATUS_STALE=3

uint8 FORM_FRONT_STEREO=1
uint8 FORM_FRONT_MULTI_VIEW=2
uint8 FORM_DOWN_DIRECT=4

uint16 class_id
uint32 instance_id
string class_name
# Canonical physical category after aliases such as xxx_front/xxx_down are
# bound together. class_id/class_name above remain the representative raw
# detector label for compatibility with legacy consumers.
string physical_class_name
uint16[] observed_class_ids

float32 world_x       # North (m)
float32 world_y       # East (m)
float32 world_z       # Down/depth (m)
float32[9] position_covariance  # Row-major 3x3 covariance (m^2)

float32 confidence
uint32 num_observations
uint32 front_stereo_count
uint32 front_multi_view_count
uint32 down_direct_count
uint8 observation_form_mask
uint8 last_observation_form
uint8 status
float32 age_sec
builtin_interfaces/Time last_observation_stamp
