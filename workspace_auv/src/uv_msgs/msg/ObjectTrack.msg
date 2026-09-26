# Fused object state. Static targets use this message as their persistent state.

uint8 STATUS_TENTATIVE=0
uint8 STATUS_STABLE=1
uint8 STATUS_STALE=2
uint8 STATUS_LOST=3

uint64 track_id
uint64 last_observation_id
uint16 class_id
string class_name
string physical_class_name
string estimate_source

float32 world_x
float32 world_y
float32 world_z
float32[9] position_covariance
float32 confidence
uint32 measurement_count
uint8 status
float32 age_sec
builtin_interfaces/Time last_measurement_stamp
