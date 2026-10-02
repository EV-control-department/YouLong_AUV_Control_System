# One visual measurement in mapping_odom (NED); header.stamp is capture time.
std_msgs/Header header
uint8 TAG=1
uint8 CONE=2
uint8 kind
int32 tag_id
int32 class_id
float32 confidence
float32 depth_m
uint32 depth_samples
float32 world_x
float32 world_y
float32 world_z
float32 pose_age_s
