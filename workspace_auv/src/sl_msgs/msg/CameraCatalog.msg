std_msgs/Header header
string source_instance_id
# Shared with the Ice frame header's camera_info_version in the SL chain.
# This is a configuration token, not a claim that calibration is available.
uint32 config_version
uint32 configured_count
uint32 enabled_count
uint32 online_count
sl_msgs/CameraDescriptor[] cameras
