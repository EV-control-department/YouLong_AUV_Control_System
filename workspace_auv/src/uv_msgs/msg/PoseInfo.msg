# Pose information published by basic_motion
builtin_interfaces/Time stamp

# Deprecated compatibility fields. Always zero; MCU owns the odom origin.
float32 origin_x
float32 origin_y
float32 origin_z
float32 origin_yaw

# robot current pose in odom frame (degrees for yaw)
float32 robot_x
float32 robot_y
float32 robot_z
float32 robot_roll    # Roll in degrees
float32 robot_pitch   # Pitch in degrees
float32 robot_yaw

# current motion target in odom frame (degrees for yaw)
float32 target_x
float32 target_y
float32 target_z
float32 target_yaw

# Metadata copied from the same MCU odom snapshot as robot pose.
bool nav_valid
bool origin_initialized
uint32 origin_generation
uint32 nav_timestamp_ms
