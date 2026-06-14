# Comprehensive robot state published by sim_bridge (or real MCU bridge)

# Position in world NED frame
float32 pos_x         # North (m)
float32 pos_y         # East (m)
float32 pos_z         # Down (m, positive = deeper)
float32 yaw           # Yaw in radians (NED frame)

# Velocity in body frame (FRD: Forward-Right-Down)
float32 vel_x         # Forward (m/s)
float32 vel_y         # Right (m/s)
float32 vel_z         # Down (m/s)
float32 yaw_rate      # Yaw rate (rad/s)

# Velocity in world frame (for external use)
float32 vel_world_x   # North (m/s)
float32 vel_world_y   # East (m/s)

# Status
bool armed
uint8 control_mode    # 0=none, 1=pos, 2=vel, 3=force
float32 battery_voltage
uint32 error_flags
float32 cycle_time_ms

# Current target (for monitoring)
float32 target_x
float32 target_y
float32 target_z
float32 target_yaw
