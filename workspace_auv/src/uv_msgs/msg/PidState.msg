# PID controller telemetry (published for debugging/tuning)

PidGains speed_mag    # Outer loop gains
PidGains vel_x        # Inner loop X gains
PidGains vel_y        # Inner loop Y gains
PidGains pos_z        # Z axis gains
PidGains yaw          # Yaw position loop
PidGains yaw_rate     # Yaw rate loop

float32 error_length
float32 error_angle_deg
float32 speed_cmd
float32 vx_body_cmd
float32 vy_body_cmd
