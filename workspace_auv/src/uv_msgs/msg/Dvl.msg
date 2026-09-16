# DVL (Doppler Velocity Log) sample in the vehicle body frame.
#
# Produced by `dvl_sim` from Stonefish's `stonefish_ros2/DVL` (SIL) and, on the
# real vehicle, by the MCU bridge (T7).  AQUA-SLAM consumes this message (T6),
# which is why it carries the per-beam speeds instead of only the fused velocity:
# its Doppler pre-integration works on the four along-beam measurements.
#
# Conventions (see docs/frames/frame_convention.md):
#   * `velocity` is the velocity at the DVL acoustic centre, expressed in the
#     **body** frame.  That is not the same as `/zit6/state/vel`, which is the
#     velocity of the body origin: they differ by `omega x r_dvl` (bug B19).
#   * `beam_velocity` holds the four along-beam speeds in the sensor's fixed
#     azimuth order 45/135/225/315 deg, beam elevation alpha = 60 deg from the
#     horizontal (scene `beam_angle = 30 deg` off vertical).

std_msgs/Header header

# Velocity at the DVL in body coordinates [m/s]
geometry_msgs/Vector3 velocity

# False when the DVL did not track the bottom (`altitude < 0`); the velocity and
# beam fields are then meaningless and must be discarded.
bool velocity_valid

# Height above the bottom [m]; negative means "no bottom lock"
float64 altitude

# Along-beam speeds [m/s], azimuth order 45/135/225/315 deg
float32[4] beam_velocity
