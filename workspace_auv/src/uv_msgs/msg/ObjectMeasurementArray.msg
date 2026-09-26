# Geometry-only observations. Long-lived state belongs to ObjectTrackArray.
# header.frame_id applies to every position and ray; localizer publishes odom.

std_msgs/Header header
ObjectMeasurement[] measurements
