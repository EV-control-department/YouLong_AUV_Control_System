# uv_rviz

RViz2 view and read-only adapters for the YouLong AUV ROS 2 graph.

Launch it from a shell where the AUV and simulation overlays are sourced:

    ros2 launch uv_rviz display.launch.py

The launch accepts ray_length:=3.0 and use_sim_time:=false. The default time source is wall time to match the current localization TF. Use simulation time only when the source nodes and TF are using /clock.

The fixed frame and all displayed positions use the project NED convention: X north, Y east, and Z depth/down. The `base_link` body frame is FRD: +X forward, +Y right, and +Z down. The view shows a dedicated labeled FRD axis at `base_link` and a NED axis at `odom`.

The canonical TF tree is published on `/auv/tf` and `/auv/tf_static`. Foxy's RViz TF listener subscribes to absolute `/tf` and `/tf_static` internally, so this launch runs a read-only bridge that mirrors the canonical AUV transforms to those root topics while RViz is open. The RViz-only topics are under /auv/visualization:

- /auv/visualization/odom (nav_msgs/Odometry)
- /auv/visualization/odom_path and /auv/visualization/planned_path (nav_msgs/Path)
- /auv/visualization/measurements and /auv/visualization/tracks (visualization_msgs/MarkerArray)

The RobotModel display reads the active URDF from the running `robot_state_publisher`, so the same RViz launch follows the simulation or real vehicle TF tree without starting another TF publisher. Its body mesh and propellers come from the Stonefish YouLong visual assets; camera, IMU, DVL and USBL frame shapes are attached to their own TF links. The TF axes are kept small so they do not hide the body.

The packaged RViz view is capped at 15 FPS and refreshes the TF display every 0.2 seconds to keep window interaction responsive alongside the simulator and perception nodes.

These topics are for display and must not be used as control or estimation inputs. The odometry conversion contains pose only because the canonical PoseInfo has no twist or covariance.
