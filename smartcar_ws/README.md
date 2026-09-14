# Smartcar Course Workspace

This workspace implements the practical environment described by the five
course guides. It is intentionally separate from the YouLong AUV workspaces in
this repository.

## Contents

| Part | Implementation |
|---|---|
| Week 01 | `src/`, `maps/`, `config/`, and `notes/` workspace layout |
| Week 02 | Dockerized ROS 1 Noetic environment with TurtleSim and `rqt_graph` |
| Week 03 | `smartcar_intro` ROS 2 publisher/subscriber package |
| Week 04 | `smartcar_gazebo` differential-drive SDF and ROS-Gazebo bridge launch |

The course documents mention ROS 2 Humble for the Ubuntu 24.04 host. Ubuntu
24.04 is running ROS 2 Jazzy on this machine, so the package and launch files
use only APIs shared by Humble and Jazzy and the setup script selects the
available ROS distribution.

## Build the ROS 2 workspace

From the repository root:

```bash
./smartcar_ws/scripts/setup_workspace.sh
```

The script creates the standard `build/`, `install/`, and `log/` directories,
which are ignored by Git. To build manually:

```bash
source /opt/ros/jazzy/setup.bash
cd smartcar_ws
colcon build --symlink-install
source install/setup.bash
```

## Week 03 publisher and subscriber

Run the two nodes in separate terminals after sourcing the workspace:

```bash
ros2 run smartcar_intro status_pub
ros2 run smartcar_intro status_sub
```

The publisher sends `std_msgs/msg/String` messages on `/student_status` once
per second. The subscriber prints each received message. A combined launch is
also available:

```bash
ros2 launch smartcar_intro status.launch.py
```

Useful checks from the guide are:

```bash
ros2 node list
ros2 topic list
ros2 topic info /student_status --verbose
ros2 topic echo /student_status
```

## Week 04 Gazebo simulation

Install the native Gazebo integration for the selected ROS distribution if it
is not already present:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz
```

Then launch the world and the bridge together:

```bash
source /opt/ros/jazzy/setup.bash
source smartcar_ws/install/setup.bash
ros2 launch smartcar_gazebo week04_sim.launch.py
```

The SDF uses the corrected wheel joint axis `0 0 1` and the Gazebo Sim
DiffDrive system. The launch file bridges:

```text
ROS 2 /cmd_vel  <->  Gazebo /cmd_vel
ROS 2 /odom     <->  Gazebo /odom
```

While the simulation is running, test it with:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.20}, angular: {z: 0.0}}"
ros2 topic echo /odom --once
```

Publish a zero Twist before leaving a terminal that is continuously publishing
velocity commands.

If the host does not have Gazebo installed, the same Jazzy + Gazebo Sim
environment is available as a container:

```bash
cd smartcar_ws/docker/jazzy_gazebo
docker compose build
docker compose up -d
docker compose exec ros2_gazebo bash
```

Inside the container, build and launch from `/smartcar_ws`:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch smartcar_gazebo week04_sim.launch.py
```

The compose service uses host networking and forwards X11 for the Gazebo GUI;
the world can also be run headlessly with `gz sim -s`.

## Week 02 ROS 1 Noetic container

The ROS 1 environment is reproducible under `docker/ros1_noetic` and includes
TurtleSim and the `rqt_graph` plugin required by the exercise:

```bash
cd smartcar_ws/docker/ros1_noetic
docker compose build
docker compose up -d
docker compose exec ros1 bash
```

Inside the container:

```bash
source /opt/ros/noetic/setup.bash
roscore
```

Use additional host terminals for `docker compose exec ros1 bash`,
`rosrun turtlesim turtlesim_node`, `rosrun turtlesim turtle_teleop_key`, and
`rqt_graph`. The compose service uses host networking and forwards the X11
socket so the TurtleSim and graph windows can be displayed.

Stop the container when finished:

```bash
docker compose down
```
