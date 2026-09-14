#!/usr/bin/env bash
set -euo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ros_distro="${ROS_DISTRO:-jazzy}"
ros_setup="/opt/ros/${ros_distro}/setup.bash"

if [[ ! -f "$ros_setup" ]]; then
  echo "ROS setup file not found: $ros_setup" >&2
  echo "Set ROS_DISTRO to an installed distribution (for example jazzy or humble)." >&2
  exit 1
fi

source "$ros_setup"
mkdir -p "$workspace_root/src" "$workspace_root/maps" "$workspace_root/config" "$workspace_root/notes"
cd "$workspace_root"

echo "Building smartcar_ws with ROS_DISTRO=$ros_distro"
colcon build --symlink-install --event-handlers console_direct+

echo
echo "Build complete. Source the overlay with:"
echo "  source $workspace_root/install/setup.bash"

if ! ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1; then
  echo
  echo "Gazebo bridge packages are not installed. For Week 04, install:"
  echo "  sudo apt install ros-${ros_distro}-ros-gz"
fi
