#!/usr/bin/env bash
# Called as root during image build; never alters the AUV bind-mounted workspace.
set -eo pipefail

case "${INSTALL_FOXGLOVE_BRIDGE:-0}" in
    0|false) exit 0 ;;
    1|true) ;;
    *) echo 'INSTALL_FOXGLOVE_BRIDGE must be 0/1 or false/true' >&2; exit 2 ;;
esac

apt-get update
if [[ "${ROS_DISTRO}" != foxy ]]; then
    # Newer supported ROS distributions use the official binary package.
    apt-get install -y --no-install-recommends "ros-${ROS_DISTRO}-foxglove-bridge"
    rm -rf /var/lib/apt/lists/*
    exit 0
fi

apt-get install -y --no-install-recommends \
    build-essential ca-certificates git cmake python3-colcon-common-extensions \
    libasio-dev libssl-dev libwebsocketpp-dev nlohmann-json3-dev \
    ros-foxy-ament-cmake ros-foxy-ament-index-cpp ros-foxy-rclcpp \
    ros-foxy-rclcpp-components ros-foxy-rosgraph-msgs ros-foxy-ros-environment \
    ros-foxy-rosbag2-cpp
rm -rf /var/lib/apt/lists/*

# Official 0.2.2 needs APIs introduced after Foxy. Apply our isolated
# rosbag2-based adapter without replacing the system rclcpp libraries.
readonly bridge_commit=80e6a977c773cbc6018db891c4ca504ab2b8293a
readonly bridge_ws=/opt/foxglove_bridge_ws
readonly bridge_src="${bridge_ws}/src/foxglove_bridge"
mkdir -p "${bridge_src}"
git -C "${bridge_src}" init
git -C "${bridge_src}" fetch --depth 1 \
    https://github.com/foxglove/ros-foxglove-bridge.git "${bridge_commit}"
git -C "${bridge_src}" checkout --detach FETCH_HEAD
test "$(git -C "${bridge_src}" rev-parse HEAD)" = "${bridge_commit}"
git -C "${bridge_src}" apply --check /opt/foxglove_bridge_build/foxy-0.2.2.patch
git -C "${bridge_src}" apply /opt/foxglove_bridge_build/foxy-0.2.2.patch

source /opt/ros/foxy/setup.bash
cd "${bridge_ws}"
MAKEFLAGS="-j${FOXGLOVE_BRIDGE_BUILD_JOBS:-2}" colcon build \
    --packages-select foxglove_bridge \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
test -x install/foxglove_bridge/lib/foxglove_bridge/foxglove_bridge
rm -rf build log "${bridge_src}/.git"
