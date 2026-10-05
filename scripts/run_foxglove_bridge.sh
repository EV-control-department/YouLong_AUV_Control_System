#!/usr/bin/env bash
set -eo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "/opt/ros/${ROS_DISTRO:-foxy}/setup.bash"

if [[ ! -f "${workspace_root}/workspace_auv/install/local_setup.bash" ]]; then
    echo 'AUV workspace is not built. Wait for auv preparation to finish, then start foxglove_bridge.' >&2
    exit 1
fi
# Current prepare_workspace.sh installs uv_msgs and zit6_interfaces together
# in workspace_auv. Also support a separately built ZIT6 overlay.
source "${workspace_root}/workspace_auv/install/local_setup.bash"
if [[ -f "${workspace_root}/third_party/AUV_zit6_cmake/install/local_setup.bash" ]]; then
    source "${workspace_root}/third_party/AUV_zit6_cmake/install/local_setup.bash"
fi
if [[ -f /opt/foxglove_bridge_ws/install/local_setup.bash ]]; then
    # local_setup preserves the AUV overlays instead of replaying build underlays.
    source /opt/foxglove_bridge_ws/install/local_setup.bash
fi
if ! ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
    echo 'Bridge is not installed. Build auv with INSTALL_FOXGLOVE_BRIDGE=1 first.' >&2
    exit 1
fi
for package in uv_msgs zit6_interfaces; do
    if ! ros2 pkg prefix "${package}" >/dev/null 2>&1; then
        echo "Required message package is missing: ${package}. Prepare the AUV workspace first." >&2
        exit 1
    fi
done

exec ros2 run foxglove_bridge foxglove_bridge --ros-args \
    -p "address:=${FOXGLOVE_BRIDGE_ADDRESS:-0.0.0.0}" \
    -p "port:=${FOXGLOVE_BRIDGE_PORT:-8765}" "$@"
