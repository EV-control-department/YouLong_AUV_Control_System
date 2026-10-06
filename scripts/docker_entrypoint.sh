#!/usr/bin/env bash
# Compose entrypoint: prepare the workspaces once, then expose simple sim/real
# commands while keeping the container useful for docker compose exec.

set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

runtime_dir="${XDG_RUNTIME_DIR:-/tmp/runtime-${HOST_UID:-1000}}"
mkdir -p "${runtime_dir}"
chmod 700 "${runtime_dir}"

# Keep rqt usable from an interactive shell without assuming NVIDIA.
# The active Compose profile decides which graphics backend is available.
if ! grep -q 'youlong-compose-rqt-wrapper-v1' "${HOME}/.bashrc" 2>/dev/null; then
    printf '%s\n' \
        '# youlong-compose-rqt-wrapper-v1' \
        'rqt() {' \
        '  command rqt "$@"' \
        '}' >> "${HOME}/.bashrc"
fi

# shellcheck disable=SC1091
set +u
source "/opt/ros/${ROS_DISTRO:-foxy}/setup.bash"
set -u
"${REPO_ROOT}/scripts/prepare_workspace.sh"

# shellcheck disable=SC1091
set +u
source "/opt/ros/${ROS_DISTRO:-foxy}/setup.bash"
set -u
if [[ -f "${REPO_ROOT}/workspace_auv/.venv/bin/activate" ]]; then
    # shellcheck disable=SC1091
    source "${REPO_ROOT}/workspace_auv/.venv/bin/activate"
    for site in "${REPO_ROOT}"/workspace_auv/.venv/lib/python*/site-packages; do
        if [[ -d "${site}" ]]; then
            export PYTHONPATH="${site}${PYTHONPATH:+:${PYTHONPATH}}"
        fi
    done
fi
if [[ -f "${REPO_ROOT}/workspace_auv/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    set +u
    source "${REPO_ROOT}/workspace_auv/install/setup.bash"
    set -u
fi
if [[ -f "${REPO_ROOT}/workspace_sim/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    set +u
    source "${REPO_ROOT}/workspace_sim/install/setup.bash"
    set -u
fi

mode="${1:-idle}"
if [[ "$#" -gt 0 ]]; then
    shift
fi

case "${mode}" in
    idle)
        exec tail -f /dev/null
        ;;
    prepare)
        exit 0
        ;;
    shell)
        exec bash --login "$@"
        ;;
    sim)
        exec ros2 launch uv_sim sim.launch.py "$@"
        ;;
    real)
        exec ros2 launch uv_bringup real.launch.py "$@"
        ;;
    record)
        exec ros2 run uv_record record "$@"
        ;;
    stream)
        exec ros2 run uv_stream camera_streamer "$@"
        ;;
    *)
        exec "${mode}" "$@"
        ;;
esac
