#!/usr/bin/env bash
# Source this file to load ROS, the workspace overlays, and venv site-packages.
# Usage: source scripts/source_workspace.sh

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    printf '请在当前 shell 中 source 此脚本：source %s\n' "$0" >&2
    exit 2
fi

_workspace_env_script="${BASH_SOURCE[0]}"
_workspace_env_root="$(cd -- "$(dirname -- "${_workspace_env_script}")/.." && pwd)"
_workspace_env_ros="/opt/ros/${ROS_DISTRO:-foxy}/setup.bash"
_workspace_env_was_nounset=0
[[ "$-" == *u* ]] && _workspace_env_was_nounset=1

if [[ ! -r "${_workspace_env_ros}" ]]; then
    printf '找不到 ROS 环境：%s（可通过 ROS_DISTRO 指定发行版）\n' "${_workspace_env_ros}" >&2
    return 1
fi
if [[ ! -r "${_workspace_env_root}/workspace_auv/install/setup.bash" ]]; then
    printf '找不到 workspace_auv/install/setup.bash；请先构建工作区。\n' >&2
    return 1
fi

set +u
source "${_workspace_env_ros}"
_workspace_env_status=$?
if (( _workspace_env_status == 0 )); then
    source "${_workspace_env_root}/workspace_auv/install/setup.bash"
    _workspace_env_status=$?
fi
if (( _workspace_env_status == 0 )) && [[ -r "${_workspace_env_root}/workspace_sim/install/setup.bash" ]]; then
    source "${_workspace_env_root}/workspace_sim/install/setup.bash"
    _workspace_env_status=$?
fi
if (( _workspace_env_was_nounset )); then set -u; fi
if (( _workspace_env_status != 0 )); then
    return "${_workspace_env_status}"
fi

for _workspace_env_site in "${_workspace_env_root}"/workspace_auv/.venv/lib/python*/site-packages; do
    if [[ -d "${_workspace_env_site}" ]]; then
        export PYTHONPATH="${_workspace_env_site}${PYTHONPATH:+:${PYTHONPATH}}"
    fi
done

if ! python3 -c 'import iceoryx2' >/dev/null 2>&1; then
    printf 'iceoryx2 未能通过当前 python3 导入。请运行 INSTALL_WORKSPACE_AI=false ./scripts/prepare_workspace.sh。\n' >&2
    return 1
fi
printf 'YouLong ROS 环境已加载（ROS %s，iceoryx2 可导入）。\n' "${ROS_DISTRO:-foxy}"
unset _workspace_env_script _workspace_env_root _workspace_env_ros _workspace_env_was_nounset _workspace_env_status _workspace_env_site
