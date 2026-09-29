#!/usr/bin/env bash
# Prepare the bind-mounted ROS workspaces for the Compose container.
#
# The ROS Python entry points in ament_python packages currently use the
# system interpreter from the ROS image.  The venv is therefore exported via
# PYTHONPATH as well as being used for the iceoryx2 build.  This keeps the
# official Python binding in-process without adding a native bridge process.

set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
readonly ICEORYX_ROOT="${REPO_ROOT}/third_party/iceoryx2"
readonly ICEORYX_MANIFEST="${ICEORYX_ROOT}/iceoryx2-ffi/python/Cargo.toml"
readonly VENV_DIR="${REPO_ROOT}/workspace_auv/.venv"
readonly VENV_PYTHON="${VENV_DIR}/bin/python"
readonly ICEORYX_CACHE="${REPO_ROOT}/.docker/iceoryx2"
readonly ICEORYX_TARGET="${ICEORYX_CACHE}/target"
readonly ICEORYX_DIST="${ICEORYX_CACHE}/dist"
readonly ICEORYX_MARKER="${ICEORYX_CACHE}/installed-revision"

log() {
    printf '[youlong] %s\n' "$*" >&2
}

require_file() {
    if [[ ! -e "$1" ]]; then
        log "缺少 $1。请先在宿主机执行: git submodule update --init --recursive"
        exit 1
    fi
}

clean_stale_colcon_workspace() {
    local workspace="$1"
    local cache
    local stale=0

    [[ -d "${workspace}/build" ]] || return 0

    while IFS= read -r -d '' cache; do
        if ! grep -q '^CMAKE_HOME_DIRECTORY:INTERNAL=/workspace/' "${cache}"; then
            stale=1
            break
        fi
    done < <(find "${workspace}/build" -type f -name CMakeCache.txt -print0)

    if (( stale )); then
        log "检测到从宿主机路径生成的旧 colcon 缓存，清理 ${workspace}/{build,install,log}"
        rm -rf -- "${workspace}/build" "${workspace}/install" "${workspace}/log"
    fi
}

ensure_iceoryx_submodule() {
    if [[ ! -f "${ICEORYX_MANIFEST}" ]]; then
        log 'iceoryx2 子模块尚未检出，尝试在容器内初始化。'
        git -C "${REPO_ROOT}" submodule update --init --recursive third_party/iceoryx2
    fi
    require_file "${ICEORYX_MANIFEST}"
}

ensure_workspace_python() {
    # This script is intentionally called with the ROS system Python first, so
    # the venv inherits cv_bridge/rclpy from the ROS image.
    INSTALL_WORKSPACE_AI="${INSTALL_WORKSPACE_AI:-true}" \
        "${REPO_ROOT}/scripts/setup_workspace_python.sh"
}

ensure_iceoryx_python() {
    local revision
    revision="$(git -C "${ICEORYX_ROOT}" rev-parse HEAD)"
    mkdir -p "${ICEORYX_TARGET}" "${ICEORYX_DIST}"

    if [[ "${FORCE_ICEORYX2_BUILD:-false}" != 'true' \
          && -f "${ICEORYX_MARKER}" \
          && "$(<"${ICEORYX_MARKER}")" == "${revision}" ]] \
        && "${VENV_PYTHON}" -c 'import iceoryx2' >/dev/null 2>&1; then
        log "iceoryx2 Python binding 已就绪 (${revision:0:12})"
        return 0
    fi

    log "构建 iceoryx2 Python binding (${revision:0:12})"
    "${VENV_PYTHON}" -m pip install \
        --disable-pip-version-check --quiet 'maturin>=1.8,<2'

    rm -f -- "${ICEORYX_DIST}"/iceoryx2-*.whl
    "${VENV_PYTHON}" -m maturin build --release \
        --manifest-path "${ICEORYX_MANIFEST}" \
        --target-dir "${ICEORYX_TARGET}" \
        --out "${ICEORYX_DIST}"

    local wheel
    wheel="$(find "${ICEORYX_DIST}" -maxdepth 1 -type f \
        -name 'iceoryx2-*.whl' -print | sort | tail -n 1)"
    if [[ -z "${wheel}" ]]; then
        log 'maturin 没有生成 iceoryx2 wheel'
        exit 1
    fi

    "${VENV_PYTHON}" -m pip install \
        --disable-pip-version-check --force-reinstall "${wheel}"

    # iceoryx2 v0.10 advertises the abi3/3.8 wheel, but its pure-Python
    # extensions use builtin generic annotations (list[str], dict[int, ...]).
    # Python 3.8 evaluates those annotations at import time and raises
    # TypeError.  Add the standard postponed-annotations future only to the
    # installed copy; the upstream submodule remains untouched.
    ICEORYX_VENV_DIR="${VENV_DIR}" "${VENV_PYTHON}" -c '
from pathlib import Path
import os
import sys

if sys.version_info < (3, 9):
    roots = list(Path(os.environ["ICEORYX_VENV_DIR"]).glob(
        "lib/python*/site-packages/iceoryx2"))
    if not roots:
        raise SystemExit("installed iceoryx2 package directory not found")
    for path in roots[0].glob("*.py"):
        text = path.read_text()
        if "from __future__ import annotations" not in text:
            path.write_text("from __future__ import annotations\n\n" + text)
'
    "${VENV_PYTHON}" -c 'import iceoryx2; print("iceoryx2 Python binding: OK")'
    printf '%s\n' "${revision}" > "${ICEORYX_MARKER}"
}

source_ros_and_python() {
    # shellcheck disable=SC1091
    set +u
    source "/opt/ros/${ROS_DISTRO:-foxy}/setup.bash"
    set -u
    # shellcheck disable=SC1091
    source "${VENV_DIR}/bin/activate"

    # ROS console scripts built by the system colcon interpreter use
    # /usr/bin/python3 in their shebang.  Make the binding and optional AI
    # packages visible to that interpreter too.
    local site
    for site in "${VENV_DIR}"/lib/python*/site-packages; do
        if [[ -d "${site}" ]]; then
            export PYTHONPATH="${site}${PYTHONPATH:+:${PYTHONPATH}}"
        fi
    done
}

build_workspaces() {
    local workers="${COLCON_WORKERS:-1}"
    clean_stale_colcon_workspace "${REPO_ROOT}/workspace_auv"
    clean_stale_colcon_workspace "${REPO_ROOT}/workspace_sim"

    log '构建 workspace_auv'
    cd "${REPO_ROOT}/workspace_auv"
    colcon build --symlink-install --parallel-workers "${workers}"
    # shellcheck disable=SC1091
    set +u
    source install/setup.bash
    set -u

    if [[ -d "${REPO_ROOT}/third_party/AUV_zit6_cmake/zit6_interfaces" ]]; then
        local zit6_paths=(
            "${REPO_ROOT}/third_party/AUV_zit6_cmake/zit6_interfaces"
        )
        local zit6_packages=(zit6_interfaces)
        local upper_config="${REPO_ROOT}/third_party/AUV_zit6_cmake/UserApp/Config/config.json"

        # upper_examples needs a machine-local ZIT6 configuration that is not
        # checked into git. Keep the ROS interface build portable, and include
        # the application only after its local configuration has been supplied.
        case "${BUILD_ZIT6_UPPER_EXAMPLES:-auto}" in
            true)
                if [[ ! -f "${upper_config}" ]]; then
                    log "缺少 ${upper_config}; BUILD_ZIT6_UPPER_EXAMPLES=true 无法继续"
                    exit 1
                fi
                zit6_paths+=("${REPO_ROOT}/third_party/AUV_zit6_cmake/upper_examples")
                zit6_packages+=(upper_examples)
                ;;
            auto)
                if [[ -f "${upper_config}" ]]; then
                    zit6_paths+=("${REPO_ROOT}/third_party/AUV_zit6_cmake/upper_examples")
                    zit6_packages+=(upper_examples)
                elif [[ -d "${REPO_ROOT}/third_party/AUV_zit6_cmake/upper_examples" ]]; then
                    log '跳过 upper_examples（未找到本机 UserApp/Config/config.json）；需要时设置 BUILD_ZIT6_UPPER_EXAMPLES=true'
                fi
                ;;
            false)
                if [[ -d "${REPO_ROOT}/third_party/AUV_zit6_cmake/upper_examples" ]]; then
                    log '按 BUILD_ZIT6_UPPER_EXAMPLES=false 跳过 upper_examples'
                fi
                ;;
            *)
                log 'BUILD_ZIT6_UPPER_EXAMPLES 只接受 auto、true 或 false'
                exit 2
                ;;
        esac

        log "构建 ZIT6 ROS 包: ${zit6_packages[*]}"
        colcon build --symlink-install --parallel-workers "${workers}" \
            --base-paths "${zit6_paths[@]}" \
            --packages-select "${zit6_packages[@]}"
        # shellcheck disable=SC1091
        set +u
        source install/setup.bash
        set -u
    fi

    log '构建 workspace_sim'
    cd "${REPO_ROOT}/workspace_sim"
    colcon build --symlink-install --parallel-workers "${workers}" \
        --cmake-force-configure
    # shellcheck disable=SC1091
    set +u
    source install/setup.bash
    set -u
}

main() {
    require_file "${REPO_ROOT}/workspace_auv/src"
    mkdir -p "${YOLO_CONFIG_DIR:-${REPO_ROOT}/.docker/ultralytics}"
    ensure_iceoryx_submodule
    ensure_workspace_python
    ensure_iceoryx_python
    source_ros_and_python
    build_workspaces

    log 'ROS、iceoryx2、uv_perception、uv_stream 和 uv_record 已准备完成'
    log "运行时 PYTHONPATH=${PYTHONPATH}"
}

main "$@"
