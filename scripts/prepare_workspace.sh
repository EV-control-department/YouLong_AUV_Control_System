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
    #
    # Nocuda mode deliberately does not install requirements-ai.txt.  On Linux,
    # installing ultralytics can pull a CUDA-enabled torch wheel even when the
    # application itself never requests a GPU.
    local install_ai="${INSTALL_WORKSPACE_AI:-false}"
    if [[ "${YOULONG_NOCUDA:-false}" == "true" ]]; then
        install_ai="false"
        log 'YOULONG_NOCUDA=true：跳过 AI/CUDA 相关 Python 依赖'
    fi

    INSTALL_WORKSPACE_AI="${install_ai}" \
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

    local wheel
    wheel="$(find "${ICEORYX_DIST}" -maxdepth 1 -type f \
        -name 'iceoryx2-*.whl' -print | sort | tail -n 1)"

    if [[ -n "${wheel}" && "${FORCE_ICEORYX2_BUILD:-false}" != 'true' ]]; then
        # The wheel is stored in the bind-mounted workspace. Reuse it first so
        # startup does not depend on PyPI being reachable; this is important on
        # machines where pypi.org TLS is intercepted or unavailable.
        log "复用已有 iceoryx2 wheel: $(basename "${wheel}")"
    else
        log "构建 iceoryx2 Python binding (${revision:0:12})"
        "${VENV_PYTHON}" -m pip install \
            --disable-pip-version-check --quiet 'maturin>=1.8,<2'

        rm -f -- "${ICEORYX_DIST}"/iceoryx2-*.whl
        "${VENV_PYTHON}" -m maturin build --release \
            --manifest-path "${ICEORYX_MANIFEST}" \
            --target-dir "${ICEORYX_TARGET}" \
            --out "${ICEORYX_DIST}"

        wheel="$(find "${ICEORYX_DIST}" -maxdepth 1 -type f \
            -name 'iceoryx2-*.whl' -print | sort | tail -n 1)"
        if [[ -z "${wheel}" ]]; then
            log 'maturin 没有生成 iceoryx2 wheel'
            exit 1
        fi
    fi

    # Install the pinned Python dependency from the bind-mounted cache first.
    # This keeps startup independent of PyPI/TLS connectivity.
    local flatbuffers_wheel
    flatbuffers_wheel="$(find "${ICEORYX_DIST}" -maxdepth 1 -type f \
        -name 'flatbuffers-25.12.19-*.whl' -print | sort | tail -n 1)"
    if [[ -z "${flatbuffers_wheel}" ]]; then
        log '缺少离线依赖 flatbuffers==25.12.19，请将对应 wheel 放入 .docker/iceoryx2/dist/'
        exit 1
    fi
    "${VENV_PYTHON}" -m pip install \
        --disable-pip-version-check --no-index --force-reinstall \
        "${flatbuffers_wheel}"

    # The wheel metadata still declares flatbuffers; --no-deps prevents pip
    # from contacting PyPI a second time while installing the local binding.
    "${VENV_PYTHON}" -m pip install \
        --disable-pip-version-check --no-index --no-deps --force-reinstall "${wheel}"

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
        # Python 3.8 evaluates list[str]/dict[str, ...] at import time.
        # Add the future import before importing any iceoryx2 module.
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

    if [[ -d "${REPO_ROOT}/third_party/AUV_zit6_cmake/upper_examples" \
          && -d "${REPO_ROOT}/third_party/AUV_zit6_cmake/zit6_interfaces" ]]; then
        if [[ -f "${REPO_ROOT}/third_party/AUV_zit6_cmake/UserApp/Config/config.json" ]]; then
            log '构建 ZIT6 ROS 接口'
            colcon build --symlink-install --parallel-workers "${workers}" \
                --base-paths \
                    "${REPO_ROOT}/third_party/AUV_zit6_cmake/upper_examples" \
                    "${REPO_ROOT}/third_party/AUV_zit6_cmake/zit6_interfaces" \
                --packages-select zit6_interfaces upper_examples
            # shellcheck disable=SC1091
            set +u
            source install/setup.bash
            set -u
        else
            log '跳过 upper_examples：缺少 UserApp/Config/config.json'
            colcon build --symlink-install --parallel-workers "${workers}" \
                --base-paths "${REPO_ROOT}/third_party/AUV_zit6_cmake/zit6_interfaces" \
                --packages-select zit6_interfaces
        fi
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
