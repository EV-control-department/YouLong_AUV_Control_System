#!/usr/bin/env bash
# Start the project without NVIDIA/CUDA dependencies.
# Usage: scripts/compose_nocuda_up.sh [docker compose arguments]

set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

compose_files=(-f "${PROJECT_ROOT}/compose.nocuda.yaml")

# Mesa is optional: do not make a non-graphical or unusual host fail at
# Compose interpolation time. The base profile remains completely device-free.
if [[ "${YOULONG_MESA:-auto}" != "0" && -e /dev/dri ]]; then
    compose_files+=(-f "${PROJECT_ROOT}/compose.nocuda.mesa.yaml")
    echo "检测到 /dev/dri，启用 Mesa 图形设备映射。" >&2
else
    echo "未启用 /dev/dri，使用纯 CPU/无固定图形设备配置。" >&2
fi

if [[ -d /dev/input && "${YOULONG_JOYSTICK:-auto}" != "0" ]]; then
    compose_files+=(-f "${PROJECT_ROOT}/compose.joystick.yaml")
    if [[ -z "${INPUT_GID:-}" ]]; then
        INPUT_GID="$(find /dev/input -maxdepth 1 -type c -printf '%G\n' 2>/dev/null | sort -n | head -n 1 || true)"
        export INPUT_GID
    fi
    echo "检测到 /dev/input，启用手柄设备映射。" >&2
fi

if [[ "${YOULONG_RUNTIME:-sim}" == "real" ]]; then
    compose_files+=(-f "${PROJECT_ROOT}/compose.real.yaml")
    echo "YOULONG_RUNTIME=real，启用 V4L2/串口设备映射。" >&2
fi

if [[ "$#" -eq 0 ]]; then
    set -- up
fi

cd "${PROJECT_ROOT}"
exec docker compose "${compose_files[@]}" "$@"
