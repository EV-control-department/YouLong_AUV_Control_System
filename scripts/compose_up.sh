#!/usr/bin/env bash
# Start Compose with automatic host input-device detection.

set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

compose_files=(
    -f "${PROJECT_ROOT}/compose.yaml"
)

if [[ -d /dev/input ]]; then
    compose_files+=(
        -f "${PROJECT_ROOT}/compose/joystick.yaml"
    )
    echo "检测到 /dev/input，启用手柄设备映射。" >&2

    # Use the host input-node group when available. An explicit INPUT_GID
    # still takes precedence through Compose's environment interpolation.
    if [[ -z "${INPUT_GID:-}" ]]; then
        detected_input_gid="$(
            find /dev/input -maxdepth 1 -type c -printf '%G\n' 2>/dev/null \
                | sort -n | head -n 1
        )"
        if [[ -n "${detected_input_gid}" ]]; then
            export INPUT_GID="${detected_input_gid}"
        fi
    fi
else
    echo "未检测到 /dev/input，跳过手柄设备映射。" >&2
fi

if [[ "${YOULONG_RUNTIME:-sim}" == "real" ]]; then
    compose_files+=(
        -f "${PROJECT_ROOT}/compose/real.yaml"
    )
    echo "YOULONG_RUNTIME=real，启用 V4L2/硬件设备映射。" >&2
fi

case "${YOULONG_GPU:-cpu}" in
    cpu|none|"")
        ;;
    nvidia)
        compose_files+=(
            -f "${PROJECT_ROOT}/compose/nvidia.yaml"
        )
        echo "YOULONG_GPU=nvidia，启用 NVIDIA GPU。" >&2
        ;;
    *)
        echo "不支持的 YOULONG_GPU 值：${YOULONG_GPU}（可选 cpu 或 nvidia）。" >&2
        exit 2
        ;;
esac

if [[ "$#" -eq 0 ]]; then
    set -- up
fi

cd "${PROJECT_ROOT}"
exec docker compose "${compose_files[@]}" "$@"
