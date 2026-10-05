#!/usr/bin/env bash
# Load the Edge runtime, validate configured V4L2 devices, then launch uv_camera.
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/source_workspace.sh"

sim_mode="${UV_CAMERA_SIM_MODE:-false}"
enable_front="${UV_CAMERA_ENABLE_FRONT:-true}"
enable_down="${UV_CAMERA_ENABLE_DOWN:-true}"
front_device="${UV_CAMERA_FRONT_DEVICE:-/dev/video2}"
down_device="${UV_CAMERA_DOWN_DEVICE:-/dev/video0}"

normalize_bool() {
    case "${1,,}" in
        true|1|yes|on) printf true ;;
        false|0|no|off) printf false ;;
        *) printf '无效布尔值：%s（请用 true/false）\n' "$1" >&2; return 2 ;;
    esac
}
sim_mode="$(normalize_bool "${sim_mode}")"
enable_front="$(normalize_bool "${enable_front}")"
enable_down="$(normalize_bool "${enable_down}")"

show_devices() {
    printf '当前 /dev/video*：\n' >&2
    local found=false device
    for device in /dev/video*; do
        [[ -e "${device}" ]] || continue
        found=true
        ls -l "${device}" >&2
    done
    if [[ "${found}" == false ]]; then
        printf '  (没有视频设备节点)\n' >&2
    fi
    if command -v v4l2-ctl >/dev/null 2>&1; then
        printf 'v4l2-ctl 设备映射：\n' >&2
        v4l2-ctl --list-devices >&2 || true
    fi
}

check_device() {
    local camera="$1" device="$2"
    if [[ ! -c "${device}" ]]; then
        printf '启用的%s相机设备不存在或不是字符设备：%s\n' "${camera}" "${device}" >&2
        show_devices
        printf '用 UV_CAMERA_%s_DEVICE=/dev/videoN 覆盖设备路径。\n' "${camera^^}" >&2
        return 1
    fi
    if [[ ! -r "${device}" || ! -w "${device}" ]]; then
        printf '当前用户无权读写%s相机设备：%s\n' "${camera}" "${device}" >&2
        ls -l "${device}" >&2
        printf '检查 video 用户组权限；修改组后需重新登录。\n' >&2
        return 1
    fi
}

if [[ "${sim_mode}" == false ]]; then
    if [[ "${enable_front}" == true ]]; then check_device front "${front_device}" || exit $?; fi
    if [[ "${enable_down}" == true ]]; then check_device down "${down_device}" || exit $?; fi
    if [[ "${enable_front}" == false && "${enable_down}" == false ]]; then
        printf '真机模式至少要启用一个相机。\n' >&2
        exit 2
    fi
fi

printf '启动 uv_camera：sim_mode=%s front=%s(%s) down=%s(%s)\n' \
    "${sim_mode}" "${enable_front}" "${front_device}" "${enable_down}" "${down_device}"
exec ros2 launch uv_camera camera_launch.py \
    "sim_mode:=${sim_mode}" \
    "enable_front:=${enable_front}" \
    "enable_down:=${enable_down}" \
    "front_camera_device:=${front_device}" \
    "down_camera_device:=${down_device}" \
    "$@"
