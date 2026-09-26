#!/usr/bin/env bash
# Convenience wrapper for the timestamp-aligned unified go2rtc session recorder.

set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

GORTC_HOST="${GORTC_HOST:-127.0.0.1}"
GORTC_PORT="${GORTC_PORT:-1984}"
RECORD_ROOT="${RECORD_ROOT:-${OUT_DIR:-${PROJECT_ROOT}/records/sessions}}"
RECORD_DIR="${RECORD_DIR:-}"
RECORD_TIMESTAMP="${RECORD_TIMESTAMP:-}"
GO2RTC_VIDEO_FORMAT="${GO2RTC_VIDEO_FORMAT:-jpeg}"
GORTC_STREAMS="${GORTC_STREAMS:-front front_annotated down down_annotated}"
DURATION_SEC="${1:-0}"
shift || true

if ! command -v ros2 >/dev/null 2>&1; then
    echo "错误：找不到 ros2。请先 source 工作区 setup.bash。" >&2
    exit 1
fi
if ! [[ "$GORTC_PORT" =~ ^[0-9]+$ ]] || (( GORTC_PORT < 1 || GORTC_PORT > 65535 )); then
    echo "错误：HTTP 端口无效：${GORTC_PORT}" >&2
    exit 1
fi
if ! [[ "$DURATION_SEC" =~ ^[0-9]+$ ]]; then
    echo "错误：录制时长必须是非负整数秒：${DURATION_SEC}" >&2
    exit 1
fi
if [[ "$GO2RTC_VIDEO_FORMAT" != "jpeg" && "$GO2RTC_VIDEO_FORMAT" != "ts" ]]; then
    echo "错误：GO2RTC_VIDEO_FORMAT 只支持 jpeg 或 ts。" >&2
    exit 1
fi

has_front=0
has_down=0
has_front_annotated=0
has_down_annotated=0
for stream in $GORTC_STREAMS; do
    case "$stream" in
        front) has_front=1 ;;
        down) has_down=1 ;;
        front_annotated) has_front_annotated=1 ;;
        down_annotated) has_down_annotated=1 ;;
        *) echo "错误：未知 go2rtc 流名称：${stream}" >&2; exit 1 ;;
    esac
done
if ((has_front != has_down || has_front_annotated != has_down_annotated)); then
    echo "错误：uv_record 按 unannotated/annotated 流组选择，不能单独选择一台相机。" >&2
    exit 1
fi
has_unannotated=$has_front
has_annotated=$has_front_annotated
if ((has_unannotated && has_annotated)); then
    STREAM_MODE=both
elif ((has_annotated)); then
    STREAM_MODE=annotated
elif ((has_unannotated)); then
    STREAM_MODE=unannotated
else
    echo "错误：GORTC_STREAMS 不能为空。" >&2
    exit 1
fi

command=(ros2 run uv_record record
    --record-mode go2rtc
    --go2rtc-stream-mode "$STREAM_MODE"
    --go2rtc-video-format "$GO2RTC_VIDEO_FORMAT"
    --host "$GORTC_HOST"
    --port "$GORTC_PORT")
if [[ -n "$RECORD_DIR" ]]; then
    command+=(--session-dir "$RECORD_DIR")
elif [[ -n "$RECORD_TIMESTAMP" ]]; then
    command+=(--session-dir "${RECORD_ROOT}/${RECORD_TIMESTAMP}")
else
    command+=(--output-root "$RECORD_ROOT")
fi
command+=("$@")

echo "开始 timestamp-aligned uv_record go2rtc session（streams=${STREAM_MODE}）。"
"${command[@]}" &
recorder_pid=$!
timer_pid=
stop_recorder() {
    if kill -0 "$recorder_pid" 2>/dev/null; then
        kill -INT "$recorder_pid" 2>/dev/null || true
    fi
}
trap stop_recorder INT TERM
if ((DURATION_SEC > 0)); then
    (sleep "$DURATION_SEC"; kill -INT "$recorder_pid" 2>/dev/null || true) &
    timer_pid=$!
else
    echo "按 Ctrl-C 停止。"
fi
set +e
wait "$recorder_pid"
result=$?
set -e
if [[ -n "$timer_pid" ]]; then
    kill "$timer_pid" 2>/dev/null || true
    wait "$timer_pid" 2>/dev/null || true
fi
trap - INT TERM
exit "$result"
