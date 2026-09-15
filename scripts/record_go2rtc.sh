#!/usr/bin/env bash
# Record uv_camera MJPEG streams as short MPEG-TS segments.

set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

GORTC_HOST="${GORTC_HOST:-192.168.16.10}"
GORTC_PORT="${GORTC_PORT:-8090}"
GORTC_STREAMS="${GORTC_STREAMS:-front front_annotated down down_annotated}"
OUTPUT_ROOT="${OUT_DIR:-${PROJECT_ROOT}/video_record}"
RECORD_DIR="${RECORD_DIR:-}"
RECORD_TIMESTAMP="${RECORD_TIMESTAMP:-}"
VIDEO_FPS="${VIDEO_FPS:-10}"
SEGMENT_SECONDS="${SEGMENT_SECONDS:-2}"
DURATION_SEC="${1:-0}"

if ! command -v ffmpeg >/dev/null 2>&1; then
    echo "错误：容器中找不到 ffmpeg。" >&2
    exit 1
fi

# MJPEG is a long-lived HTTP response. Only inspect headers here. Streams that
# are unavailable are skipped; recording starts as long as at least one
# requested stream is available.
probe_stream() {
    local stream="$1"
    local probe_headers
    probe_headers="$(
        curl -sS --max-time 2 -D - \
            "http://${GORTC_HOST}:${GORTC_PORT}/${stream}" \
            -o /dev/null 2>/dev/null || true
    )"
    if grep -Eiq '^HTTP/[0-9.]+[[:space:]]+200([[:space:]]|$)' <<<"$probe_headers" \
       && grep -Eiq '^Content-[Tt]ype:[[:space:]]*multipart/x-mixed-replace' <<<"$probe_headers"; then
        return 0
    fi
    echo "不可用：${stream} (${GORTC_HOST}:${GORTC_PORT})" >&2
    return 1
}

available_streams=()
missing_streams=()
for stream in $GORTC_STREAMS; do
    if probe_stream "$stream"; then
        available_streams+=("$stream")
    else
        missing_streams+=("$stream")
    fi
done
if [ "${#missing_streams[@]}" -ne 0 ]; then
    echo "跳过不可用视频流：${missing_streams[*]}" >&2
fi
if [ "${#available_streams[@]}" -eq 0 ]; then
    echo "未开始录制：没有可用的视频流。" >&2
    exit 2
fi
GORTC_STREAMS="${available_streams[*]}"

# Each invocation gets its own time-stamped directory. RECORD_DIR and
# RECORD_TIMESTAMP let the GUI display and control the exact same path.
TIMESTAMP="${RECORD_TIMESTAMP:-$(date +%Y%m%d_%H%M%S)}"
OUT_DIR="${RECORD_DIR:-${OUTPUT_ROOT}/${TIMESTAMP}}"
mkdir -p "$OUT_DIR"

declare -A PIDS=()
declare -A PLAYLISTS=()
SYNC_PID=""
STOP_REQUESTED=0
RECORDER_FAILURE=0

stop_recording() {
    STOP_REQUESTED=1
    local stream
    for stream in "${!PIDS[@]}"; do
        kill -INT "${PIDS[$stream]}" 2>/dev/null || true
    done
}

sync_loop() {
    declare -A observed=()
    declare -A synced=()
    local file size
    while :; do
        for file in "$OUT_DIR"/*_"$TIMESTAMP"_*.ts; do
            [ -f "$file" ] || continue
            size="$(stat -c '%s' "$file" 2>/dev/null || true)"
            [ -n "$size" ] || continue
            if [ "${observed[$file]:-}" = "$size" ] \
               && [ -z "${synced[$file]:-}" ]; then
                sync -d "$file" 2>/dev/null || sync "$file" || true
                synced["$file"]=1
            else
                observed["$file"]="$size"
                unset 'synced[$file]'
            fi
        done
        sleep 1
    done
}

record_one() {
    local stream="$1"
    local url="http://${GORTC_HOST}:${GORTC_PORT}/${stream}"
    local pattern="$OUT_DIR/${stream}_${TIMESTAMP}_%06d.ts"
    local playlist="$OUT_DIR/${stream}_${TIMESTAMP}.m3u8"

    # Ubuntu 20.04/Foxy ships FFmpeg 4.2, which uses -vsync rather than
    # the newer -fps_mode option.
    ffmpeg -hide_banner -loglevel warning \
        -nostdin -f mpjpeg -i "$url" \
        -map 0:v:0 -an \
        -c:v libx264 -preset ultrafast -tune zerolatency \
        -pix_fmt yuv420p -r "$VIDEO_FPS" -vsync cfr \
        -force_key_frames "expr:gte(t,n_forced*${SEGMENT_SECONDS})" \
        -flush_packets 1 \
        -f segment -segment_time "$SEGMENT_SECONDS" \
        -segment_format mpegts -reset_timestamps 1 \
        -segment_list "$playlist" -segment_list_type m3u8 \
        -segment_list_size 0 -y "$pattern" &
    PIDS["$stream"]=$!
    PLAYLISTS["$stream"]="$playlist"
    echo "[$stream] -> $OUT_DIR/${stream}_${TIMESTAMP}_*.ts"
}

trap stop_recording INT TERM

for stream in $GORTC_STREAMS; do
    record_one "$stream"
done

sync_loop &
SYNC_PID=$!

if [ "$DURATION_SEC" -gt 0 ]; then
    echo "录制 ${DURATION_SEC}s 后自动停止..."
    sleep "$DURATION_SEC" || true
    stop_recording
else
    echo "正在录制：${GORTC_HOST}:${GORTC_PORT} [$GORTC_STREAMS]"
    echo "按 Ctrl-C 停止。"
fi

for stream in "${!PIDS[@]}"; do
    if ! wait "${PIDS[$stream]}" 2>/dev/null; then
        [ "$STOP_REQUESTED" -ne 0 ] || RECORDER_FAILURE=1
    fi
done

if [ -n "$SYNC_PID" ]; then
    kill "$SYNC_PID" 2>/dev/null || true
    wait "$SYNC_PID" 2>/dev/null || true
fi

for stream in "${!PLAYLISTS[@]}"; do
    playlist="${PLAYLISTS[$stream]}"
    if [ -f "$playlist" ]; then
        sync -d "$playlist" 2>/dev/null || sync "$playlist" || true
        echo "完成：$playlist"
    fi
done

if [ "$RECORDER_FAILURE" -ne 0 ]; then
    echo "错误：至少一路视频录制失败。" >&2
    exit 1
fi

echo "录制结束，输出目录：$OUT_DIR"
