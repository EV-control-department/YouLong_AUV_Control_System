#!/usr/bin/env bash
# 录制 uv_camera 的原始 MJPEG 视频流到 $R/datas/video_records。
#
# 为什么拉 8090 而不是 go2rtc 1984:
#   实测(go2rtc 1.9.14)对纯 HTTP 源不暴露 ffmpeg 可拉的 .mjs/.mjpeg/.mp4 端点,
#   /api/stream 也 404,只有 WebRTC(浏览器)。而 go2rtc 这 4 路正是从 uv_camera
#   的 8090 MJPEG 服务拉来的(http://127.0.0.1:8090/front 等)。
#   8090 bind 0.0.0.0,可跨机访问 —— 所以直接拉 8090,内容与 go2rtc 一致。
#
# 输出格式: 短分段 MPEG-TS(.ts)。TS 无 moov/tail 依赖,已经完成的段在断电后仍可播。
#
# 默认 2 路原始流:
#   front / down
#
# 用法:
#   ./scripts/record_go2rtc.sh                 # 一直录, Ctrl-C 停止
#   ./scripts/record_go2rtc.sh 60              # 录 60 秒后自动停
# 覆盖默认(环境变量):
#   GORTC_HOST=192.168.16.10 GORTC_PORT=8090 ./scripts/record_go2rtc.sh
#   GORTC_STREAMS="front down" VIDEO_FPS=10 ./scripts/record_go2rtc.sh

set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

# 默认主机是 AGX(跑 uv_camera + go2rtc)。端口默认 8090(uv_camera MJPEG 源)。
GORTC_HOST="${GORTC_HOST:-192.168.16.10}"
GORTC_PORT="${GORTC_PORT:-8090}"
GORTC_STREAMS="${GORTC_STREAMS:-front down}"
OUT_DIR="${OUT_DIR:-${PROJECT_ROOT}/datas/video_records}"
VIDEO_FPS="${VIDEO_FPS:-10}"
SEGMENT_SECONDS="${SEGMENT_SECONDS:-2}"

# 录多久(秒);0 或空 = 无限(需 Ctrl-C)
DURATION_SEC="${1:-0}"

if ! command -v ffmpeg >/dev/null 2>&1; then
  echo "错误:需要 ffmpeg 才能录制(转 mp4)。" >&2
  exit 1
fi

mkdir -p "$OUT_DIR"

# ── 探测目标是否在线 ─────────────────────────────────────────────
# MJPEG 是不会主动结束的长连接，不能用 curl 的退出码判断：--max-time
# 到期时正常流也会返回 28。只检查已经收到的 HTTP 状态和 Content-Type。
probe_headers="$(
  curl -sS --max-time 2 -D - \
    "http://${GORTC_HOST}:${GORTC_PORT}/front" \
    -o /dev/null 2>/dev/null || true
)"
if ! grep -Eiq '^HTTP/[0-9.]+[[:space:]]+200([[:space:]]|$)' <<<"$probe_headers" \
   || ! grep -Eiq '^Content-[Tt]ype:[[:space:]]*multipart/x-mixed-replace' <<<"$probe_headers"; then
  echo "提示:/front 探测失败。确认 uv_camera 在 ${GORTC_HOST}:${GORTC_PORT} 已运行;" >&2
  echo "     仍将尝试(若 ffmpeg 能连上即可)。" >&2
fi

ts="$(date +%Y%m%d_%H%M%S)"
declare -A PIDS
declare -A FILES
SYNC_PID=''

stop_recording() {
  local stream
  for stream in "${!PIDS[@]}"; do
    kill -INT "${PIDS[$stream]}" 2>/dev/null || true
  done
}

wait_for_recorders() {
  local stream
  for stream in "${!PIDS[@]}"; do
    wait "${PIDS[$stream]}" 2>/dev/null || true
  done
}

sync_recorded_files() {
  local file
  for file in "$OUT_DIR"/*_"$ts"_*.ts; do
    if [ -f "$file" ]; then
      # GNU sync -d calls fdatasync for this file without flushing unrelated
      # files on the machine.  The fallback keeps the script portable.
      sync -d "$file" 2>/dev/null || sync "$file" || true
    fi
  done
}

sync_loop() {
  declare -A observed=()
  declare -A synced=()
  local file size
  while :; do
    for file in "$OUT_DIR"/*_"$ts"_*.ts; do
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

trap stop_recording INT TERM

record_one() {
  local stream="$1"
  local url="http://${GORTC_HOST}:${GORTC_PORT}/${stream}"
  local pattern="$OUT_DIR/${stream}_${ts}_%06d.ts"
  local playlist="$OUT_DIR/${stream}_${ts}.m3u8"
  # MPEG-TS: 无 moov/tail 依赖;每 2 秒结束一个文件,降低断电损失范围。
  ffmpeg -hide_banner -loglevel warning \
    -f mpjpeg \
    -i "$url" \
    -map 0:v:0 -an \
    -c:v libx264 -preset ultrafast -tune zerolatency \
    -pix_fmt yuv420p -r "$VIDEO_FPS" -fps_mode cfr \
    -force_key_frames "expr:gte(t,n_forced*${SEGMENT_SECONDS})" \
    -flush_packets 1 \
    -f segment -segment_time "$SEGMENT_SECONDS" \
    -segment_format mpegts -reset_timestamps 1 \
    -segment_list "$playlist" -segment_list_type m3u8 \
    -segment_list_size 0 -y "$pattern" &
  PIDS["$stream"]=$!
  FILES["$stream"]="$playlist"
  echo "[$stream] -> $OUT_DIR/${stream}_${ts}_*.ts (${VIDEO_FPS} FPS)"
}

for s in $GORTC_STREAMS; do
  record_one "$s"
done

# Sync only completed/stable segments.  This bounds power-loss damage while
# avoiding a global `sync` on every frame.
sync_loop &
SYNC_PID=$!

if [ "$DURATION_SEC" -gt 0 ]; then
  echo "录制 ${DURATION_SEC}s 后自动停止..."
  sleep "$DURATION_SEC" || true
  # SIGINT 让 ffmpeg 优雅收尾 TS;即便强杀,TS 已写部分也可播。
  stop_recording
  wait_for_recorders
else
  echo "无限录制,按 Ctrl-C 停止。"
  wait_for_recorders
fi

if [ -n "$SYNC_PID" ]; then
  kill "$SYNC_PID" 2>/dev/null || true
  wait "$SYNC_PID" 2>/dev/null || true
fi
sync_recorded_files

trap - INT TERM
for s in "${!PIDS[@]}"; do
  if [ -f "${FILES[$s]}" ]; then
    echo "✓ ${FILES[$s]} ($(du -h "${FILES[$s]}" | cut -f1))"
  fi
done
echo "完成。输出目录: $OUT_DIR"
