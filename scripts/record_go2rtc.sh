#!/usr/bin/env bash
# 录制 uv_camera 的 4 路 MJPEG 视频流到 $R/datas/video_records。
#
# 为什么拉 8090 而不是 go2rtc 1984:
#   实测(go2rtc 1.9.14)对纯 HTTP 源不暴露 ffmpeg 可拉的 .mjs/.mjpeg/.mp4 端点,
#   /api/stream 也 404,只有 WebRTC(浏览器)。而 go2rtc 这 4 路正是从 uv_camera
#   的 8090 MJPEG 服务拉来的(http://127.0.0.1:8090/front 等)。
#   8090 bind 0.0.0.0,可跨机访问 —— 所以直接拉 8090,内容与 go2rtc 一致。
#
# 输出格式: MPEG-TS(.ts)。TS 无 moov/tail 依赖,断电/强杀后已写部分必然可播(断电安全)。
#
# 4 路:
#   front / down / front_annotated / down_annotated
#
# 用法:
#   ./scripts/record_go2rtc.sh                 # 一直录, Ctrl-C 停止
#   ./scripts/record_go2rtc.sh 60              # 录 60 秒后自动停
# 覆盖默认(环境变量):
#   GORTC_HOST=192.168.16.10 GORTC_PORT=8090 ./scripts/record_go2rtc.sh
#   GORTC_STREAMS="front down" ./scripts/record_go2rtc.sh

set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

# 默认主机是 AGX(跑 uv_camera + go2rtc)。端口默认 8090(uv_camera MJPEG 源)。
GORTC_HOST="${GORTC_HOST:-192.168.16.10}"
GORTC_PORT="${GORTC_PORT:-8090}"
GORTC_STREAMS="${GORTC_STREAMS:-front down front_annotated down_annotated}"
OUT_DIR="${OUT_DIR:-${PROJECT_ROOT}/datas/video_records}"

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

trap stop_recording INT TERM

record_one() {
  local stream="$1"
  local url="http://${GORTC_HOST}:${GORTC_PORT}/${stream}"
  local file="$OUT_DIR/${stream}_${ts}.ts"
  # MPEG-TS: 无 moov/tail 依赖,断电/强杀后已写部分必然可播。
  ffmpeg -hide_banner -loglevel warning \
    -i "$url" \
    -c:v libx264 -preset veryfast -pix_fmt yuv420p \
    -fflags +genpts \
    -f mpegts \
    -y "$file" &
  PIDS["$stream"]=$!
  FILES["$stream"]=$file
  echo "[$stream] -> $file"
}

for s in $GORTC_STREAMS; do
  record_one "$s"
done

if [ "$DURATION_SEC" -gt 0 ]; then
  echo "录制 ${DURATION_SEC}s 后自动停止..."
  sleep "$DURATION_SEC" || true
  # SIGINT 让 ffmpeg 优雅收尾 TS;即便强杀,TS 已写部分也可播。
  stop_recording
else
  echo "无限录制,按 Ctrl-C 停止。"
  wait_for_recorders
fi

trap - INT TERM
for s in "${!PIDS[@]}"; do
  if [ -f "${FILES[$s]}" ]; then
    echo "✓ ${FILES[$s]} ($(du -h "${FILES[$s]}" | cut -f1))"
  fi
done
echo "完成。输出目录: $OUT_DIR"
