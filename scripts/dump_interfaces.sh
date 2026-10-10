#!/usr/bin/env bash
#
# T3b: snapshot the live ROS 2 interface surface of a running stack.
#
# Why a live snapshot when docs/interfaces/topic_contract.md is already checked
# against the source?  Because the contract can only be as good as what the source
# says, and some endpoints are not visible in the source at all:
#
#   * topics/services/actions that exist only at runtime (f-string built names,
#     nodes started by a preset, micro-ROS endpoints in HIL);
#   * the **message type of scene-declared publishers** -- a Stonefish
#     `<ros_publisher topic=.../>` is written by C++ we do not parse, so the
#     checker only verifies that the scene *declares* the topic.  The first run of
#     this script immediately found one wrong row that way
#     (`/auv/dvl_altitude` is `sensor_msgs/Range`, not `std_msgs/Float32`);
#   * QoS, which no amount of AST reading can tell you, and which is where the
#     big-image rate problems come from.
#
# Usage::
#
#     source /opt/ros/jazzy/setup.bash
#     source workspace_auv/install/setup.bash
#     source workspace_sim/install/setup.bash
#     ros2 launch uv_sim_bringup sim_bringup.py headless:=true ...   # other terminal
#     scripts/dump_interfaces.sh                          # -> snapshots/<date>_<mode>/
#     scripts/dump_interfaces.sh --rates                  # + per-topic Hz (slow)
#     scripts/dump_interfaces.sh --rates --out /tmp/x
#
# The directory is named `<YYYY-MM-DD>_<mode>`, where `<mode>` comes from
# `--mode` (default `sil-console`): a snapshot without its mode is a trap, since
# the GPU stack has camera images and the console stack does not, and HIL has a
# different owner for every `/zit6/state/*` topic.
#
# `rates.txt` is deliberately **opt-in**: measuring needs ~6 s per topic, and the
# numbers are only meaningful while the stack is actually running.  Topics with no
# publisher are recorded as `-` rather than skipped, so a later reader can tell
# "silent" from "not measured".

set -u

RATES=0
MODE="sil-console"
OUT=""
while [ $# -gt 0 ]; do
    case "$1" in
        --rates) RATES=1; shift ;;
        --mode)  MODE="${2:?--mode needs a value}"; shift 2 ;;
        --out)   OUT="${2:?--out needs a value}"; shift 2 ;;
        -h|--help) sed -n '2,40p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DATE="$(date +%F)"
OUT="${OUT:-${REPO_ROOT}/docs/interfaces/snapshots/${DATE}_${MODE}}"
mkdir -p "${OUT}"

if ! command -v ros2 >/dev/null 2>&1; then
    echo "ros2 not found: source /opt/ros/jazzy/setup.bash and both workspaces first" >&2
    exit 2
fi

# `ros2` prints Fast DDS shared-memory chatter on stderr in sandboxes; keep the
# artifacts to what the tools actually report.
ROS2="ros2"
strip() { grep -v -E "RTPS_|SHM Transport|Shared Memory Manager|Failed to create segment" || true; }

echo "== nodes =="
${ROS2} node list 2>&1 | strip | grep -v '^$' | sort > "${OUT}/nodes.txt"

echo "== topics =="
${ROS2} topic list -t 2>&1 | strip | grep -v '^$' | sort > "${OUT}/topics.txt"

echo "== services =="
${ROS2} service list -t 2>&1 | strip | grep -v '^$' | sort > "${OUT}/services.txt"

echo "== actions =="
${ROS2} action list -t 2>&1 | strip | grep -v '^$' | sort > "${OUT}/actions.txt"

echo "== QoS (per topic with at least one publisher) =="
: > "${OUT}/qos.txt"
while read -r topic _type; do
    [ -n "${topic}" ] || continue
    info="$(${ROS2} topic info -v "${topic}" 2>&1 | strip)"
    case "${info}" in
        *"Publisher count: 0"*) continue ;;
    esac
    {
        echo "### ${topic}"
        echo "${info}"
        echo
    } >> "${OUT}/qos.txt"
done < "${OUT}/topics.txt"

if [ "${RATES}" -eq 1 ]; then
    echo "== rates (this takes ~7 s per topic) =="
    : > "${OUT}/rates.txt"
    printf '# measured while the stack was running; "-" means no publisher (not "unmeasured")\n' \
        >> "${OUT}/rates.txt"
    while read -r topic _type; do
        [ -n "${topic}" ] || continue
        rate="$(timeout 7 ${ROS2} topic hz "${topic}" 2>/dev/null \
                | strip | grep -m1 "average rate" | sed 's/.*average rate: //')"
        [ -n "${rate}" ] || rate="-"
        printf '%-46s %s\n' "${topic}" "${rate}" >> "${OUT}/rates.txt"
    done < "${OUT}/topics.txt"
fi

{
    echo "date        : $(date -Is)"
    echo "mode        : ${MODE}"
    echo "repo        : ${REPO_ROOT}"
    echo "ros_distro  : ${ROS_DISTRO:-unknown}"
    echo "ros_domain  : ${ROS_DOMAIN_ID:-0}"
    echo "with_rates  : ${RATES}"
    echo "generated_by: scripts/dump_interfaces.sh"
    echo
    echo "Read the mode before comparing snapshots: sil-console has no camera images,"
    echo "sil-gpu has them, hil has a real MCU owning /zit6/state/*.  QoS depth shows"
    echo "UNKNOWN on this rmw (Fast DDS + ros2 topic info -v): reliability/durability"
    echo "are trustworthy, the KEEP_LAST depth is not reported."
} > "${OUT}/meta.txt"

echo
echo "wrote ${OUT}"
ls -1 "${OUT}"
