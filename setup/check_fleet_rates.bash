#!/usr/bin/env bash
set -euo pipefail

setup_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ws_root="$(cd "${setup_dir}/../../.." && pwd)"
seconds="${NSL_RATE_SECONDS:-8}"
topic_name="${NSL_RATE_TOPIC:-point_cloud_rgb}"

usage() {
    printf 'Usage: %s [cam_id ...]\n' "$0"
    printf '  cam_id examples: 51 52 cam_53\n'
    printf '  env: NSL_RATE_SECONDS=8 NSL_RATE_TOPIC=point_cloud_rgb|point_cloud|rgb/image_raw\n'
}

source_ros_setup() {
    local setup_file="$1"
    if [[ -f "$setup_file" ]]; then
        set +u
        # shellcheck disable=SC1090
        source "$setup_file"
        set -u
    fi
}

normalize_cam_id() {
    local cam="$1"
    cam="${cam#/}"
    cam="${cam#cam_}"
    printf '%s\n' "$cam"
}

detect_cameras() {
    ros2 topic list 2>/dev/null |
        awk -F/ -v topic="$topic_name" '$0 ~ "^/cam_[^/]+/camera/" topic "$" {print $2}' |
        sed 's/^cam_//' |
        sort -V
}

measure_hz() {
    local topic="$1"
    local log="$2"
    timeout "${seconds}s" ros2 topic hz "$topic" --window 20 >"$log" 2>&1 || true
    awk '/average rate:/ {rate=$3} END {print rate ? rate : "n/a"}' "$log"
}

measure_bw() {
    local topic="$1"
    local log="$2"
    timeout "${seconds}s" ros2 topic bw "$topic" >"$log" 2>&1 || true
    awk '/MB\/s/ {bw=$1} END {print bw ? bw : "n/a"}' "$log"
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
fi

source_ros_setup /opt/ros/humble/setup.bash
source_ros_setup "${ws_root}/install/setup.bash"

cams=()
if (( $# )); then
    for cam in "$@"; do
        cams+=("$(normalize_cam_id "$cam")")
    done
else
    while IFS= read -r cam; do
        [[ -n "$cam" ]] && cams+=("$cam")
    done < <(detect_cameras)
fi

if (( ${#cams[@]} == 0 )); then
    printf '[rates] ERROR: no /cam_*/camera/%s topics found.\n' "$topic_name" >&2
    exit 1
fi

printf '[rates] measuring /cam_N/camera/%s for %ss per metric\n' "$topic_name" "$seconds"
printf '%-8s %-12s %-12s %s\n' 'camera' 'hz' 'MB/s' 'topic'

tmpdir="$(mktemp -d)"
trap 'rm -rf "$tmpdir"' EXIT

best_hz=0
rows=()
for cam in "${cams[@]}"; do
    topic="/cam_${cam}/camera/${topic_name}"
    hz_log="${tmpdir}/cam_${cam}_hz.log"
    bw_log="${tmpdir}/cam_${cam}_bw.log"
    hz="$(measure_hz "$topic" "$hz_log")"
    bw="$(measure_bw "$topic" "$bw_log")"
    rows+=("${cam}|${hz}|${bw}|${topic}")
    if [[ "$hz" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
        best_hz="$(awk -v a="$best_hz" -v b="$hz" 'BEGIN {print b > a ? b : a}')"
    fi
done

for row in "${rows[@]}"; do
    IFS='|' read -r cam hz bw topic <<< "$row"
    note=''
    if [[ "$hz" =~ ^[0-9]+([.][0-9]+)?$ && "$best_hz" != "0" ]]; then
        ratio="$(awk -v h="$hz" -v b="$best_hz" 'BEGIN {print h / b}')"
        if awk -v r="$ratio" 'BEGIN {exit !(r < 0.8)}'; then
            note='LOW: check that Edge CPU/RViz/rqt are off and sensor tuning matches'
        fi
    fi
    printf '%-8s %-12s %-12s %s %s\n' "cam_${cam}" "$hz" "$bw" "$topic" "$note"
done
