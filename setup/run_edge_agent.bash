#!/usr/bin/env bash
# Start the edge camera driver and single-view human pose runtime together.

set -euo pipefail

ws="${COLCON_WS:-${HOME}/colcon_ws}"
nsl_src="${NSL_SRC_DIR:-${ws}/src/NSL-3130AA-ROS2}"
pose_src="${ROS_HUMANPOSE_SRC_DIR:-${ws}/src/ros_humanpose}"
runtime_env="${HOME}/.ros/nsl_runtime.env"

set +u
source /opt/ros/humble/setup.bash
if [[ -f "${ws}/install/setup.bash" ]]; then
    source "${ws}/install/setup.bash"
fi
if [[ -f "${runtime_env}" ]]; then
    source "${runtime_env}"
elif [[ -f "${nsl_src}/setup/setup_dds_interface.bash" ]]; then
    source "${nsl_src}/setup/setup_dds_interface.bash"
fi
set -u

detect_edge_octet() {
    local octet="${ROS_HUMANPOSE_OCTET:-${NSL_EDGE_OCTET:-}}"
    if [[ -z "$octet" ]]; then
        octet="$(ip -4 -o addr show 2>/dev/null | grep -oE '192\.168\.0\.[0-9]+' | head -n1 | awk -F. '{print $4}' || true)"
    fi
    printf '%s\n' "$octet"
}

wait_for_edge_lan() {
    local octet="$1"
    local timeout="${NSL_EDGE_LAN_WAIT_SEC:-30}"
    local elapsed=0
    local pattern='192\.168\.0\.'
    [[ -n "$octet" ]] && pattern="192\\.168\\.0\\.${octet}"

    while (( elapsed < timeout )); do
        if ip -4 -o addr show 2>/dev/null | grep -qE "$pattern"; then
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    echo "[edge-agent] WARNING: Edge/Host network 192.168.0.${octet:-x} not ready after ${timeout}s"
    return 1
}

wait_for_camera_ip() {
    local octet="$1"
    local timeout="${NSL_CAMERA_WAIT_SEC:-45}"
    local elapsed=0
    local camera_ip="${NSL_CAMERA_IP:-}"

    if [[ -z "$camera_ip" || "${camera_ip,,}" == "auto" ]]; then
        [[ "$octet" =~ ^[0-9]+$ ]] || return 0
        camera_ip="192.168.2.$((octet + 150))"
    fi

    while (( elapsed < timeout )); do
        if ping -c 1 -W 1 "$camera_ip" >/dev/null 2>&1; then
            echo "[edge-agent] camera reachable: ${camera_ip}"
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done

    echo "[edge-agent] camera ${camera_ip} not reachable after ${timeout}s"
    [[ "${NSL_EDGE_REQUIRE_CAMERA:-1}" =~ ^(1|true|yes)$ ]] && return 1
    return 0
}

edge_octet="$(detect_edge_octet)"
if [[ -n "$edge_octet" ]]; then
    export NSL_EDGE_OCTET="$edge_octet"
    export ROS_HUMANPOSE_OCTET="${ROS_HUMANPOSE_OCTET:-$edge_octet}"
    export NSL_EDGE_NAMESPACE="cam_${edge_octet}"
fi

wait_for_edge_lan "$edge_octet" || true
if ! wait_for_camera_ip "$edge_octet"; then
    echo "[edge-agent] exiting so systemd can retry after RestartSec"
    exit 1
fi

sync_singleview_weight() {
    [[ "${ROS_HUMANPOSE_SYNC_WEIGHTS_ON_START:-1}" =~ ^(1|true|yes)$ ]] || return 0
    local octet="${ROS_HUMANPOSE_OCTET:-${NSL_EDGE_OCTET:-}}"
    local host="${ROS_HUMANPOSE_WEIGHT_HOST:-${NSL_HOST_IP:-192.168.0.61}}"
    local user_name="${ROS_HUMANPOSE_WEIGHT_USER:-${USER}}"
    local remote_dir="${ROS_HUMANPOSE_WEIGHT_REMOTE_DIR:-~/colcon_ws/src/ros_humanpose/weight}"
    local local_dir="${ROS_HUMANPOSE_WEIGHT_LOCAL_DIR:-${pose_src}/weight}"
    if [[ -z "$octet" ]]; then
        octet="$(detect_edge_octet)"
    fi
    [[ -n "$octet" ]] || return 0
    mkdir -p "$local_dir"
    local file="3d_pose_cam${octet}.pkl"
    local tmp="${local_dir}/.${file}.tmp"
    echo "[edge-agent] sync weight ${user_name}@${host}:${remote_dir}/${file}"
    if rsync -az --partial --timeout=10 "${user_name}@${host}:${remote_dir}/${file}" "$tmp"; then
        mv "$tmp" "${local_dir}/${file}"
    else
        rm -f "$tmp"
        echo "[edge-agent] weight sync skipped; using local copy if present"
    fi
}

sync_singleview_weight

camera_args=(
    namespace:="${NSL_EDGE_NAMESPACE:-auto}"
    use_rviz:=false
    use_rqt:=false
    use_rgb_compressor:="${NSL_USE_RGB_COMPRESSOR:-true}"
    rgb_jpeg_quality:="${NSL_RGB_JPEG_QUALITY:-80}"
    rgb_compressed_width:="${NSL_RGB_COMPRESSED_WIDTH:-0}"
    rgb_compressed_frame_skip:="${NSL_RGB_COMPRESSED_FRAME_SKIP:-0}"
)

pose_args=(
    namespace:="${NSL_EDGE_NAMESPACE:-auto}"
    use_rviz:=false
)

echo "[edge-agent] starting camera.launch.py ${camera_args[*]}"
ros2 launch roboscan_nsl3130 camera.launch.py "${camera_args[@]}" &
camera_pid=$!

sleep "${NSL_POSE_START_DELAY_SEC:-8}"
echo "[edge-agent] starting singleview_pose.launch.py ${pose_args[*]}"
ros2 launch ros_humanpose singleview_pose.launch.py "${pose_args[@]}" &
pose_pid=$!

shutdown() {
    echo "[edge-agent] stopping child processes"
    kill "${camera_pid}" "${pose_pid}" 2>/dev/null || true
    wait "${camera_pid}" "${pose_pid}" 2>/dev/null || true
}
trap shutdown INT TERM EXIT

wait -n "${camera_pid}" "${pose_pid}"
exit $?
