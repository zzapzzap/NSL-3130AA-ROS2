#!/usr/bin/env bash
# Start the edge camera driver and single-view human pose runtime together.

set -euo pipefail

ws="${SIRIUS_ROBOT_DT_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"
nsl_src="${NSL_SRC_DIR:-${ws}/controller/src/NSL-3130AA-ROS2}"
pose_src="${ROS_HUMANPOSE_SRC_DIR:-${ws}/controller/src/ros_humanpose}"

set +u
source "${ws}/setup/env.bash"
if [[ -f "${ws}/controller/install/local_setup.bash" ]]; then
    source "${ws}/controller/install/local_setup.bash"
fi
if [[ -f "${nsl_src}/setup/setup_dds_interface.bash" ]]; then
    source "${ws}/setup/fleet_env.bash"
fi
set -u

detect_edge_octet() {
    local address octet
    address="$(ip -4 -o addr show 2>/dev/null | grep -oE '192\.168\.3\.[0-9]+' | head -n1 || true)"
    octet="${address##*.}"
    if [[ "$octet" =~ ^[0-9]+$ ]] && ((octet >= 161 && octet <= 165)); then
        printf '%s\n' "$octet"
        return 0
    fi
    octet="${ROS_HUMANPOSE_OCTET:-${NSL_EDGE_OCTET:-}}"
    if [[ "$octet" =~ ^[0-9]+$ ]] && ((octet >= 161 && octet <= 165)); then
        printf '%s\n' "$octet"
    fi
}

wait_for_edge_lan() {
    local octet="$1"
    local timeout="${NSL_EDGE_LAN_WAIT_SEC:-30}"
    local elapsed=0
    local pattern='192\.168\.3\.'
    [[ -n "$octet" ]] && pattern="192\\.168\\.3\\.${octet}"

    while (( elapsed < timeout )); do
        if ip -4 -o addr show 2>/dev/null | grep -qE "$pattern"; then
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    echo "[edge-agent] WARNING: Edge/Host network 192.168.3.${octet:-x} not ready after ${timeout}s"
    return 1
}

wait_for_camera_ip() {
    local octet="$1"
    local timeout="${NSL_CAMERA_WAIT_SEC:-45}"
    local elapsed=0
    local camera_ip="${NSL_CAMERA_IP:-}"
    [[ "${NSL_CONNECTION:-ethernet}" != usb ]] || return 0

    if [[ -z "$camera_ip" || "${camera_ip,,}" == "auto" ]]; then
        [[ "$octet" =~ ^[0-9]+$ ]] || return 0
        camera_ip="$(python3 "${nsl_src}/setup/camera_network.py" --octet "$octet" --field sensor_host)" || return
        [[ -n "$camera_ip" ]] || { echo "[edge-agent] sensor_host is not configured for cam_${octet}" >&2; return 1; }
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

singleview_weight_name() {
    local octet="${ROS_HUMANPOSE_OCTET:-${NSL_EDGE_OCTET:-}}"
    if [[ -z "$octet" ]]; then
        octet="$(detect_edge_octet)"
    fi
    [[ -n "$octet" ]] || return 1
    printf '3d_pose_cam%s.pkl\n' "$octet"
}

singleview_weight_dir() {
    printf '%s\n' "${ROS_HUMANPOSE_WEIGHT_LOCAL_DIR:-${ws}/controller/src/ros_humanpose/weight}"
}

singleview_weight_path() {
    local file local_dir
    file="$(singleview_weight_name)" || return 1
    local_dir="$(singleview_weight_dir)"
    printf '%s/%s\n' "$local_dir" "$file"
}

fetch_singleview_weight() {
    [[ "${ROS_HUMANPOSE_SYNC_WEIGHTS_ON_START:-1}" =~ ^(1|true|yes)$ ]] || return 1
    local file local_dir tmp
    file="$(singleview_weight_name)" || return 1
    local_dir="$(singleview_weight_dir)"
    mkdir -p "$local_dir"
    tmp="${local_dir}/.${file}.tmp"
    # SSH-free: fetch from the host's weight server over ROS2
    # (/fleet/weights/get, nsl-weight-server@.service on the host). No keys
    # needed — a new edge only has to join the DDS domain.
    echo "[edge-agent] fetch weight ${file} via /fleet/weights/get"
    if ros2 run ros_humanpose weight_client.py get "$file" "$tmp" \
            --wait "${ROS_HUMANPOSE_WEIGHT_WAIT_SEC:-15}"; then
        mv "$tmp" "${local_dir}/${file}"
        return 0
    else
        rm -f "$tmp" "${tmp}.part"
        echo "[edge-agent] weight fetch skipped; waiting for local or host copy if needed"
        return 1
    fi
}

sync_singleview_weight() {
    fetch_singleview_weight || true
}

wait_for_singleview_weight() {
    [[ "${ROS_HUMANPOSE_WAIT_FOR_SINGLEVIEW_WEIGHT:-1}" =~ ^(1|true|yes)$ ]] || return 0
    local camera_pid="$1"
    local retry_sec="${ROS_HUMANPOSE_WEIGHT_RETRY_SEC:-15}"
    local path file

    path="$(singleview_weight_path)" || {
        echo "[edge-agent] WARNING: no edge octet; cannot derive single-view weight name"
        return 0
    }
    file="$(basename "$path")"

    while [[ ! -s "$path" ]]; do
        echo "[edge-agent] single-view weight missing: ${path}"
        fetch_singleview_weight || true
        if [[ -s "$path" ]]; then
            break
        fi
        if ! kill -0 "$camera_pid" 2>/dev/null; then
            echo "[edge-agent] camera process exited while waiting for ${file}"
            return 1
        fi
        echo "[edge-agent] camera stays up; waiting ${retry_sec}s for ${file}"
        sleep "$retry_sec"
    done

    echo "[edge-agent] single-view weight ready: ${path}"
}

camera_args=(
    connection:="${NSL_CONNECTION:-ethernet}"
    namespace:="${NSL_EDGE_NAMESPACE:-auto}"
    use_rviz:=false
    use_rqt:=false
    use_multiview_tf:="${NSL_USE_MULTIVIEW_TF:-true}"
    use_rgb_compressor:="${NSL_USE_RGB_COMPRESSOR:-true}"
    rgb_jpeg_quality:="${NSL_RGB_JPEG_QUALITY:-80}"
    # Native 1920x1080 JPEG (no downscaling). The compressed image is RELIABLE
    # (rgb_compressor_node.py): it is fragmented for the wire and reassembled
    # losslessly at the host, so the full-resolution frame arrives complete for
    # host-side image analysis. Set NSL_RGB_COMPRESSED_WIDTH only if you ever
    # want a smaller panel; default keeps full res.
    rgb_compressed_width:="${NSL_RGB_COMPRESSED_WIDTH:-0}"
    rgb_compressed_frame_skip:="${NSL_RGB_COMPRESSED_FRAME_SKIP:-0}"
)
calib_args="${NSL_CALIB_ARGS:-}"
if [[ -z "$calib_args" ]]; then
    # Reject implausibly close planes and gross LiDAR pulls while preserving
    # the camera-only PnP estimate as a safe fallback.
    calib_args="--slide-min-range ${NSL_CALIB_MIN_RANGE:-0.5} --max-depth-delta ${NSL_CALIB_MAX_DEPTH_DELTA:-0.8}"
    if [[ "${NSL_DEBUG_ROI:-1}" =~ ^(1|true|yes)$ ]]; then
        calib_args+=" --debug-roi true"
    fi
fi
if [[ -n "$calib_args" ]]; then
    camera_args+=(calib_args:="$calib_args")
fi

pose_args=(
    namespace:="${NSL_EDGE_NAMESPACE:-auto}"
    use_rviz:=false
)
if [[ -n "${NSL_POSE_CONFIG:-}" ]]; then
    case "$NSL_POSE_CONFIG" in
        controller/*) pose_config="${SIRIUS_ROBOT_DT_ROOT}/${NSL_POSE_CONFIG}" ;;
        /*) pose_config="$NSL_POSE_CONFIG" ;;
        *) echo '[edge-agent] NSL_POSE_CONFIG must be a project-relative or absolute module path.' >&2; exit 2 ;;
    esac
    [[ -f "$pose_config" ]] || { echo "[edge-agent] Missing prepared pose config: $pose_config" >&2; exit 1; }
    pose_args+=("config_path:=$pose_config")
fi

camera_pid=""
pose_pid=""
shutdown() {
    echo "[edge-agent] stopping child processes"
    local pids=()
    [[ -n "${camera_pid:-}" ]] && pids+=("$camera_pid")
    [[ -n "${pose_pid:-}" ]] && pids+=("$pose_pid")
    if ((${#pids[@]})); then
        kill "${pids[@]}" 2>/dev/null || true
        wait "${pids[@]}" 2>/dev/null || true
    fi
}
trap shutdown INT TERM EXIT

echo "[edge-agent] starting camera.launch.py ${camera_args[*]}"
ros2 launch roboscan_nsl3130 camera.launch.py "${camera_args[@]}" &
camera_pid=$!

if [[ "${NSL_ENABLE_POSE:-true}" == false ]]; then
    echo '[edge-agent] camera-only mode; inference and weight wait are disabled'
    wait "$camera_pid"
    exit $?
fi
sleep "${NSL_POSE_START_DELAY_SEC:-8}"
if ! wait_for_singleview_weight "$camera_pid"; then
    echo "[edge-agent] exiting so systemd can retry after RestartSec"
    exit 1
fi
echo "[edge-agent] starting singleview_pose.launch.py ${pose_args[*]}"
ros2 launch ros_humanpose singleview_pose.launch.py "${pose_args[@]}" &
pose_pid=$!

wait -n "${camera_pid}" "${pose_pid}"
exit $?
