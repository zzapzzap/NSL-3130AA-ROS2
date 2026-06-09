#!/usr/bin/env bash
set -euo pipefail

setup_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${setup_dir}/.." && pwd)"
camera_ip="${NSL_CAMERA_IP:-192.168.2.220}"
camera_netmask="${NSL_CAMERA_NETMASK:-255.255.255.0}"
camera_gateway="${NSL_CAMERA_GATEWAY:-192.168.2.1}"
factory_camera_ip="${NSL_FACTORY_CAMERA_IP:-192.168.0.220}"
factory_host_ip="${NSL_FACTORY_HOST_IP:-192.168.0.190}"
domain_id="${ROS_DOMAIN_ID:-42}"

usage() {
    printf 'Usage: %s [--set-camera-ip]\n' "$0"
    printf '  --set-camera-ip  Use the attached USB camera to write %s/%s gw %s, then stop.\n' \
        "$camera_ip" "$camera_netmask" "$camera_gateway"
}

ensure_bashrc_line() {
    local line="$1"
    touch "${HOME}/.bashrc"
    if ! grep -Fqx "$line" "${HOME}/.bashrc"; then
        printf '%s\n' "$line" >> "${HOME}/.bashrc"
        printf '[fleet] appended to ~/.bashrc: %s\n' "$line"
    fi
}

ensure_single_dds_source() {
    local line="source ${setup_dir}/setup_dds_interface.bash"
    local tmp
    tmp="$(mktemp)"
    awk -v exact="$line" '
        /NSL-3130AA-ROS2\/(setup\/)?setup_dds_interface\.bash/ {
            if (!seen) {
                print exact
                seen = 1
            }
            next
        }
        { print }
        END {
            if (!seen) {
                print exact
            }
        }
    ' "${HOME}/.bashrc" > "$tmp"
    if ! cmp -s "$tmp" "${HOME}/.bashrc"; then
        mv "$tmp" "${HOME}/.bashrc"
        printf '[fleet] normalized ~/.bashrc DDS source: %s\n' "$line"
    else
        rm -f "$tmp"
    fi
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

ping_once() {
    ping -c 1 -W 1 "$1" >/dev/null 2>&1
}

cleanup_factory_route() {
    local iface="${1:-}"
    [[ -n "$iface" ]] || return 0
    sudo ip route del "${factory_camera_ip}/32" dev "$iface" 2>/dev/null || true
    sudo ip addr del "${factory_host_ip}/24" dev "$iface" 2>/dev/null || true
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
fi

lan_ip="$(ip -4 -o addr show 2>/dev/null | grep -oE '192\.168\.0\.[0-9]+' | head -n1 || true)"
cam_nic_ip="$(ip -4 -o addr show 2>/dev/null | grep -oE '192\.168\.2\.[0-9]+' | head -n1 || true)"
cam_iface="$(ip -4 -o addr show 2>/dev/null | awk '/192\.168\.2\./ {print $2; exit}' || true)"

if [[ -z "$lan_ip" ]]; then
    printf '[fleet] WARNING: no 192.168.0.x LAN IP found; namespace:=auto and DDS LAN pinning will not work.\n' >&2
else
    octet="${lan_ip##*.}"
    if (( octet < 50 || octet > 59 )); then
        printf '[fleet] NOTE: LAN IP is %s. Camera Edge range is expected to be 192.168.0.50-59; hosts may be outside this range.\n' "$lan_ip"
    else
        printf '[fleet] Edge LAN IP %s -> namespace /cam_%s\n' "$lan_ip" "$octet"
    fi
fi

if [[ -z "$cam_nic_ip" ]]; then
    printf '[fleet] WARNING: no 192.168.2.x camera NIC IP found. Configure the Edge-camera NIC before streaming.\n' >&2
else
    printf '[fleet] Camera NIC IP %s; target camera IP %s\n' "$cam_nic_ip" "$camera_ip"
fi

ensure_bashrc_line "export ROS_DOMAIN_ID=${domain_id}"
ensure_single_dds_source

source_ros_setup /opt/ros/humble/setup.bash
source_ros_setup "${HOME}/colcon_ws/install/setup.bash"

if [[ "${1:-}" == "--set-camera-ip" ]]; then
    if ping_once "$camera_ip"; then
        printf '[fleet] Camera already reachable at %s. No IP write needed.\n' "$camera_ip"
        exit 0
    fi

    serial="$(python3 "${repo_root}/NSL3130_driver/src/roboscan_nsl3130/scripts/detect_camera_id.py" 2>/dev/null || true)"
    if [[ -n "$serial" ]]; then
        printf '[fleet] Trying USB IP write for serial %s -> %s\n' "$serial" "$camera_ip"
        if ros2 run roboscan_nsl3130 change_camera_ip "$camera_ip" "$camera_netmask" "$camera_gateway" "$serial"; then
            printf '[fleet] Power-cycle the camera now, then verify: ping -c 1 %s\n' "$camera_ip"
            exit 0
        fi
        printf '[fleet] USB SDK open/write failed; trying factory Ethernet fallback (%s).\n' "$factory_camera_ip" >&2
    else
        printf '[fleet] USB camera serial not detected; trying factory Ethernet fallback (%s).\n' "$factory_camera_ip" >&2
    fi

    if [[ -z "$cam_iface" ]]; then
        printf '[fleet] ERROR: no 192.168.2.x camera NIC found for Ethernet fallback.\n' >&2
        exit 1
    fi
    if ! sudo -n true 2>/dev/null; then
        printf '[fleet] ERROR: sudo is needed to add a temporary %s route on %s.\n' "$factory_camera_ip" "$cam_iface" >&2
        exit 1
    fi

    cleanup_factory_route "$cam_iface"
    trap 'cleanup_factory_route "$cam_iface"' EXIT
    sudo ip addr add "${factory_host_ip}/24" dev "$cam_iface" noprefixroute
    sudo ip route replace "${factory_camera_ip}/32" dev "$cam_iface" src "$factory_host_ip"

    if ! ping_once "$factory_camera_ip"; then
        printf '[fleet] ERROR: camera is not reachable at target %s or factory %s.\n' "$camera_ip" "$factory_camera_ip" >&2
        exit 1
    fi

    printf '[fleet] Writing camera IP over Ethernet %s -> %s\n' "$factory_camera_ip" "$camera_ip"
    if ! ros2 run roboscan_nsl3130 change_camera_ip "$camera_ip" "$camera_netmask" "$camera_gateway" "$factory_camera_ip"; then
        if ! ping_once "$camera_ip"; then
            printf '[fleet] ERROR: Ethernet fallback failed and %s is still unreachable.\n' "$camera_ip" >&2
            exit 1
        fi
    fi
    cleanup_factory_route "$cam_iface"
    trap - EXIT
    if ping_once "$camera_ip"; then
        printf '[fleet] Camera reachable at %s. Power-cycle once to confirm the IP persists.\n' "$camera_ip"
    else
        printf '[fleet] IP write command finished. Power-cycle the camera, then verify: ping -c 1 %s\n' "$camera_ip"
    fi
    exit 0
fi

printf '[fleet] Runtime launch:\n'
printf '  ros2 launch roboscan_nsl3130 camera.launch.py use_rviz:=false use_rqt:=false\n'
printf '[fleet] Recovery-only USB IP write:\n'
printf '  %s --set-camera-ip\n' "$0"
