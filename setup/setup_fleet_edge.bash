#!/usr/bin/env bash
set -euo pipefail

setup_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${setup_dir}/.." && pwd)"
fleet_first_octet=51
fleet_last_octet=59
camera_host_offset=50
camera_sensor_offset=150
camera_ip="${NSL_CAMERA_IP:-auto}"
camera_netmask="${NSL_CAMERA_NETMASK:-255.255.255.0}"
camera_gateway="${NSL_CAMERA_GATEWAY:-192.168.2.1}"
factory_camera_ip="${NSL_FACTORY_CAMERA_IP:-192.168.0.220}"
factory_host_ip="${NSL_FACTORY_HOST_IP:-192.168.0.190}"
legacy_camera_ip="${NSL_LEGACY_CAMERA_IP:-192.168.2.220}"
camera_apply_wait_sec="${NSL_CAMERA_APPLY_WAIT_SEC:-15}"
domain_id="${ROS_DOMAIN_ID:-42}"

usage() {
    printf 'Usage: %s [--check|--set-camera-ip|--host-only]\n' "$0"
    printf '  no args          Configure host DDS/ROS env, then ensure the camera IP from the Set number.\n'
    printf '  --check          Print the rule-based addresses only; do not change files or camera IP.\n'
    printf '  --set-camera-ip  Write the rule-based camera IP, then stop.\n'
    printf '  --host-only      Configure host DDS/ROS env only; do not touch the camera.\n'
    printf '\nRule: Set N=%s-%s -> LAN 192.168.0.N, camera NIC 192.168.2.(N+%s), camera 192.168.2.(N+%s), gateway 192.168.2.1\n' \
        "$fleet_first_octet" "$fleet_last_octet" "$camera_host_offset" "$camera_sensor_offset"
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

wait_for_ping() {
    local ip="$1"
    local seconds="${2:-8}"
    local elapsed=0

    while (( elapsed < seconds )); do
        if ping_once "$ip"; then
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    return 1
}

cleanup_factory_route() {
    local iface="${1:-}"
    [[ -n "$iface" ]] || return 0
    sudo ip route del "${factory_camera_ip}/32" dev "$iface" 2>/dev/null || true
    sudo ip addr del "${factory_host_ip}/24" dev "$iface" 2>/dev/null || true
}

detect_usb_serial() {
    python3 "${repo_root}/NSL3130_driver/src/roboscan_nsl3130/scripts/detect_camera_id.py" 2>/dev/null || true
}

run_change_camera_ip() {
    local endpoint="$1"
    local output status current_ip

    set +e
    output="$(ros2 run roboscan_nsl3130 change_camera_ip "$camera_ip" "$camera_netmask" "$camera_gateway" "$endpoint" 2>&1)"
    status=$?
    set -e

    printf '%s\n' "$output"

    current_ip="$(printf '%s\n' "$output" | awk '
        /Camera current IP/ {
            for (i = 1; i <= NF; i++) {
                if ($i ~ /^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$/) {
                    print $i
                    exit
                }
            }
        }
    ')"
    if [[ -n "$current_ip" ]]; then
        last_reported_camera_ip="$current_ip"
    fi

    return "$status"
}

try_reachable_ip_write() {
    local source_ip="$1"
    local label="$2"

    [[ -n "$source_ip" ]] || return 1
    [[ "$source_ip" != "$camera_ip" ]] || return 1
    if ! ping_once "$source_ip"; then
        return 1
    fi

    printf '[fleet] Trying Ethernet IP write via %s %s -> %s\n' "$label" "$source_ip" "$camera_ip"
    if run_change_camera_ip "$source_ip"; then
        printf '[fleet] Ethernet IP write via %s saved. Waiting up to %ss for %s ...\n' "$source_ip" "$camera_apply_wait_sec" "$camera_ip"
    else
        printf '[fleet] Ethernet write via %s returned nonzero. Checking %s anyway ...\n' "$source_ip" "$camera_ip" >&2
    fi

    if wait_for_ping "$camera_ip" "$camera_apply_wait_sec"; then
        printf '[fleet] Camera reachable at %s.\n' "$camera_ip"
        return 0
    fi

    return 1
}

is_fleet_octet() {
    local octet="${1:-}"
    [[ "$octet" =~ ^[0-9]+$ ]] && (( octet >= fleet_first_octet && octet <= fleet_last_octet ))
}

resolve_rule_addresses() {
    edge_octet=""
    expected_cam_nic_ip=""
    expected_camera_ip=""

    if [[ -n "$lan_ip" ]]; then
        edge_octet="${lan_ip##*.}"
    fi

    if is_fleet_octet "$edge_octet"; then
        expected_cam_nic_ip="192.168.2.$((edge_octet + camera_host_offset))"
        expected_camera_ip="192.168.2.$((edge_octet + camera_sensor_offset))"
    fi

    if [[ "${camera_ip,,}" == "auto" ]]; then
        if [[ -z "$expected_camera_ip" ]]; then
            printf '[fleet] ERROR: camera_ip=auto needs Edge LAN IP 192.168.0.%s-%s. Current LAN IP: %s\n' \
                "$fleet_first_octet" "$fleet_last_octet" "${lan_ip:-none}" >&2
            return 1
        fi
        camera_ip="$expected_camera_ip"
    fi

    if [[ "${camera_gateway,,}" == "auto" ]]; then
        camera_gateway="192.168.2.1"
    fi

    return 0
}

print_address_rule() {
    if [[ -n "${edge_octet:-}" && -n "${expected_camera_ip:-}" ]]; then
        printf '[fleet] Set %s rule: LAN %s -> namespace /cam_%s, camera NIC %s, camera %s gw %s\n' \
            "$edge_octet" "$lan_ip" "$edge_octet" "$expected_cam_nic_ip" "$camera_ip" "$camera_gateway"
    fi
}

ensure_camera_ip() {
    local serial
    local attempted_usb=0
    last_reported_camera_ip=""

    if ping_once "$camera_ip"; then
        printf '[fleet] Camera already reachable at %s. No IP write needed.\n' "$camera_ip"
        return 0
    fi

    serial="$(detect_usb_serial)"
    if [[ -n "$serial" ]]; then
        attempted_usb=1
        printf '[fleet] Trying USB IP write for serial %s -> %s\n' "$serial" "$camera_ip"
        if run_change_camera_ip "$serial"; then
            printf '[fleet] USB IP write saved. Waiting up to %ss for %s ...\n' "$camera_apply_wait_sec" "$camera_ip"
            if wait_for_ping "$camera_ip" "$camera_apply_wait_sec"; then
                printf '[fleet] Camera reachable at %s.\n' "$camera_ip"
                return 0
            fi
            printf '[fleet] USB path did not make %s reachable; trying reachable Ethernet IPs before factory setup.\n' "$camera_ip" >&2
        else
            printf '[fleet] USB write returned nonzero. Checking %s before Ethernet setup ...\n' "$camera_ip" >&2
            if wait_for_ping "$camera_ip" "$camera_apply_wait_sec"; then
                printf '[fleet] Camera reachable at %s.\n' "$camera_ip"
                return 0
            fi
            printf '[fleet] USB write did not make %s reachable; trying reachable Ethernet IPs before factory setup.\n' "$camera_ip" >&2
        fi

        if try_reachable_ip_write "$last_reported_camera_ip" "reported current IP"; then
            return 0
        fi
        if [[ "$legacy_camera_ip" != "$last_reported_camera_ip" ]] && try_reachable_ip_write "$legacy_camera_ip" "legacy IP"; then
            return 0
        fi
    else
        printf '[fleet] USB camera serial not detected; trying Ethernet setup path (%s).\n' "$factory_camera_ip" >&2
    fi

    if [[ -z "$cam_iface" ]]; then
        printf '[fleet] ERROR: no 192.168.2.x camera NIC found for Ethernet setup.\n' >&2
        return 1
    fi
    if ! sudo -n true 2>/dev/null; then
        printf '[fleet] ERROR: sudo is needed to add a temporary %s route on %s.\n' "$factory_camera_ip" "$cam_iface" >&2
        return 1
    fi

    cleanup_factory_route "$cam_iface"
    if ! sudo ip addr add "${factory_host_ip}/24" dev "$cam_iface" noprefixroute; then
        printf '[fleet] ERROR: failed to add temporary %s/24 on %s.\n' "$factory_host_ip" "$cam_iface" >&2
        cleanup_factory_route "$cam_iface"
        return 1
    fi
    if ! sudo ip route replace "${factory_camera_ip}/32" dev "$cam_iface" src "$factory_host_ip"; then
        printf '[fleet] ERROR: failed to add temporary route to %s on %s.\n' "$factory_camera_ip" "$cam_iface" >&2
        cleanup_factory_route "$cam_iface"
        return 1
    fi

    if ! ping_once "$factory_camera_ip"; then
        if (( attempted_usb )); then
            printf '[fleet] Camera IP may be saved, but neither target %s nor factory %s is reachable now.\n' "$camera_ip" "$factory_camera_ip" >&2
            printf '[fleet] Power-cycle/replug the camera, then run: ping -c 1 %s\n' "$camera_ip" >&2
            cleanup_factory_route "$cam_iface"
            return 2
        fi
        printf '[fleet] ERROR: camera is not reachable at target %s or factory %s.\n' "$camera_ip" "$factory_camera_ip" >&2
        cleanup_factory_route "$cam_iface"
        return 1
    fi

    printf '[fleet] Writing camera IP over Ethernet %s -> %s\n' "$factory_camera_ip" "$camera_ip"
    if ! run_change_camera_ip "$factory_camera_ip"; then
        printf '[fleet] Ethernet write returned nonzero. Checking %s before failing ...\n' "$camera_ip" >&2
        if ! wait_for_ping "$camera_ip" "$camera_apply_wait_sec"; then
            printf '[fleet] ERROR: Ethernet setup failed and %s is still unreachable.\n' "$camera_ip" >&2
            cleanup_factory_route "$cam_iface"
            return 1
        fi
        printf '[fleet] Camera reachable at %s.\n' "$camera_ip"
        cleanup_factory_route "$cam_iface"
        return 0
    fi
    cleanup_factory_route "$cam_iface"

    printf '[fleet] Ethernet IP write finished. Waiting up to %ss for %s ...\n' "$camera_apply_wait_sec" "$camera_ip"
    if wait_for_ping "$camera_ip" "$camera_apply_wait_sec"; then
        printf '[fleet] Camera reachable at %s.\n' "$camera_ip"
        return 0
    fi

    printf '[fleet] IP write is saved but %s is not reachable yet. Power-cycle the camera, then verify: ping -c 1 %s\n' "$camera_ip" "$camera_ip"
    return 2
}

mode="${1:-}"
if (( $# > 1 )); then
    usage >&2
    exit 2
fi

case "$mode" in
    -h|--help)
        usage
        exit 0
        ;;
    "" )
        camera_mode="best_effort"
        ;;
    --check)
        camera_mode="check"
        ;;
    --set-camera-ip|--ensure-camera-ip)
        camera_mode="strict"
        ;;
    --host-only)
        camera_mode="none"
        ;;
    *)
        usage >&2
        exit 2
        ;;
esac

lan_ip="$(ip -4 -o addr show 2>/dev/null | grep -oE '192\.168\.0\.[0-9]+' | head -n1 || true)"
cam_nic_ip="$(ip -4 -o addr show 2>/dev/null | grep -oE '192\.168\.2\.[0-9]+' | head -n1 || true)"
cam_iface="$(ip -4 -o addr show 2>/dev/null | awk '/192\.168\.2\./ {print $2; exit}' || true)"

if [[ -z "$lan_ip" ]]; then
    printf '[fleet] WARNING: no 192.168.0.x LAN IP found; namespace:=auto and DDS LAN pinning will not work.\n' >&2
else
    edge_octet="${lan_ip##*.}"
    if ! is_fleet_octet "$edge_octet"; then
        printf '[fleet] NOTE: LAN IP is %s. Edge Set range is 192.168.0.%s-%s; hosts may be outside this range.\n' \
            "$lan_ip" "$fleet_first_octet" "$fleet_last_octet"
    else
        printf '[fleet] Edge LAN IP %s -> namespace /cam_%s\n' "$lan_ip" "$edge_octet"
    fi
fi

if [[ "$camera_mode" != "none" ]]; then
    if ! resolve_rule_addresses; then
        exit 1
    fi
    print_address_rule

    if [[ -z "$cam_nic_ip" ]]; then
        printf '[fleet] WARNING: no 192.168.2.x camera NIC IP found. Expected camera NIC: %s\n' \
            "${expected_cam_nic_ip:-192.168.2.101}" >&2
        if [[ "$camera_mode" != "check" ]]; then
            printf '[fleet] ERROR: configure the camera NIC first, or run --check to inspect only.\n' >&2
            exit 1
        fi
    elif [[ -n "${expected_cam_nic_ip:-}" && "$cam_nic_ip" != "$expected_cam_nic_ip" ]]; then
        printf '[fleet] WARNING: camera NIC IP is %s, but Set %s rule expects %s.\n' \
            "$cam_nic_ip" "$edge_octet" "$expected_cam_nic_ip" >&2
        if [[ "$camera_mode" != "check" ]]; then
            printf '[fleet] ERROR: fix the camera NIC IP before writing the camera address.\n' >&2
            exit 1
        fi
    else
        printf '[fleet] Camera NIC IP %s OK; target camera IP %s\n' "$cam_nic_ip" "$camera_ip"
    fi
fi

if [[ "$camera_mode" == "check" ]]; then
    exit 0
fi

ensure_bashrc_line "export ROS_DOMAIN_ID=${domain_id}"
ensure_single_dds_source

source_ros_setup "${setup_dir}/setup_dds_interface.bash"
source_ros_setup /opt/ros/humble/setup.bash
source_ros_setup "${HOME}/colcon_ws/install/setup.bash"

camera_result=0
if [[ "$camera_mode" != "none" ]]; then
    if ensure_camera_ip; then
        camera_result=0
    else
        camera_result=$?
    fi

    if [[ "$camera_mode" == "strict" ]]; then
        if (( camera_result == 1 )); then
            exit 1
        fi
        exit 0
    fi
fi

printf '[fleet] Runtime launch:\n'
printf '  ros2 launch roboscan_nsl3130 camera.launch.py use_rviz:=false use_rqt:=false\n'
if (( camera_result == 1 )); then
    printf '[fleet] NOTE: camera IP was not verified. Check camera power/USB/Ethernet, or run:\n'
    printf '  %s --set-camera-ip\n' "$0"
elif (( camera_result == 2 )); then
    printf '[fleet] NOTE: camera IP is saved but needs a camera power-cycle before strict Ethernet launch.\n'
fi
