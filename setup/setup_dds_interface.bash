#!/usr/bin/env bash
# Restrict FastDDS (the default ROS 2 RMW) to the 192.168.0.x machine-to-machine
# LAN so it never advertises the camera-only NIC (192.168.2.x). Also add unicast
# initial peers for 192.168.0.51-70, so discovery still works on switches/routers
# that do not forward multicast reliably. Without this, remote peers may show up
# partially or not at all and point clouds / TF (/tf_static) stop flowing.
#
# The 192.168.2.x link is host<->ToF-sensor only (used by the driver's nanolib,
# NOT by DDS), so DDS never needs it.
#
# Add ONE line to ~/.bashrc (after you source ROS / the workspace), on EVERY machine:
#     source ~/colcon_ws/src/NSL-3130AA-ROS2/setup/setup_dds_interface.bash
#
# It re-detects the local 192.168.0.x address each shell. Camera Edge machines
# are expected to live in 192.168.0.51-60 (namespace /cam_51 ... /cam_60);
# Host/viewer machines are expected to live in 192.168.0.61-70. Restart all ROS 2
# processes after enabling it.

_nsl_lan_ip="$(ip -4 -o addr show 2>/dev/null | grep -oE '192\.168\.0\.[0-9]+' | head -n1)"
_nsl_peer_spec="${NSL_DDS_INITIAL_PEERS:-51-70}"

_nsl_expand_peer_spec() {
    local spec="$1"
    local part start end value
    [[ -n "$spec" && "${spec,,}" != "off" && "${spec,,}" != "false" && "${spec,,}" != "none" ]] || return 0
    spec="${spec//,/ }"
    for part in $spec; do
        if [[ "$part" =~ ^([0-9]+)-([0-9]+)$ ]]; then
            start="${BASH_REMATCH[1]}"
            end="${BASH_REMATCH[2]}"
            for ((value=start; value<=end; value++)); do
                printf '%s\n' "$value"
            done
        elif [[ "$part" =~ ^192\.168\.0\.([0-9]+)$ ]]; then
            printf '%s\n' "${BASH_REMATCH[1]}"
        elif [[ "$part" =~ ^[0-9]+$ ]]; then
            printf '%s\n' "$part"
        fi
    done | awk '!seen[$0]++'
}

_nsl_initial_peers_xml() {
    local octet
    while IFS= read -r octet; do
        [[ -n "$octet" ]] || continue
        cat <<XML
                        <locator>
                            <udpv4>
                                <address>192.168.0.${octet}</address>
                            </udpv4>
                        </locator>
XML
    done < <(_nsl_expand_peer_spec "$_nsl_peer_spec")
}

if [ -n "${_nsl_lan_ip}" ]; then
    _nsl_dds_xml="${HOME}/.ros/fastdds_nsl.xml"
    mkdir -p "${HOME}/.ros"
    cat > "${_nsl_dds_xml}" <<XML
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <profiles>
        <transport_descriptors>
            <transport_descriptor>
                <transport_id>nsl_lan</transport_id>
                <type>UDPv4</type>
                <maxInitialPeersRange>40</maxInitialPeersRange>
                <interfaceWhiteList>
                    <address>${_nsl_lan_ip}</address>
                </interfaceWhiteList>
            </transport_descriptor>
        </transport_descriptors>
        <participant profile_name="nsl_lan_only" is_default_profile="true">
            <rtps>
                <userTransports>
                    <transport_id>nsl_lan</transport_id>
                </userTransports>
                <useBuiltinTransports>false</useBuiltinTransports>
                <builtin>
                    <initialPeersList>
$(_nsl_initial_peers_xml)
                    </initialPeersList>
                </builtin>
            </rtps>
        </participant>
    </profiles>
</dds>
XML
    export FASTRTPS_DEFAULT_PROFILES_FILE="${_nsl_dds_xml}"
    export NSL_DDS_INITIAL_PEERS="${_nsl_peer_spec}"
else
    echo "[nsl-dds] WARNING: no 192.168.0.x address found — DDS interface not restricted." >&2
fi
unset _nsl_lan_ip _nsl_dds_xml _nsl_peer_spec
unset -f _nsl_expand_peer_spec _nsl_initial_peers_xml
