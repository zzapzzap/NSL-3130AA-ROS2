#!/usr/bin/env bash
# Configure FastDDS for the NSL fleet. The default discovery-server mode works
# on LANs that block multicast; multicast mode keeps the original interface-
# restricted profile available for networks where multicast is reliable.
# DDS participants use 192.168.3.161-169; camera discovery remains 161-165.
# This validates a local LAN address, not remote peers or a security ACL.

_nsl_lan_ip="$(ip -4 -o addr show 2>/dev/null | awk '
    {
        for (field = 1; field < NF; field++) {
            if ($field != "inet") continue
            split($(field + 1), address, "/")
            if (address[1] ~ /^192[.]168[.]3[.]16[1-9]$/) {
                print address[1]
                exit
            }
        }
    }
')"
_nsl_dds_mode="${NSL_DDS_MODE:-server}"

if [ -z "${_nsl_lan_ip}" ]; then
    unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT
    unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
    echo "[nsl-dds] WARNING: no local 192.168.3.161-169 address found; DDS fleet setup skipped." >&2
elif [ "${_nsl_dds_mode}" = "server" ]; then
    export ROS_LOCALHOST_ONLY=0
    _nsl_host_ip="${NSL_HOST_IP:-192.168.3.166}"
    _nsl_discovery_port="${NSL_DISCOVERY_PORT:-11811}"
    _nsl_lan_octet="${_nsl_lan_ip##*.}"

    # Use the built-in transports verified with this fleet's discovery server.
    # Clear the legacy profile inherited from shells using multicast mode.
    unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
    export ROS_DISCOVERY_SERVER="${_nsl_host_ip}:${_nsl_discovery_port}"

    # Host tools need the complete graph for topic/node introspection. Edge
    # nodes can remain regular clients and only receive relevant matches.
    if [[ "${_nsl_lan_octet}" =~ ^[0-9]+$ ]] && \
            (( _nsl_lan_octet >= 166 && _nsl_lan_octet <= 169 )); then
        export ROS_SUPER_CLIENT=TRUE
    else
        unset ROS_SUPER_CLIENT
    fi
elif [ "${_nsl_dds_mode}" = "multicast" ]; then
    export ROS_LOCALHOST_ONLY=0
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
            </rtps>
        </participant>
    </profiles>
</dds>
XML
    unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT FASTDDS_DEFAULT_PROFILES_FILE
    export FASTRTPS_DEFAULT_PROFILES_FILE="${_nsl_dds_xml}"
else
    unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT
    unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
    echo "[nsl-dds] WARNING: unknown NSL_DDS_MODE=${_nsl_dds_mode}; expected server or multicast." >&2
fi

unset _nsl_lan_ip _nsl_dds_mode _nsl_host_ip _nsl_discovery_port
unset _nsl_lan_octet _nsl_dds_xml
