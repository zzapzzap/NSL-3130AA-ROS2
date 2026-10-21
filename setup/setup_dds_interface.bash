#!/usr/bin/env bash
# Fleet DDS uses unicast discovery at .166 and only the equipment LAN NIC.
_nsl_lan_ip="$(ip -4 -o addr show 2>/dev/null | awk '
{for (i=1;i<NF;i++) if ($i=="inet") {split($(i+1),a,"/");
 if (a[1] ~ /^192[.]168[.]3[.](16[1-4]|166)$/) {print a[1];exit}}}')"
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
if [[ -z "$_nsl_lan_ip" ]]; then
    echo '[nsl-dds] WARNING: no local DDS address (.161-.164 or .166); fleet setup skipped.' >&2
elif [[ "${NSL_DDS_MODE:-server}" != server ]]; then
    echo '[nsl-dds] WARNING: only server mode is allowed; multicast DDS is disabled.' >&2
else
    _nsl_dds_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    _nsl_dds_xml="${HOME}/.ros/fastdds_sirius_lan.xml"
    if python3 "${_nsl_dds_dir}/dds_profile.py" --ip "$_nsl_lan_ip" --output "$_nsl_dds_xml" >/dev/null; then
        export ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        export ROS_DISCOVERY_SERVER=192.168.3.166:11811
        export FASTRTPS_DEFAULT_PROFILES_FILE="$_nsl_dds_xml"
        # Interactive Edge terminals need the complete graph for `ros2 topic list`.
        # Managed Edge services remain regular clients to limit discovery traffic.
        [[ "$_nsl_lan_ip" != 192.168.3.166 && $- != *i* ]] || export ROS_SUPER_CLIENT=TRUE
    else
        echo '[nsl-dds] ERROR: failed to generate LAN-only DDS profile.' >&2
        return 1
    fi
fi
unset _nsl_lan_ip _nsl_dds_dir _nsl_dds_xml
