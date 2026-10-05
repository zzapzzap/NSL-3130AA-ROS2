#!/usr/bin/env bash
set -euo pipefail

project_root="${SIRIUS_ROBOT_DT_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"
set +u
source "${project_root}/setup/env.bash"
set -u
NSL_HOST_IP="$(python3 "${project_root}/controller/src/NSL-3130AA-ROS2/setup/camera_network.py" --network-field host)"

host_ip="${NSL_HOST_IP:-192.168.3.166}"
port="${NSL_DISCOVERY_PORT:-11811}"

# The server itself must not inherit client discovery or a custom transport
# profile from an interactive shell.
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE

exec fastdds discovery -i 0 -l "${host_ip}" -p "${port}"
