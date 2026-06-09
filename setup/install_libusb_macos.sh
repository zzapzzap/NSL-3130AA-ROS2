#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "${repo_root}/NSL3130_driver/src/roboscan_nsl3130/nsl_lib/script/install_libusb_macos.sh" "$@"
