#!/usr/bin/env bash
# Intrinsic calibration for NSL-3130AA RGB camera.
#
# Usage: ./setup/intrinsic_calib.sh [board_size] [square_size] [image_topic] [camera_id]
#   board_size  : inner corners WxH           (default 8x13)
#   square_size : square side in metres       (default 0.04)
#   image_topic : ROS topic for RGB image     (default /rgb/image_raw)
#   camera_id   : ID tag written into YML     (default nsl)
#
# Requires: camera.launch.py running first (provides the image topic)
# Output:   <repo>/calib_output/<camera_id>/intrinsic.yml

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    sed -n '2,10p' "$0"
    exit 0
fi

BOARD_SIZE="${1:-8x13}"
SQUARE_SIZE="${2:-0.04}"
IMAGE_TOPIC="${3:-/rgb/image_raw}"
CAMERA_ID="${4:-}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CALIB_DIR="$REPO_ROOT/calib_output"
DETECT_SCRIPT="$REPO_ROOT/NSL3130_driver/src/roboscan_nsl3130/scripts/detect_camera_id.py"
SCRIPT="$REPO_ROOT/NSL3130_driver/src/roboscan_nsl3130/scripts/intrinsic_calibration_node.py"

mkdir -p "$CALIB_DIR"

source /opt/ros/humble/setup.bash
WS_SETUP="$REPO_ROOT/../../install/setup.bash"
[ -f "$WS_SETUP" ] && source "$WS_SETUP"

# Auto-detect camera serial via USB if not provided
if [ -z "$CAMERA_ID" ]; then
    CAMERA_ID=$(python3 "$DETECT_SCRIPT" 2>/dev/null)
    if [ -n "$CAMERA_ID" ]; then
        echo "[intrinsic] Auto-detected camera_id: $CAMERA_ID"
    else
        echo "[intrinsic] WARNING: Camera not detected via USB. Using 'nsl' as camera_id."
        echo "            Connect the camera or pass as 4th argument: ./setup/intrinsic_calib.sh 8x13 0.04 /topic N00A5060D"
        CAMERA_ID="nsl"
    fi
fi

echo ""
echo "  Board:  $BOARD_SIZE    Square: ${SQUARE_SIZE} m    Topic: $IMAGE_TOPIC    ID: $CAMERA_ID"
echo "  Output: $CALIB_DIR"
echo ""

python3 "$SCRIPT" \
    --camera-id   "$CAMERA_ID" \
    --size        "$BOARD_SIZE" \
    --square      "$SQUARE_SIZE" \
    --image-topic "$IMAGE_TOPIC" \
    --output-dir  "$CALIB_DIR"
