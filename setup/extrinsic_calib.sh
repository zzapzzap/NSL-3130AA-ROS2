#!/usr/bin/env bash
# Extrinsic calibration for NSL-3130AA (LiDAR <-> Camera).
#
# Usage: ./setup/extrinsic_calib.sh [camera_id] [image_topic] [lidar_topic] [amplitude_topic] [points_per_frame]
#   camera_id        : camera serial or ID; auto-detected from calib_output if omitted
#   image_topic      : ROS topic for RGB image   (default /rgb/image_raw)
#   lidar_topic      : ROS topic for point cloud (default /point_cloud)
#   amplitude_topic  : ROS topic for amplitude image (default /roboscanAmpl, empty disables assisted picker)
#   points_per_frame : number of marker correspondences stored per frame (default 5)
#
# Requires:
#   1. camera.launch.py running (provides image + point_cloud topics)
#   2. setup/intrinsic_calib.sh completed first
#
# Interactive keys (once running):
#   [s] store frame  [c] calibrate  [r] reset  [q] quit
#
# Output: <repo>/calib_output/<camera_id>/extrinsic.yml

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    sed -n '2,19p' "$0"
    exit 0
fi

CAMERA_ID="${1:-}"
IMAGE_TOPIC="${2:-/rgb/image_raw}"
LIDAR_TOPIC="${3:-/point_cloud}"
AMPLITUDE_TOPIC="${4:-/roboscanAmpl}"
POINTS_PER_FRAME="${5:-5}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CALIB_DIR="$REPO_ROOT/calib_output"
DETECT_SCRIPT="$REPO_ROOT/NSL3130_driver/src/roboscan_nsl3130/scripts/detect_camera_id.py"

source /opt/ros/humble/setup.bash
WS_SETUP="$REPO_ROOT/../../install/setup.bash"
[ -f "$WS_SETUP" ] && source "$WS_SETUP"

# Auto-detect: USB serial first, then fall back to intrinsic.yml filename
if [ -z "$CAMERA_ID" ]; then
    CAMERA_ID=$(python3 "$DETECT_SCRIPT" 2>/dev/null)
    if [ -n "$CAMERA_ID" ]; then
        echo "[extrinsic] Auto-detected camera_id: $CAMERA_ID"
    else
        YML=$(ls "$CALIB_DIR"/*/intrinsic.yml 2>/dev/null | head -1)
        if [ -z "$YML" ]; then
            echo "[ERROR] Camera not detected via USB and no */intrinsic.yml found in $CALIB_DIR"
            echo "        Connect the camera, or run ./setup/intrinsic_calib.sh first."
            exit 1
        fi
        CAMERA_ID=$(basename "$(dirname "$YML")")
        echo "[extrinsic] Using camera_id from intrinsic.yml: $CAMERA_ID"
    fi
fi

echo ""
echo "  Camera: $CAMERA_ID    Image: $IMAGE_TOPIC    LiDAR: $LIDAR_TOPIC"
echo "  Amplitude: ${AMPLITUDE_TOPIC:-disabled}    Points/frame: $POINTS_PER_FRAME"
echo "  Output: $CALIB_DIR"
echo ""

ros2 run roboscan_nsl3130 extrinsic_calibration_node.py \
    --camera-id   "$CAMERA_ID" \
    --image-topic "$IMAGE_TOPIC" \
    --lidar-topic "$LIDAR_TOPIC" \
    --amplitude-topic "$AMPLITUDE_TOPIC" \
    --points-per-frame "$POINTS_PER_FRAME" \
    --output-dir  "$CALIB_DIR"
