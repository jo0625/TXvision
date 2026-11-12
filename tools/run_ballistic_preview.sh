#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"
export PATH="$(dirname "$ROOT")/.venv/bin:$PATH"
export YOLO_ARMOR_MODEL="$ROOT/deployment/yolo_v8_gray140_candidate_20260826/model.pt"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI="file://$ROOT/tools/handeye_calibration/cyclonedds_wsl_local.xml"
export TZ=Asia/Shanghai
PREVIEW_OUTPUT="$(dirname "$ROOT")/outputs/ballistic_preview_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$PREVIEW_OUTPUT"
export ROS_HOME="$PREVIEW_OUTPUT/.ros"
export ROS_LOG_DIR="$ROS_HOME/log"
export YOLO_CONFIG_DIR="$(dirname "$ROOT")/.ros"
export MPLCONFIGDIR="$YOLO_CONFIG_DIR/matplotlib"
exec python3 "$ROOT/rm_aimer/scripts/ballistic_image_overlay.py" --output "$PREVIEW_OUTPUT" --start-pipeline "$@"
