#!/usr/bin/env bash
set -eo pipefail

ROOT="${RM_AUTO_AIM_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"
set -u

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$ROOT/tools/handeye_calibration/cyclonedds_wsl_local.xml"

case "${1:-}" in
  camera)
    exec ros2 launch hik_camera hik_camera.launch.py \
      publish_bayer:=true params_file:="$ROOT/hik_camera/config/camera_params.yaml"
    ;;
  tf)
    exec ros2 run rm_serial_driver gimbal_tf_broadcaster.py --ros-args \
      --params-file "$ROOT/rm_serial_driver/config/gimbal_tf_provisional_handeye.yaml"
    ;;
  vision)
    export PATH="$(dirname "$ROOT")/.venv/bin:$PATH"
    export YOLO_ARMOR_MODEL="$ROOT/deployment/yolo_v8_gray140_candidate_20260826/model.pt"
    exec ros2 launch auto_aim_bringup yolo_auto_aim.launch.py \
      yolo_params:="$ROOT/auto_aim_bringup/params/yolo_detector_params_v8_frozen_candidate.yaml" \
      tracker_params:="$ROOT/auto_aim_bringup/params/tracker_params.yaml" \
      publish_debug:=true start_static_tf:=false
    ;;
  aimer)
    exec ros2 run rm_aimer basic_aimer_node.py --ros-args \
      --params-file "$ROOT/rm_aimer/config/basic_aimer.yaml" \
      --params-file "$ROOT/rm_aimer/config/controller_preview.yaml"
    ;;
  preview)
    exec /usr/bin/python3 "$ROOT/rm_aimer/scripts/controller_command_preview.py" \
      --ros-args --params-file "$ROOT/rm_aimer/config/controller_preview.yaml"
    ;;
  rviz)
    exec ros2 run rviz2 rviz2 -d \
      "$ROOT/auto_aim_bringup/rviz/vision_display_test.rviz"
    ;;
  serial-readonly)
    exec /usr/bin/python3 \
      "$ROOT/install/rm_serial_driver/lib/rm_serial_driver/serial_bridge_node.py" \
      --ros-args --params-file "$ROOT/rm_serial_driver/config/serial_bridge.yaml" \
      -p transport:=windows -p port:=COM3 -p receive_only:=true
    ;;
  serial-duplex)
    exec /usr/bin/python3 \
      "$ROOT/install/rm_serial_driver/lib/rm_serial_driver/serial_bridge_node.py" \
      --ros-args --params-file "$ROOT/rm_serial_driver/config/serial_bridge.yaml" \
      -p transport:=windows_duplex -p port:=COM3 -p receive_only:=false
    ;;
  *)
    echo "Unknown component: ${1:-<missing>}" >&2
    exit 2
    ;;
esac
