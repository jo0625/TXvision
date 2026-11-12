#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPONENT="$ROOT/tools/complete_system_component.sh"
RESULTS="$ROOT/test_results/complete_system"
PREFIX="rm-complete"
LEGACY_SERIAL_UNITS=(
  rm-feedback-receiver-20260923.service
  rm-complete-duplex-20260923.service
)

source_ros() {
  # ROS environment hooks probe optional variables that may be unset.
  set +u
  source /opt/ros/humble/setup.bash
  source "$ROOT/install/setup.bash"
  set -u
  export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI="file://$ROOT/tools/handeye_calibration/cyclonedds_wsl_local.xml"
}

unit_name() {
  printf '%s-%s.service\n' "$PREFIX" "$1"
}

stop_component() {
  local unit
  unit="$(unit_name "$1")"
  systemctl --user stop "$unit" 2>/dev/null || true
  for _ in {1..30}; do
    systemctl --user is-active --quiet "$unit" 2>/dev/null || return 0
    sleep 0.1
  done
}

start_component() {
  local name="$1" component="$2" unit
  unit="$(unit_name "$name")"
  if systemctl --user is-active --quiet "$unit"; then
    return 0
  fi
  stop_component "$name"
  echo "Starting $name ($component)..."
  systemd-run --user --quiet --collect --unit="$unit" \
    /bin/bash "$COMPONENT" "$component"
  sleep 0.5
  if ! systemctl --user is-active --quiet "$unit"; then
    echo "Failed to start $unit" >&2
    journalctl --user -u "$unit" --no-pager -n 20 >&2 || true
    return 1
  fi
}

restore_readonly() {
  stop_component serial-duplex
  start_component serial serial-readonly
}

stop_legacy_serial() {
  local unit
  for unit in "${LEGACY_SERIAL_UNITS[@]}"; do
    systemctl --user stop "$unit" 2>/dev/null || true
  done
}

start_all() {
  stop_legacy_serial
  for name in serial-duplex preview aimer vision tf camera serial; do
    stop_component "$name"
  done
  start_component serial serial-readonly
  start_component camera camera
  start_component tf tf
  start_component vision vision
  start_component aimer aimer
  start_component preview preview
  sleep 3
  echo "Pipeline started in receive-only mode."
}

stop_all() {
  stop_legacy_serial
  for name in rviz serial-duplex preview aimer vision tf camera serial; do
    stop_component "$name"
  done
  echo "Pipeline stopped."
}

start_rviz() {
  systemctl --user import-environment DISPLAY WAYLAND_DISPLAY XDG_RUNTIME_DIR
  start_component rviz rviz
  sleep 2
  if ! systemctl --user is-active --quiet "$(unit_name rviz)"; then
    echo "RViz exited during startup" >&2
    journalctl --user -u "$(unit_name rviz)" --no-pager -n 30 >&2 || true
    return 1
  fi
  echo "RViz started on /detector/result_img."
}

stop_rviz() {
  stop_component rviz
  echo "RViz stopped."
}

show_status() {
  for name in serial serial-duplex camera tf vision aimer preview rviz; do
    local unit state
    unit="$(unit_name "$name")"
    state="$(systemctl --user is-active "$unit" 2>/dev/null || true)"
    printf '%-40s %s\n' "$unit" "${state:-not-found}"
  done
  local unit state
  for unit in "${LEGACY_SERIAL_UNITS[@]}"; do
    state="$(systemctl --user is-active "$unit" 2>/dev/null || true)"
    printf '%-40s %s\n' "$unit" "${state:-not-found}"
  done
  echo "Windows serial ports:"
  powershell.exe -NoProfile -NonInteractive -Command \
    '[System.IO.Ports.SerialPort]::GetPortNames()' 2>/dev/null | tr -d '\r' || true
  echo "USB/IP devices:"
  usbipd.exe list 2>/dev/null | tr -d '\r' || true
}

show_config() {
  cat <<EOF
root=$ROOT
camera=$ROOT/hik_camera/config/camera_params.yaml
detector=$ROOT/auto_aim_bringup/params/yolo_detector_params_v8_frozen_candidate.yaml
tracker=$ROOT/auto_aim_bringup/params/tracker_params.yaml
tf=$ROOT/rm_serial_driver/config/gimbal_tf_provisional_handeye.yaml
aimer=$ROOT/rm_aimer/config/basic_aimer.yaml
preview=$ROOT/rm_aimer/config/controller_preview.yaml
serial=$ROOT/rm_serial_driver/config/serial_bridge.yaml
model=$ROOT/deployment/yolo_v8_gray140_candidate_20260826/model.pt
EOF
}

preflight() {
  local duration="${1:-10}" stamp output
  mkdir -p "$RESULTS"
  stamp="$(date +%Y%m%d-%H%M%S)"
  output="$RESULTS/preflight-$stamp.json"
  source_ros
  /usr/bin/python3 "$ROOT/tools/complete_system_test.py" \
    --duration "$duration" --require-target --require-readonly --output "$output"
  echo "Saved: $output"
}

follow() {
  local duration="${1:-10}" max_error="${2:-5}" max_excursion="${3:-5}"
  local confirmation="${4:-}" angle_limits="${5:-enabled}" stamp output rc=0
  local result_name="follow"
  local -a angle_args=(
    --angle-deadband-deg 0.05
    --max-error-deg "$max_error"
    --max-excursion-deg "$max_excursion"
  )
  if [[ "$confirmation" != "--confirm-motion" ]]; then
    echo "Motion test requires the final argument --confirm-motion" >&2
    echo "Example: $0 follow 10 5 5 --confirm-motion" >&2
    return 2
  fi
  if [[ "$angle_limits" == "disabled" ]]; then
    result_name="follow-controller-limited"
    angle_args=(--angle-deadband-deg 0 --no-angle-limits)
  fi

  preflight 5
  trap restore_readonly EXIT INT TERM
  stop_component serial
  start_component serial-duplex serial-duplex
  sleep 3

  mkdir -p "$RESULTS"
  stamp="$(date +%Y%m%d-%H%M%S)"
  output="$RESULTS/$result_name-$stamp.json"
  source_ros
  /usr/bin/python3 "$ROOT/tools/dynamic_follow_test.py" \
    --duration "$duration" --start-delay 2 \
    "${angle_args[@]}" \
    --output "$output" || rc=$?
  restore_readonly
  trap - EXIT INT TERM
  echo "Saved: $output"
  return "$rc"
}

follow_controller_limited() {
  local duration="${1:-10}" confirmation="${2:-}"
  if [[ "$confirmation" != "--confirm-motion" ]]; then
    echo "Controller-limited motion test requires --confirm-motion" >&2
    echo "Example: $0 follow-controller-limited 180 --confirm-motion" >&2
    return 2
  fi
  start_rviz
  follow "$duration" 5 5 "$confirmation" disabled
}

complete_test() {
  local duration="${1:-10}" max_error="${2:-5}" max_excursion="${3:-5}"
  local confirmation="${4:-}"
  if [[ "$confirmation" != "--confirm-motion" ]]; then
    echo "Complete motion test requires the final argument --confirm-motion" >&2
    echo "Example: $0 complete 10 5 5 --confirm-motion" >&2
    return 2
  fi
  start_all
  follow "$duration" "$max_error" "$max_excursion" "$confirmation"
}

complete_controller_limited() {
  local duration="${1:-10}" confirmation="${2:-}"
  if [[ "$confirmation" != "--confirm-motion" ]]; then
    echo "Complete controller-limited test requires --confirm-motion" >&2
    echo "Example: $0 complete-controller-limited 180 --confirm-motion" >&2
    return 2
  fi
  start_all
  follow_controller_limited "$duration" "$confirmation"
}

usage() {
  cat <<EOF
Usage: $0 COMMAND [ARGS]

  start                              Start the full pipeline, receive-only
  preflight [seconds]                Read-only test; lit armor must be visible
  follow [seconds] [step_deg] [travel_deg] --confirm-motion
                                     Bounded live follow, then restore read-only
  complete [seconds] [step_deg] [travel_deg] --confirm-motion
                                     Start, preflight, follow, restore read-only
  follow-controller-limited [seconds] --confirm-motion
                                     Follow with angle limits delegated to controller
  complete-controller-limited [seconds] --confirm-motion
                                     Full test with angle limits delegated to controller
  display                            Start RViz without duplicating camera or vision
  display-stop                       Stop only RViz
  status                             Show service and COM-port state
  config                             Print all active configuration paths
  stop                               Stop all test services
EOF
}

case "${1:-}" in
  start) start_all ;;
  preflight) preflight "${2:-10}" ;;
  follow) follow "${2:-10}" "${3:-5}" "${4:-5}" "${5:-}" ;;
  complete) complete_test "${2:-10}" "${3:-5}" "${4:-5}" "${5:-}" ;;
  follow-controller-limited) follow_controller_limited "${2:-10}" "${3:-}" ;;
  complete-controller-limited) complete_controller_limited "${2:-10}" "${3:-}" ;;
  display) start_rviz ;;
  display-stop) stop_rviz ;;
  status) show_status ;;
  config) show_config ;;
  stop) stop_all ;;
  *) usage; exit 2 ;;
esac
