#!/usr/bin/env bash
set -euo pipefail
HAND_EYE_TEMPLATE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
case "${1:-}" in
  ""|--check) ;;
  *) echo "Usage: bash $0 [--check]" >&2; exit 2 ;;
esac
/usr/bin/python3 -B "$HAND_EYE_TEMPLATE/check_config.py"
if [[ "${1:-}" == "--check" ]]; then exit 0; fi
HAND_EYE_RUN_ROOT="$(/usr/bin/python3 -B -c 'import json,sys; print(json.load(open(sys.argv[1]))["runs_root"])' "$HAND_EYE_TEMPLATE/runtime_paths.json")"
mkdir -p "$HAND_EYE_RUN_ROOT"
HAND_EYE_RUN="$(mktemp -d "$HAND_EYE_RUN_ROOT/full_aim_$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)_XXXXXX")"
cp "$HAND_EYE_TEMPLATE"/{start_full_aim.sh,capture.py,aimer_validation.py,run_test.py,check_config.py,production_before.json,runtime_paths.json,configuration_review.json} "$HAND_EYE_RUN/"
cp -r "$HAND_EYE_TEMPLATE/source_snapshot" "$HAND_EYE_TEMPLATE/reviewed_configuration" "$HAND_EYE_RUN/"
echo "本轮记录目录：$HAND_EYE_RUN"
echo "15秒预热 → 30秒只接收预检 → 通过后60秒真实视觉跟随。Ctrl+C停止。"
exec /usr/bin/python3 -B "$HAND_EYE_RUN/run_test.py"
