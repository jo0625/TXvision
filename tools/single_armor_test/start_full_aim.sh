#!/usr/bin/env bash
set -euo pipefail
TEST_TEMPLATE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TEST_MODEL=single_armor
TEST_CHECK=false
TEST_RECORD_ROSBAG=true
for arg in "$@"; do
  case "$arg" in
    single_armor|rotating) TEST_MODEL="$arg" ;;
    --check) TEST_CHECK=true ;;
    --no-bag) TEST_RECORD_ROSBAG=false ;;
    *) echo "Usage: bash $0 [--check] [--no-bag] [single_armor|rotating]" >&2; exit 2 ;;
  esac
done
/usr/bin/python3 -B "$TEST_TEMPLATE/check_config.py" --model "$TEST_MODEL"
echo "rosbag 自动记录：$TEST_RECORD_ROSBAG（数值话题，不含图像）"
if "$TEST_CHECK"; then exit 0; fi
TEST_RUN_ROOT="$(/usr/bin/python3 -B -c 'import json,sys; print(json.load(open(sys.argv[1]))["runs_root"])' "$TEST_TEMPLATE/runtime_paths.json")"
mkdir -p "$TEST_RUN_ROOT"
TEST_RUN="$(mktemp -d "$TEST_RUN_ROOT/${TEST_MODEL}_$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)_XXXXXX")"
cp "$TEST_TEMPLATE"/{start_full_aim.sh,capture.py,rosbag_qos.yaml,aimer_validation.py,run_test.py,check_config.py,runtime_paths.json,configuration_review.json,baseline_provenance.json} "$TEST_RUN/"
cp -r "$TEST_TEMPLATE/source_snapshot" "$TEST_TEMPLATE/reviewed_configuration" "$TEST_RUN/"
/usr/bin/python3 -B - "$TEST_RUN/runtime_paths.json" "$TEST_MODEL" "$TEST_RECORD_ROSBAG" <<'PYCONFIG'
from pathlib import Path
import json
import sys
path = Path(sys.argv[1])
settings = json.loads(path.read_text())
if sys.argv[2] == "rotating":
    settings["tracker_params"] = settings["tracker_params_rotating"]
settings["motion_model"] = sys.argv[2]
settings["record_rosbag"] = sys.argv[3] == "true"
path.write_text(json.dumps(settings, indent=2) + "\n")
PYCONFIG
echo "运动模型：$TEST_MODEL；本轮记录目录：$TEST_RUN"
echo "7秒预热 → 15秒只接收预检 → 通过后60秒真实视觉跟随。Ctrl+C停止。"
exec /usr/bin/python3 -B "$TEST_RUN/run_test.py"
