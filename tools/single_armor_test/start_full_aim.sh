#!/usr/bin/env bash
set -euo pipefail
TEST_TEMPLATE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TEST_MODEL=single_armor
TEST_CHECK=false
for arg in "$@"; do
  case "$arg" in
    single_armor|rotating) TEST_MODEL="$arg" ;;
    --check) TEST_CHECK=true ;;
    *) echo "Usage: bash $0 [--check] [single_armor|rotating]" >&2; exit 2 ;;
  esac
done
/usr/bin/python3 -B "$TEST_TEMPLATE/check_config.py" --model "$TEST_MODEL"
if "$TEST_CHECK"; then exit 0; fi
TEST_RUN_ROOT="$(/usr/bin/python3 -B -c 'import json,sys; print(json.load(open(sys.argv[1]))["runs_root"])' "$TEST_TEMPLATE/runtime_paths.json")"
mkdir -p "$TEST_RUN_ROOT"
TEST_RUN="$(mktemp -d "$TEST_RUN_ROOT/${TEST_MODEL}_$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)_XXXXXX")"
cp "$TEST_TEMPLATE"/{start_full_aim.sh,capture.py,aimer_validation.py,run_test.py,check_config.py,runtime_paths.json,configuration_review.json,baseline_provenance.json} "$TEST_RUN/"
cp -r "$TEST_TEMPLATE/source_snapshot" "$TEST_TEMPLATE/reviewed_configuration" "$TEST_RUN/"
/usr/bin/python3 -B - "$TEST_RUN/runtime_paths.json" "$TEST_MODEL" <<'PYCONFIG'
from pathlib import Path
import json
import sys
path = Path(sys.argv[1])
settings = json.loads(path.read_text())
if sys.argv[2] == "rotating":
    settings["tracker_params"] = settings["tracker_params_rotating"]
settings["motion_model"] = sys.argv[2]
path.write_text(json.dumps(settings, indent=2) + "\n")
PYCONFIG
echo "运动模型：$TEST_MODEL；本轮记录目录：$TEST_RUN"
echo "7秒预热 → 15秒只接收预检 → 通过后60秒真实视觉跟随。Ctrl+C停止。"
exec /usr/bin/python3 -B "$TEST_RUN/run_test.py"
