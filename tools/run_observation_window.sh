#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 2 ]]; then
  echo "usage: $0 OUTPUT_PREFIX DURATION_SECONDS" >&2
  exit 2
fi

output_prefix=$1
duration_seconds=$2
python_bin=/home/jo/Documents/Codex/rm_vision/.venv/bin/python
tools_dir=/home/jo/Documents/Codex/rm_vision/rm_auto_aim/tools

set +u
source /opt/ros/humble/setup.bash
source /home/jo/Documents/Codex/rm_vision/rm_auto_aim/install/setup.bash
set -u

echo "START $(date '+%Y-%m-%d %H:%M:%S %Z')"
"${python_bin}" "${tools_dir}/resource_probe.py" \
  --output "${output_prefix}_resources.json" \
  --duration "${duration_seconds}" --interval 1 &
resource_pid=$!

set +e
timeout --signal=INT --kill-after=5s "${duration_seconds}s" \
  "${python_bin}" "${tools_dir}/latency_probe.py" observe \
  --output "${output_prefix}_observations.json"
observer_status=$?
set -e

wait "${resource_pid}"
echo "END $(date '+%Y-%m-%d %H:%M:%S %Z') observer_status=${observer_status}"

# timeout exits 124 after delivering SIGINT; the observer still saves cleanly.
if [[ ${observer_status} -ne 0 && ${observer_status} -ne 124 ]]; then
  exit "${observer_status}"
fi
