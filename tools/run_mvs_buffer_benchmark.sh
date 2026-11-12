#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd "${script_dir}/.." && pwd)"
binary_dir="${repo_dir}/build/mvs_buffer_benchmark"
binary_path="${binary_dir}/mvs_buffer_benchmark"
run_stamp="$(date +%Y%m%d_%H%M%S)"
run_dir="${repo_dir}/test_results/mvs_buffer_nodes_${run_stamp}"
if (($# > 0)) && [[ "$1" != -* ]]; then
  run_dir="$1"
  shift
fi
benchmark_args=("$@")

mkdir -p -- "${binary_dir}" "${run_dir}"

g++ -std=c++17 -O2 -Wall -Wextra -Werror -Wpedantic \
  -I/opt/MVS/include \
  "${script_dir}/mvs_buffer_benchmark.cpp" \
  -L/opt/MVS/lib/64 -Wl,-rpath,/opt/MVS/lib/64 \
  -lMvCameraControl -pthread \
  -o "${binary_path}"

if [[ " ${benchmark_args[*]} " == *" --help "* ]] || [[ " ${benchmark_args[*]} " == *" -h "* ]]; then
  "${binary_path}" --help
  exit 0
fi

"${binary_path}" --output "${run_dir}" "${benchmark_args[@]}"
python3 "${script_dir}/analyze_mvs_buffer_benchmark.py" "${run_dir}"

echo "Benchmark report: ${run_dir}/report.md"
