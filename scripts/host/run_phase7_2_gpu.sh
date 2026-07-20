#!/usr/bin/env bash
# Focused Phase 7.2 GPU integration (host Isaac Sim Python + cuRobo).
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck source=./env.isaac_host.sh
# shellcheck disable=SC1091
source "${root}/scripts/host/env.isaac_host.sh"
spark_host_require_native_shell
spark_host_apply_env

export PYTHONPATH="${root}/src:${root}${PYTHONPATH:+:${PYTHONPATH}}"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  "${ISAACSIM_PYTHON_EXE}" -m pytest \
  "${root}/tests/integration/test_phase7_2_multi_target_gpu.py" \
  -q --tb=short "$@"
