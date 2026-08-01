#!/usr/bin/env bash
# Phase 1.1 Option B GPU gates (dual overlay + 7.1/7.2 suites on scaffolding
# or trial-armed dual — tests create their own trial YAML).
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
  "${root}/tests/integration/test_phase1_1_collision_spheres_gpu.py" \
  "${root}/tests/integration/test_phase1_1_option_b_planning_gpu.py" \
  "${root}/tests/integration/test_phase7_1_cube_suite_gpu.py" \
  "${root}/tests/integration/test_phase7_2_multi_target_gpu.py" \
  -q --tb=short "$@"
