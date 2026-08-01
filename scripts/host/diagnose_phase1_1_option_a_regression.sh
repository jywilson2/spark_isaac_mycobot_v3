#!/usr/bin/env bash
# Host GPU diagnosis of the Option A armed-overlay planning regression.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck source=./env.isaac_host.sh
# shellcheck disable=SC1091
source "${root}/scripts/host/env.isaac_host.sh"
spark_host_require_native_shell
spark_host_apply_env

export PYTHONPATH="${root}/src:${root}${PYTHONPATH:+:${PYTHONPATH}}"
exec "${ISAACSIM_PYTHON_EXE}" \
  "${root}/scripts/host/diagnose_phase1_1_option_a_regression.py" "$@"
