#!/usr/bin/env bash
# Inspect and warm the Phase 1 MyCobot model in the host Isaac Sim runtime.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.isaac_host.sh
source "${SCRIPT_DIR}/env.isaac_host.sh"
spark_host_apply_env

exec "${ISAACSIM_PYTHON_EXE}" \
  "${SPARK_REPO_ROOT}/scripts/inspect_robot_model.py" \
  --gpu "$@"
