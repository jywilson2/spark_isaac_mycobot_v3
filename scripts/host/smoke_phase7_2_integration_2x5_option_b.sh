#!/usr/bin/env bash
# Integration 2×5 smoke with Phase 1.1 Option B dual overlay trial-armed.
#
# Writes a temporary robot YAML (scaffolding + dense *_world_cover spheres) and
# a temporary app.yml that points at it, then runs the standard integration
# 2×5 smoke. Does not modify the default disarmed robot YAML.
#
# Usage:
#   ./scripts/host/smoke_phase7_2_integration_2x5_option_b.sh --headless|--gui
#   ./scripts/host/smoke_phase7_2_integration_2x5_option_b.sh --gui --auto-exit --root-seed 4242
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
trial_robot="${root}/config/robots/_tmp_option_b_integration_trial.yml"
trial_app="${root}/config/_tmp_app_option_b_integration.yml"

cleanup() {
  rm -f "${trial_robot}" "${trial_app}"
}
trap cleanup EXIT

# shellcheck source=./env.isaac_host.sh
# shellcheck disable=SC1091
source "${root}/scripts/host/env.isaac_host.sh"
spark_host_require_native_shell
spark_host_apply_env

"${ISAACSIM_PYTHON_EXE}" "${root}/scripts/host/write_option_b_trial_app.py" \
  --repo-root "${root}" \
  --trial-robot "${trial_robot}" \
  --trial-app "${trial_app}"

export SPARK_PHASE7_2_REPORT="${SPARK_PHASE7_2_REPORT:-${root}/artifacts/reports/phase7_2_multi_target_integration_2x5_option_b.json}"
export SPARK_PHASE7_2_BUNDLE="${SPARK_PHASE7_2_BUNDLE:-${root}/artifacts/reports/phase7_2_multi_target_integration_2x5_option_b.bundle.json}"

# Do not exec: keep this shell so the EXIT trap removes trial YAML files.
set +e
bash "${root}/scripts/host/smoke_phase7_2_integration_2x5.sh" \
  --app-config "${trial_app}" \
  "$@"
status=$?
set -e
exit "${status}"
