#!/usr/bin/env bash
# Integration-only Phase 7.2 smoke: 2 episodes × 5 targets with distinct
# per-episode placement and planner seeds.
#
# Root seed: omit --root-seed so each episode draws an independent random seed
# (coverage). Pass --root-seed N to reproduce a deterministic suite (logged as
# phase7_2_plan: root_seed=N / episode_seed=… and stored in the bundle).
#
# Not part of the default spark GUI gate. Enable via:
#   ./scripts/run_verification.sh spark --with-integration-smoke
#   SPARK_RUN_INTEGRATION_SMOKE=1 ./scripts/run_verification.sh spark
# Or run directly:
#   ./scripts/host/smoke_phase7_2_integration_2x5.sh --headless|--gui
#   ./scripts/host/smoke_phase7_2_integration_2x5.sh --gui --auto-exit --root-seed 4242
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
config="${root}/config/phase7_2_multi_target_integration_2x5.yml"
export SPARK_PHASE7_2_REPORT="${SPARK_PHASE7_2_REPORT:-${root}/artifacts/reports/phase7_2_multi_target_integration_2x5.json}"
export SPARK_PHASE7_2_BUNDLE="${SPARK_PHASE7_2_BUNDLE:-${root}/artifacts/reports/phase7_2_multi_target_integration_2x5.bundle.json}"

# Force the integration size after caller mode flags so counts stay 2×5.
exec bash "${root}/scripts/host/smoke_phase7_2_multi_target.sh" \
  --config "${config}" \
  "$@" \
  --targets 5 \
  --episodes 2
