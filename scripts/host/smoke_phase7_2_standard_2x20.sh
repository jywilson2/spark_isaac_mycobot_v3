#!/usr/bin/env bash
# Standard Phase 7.2 smoke: 2 episodes × 20 targets on a two-ring manual field
# (14 mm cubes; placement is identical across episodes, shuffle order and
# planner seeds vary).
#
# Root seed: omit --root-seed so each episode draws an independent random seed
# (coverage). Pass --root-seed N to reproduce a deterministic suite (logged as
# phase7_2_plan: root_seed=N / episode_seed=… and stored in the bundle).
#
# Not part of the default spark GUI gate. Run directly:
#   ./scripts/host/smoke_phase7_2_standard_2x20.sh --headless|--gui
#   ./scripts/host/smoke_phase7_2_standard_2x20.sh --gui --auto-exit --root-seed 4242
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
config="${root}/config/phase7_2_multi_target_standard_2x20.yml"
export SPARK_PHASE7_2_REPORT="${SPARK_PHASE7_2_REPORT:-${root}/artifacts/reports/phase7_2_multi_target_standard_2x20.json}"
export SPARK_PHASE7_2_BUNDLE="${SPARK_PHASE7_2_BUNDLE:-${root}/artifacts/reports/phase7_2_multi_target_standard_2x20.bundle.json}"

# Force the standard size after caller mode flags so counts stay 2×20.
exec bash "${root}/scripts/host/smoke_phase7_2_multi_target.sh" \
  --config "${config}" \
  "$@" \
  --targets 20 \
  --episodes 2
