#!/usr/bin/env bash
# Phase 7.4 densest-style smoke: 2×20 with regenerated centres and
# delta_z_m=0.30 (unclamped absolute Z band + dexterous-reach screening).
#
#   ./scripts/host/smoke_phase7_4_standard_2x20_delta_z_0_30.sh --gui --no-auto-exit
#   ./scripts/host/smoke_phase7_4_standard_2x20_delta_z_0_30.sh --gui --auto-exit --root-seed 4242
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
config="${root}/config/phase7_4_multi_target_standard_2x20_delta_z_0_30.yml"
# Always pin report/bundle paths for this suite (do not inherit a different
# SPARK_PHASE7_2_* value from a prior smoke / GUI loop in the same shell).
export SPARK_PHASE7_2_REPORT="${root}/artifacts/reports/phase7_4_multi_target_standard_2x20_delta_z_0_30.json"
export SPARK_PHASE7_2_BUNDLE="${root}/artifacts/reports/phase7_4_multi_target_standard_2x20_delta_z_0_30.bundle.json"

exec bash "${root}/scripts/host/smoke_phase7_2_multi_target.sh" \
  --config "${config}" \
  "$@" \
  --targets 20 \
  --episodes 2
