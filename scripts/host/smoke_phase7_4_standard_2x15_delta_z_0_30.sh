#!/usr/bin/env bash
# Phase 7.4 densest-style smoke: 2×15 with regenerated centres and
# delta_z_m=0.30 (tip-IK pre-screen + field regen; lighter pack than 2×20).
#
#   ./scripts/host/smoke_phase7_4_standard_2x15_delta_z_0_30.sh --gui --no-auto-exit
#   ./scripts/host/smoke_phase7_4_standard_2x15_delta_z_0_30.sh --headless --auto-exit
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
config="${root}/config/phase7_4_multi_target_standard_2x15_delta_z_0_30.yml"
export SPARK_PHASE7_2_REPORT="${root}/artifacts/reports/phase7_4_multi_target_standard_2x15_delta_z_0_30.json"
export SPARK_PHASE7_2_BUNDLE="${root}/artifacts/reports/phase7_4_multi_target_standard_2x15_delta_z_0_30.bundle.json"

exec bash "${root}/scripts/host/smoke_phase7_2_multi_target.sh" \
  --config "${config}" \
  "$@" \
  --targets 15 \
  --episodes 2
