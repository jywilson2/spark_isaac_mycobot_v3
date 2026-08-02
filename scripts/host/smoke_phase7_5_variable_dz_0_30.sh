#!/usr/bin/env bash
# Phase 7.5 variable-target-count Z-density smoke (dz=0.30).
# Headless plan+play by default; pass --gui for Kit playback of the frozen bundle.
# Does not pass --targets (achieved count is an outcome).
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
config="${root}/config/phase7_5_variable_targets_dz_0_30.yml"
export SPARK_PHASE7_2_REPORT="${root}/artifacts/reports/phase7_5_variable_targets_dz_0_30.json"
export SPARK_PHASE7_2_BUNDLE="${root}/artifacts/reports/phase7_5_variable_targets_dz_0_30.bundle.json"
exec bash "${root}/scripts/host/smoke_phase7_2_multi_target.sh" \
  --config "${config}" \
  --root-seed 4242 \
  "$@"
