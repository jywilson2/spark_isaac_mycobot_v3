#!/usr/bin/env bash
# Stream Phase 7.4 suite target-placement generation (CPU-only; no Isaac).
#
#   ./scripts/host/generate_phase7_4_placement.sh \
#     --config config/phase7_4_multi_target_standard_2x20_delta_z_0_30.yml \
#     --root-seed 4242
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export PYTHONPATH="${root}/src${PYTHONPATH:+:${PYTHONPATH}}"
export PYTHONUNBUFFERED=1
exec stdbuf -oL -eL python3 "${root}/scripts/generate_multi_target_placement.py" "$@"
