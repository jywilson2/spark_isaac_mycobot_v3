#!/usr/bin/env bash
# Host entry for Phase 8 residual training (sim-only).
#
# Default backend is the offline synthetic trainer in mycobot_curobo (no Kit).
# Live Isaac Lab env wiring remains optional; this script never enables hardware.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

if [[ "${ENABLE_MYCOBOT_HARDWARE_TESTS:-0}" == "1" ]]; then
  echo "Refusing Phase 8 training with ENABLE_MYCOBOT_HARDWARE_TESTS=1 (sim-only)." >&2
  exit 2
fi

export PYTHONPATH="${REPO_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}"
python3 "${REPO_ROOT}/scripts/train_residual_policy.py" "$@"
