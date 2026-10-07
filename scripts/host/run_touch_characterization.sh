#!/usr/bin/env bash
# Opt-in touch-failure characterization on the Spark host (Isaac python).
# Does not launch Phase 7.2 smoke and does not change pass/fail gates.
#
#   ./scripts/host/spark_host_exec.sh ./scripts/host/run_touch_characterization.sh
#   ./scripts/host/run_touch_characterization.sh --no-planner
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# A parent shell may set the loaded-guard without exporting the functions.
unset SPARK_ISAAC_HOST_ENV_LOADED
# shellcheck disable=SC1091
source "${root}/scripts/host/env.isaac_host.sh"

if [[ "${1:-}" == "--cpu" ]]; then
  shift
  exec python3 "${root}/scripts/run_touch_characterization.py" "$@"
fi

spark_host_apply_env
exec "${ISAACSIM_PYTHON_EXE}" "${root}/scripts/run_touch_characterization.py" \
  --with-planner \
  --fields 20 \
  --targets 2 \
  --seed 4242 \
  --draws 200 \
  --joint-samples 8 \
  --output-dir "${root}/docs/figures" \
  --bundle-out "${root}/artifacts/characterization/touch_characterization_bundle.json" \
  "$@"
