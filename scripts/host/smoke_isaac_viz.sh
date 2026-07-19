#!/usr/bin/env bash
# Phase 7 validated-plan Isaac Sim smoke. Run natively on the DGX Spark host.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MODE="--headless"
AUTO_EXIT="--auto-exit"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --headless|--gui) MODE="$1"; shift ;;
    --auto-exit) AUTO_EXIT="--auto-exit"; shift ;;
    --no-auto-exit) AUTO_EXIT="--no-auto-exit"; shift ;;
    -h|--help)
      echo "Usage: $0 [--headless|--gui] [--auto-exit|--no-auto-exit]"
      exit 0
      ;;
    *) echo "ERROR: unknown argument: $1" >&2; exit 2 ;;
  esac
done

# shellcheck source=env.isaac_host.sh
source "${ROOT}/scripts/host/env.isaac_host.sh"
spark_host_require_native_shell
"${ROOT}/scripts/host/check_prereqs.sh"

VENDOR_URDF="${ROOT}/third_party/mycobot_ros2/mycobot_description/urdf/mycobot_280_m5/mycobot_280_m5.urdf"
PREPARED_USD="${ROOT}/assets/mycobot_280_m5/prepared/mycobot_280_m5.usd"
NESTED_PREPARED_USD="${ROOT}/assets/mycobot_280_m5/prepared/mycobot_280_m5/mycobot_280_m5.usda"
if [[ ! -f "${VENDOR_URDF}" ]]; then
  "${ROOT}/scripts/download_mycobot_ros2.sh"
fi
if [[ ! -f "${PREPARED_USD}" && ! -f "${NESTED_PREPARED_USD}" ]]; then
  "${ROOT}/scripts/convert_urdf_to_usd.sh"
fi
if [[ ! -f "${PREPARED_USD}" ]]; then
  PREPARED_USD="${NESTED_PREPARED_USD}"
fi
if [[ ! -f "${PREPARED_USD}" ]]; then
  echo "ERROR: URDF conversion produced no prepared USD: ${PREPARED_USD}" >&2
  exit 1
fi

METRICS="${SPARK_PHASE7_METRICS:-${ROOT}/artifacts/reports/phase7_sim_metrics.json}"
mkdir -p "$(dirname "${METRICS}")"
spark_host_run_python "${ROOT}/isaac_sim/play_nominal_plan.py" \
  --repo-root "${ROOT}" \
  --plan "${ROOT}/tests/data/phase7_validated_plan.json" \
  --usd "${PREPARED_USD}" \
  "${MODE}" \
  "${AUTO_EXIT}" \
  --output-metrics "${METRICS}"
python3 -c '
import json
import sys
payload = json.load(open(sys.argv[1], encoding="utf-8"))
if payload.get("joint_playback_completed") is not True:
    raise SystemExit(f"joint playback did not complete: {payload}")
' "${METRICS}"
