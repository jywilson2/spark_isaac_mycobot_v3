#!/usr/bin/env bash
# Phase 7.1 cube-suite Isaac Sim smoke. Run natively on the DGX Spark host.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MODE="--headless"
AUTO_EXIT="--auto-exit"
ALL_MODES=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --headless|--gui) MODE="$1"; shift ;;
    --auto-exit) AUTO_EXIT="--auto-exit"; shift ;;
    --no-auto-exit) AUTO_EXIT="--no-auto-exit"; shift ;;
    --all-modes) ALL_MODES=1; shift ;;
    -h|--help)
      echo "Usage: $0 [--headless|--gui] [--auto-exit|--no-auto-exit] [--all-modes]"
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

REPORT="${SPARK_PHASE7_1_REPORT:-${ROOT}/artifacts/reports/phase7_1_cube_suite.json}"
BUNDLE="${SPARK_PHASE7_1_BUNDLE:-${ROOT}/artifacts/reports/phase7_1_cube_suite.bundle.json}"
mkdir -p "$(dirname "${REPORT}")"
PLAN_ARGS=()
if [[ "${ALL_MODES}" -eq 1 ]]; then
  PLAN_ARGS+=(--all-modes)
fi

# Plan/validate in a process that never imports Isaac Kit. cuRobo/Warp must not
# share an address space with SimulationApp on this host stack.
spark_host_run_python "${ROOT}/isaac_sim/plan_cube_suite.py" \
  --config "${ROOT}/config/phase7_1_cube_suite.yml" \
  ${PLAN_ARGS[@]+"${PLAN_ARGS[@]}"} \
  --output-bundle "${BUNDLE}"

set +e
spark_host_run_python "${ROOT}/isaac_sim/play_cube_suite.py" \
  --repo-root "${ROOT}" \
  --bundle "${BUNDLE}" \
  --usd "${PREPARED_USD}" \
  "${MODE}" \
  "${AUTO_EXIT}" \
  --output-report "${REPORT}"
suite_status=$?
set -e
if [[ ! -f "${REPORT}" ]]; then
  echo "ERROR: Phase 7.1 suite did not write report: ${REPORT}" >&2
  exit 1
fi
python3 -c '
import json
import sys

payload = json.load(open(sys.argv[1], encoding="utf-8"))
required = ("schema_version", "joint_playback_completed", "lighting_ready", "tip_metrics_status", "summary")
missing = [key for key in required if key not in payload]
if missing:
    raise SystemExit(f"Phase 7.1 report missing {missing}: {payload}")
if payload["schema_version"] != 1 or not payload["lighting_ready"]:
    raise SystemExit(f"invalid Phase 7.1 report: {payload}")
if payload["tip_metrics_status"] != "not_evaluated":
    raise SystemExit(f"unexpected Phase 7.1 tip metric state: {payload}")
if payload.get("tip_position_error_m") is not None or payload.get("tip_orientation_error_rad") is not None:
    raise SystemExit(f"Phase 7.1 tip metrics must remain null: {payload}")
summary = payload["summary"]
if int(summary.get("total_episodes", 0)) < 1:
    raise SystemExit(f"Phase 7.1 summary missing episodes: {payload}")
print(
    json.dumps(
        {
            "lighting_ready": True,
            "tip_metrics_status": "not_evaluated",
            "suite_status": int(sys.argv[2]),
            "summary": summary,
        },
        sort_keys=True,
    )
)
' "${REPORT}" "${suite_status}"
