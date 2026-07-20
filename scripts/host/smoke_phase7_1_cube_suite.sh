#!/usr/bin/env bash
# Phase 7.1 cube-suite Isaac Sim smoke. Run natively on the DGX Spark host.
set -euo pipefail

usage() {
  printf 'Usage: %s [--headless|--gui] [--auto-exit|--no-auto-exit] [--all-modes]\n' "$0"
}

main() {
  local root mode auto_exit all_modes vendor_urdf prepared_usd nested_prepared_usd
  local report bundle suite_status
  local -a plan_args

  root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
  mode="--headless"
  auto_exit="--auto-exit"
  all_modes=0
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --headless|--gui) mode="$1"; shift ;;
      --auto-exit) auto_exit="--auto-exit"; shift ;;
      --no-auto-exit) auto_exit="--no-auto-exit"; shift ;;
      --all-modes) all_modes=1; shift ;;
      -h|--help)
        usage
        return 0
        ;;
      *)
        printf 'ERROR: unknown argument: %s\n' "$1" >&2
        return 2
        ;;
    esac
  done

  # shellcheck source=./env.isaac_host.sh
  # shellcheck disable=SC1091
  source "${root}/scripts/host/env.isaac_host.sh"
  spark_host_require_native_shell
  "${root}/scripts/host/check_prereqs.sh"

  vendor_urdf="${root}/third_party/mycobot_ros2/mycobot_description/urdf/mycobot_280_m5/mycobot_280_m5.urdf"
  prepared_usd="${root}/assets/mycobot_280_m5/prepared/mycobot_280_m5.usd"
  nested_prepared_usd="${root}/assets/mycobot_280_m5/prepared/mycobot_280_m5/mycobot_280_m5.usda"
  if [[ ! -f "${vendor_urdf}" ]]; then
    "${root}/scripts/download_mycobot_ros2.sh"
  fi
  if [[ ! -f "${prepared_usd}" && ! -f "${nested_prepared_usd}" ]]; then
    "${root}/scripts/convert_urdf_to_usd.sh"
  fi
  if [[ ! -f "${prepared_usd}" ]]; then
    prepared_usd="${nested_prepared_usd}"
  fi
  if [[ ! -f "${prepared_usd}" ]]; then
    printf 'ERROR: URDF conversion produced no prepared USD: %s\n' "${prepared_usd}" >&2
    return 1
  fi

  report="${SPARK_PHASE7_1_REPORT:-${root}/artifacts/reports/phase7_1_cube_suite.json}"
  bundle="${SPARK_PHASE7_1_BUNDLE:-${root}/artifacts/reports/phase7_1_cube_suite.bundle.json}"
  mkdir -p "$(dirname "${report}")"
  plan_args=()
  if [[ "${all_modes}" -eq 1 ]]; then
    plan_args+=(--all-modes)
  fi

  # Plan/validate in a process that never imports Isaac Kit. cuRobo/Warp must not
  # share an address space with SimulationApp on this host stack.
  spark_host_run_python "${root}/isaac_sim/plan_cube_suite.py" \
    --config "${root}/config/phase7_1_cube_suite.yml" \
    ${plan_args[@]+"${plan_args[@]}"} \
    --output-bundle "${bundle}"

  set +e
  spark_host_run_python "${root}/isaac_sim/play_cube_suite.py" \
    --repo-root "${root}" \
    --bundle "${bundle}" \
    --usd "${prepared_usd}" \
    "${mode}" \
    "${auto_exit}" \
    --output-report "${report}"
  suite_status=$?
  set -e
  if [[ ! -f "${report}" ]]; then
    printf 'ERROR: Phase 7.1 suite did not write report: %s\n' "${report}" >&2
    return 1
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
' "${report}" "${suite_status}"
}

main "$@"
