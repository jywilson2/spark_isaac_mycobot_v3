#!/usr/bin/env bash
# Mode B chained Phase 7.1 cube GUI: EE continues from the last success
# (no home/bank reset between cubes). Run on the DGX Spark host.
#
#   ./scripts/host/run_phase7_1_chained_gui.sh --GUI
#   ./scripts/host/run_phase7_1_chained_gui.sh --gui --episodes 20
#   ./scripts/host/spark_host_exec.sh ./scripts/host/run_phase7_1_chained_gui.sh --GUI
set -euo pipefail

usage() {
  printf 'Usage: %s [--GUI|--gui|--headless] [--episodes N] [--auto-exit|--no-auto-exit]\n' "$0"
}

main() {
  local root episodes mode auto_exit vendor_urdf prepared_usd nested_prepared_usd
  local bundle report suite_status host_home

  root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
  episodes=20
  mode="--gui"
  auto_exit="--no-auto-exit"
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --episodes)
        episodes="${2:?--episodes requires a positive integer}"
        shift 2
        ;;
      --GUI|--gui) mode="--gui"; shift ;;
      --headless) mode="--headless"; shift ;;
      --auto-exit) auto_exit="--auto-exit"; shift ;;
      --no-auto-exit) auto_exit="--no-auto-exit"; shift ;;
      -h|--help)
        usage
        return 0
        ;;
      *)
        printf 'ERROR: unknown argument: %s\n' "$1" >&2
        usage >&2
        return 2
        ;;
    esac
  done
  if ! [[ "${episodes}" =~ ^[1-9][0-9]*$ ]]; then
    printf 'ERROR: --episodes must be a positive integer (got %s)\n' "${episodes}" >&2
    return 2
  fi

  # shellcheck source=./env.isaac_host.sh
  # shellcheck disable=SC1091
  source "${root}/scripts/host/env.isaac_host.sh"
  # shellcheck source=./spark_host_exec.sh
  # shellcheck disable=SC1091
  source "${root}/scripts/host/spark_host_exec.sh"
  spark_host_require_native_shell
  "${root}/scripts/host/check_prereqs.sh"

  if [[ "${mode}" == "--gui" ]]; then
    host_home="${HOME:-/home/${USER:-admin}}"
    spark_require_gui_display "${host_home}"
    printf '=== Mode B chained GUI on DISPLAY=%s ===\n' "${DISPLAY}"
  else
    printf '=== Mode B chained headless: %s episodes ===\n' "${episodes}"
  fi

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
    printf 'ERROR: prepared USD missing: %s\n' "${prepared_usd}" >&2
    return 1
  fi

  bundle="${root}/artifacts/reports/phase7_1_chained_${episodes}.bundle.json"
  report="${root}/artifacts/reports/phase7_1_chained_${episodes}.json"
  mkdir -p "$(dirname "${report}")"

  printf '=== Planning Mode B chained: %s episodes ===\n' "${episodes}"
  spark_host_run_python "${root}/isaac_sim/plan_cube_suite.py" \
    --config "${root}/config/phase7_1_cube_suite.yml" \
    --chained \
    --episodes "${episodes}" \
    --output-bundle "${bundle}"

  printf '=== Playing suite (%s) ===\n' "${mode}"
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
    printf 'ERROR: missing report %s\n' "${report}" >&2
    return 1
  fi
  python3 -c '
import json
import sys

payload = json.load(open(sys.argv[1], encoding="utf-8"))
summary = payload.get("summary") or {}
print(
    json.dumps(
        {
            "lighting_ready": payload.get("lighting_ready"),
            "stage_lighting_mode": payload.get("stage_lighting_mode"),
            "suite_status": int(sys.argv[2]),
            "summary": summary,
            "report": sys.argv[1],
        },
        sort_keys=True,
        indent=2,
    )
)
if not payload.get("lighting_ready"):
    raise SystemExit(2)
raise SystemExit(int(sys.argv[2]))
' "${report}" "${suite_status}"
}

main "$@"
