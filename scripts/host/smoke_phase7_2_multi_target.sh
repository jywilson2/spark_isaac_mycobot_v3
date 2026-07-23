#!/usr/bin/env bash
# Phase 7.2 multi-target tip-contact Isaac Sim smoke. Run on the DGX Spark host.
set -euo pipefail

usage() {
  printf 'Usage: %s [--headless|--gui] [--auto-exit|--no-auto-exit] [--manual] [--config PATH] [--targets N] [--episodes N] [--root-seed N]\n' "$0"
}

main() {
  local root mode auto_exit manual vendor_urdf prepared_usd nested_prepared_usd
  local report bundle suite_status config targets episodes root_seed artifact_tag config_override
  local -a plan_args

  root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
  mode="--headless"
  auto_exit="--auto-exit"
  manual=0
  targets=""
  episodes=""
  root_seed=""
  config_override=""
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --headless|--gui) mode="$1"; shift ;;
      --auto-exit) auto_exit="--auto-exit"; shift ;;
      --no-auto-exit) auto_exit="--no-auto-exit"; shift ;;
      --manual) manual=1; shift ;;
      --config)
        if [[ $# -lt 2 ]]; then
          printf 'ERROR: --config requires a YAML path\n' >&2
          return 2
        fi
        config_override="$2"
        shift 2
        ;;
      --targets)
        if [[ $# -lt 2 ]]; then
          printf 'ERROR: --targets requires a positive integer\n' >&2
          return 2
        fi
        targets="$2"
        shift 2
        ;;
      --episodes)
        if [[ $# -lt 2 ]]; then
          printf 'ERROR: --episodes requires a positive integer\n' >&2
          return 2
        fi
        episodes="$2"
        shift 2
        ;;
      --root-seed)
        if [[ $# -lt 2 ]]; then
          printf 'ERROR: --root-seed requires a non-negative integer\n' >&2
          return 2
        fi
        root_seed="$2"
        shift 2
        ;;
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

  if [[ -n "${targets}" ]]; then
    if ! [[ "${targets}" =~ ^[1-9][0-9]*$ ]]; then
      printf 'ERROR: --targets must be a positive integer, got %s\n' "${targets}" >&2
      return 2
    fi
  fi
  if [[ -n "${episodes}" ]]; then
    if ! [[ "${episodes}" =~ ^[1-9][0-9]*$ ]]; then
      printf 'ERROR: --episodes must be a positive integer, got %s\n' "${episodes}" >&2
      return 2
    fi
  fi
  if [[ -n "${root_seed}" ]]; then
    if ! [[ "${root_seed}" =~ ^(0|[1-9][0-9]*)$ ]]; then
      printf 'ERROR: --root-seed must be a non-negative integer, got %s\n' "${root_seed}" >&2
      return 2
    fi
  fi

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

  if [[ -n "${config_override}" ]]; then
    if [[ "${manual}" -eq 1 ]]; then
      printf 'ERROR: --config and --manual are mutually exclusive\n' >&2
      return 2
    fi
    if [[ "${config_override}" != /* ]]; then
      config_override="${root}/${config_override}"
    fi
    if [[ ! -f "${config_override}" ]]; then
      printf 'ERROR: suite config not found: %s\n' "${config_override}" >&2
      return 2
    fi
    config="${config_override}"
    report="${SPARK_PHASE7_2_REPORT:-${root}/artifacts/reports/phase7_2_multi_target_custom.json}"
    bundle="${SPARK_PHASE7_2_BUNDLE:-${root}/artifacts/reports/phase7_2_multi_target_custom.bundle.json}"
  elif [[ "${manual}" -eq 1 ]]; then
    config="${root}/config/phase7_2_multi_target_manual.yml"
    report="${SPARK_PHASE7_2_REPORT:-${root}/artifacts/reports/phase7_2_multi_target_manual.json}"
    bundle="${SPARK_PHASE7_2_BUNDLE:-${root}/artifacts/reports/phase7_2_multi_target_manual.bundle.json}"
  else
    config="${root}/config/phase7_2_multi_target.yml"
    report="${SPARK_PHASE7_2_REPORT:-${root}/artifacts/reports/phase7_2_multi_target.json}"
    bundle="${SPARK_PHASE7_2_BUNDLE:-${root}/artifacts/reports/phase7_2_multi_target.bundle.json}"
  fi
  artifact_tag=""
  if [[ -n "${targets}" && -n "${episodes}" ]]; then
    artifact_tag="${targets}x${episodes}"
  elif [[ -n "${targets}" ]]; then
    artifact_tag="${targets}"
  elif [[ -n "${episodes}" ]]; then
    artifact_tag="ep${episodes}"
  fi
  if [[ -n "${artifact_tag}" && -z "${SPARK_PHASE7_2_REPORT:-}" ]]; then
    report="${root}/artifacts/reports/phase7_2_multi_target_${artifact_tag}.json"
    bundle="${root}/artifacts/reports/phase7_2_multi_target_${artifact_tag}.bundle.json"
  fi
  mkdir -p "$(dirname "${report}")"

  plan_args=(--config "${config}" --output-bundle "${bundle}")
  if [[ -n "${targets}" ]]; then
    plan_args+=(--targets "${targets}")
  fi
  if [[ -n "${episodes}" ]]; then
    plan_args+=(--episodes "${episodes}")
  fi
  if [[ -n "${root_seed}" ]]; then
    plan_args+=(--root-seed "${root_seed}")
  fi

  # Always attempt playback when a bundle exists, even if planning reported
  # incomplete episodes, so --gui --no-auto-exit can hold the Kit window.
  set +e
  spark_host_run_python "${root}/isaac_sim/plan_multi_target_suite.py" \
    "${plan_args[@]}"
  plan_status=$?
  set -e
  if [[ ! -f "${bundle}" ]]; then
    printf 'ERROR: Phase 7.2 planner did not write bundle: %s (exit %s)\n' \
      "${bundle}" "${plan_status}" >&2
    return 1
  fi
  if [[ "${plan_status}" -ne 0 ]]; then
    printf 'WARNING: Phase 7.2 planner exit %s; continuing to playback for visualization\n' \
      "${plan_status}" >&2
  fi

  set +e
  spark_host_run_python "${root}/isaac_sim/play_multi_target_suite.py" \
    --repo-root "${root}" \
    --bundle "${bundle}" \
    --usd "${prepared_usd}" \
    "${mode}" \
    "${auto_exit}" \
    --output-report "${report}"
  suite_status=$?
  set -e
  if [[ ! -f "${report}" ]]; then
    printf 'ERROR: Phase 7.2 suite did not write report: %s\n' "${report}" >&2
    return 1
  fi
  python3 -c '
import json
import sys

payload = json.load(open(sys.argv[1], encoding="utf-8"))
required = ("schema_version", "joint_playback_completed", "lighting_ready", "summary")
missing = [key for key in required if key not in payload]
if missing:
    raise SystemExit(f"Phase 7.2 report missing {missing}: {payload}")
if payload["schema_version"] != 1 or not payload["lighting_ready"]:
    raise SystemExit(f"invalid Phase 7.2 report: {payload}")
summary = payload["summary"]
if int(summary.get("total_episodes", 0)) < 1:
    raise SystemExit(f"Phase 7.2 summary missing episodes: {payload}")
expected_targets = int(sys.argv[3]) if sys.argv[3] else None
expected_episodes = int(sys.argv[4]) if sys.argv[4] else None
auto_exit = sys.argv[5] == "--auto-exit"
if expected_targets is not None:
    results = payload.get("results") or []
    if not results:
        raise SystemExit(f"Phase 7.2 report missing results for --targets check: {payload}")
    actual = len(results[0].get("episode", {}).get("field", {}).get("targets") or [])
    if actual != expected_targets:
        raise SystemExit(
            f"Phase 7.2 expected {expected_targets} targets, found {actual}: {payload}"
        )
if expected_episodes is not None:
    actual_episodes = int(summary.get("total_episodes", -1))
    if actual_episodes != expected_episodes:
        raise SystemExit(
            f"Phase 7.2 expected {expected_episodes} episodes, found {actual_episodes}: {payload}"
        )
# With --no-auto-exit the operator reviews the held GUI; do not hard-fail the
# smoke wrapper solely on incomplete tip clearance (play/plan exit codes remain).
if auto_exit and int(summary.get("successes", 0)) != int(summary.get("total_episodes", -1)):
    raise SystemExit(f"Phase 7.2 suite did not fully succeed: {summary}")
print(
    json.dumps(
        {
            "lighting_ready": True,
            "suite_status": int(sys.argv[2]),
            "plan_status": int(sys.argv[6]),
            "targets": expected_targets,
            "episodes": expected_episodes,
            "summary": summary,
        },
        sort_keys=True,
    )
)
' "${report}" "${suite_status}" "${targets:-}" "${episodes:-}" "${auto_exit}" "${plan_status}"
  if [[ "${plan_status}" -ne 0 && "${suite_status}" -eq 0 ]]; then
    return "${plan_status}"
  fi
  return "${suite_status}"
}

main "$@"
