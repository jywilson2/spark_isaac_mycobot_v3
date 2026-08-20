#!/usr/bin/env bash
# Phase 8 GUI smoke: A/B residual-off (biased) then residual-on (corrected).
#
# Default mode is demo-visible (exaggerated residual bounds + tip bias) so the
# correction is obvious in GUI. Use --subtle for production-like bounds.
#
# Pass A: residual-off — tip bias injected into nominal joints (misaligned)
# Pass B: residual-on — policy cancels that tip bias (toward alignment)
#
# Usage (host native shell):
#   ./scripts/host/smoke_phase8_residual_gui.sh --gui --auto-exit
#   ./scripts/host/smoke_phase8_residual_gui.sh --gui --auto-exit --subtle
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${REPO_ROOT}"

# shellcheck source=env.isaac_host.sh
source "${SCRIPT_DIR}/env.isaac_host.sh"
spark_host_apply_env || {
  echo "Isaac Sim python.sh not found. Set ISAACSIM_PATH." >&2
  exit 1
}

if [[ "${ENABLE_MYCOBOT_HARDWARE_TESTS:-0}" == "1" ]]; then
  echo "Refusing Phase 8 residual GUI smoke with hardware tests enabled." >&2
  exit 2
fi

MODE_ARGS=()
DEMO_MODE=1
while [[ $# -gt 0 ]]; do
  case "$1" in
    --gui|--headless|--auto-exit|--no-auto-exit)
      MODE_ARGS+=("$1")
      shift
      ;;
    --demo)
      DEMO_MODE=1
      shift
      ;;
    --subtle)
      DEMO_MODE=0
      shift
      ;;
    *)
      echo "Unknown argument: $1" >&2
      echo "Usage: $0 [--gui|--headless] [--auto-exit|--no-auto-exit] [--demo|--subtle]" >&2
      exit 2
      ;;
  esac
done
if [[ ${#MODE_ARGS[@]} -eq 0 ]]; then
  MODE_ARGS=(--gui --auto-exit)
fi

SOURCE_BUNDLE="${SPARK_PHASE8_SOURCE_BUNDLE:-${REPO_ROOT}/artifacts/reports/phase7_2_multi_target_integration_2x5.bundle.json}"
EPISODE_INDEX="${SPARK_PHASE8_EPISODE_INDEX:-0}"
NOISE_STD="${SPARK_PHASE8_NOISE_STD_RAD:-0.01}"

if [[ "${DEMO_MODE}" -eq 1 ]]; then
  SAFETY_PROFILE="${SPARK_PHASE8_SAFETY_PROFILE:-simulation_demo_visible}"
  CHECKPOINT="${SPARK_PHASE8_CHECKPOINT:-${REPO_ROOT}/artifacts/residuals/phase8_gui_demo_visible_policy.json}"
  # 8 mm default: visible on ~14 mm cubes, recoverable under demo bounds.
  TIP_BIAS_X="${SPARK_PHASE8_TIP_BIAS_X:-0.008}"
  TIP_BIAS_Y="${SPARK_PHASE8_TIP_BIAS_Y:-0.0}"
  TIP_BIAS_Z="${SPARK_PHASE8_TIP_BIAS_Z:-0.0}"
  BIASED_BUNDLE="${SPARK_PHASE8_OFF_BUNDLE:-${REPO_ROOT}/artifacts/reports/phase8_residual_gui_demo_off.bundle.json}"
  RESIDUAL_BUNDLE="${SPARK_PHASE8_BUNDLE:-${REPO_ROOT}/artifacts/reports/phase8_residual_gui_demo_on.bundle.json}"
  REPORT_OFF="${SPARK_PHASE8_REPORT_OFF:-${REPO_ROOT}/artifacts/reports/phase8_residual_gui_demo_off.json}"
  REPORT_ON="${SPARK_PHASE8_REPORT_ON:-${REPO_ROOT}/artifacts/reports/phase8_residual_gui_demo_on.json}"
  FORCE_RETRAIN_FLAG=(--force-retrain)
  echo "phase8_residual_gui: mode=DEMO_VISIBLE profile=${SAFETY_PROFILE} tip_bias=(${TIP_BIAS_X},${TIP_BIAS_Y},${TIP_BIAS_Z})"
else
  SAFETY_PROFILE="${SPARK_PHASE8_SAFETY_PROFILE:-simulation_bounded_residual}"
  CHECKPOINT="${SPARK_PHASE8_CHECKPOINT:-${REPO_ROOT}/artifacts/residuals/phase8_gui_smoke_policy.json}"
  TIP_BIAS_X="${SPARK_PHASE8_TIP_BIAS_X:-0.001}"
  TIP_BIAS_Y="${SPARK_PHASE8_TIP_BIAS_Y:-0.0}"
  TIP_BIAS_Z="${SPARK_PHASE8_TIP_BIAS_Z:-0.0}"
  BIASED_BUNDLE="${SPARK_PHASE8_OFF_BUNDLE:-${REPO_ROOT}/artifacts/reports/phase8_residual_gui_off.bundle.json}"
  RESIDUAL_BUNDLE="${SPARK_PHASE8_BUNDLE:-${REPO_ROOT}/artifacts/reports/phase8_residual_gui_integration_2x5.bundle.json}"
  REPORT_OFF="${SPARK_PHASE8_REPORT_OFF:-${REPO_ROOT}/artifacts/reports/phase8_residual_gui_off.json}"
  REPORT_ON="${SPARK_PHASE8_REPORT_ON:-${REPO_ROOT}/artifacts/reports/phase8_residual_gui_on.json}"
  FORCE_RETRAIN_FLAG=()
  echo "phase8_residual_gui: mode=SUBTLE profile=${SAFETY_PROFILE} tip_bias=(${TIP_BIAS_X},${TIP_BIAS_Y},${TIP_BIAS_Z})"
fi

if [[ ! -f "${SOURCE_BUNDLE}" ]]; then
  echo "Source bundle missing: ${SOURCE_BUNDLE}" >&2
  echo "Run Phase 7.2 integration smoke first, or set SPARK_PHASE8_SOURCE_BUNDLE." >&2
  exit 1
fi

export PYTHONPATH="${REPO_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}"

USD="${REPO_ROOT}/assets/mycobot_280_m5/prepared/mycobot_280_m5.usd"
NESTED_USD="${REPO_ROOT}/assets/mycobot_280_m5/prepared/mycobot_280_m5/mycobot_280_m5.usda"
if [[ ! -f "${USD}" && ! -f "${NESTED_USD}" ]]; then
  "${REPO_ROOT}/scripts/convert_urdf_to_usd.sh"
fi
if [[ ! -f "${USD}" ]]; then
  USD="${NESTED_USD}"
fi
if [[ ! -f "${USD}" ]]; then
  echo "ERROR: prepared robot USD not found after conversion attempt: ${USD}" >&2
  exit 1
fi

play_one() {
  local label="$1"
  local bundle="$2"
  local report="$3"
  local hud_text="$4"
  echo "============================================================"
  echo "phase8_residual_gui: PASS ${label}"
  echo "  bundle=${bundle}"
  echo "  report=${report}"
  echo "  hud=${hud_text}"
  echo "============================================================"
  set +e
  "${ISAACSIM_PYTHON_EXE}" "${REPO_ROOT}/isaac_sim/play_multi_target_suite.py" \
    --repo-root "${REPO_ROOT}" \
    --bundle "${bundle}" \
    --usd "${USD}" \
    --output-report "${report}" \
    --episode-index "${EPISODE_INDEX}" \
    --run-label "${hud_text}" \
    "${MODE_ARGS[@]}"
  local play_rc=$?
  set -e
  if [[ ${play_rc} -ne 0 ]]; then
    echo "phase8_residual_gui: PASS ${label} play EXIT:${play_rc}" >&2
    return "${play_rc}"
  fi
  if ! python3 -c "import json,sys; p=json.load(open(sys.argv[1])); sys.exit(0 if p.get('joint_playback_completed') and not p.get('error') else 2)" "${report}"; then
    echo "phase8_residual_gui: PASS ${label} report incomplete or errored" >&2
    return 2
  fi
  python3 -c "import json,sys; p=json.load(open(sys.argv[1])); s=p.get('summary') or {}; print(f\"phase8_residual_gui: PASS {sys.argv[2]} tip={s.get('total_tip_contacts')} body={s.get('total_body_contacts')} self={s.get('total_self_collisions')} success_rate={s.get('success_rate')}\")" "${report}" "${label}"
  return 0
}

echo "phase8_residual_gui: inject tip bias (PASS A prep / residual-off)"
python3 "${REPO_ROOT}/scripts/apply_residual_noise_to_bundle.py" \
  --bundle "${SOURCE_BUNDLE}" \
  --output-bundle "${BIASED_BUNDLE}" \
  --inject-tip-bias-only \
  --tip-bias-m "${TIP_BIAS_X}" "${TIP_BIAS_Y}" "${TIP_BIAS_Z}" \
  --residual-safety-profile "${SAFETY_PROFILE}" \
  --noise-seed 8008

echo "phase8_residual_gui: train/apply residual on biased bundle (PASS B prep)"
python3 "${REPO_ROOT}/scripts/apply_residual_noise_to_bundle.py" \
  --bundle "${BIASED_BUNDLE}" \
  --output-bundle "${RESIDUAL_BUNDLE}" \
  --checkpoint "${CHECKPOINT}" \
  --train-if-missing \
  "${FORCE_RETRAIN_FLAG[@]}" \
  --measurement-noise-std-rad "${NOISE_STD}" \
  --noise-seed 8008 \
  --tip-bias-m "${TIP_BIAS_X}" "${TIP_BIAS_Y}" "${TIP_BIAS_Z}" \
  --residual-safety-profile "${SAFETY_PROFILE}"

# A/B: residual-off (biased) then residual-on (corrected).
if [[ "${DEMO_MODE}" -eq 1 ]]; then
  HUD_MODE="DEMO"
else
  HUD_MODE="SUBTLE"
fi
# Prefer the shared formatter so smoke and unit tests stay aligned.
HUD_A="$(
  PYTHONPATH="${REPO_ROOT}/src:${REPO_ROOT}" python3 - <<PY
from isaac_sim.playback_hud import format_phase8_ab_hud_text
print(format_phase8_ab_hud_text(pass_id="Pass A", residual_on=False, mode="${HUD_MODE}"))
PY
)"
HUD_B="$(
  PYTHONPATH="${REPO_ROOT}/src:${REPO_ROOT}" python3 - <<PY
from isaac_sim.playback_hud import format_phase8_ab_hud_text
print(format_phase8_ab_hud_text(pass_id="Pass B", residual_on=True, mode="${HUD_MODE}"))
PY
)"
play_one "A_residual_off" "${BIASED_BUNDLE}" "${REPORT_OFF}" "${HUD_A}"
play_one "B_residual_on" "${RESIDUAL_BUNDLE}" "${REPORT_ON}" "${HUD_B}"

echo "phase8_residual_gui: A/B complete"
echo "  off_report=${REPORT_OFF}"
echo "  on_report=${REPORT_ON}"
echo "phase8_residual_gui: EXIT:0"
