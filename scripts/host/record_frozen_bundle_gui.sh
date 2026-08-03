#!/usr/bin/env bash
# Record a frozen multi-target bundle GUI replay to mp4 (host only).
# Usage: record_frozen_bundle_gui.sh BUNDLE.json OUTPUT.mp4
set -euo pipefail

if [[ $# -ne 2 ]]; then
  printf 'Usage: %s BUNDLE.json OUTPUT.mp4\n' "$0" >&2
  exit 2
fi

bundle="$1"
output="$2"
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

if [[ "${bundle}" != /* ]]; then
  bundle="${root}/${bundle}"
fi
if [[ "${output}" != /* ]]; then
  output="${PWD}/${output}"
fi
if [[ ! -f "${bundle}" ]]; then
  printf 'ERROR: bundle not found: %s\n' "${bundle}" >&2
  exit 2
fi

# shellcheck source=./env.isaac_host.sh
# shellcheck disable=SC1091
source "${root}/scripts/host/env.isaac_host.sh"
spark_host_require_native_shell

# Reuse recorder helpers from the Phase 7.2 smoke wrapper.
# shellcheck disable=SC1090
source <(sed -n '/^resolve_record_ffmpeg()/,/^}$/p; /^record_kit_window()/,/^}$/p' \
  "${root}/scripts/host/smoke_phase7_2_multi_target.sh")

if ! command -v xwininfo >/dev/null 2>&1; then
  printf 'ERROR: xwininfo required (x11-utils)\n' >&2
  exit 2
fi
ffmpeg_bin="$(resolve_record_ffmpeg || true)"
if [[ -z "${ffmpeg_bin}" || ! -x "${ffmpeg_bin}" ]]; then
  printf 'ERROR: no ffmpeg found\n' >&2
  exit 2
fi

prepared_usd="${root}/assets/mycobot_280_m5/prepared/mycobot_280_m5.usd"
nested_prepared_usd="${root}/assets/mycobot_280_m5/prepared/mycobot_280_m5/mycobot_280_m5.usda"
if [[ ! -f "${prepared_usd}" ]]; then
  prepared_usd="${nested_prepared_usd}"
fi
if [[ ! -f "${prepared_usd}" ]]; then
  printf 'ERROR: prepared USD missing\n' >&2
  exit 1
fi

mkdir -p "$(dirname "${output}")"
tmp_out="$(mktemp "${TMPDIR:-/tmp}/spark_record.XXXXXX.mp4")"
rm -f "${tmp_out}"

printf 'phase7_5_record: bundle=%s\n' "${bundle}"
printf 'phase7_5_record: ffmpeg=%s\n' "${ffmpeg_bin}"
printf 'phase7_5_record: tmp=%s\n' "${tmp_out}"
printf 'phase7_5_record: final=%s\n' "${output}"

record_kit_window "${tmp_out}" "${ffmpeg_bin}" &
recorder_pid=$!

set +e
spark_host_run_python "${root}/isaac_sim/play_multi_target_suite.py" \
  --repo-root "${root}" \
  --bundle "${bundle}" \
  --usd "${prepared_usd}" \
  --gui --auto-exit \
  --output-report "${root}/artifacts/reports/phase7_5_variable_targets_dz_0_30.gui.json"
play_status=$?
set -e

kill -INT "${recorder_pid}" 2>/dev/null || true
wait "${recorder_pid}" 2>/dev/null || true

if [[ ! -s "${tmp_out}" ]]; then
  printf 'phase7_5_record: ERROR no video written\n' >&2
  printf 'phase7_5_record: EXIT:%s\n' "${play_status}"
  exit 1
fi

cp -f "${tmp_out}" "${output}"
ls -lh "${tmp_out}" "${output}"
printf 'phase7_5_record: wrote %s\n' "${output}"
printf 'phase7_5_record: EXIT:%s\n' "${play_status}"
exit "${play_status}"
