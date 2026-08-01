#!/usr/bin/env bash
# Unseeded 2×20 robustness batch with Option B dual overlay armed.
# Default: 10 sequential headless runs (no --root-seed).
#
#   ./scripts/host/run_phase1_1_option_b_unseeded_2x20.sh
#   RUNS=5 ./scripts/host/run_phase1_1_option_b_unseeded_2x20.sh
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck source=./env.isaac_host.sh
# shellcheck disable=SC1091
source "${root}/scripts/host/env.isaac_host.sh"
spark_host_require_native_shell
spark_host_apply_env

runs="${RUNS:-10}"
out_dir="${root}/artifacts/reports/phase1_1_option_b_unseeded_2x20"
mkdir -p "${out_dir}"
log_dir="${out_dir}/logs"
mkdir -p "${log_dir}"

failures=0
for i in $(seq 1 "${runs}"); do
  tag="$(printf 'run_%02d' "${i}")"
  report="${out_dir}/${tag}.json"
  bundle="${out_dir}/${tag}.bundle.json"
  log="${log_dir}/${tag}.log"
  echo "=== ${tag} (Option B dual, unseeded 2×20) ==="
  set +e
  SPARK_PHASE7_2_REPORT="${report}" \
  SPARK_PHASE7_2_BUNDLE="${bundle}" \
    bash "${root}/scripts/host/smoke_phase7_2_standard_2x20_option_b.sh" \
      --headless --auto-exit \
      >"${log}" 2>&1
  status=$?
  set -e
  echo "${tag}: exit=${status}"
  if [[ "${status}" -ne 0 ]]; then
    failures=$((failures + 1))
  fi
done

"${ISAACSIM_PYTHON_EXE}" - <<PY
import json
import re
from pathlib import Path

out = Path(${out_dir@Q})
shell_exits = {}
for i in range(1, int(${runs@Q}) + 1):
    tag = f"run_{i:02d}"
    # Prefer the echo lines written to stdout during the batch.
shell_log = Path("/tmp/option_b_unseeded_2x20.log")
if shell_log.is_file():
    for line in shell_log.read_text(errors="replace").splitlines():
        m = re.match(r"(run_\\d+): exit=(\\d+)", line)
        if m:
            shell_exits[m.group(1)] = int(m.group(2))
runs = []
for path in sorted(out.glob("run_[0-9][0-9].json")):
    data = json.loads(path.read_text())
    summary = data.get("summary", {})
    log_path = out / "logs" / f"{path.stem}.log"
    log = log_path.read_text(errors="replace") if log_path.is_file() else ""
    shell_exit = shell_exits.get(path.stem)
    success_rate = summary.get("success_rate")
    failed_episodes = int(summary.get("failed_episodes") or 0)
    target_fails = int(summary.get("total_target_failures") or 0)
    suite_ok = (
        success_rate == 1.0
        and failed_episodes == 0
        and target_fails == 0
        and (shell_exit is None or shell_exit == 0)
    )
    runs.append({
        "run": path.stem,
        "shell_exit": shell_exit,
        "suite_ok": suite_ok,
        "success_rate": success_rate,
        "failed_episodes": failed_episodes,
        "total_planning_failures": summary.get("total_planning_failures"),
        "total_target_failures": target_fails,
        "total_tip_contacts": summary.get("total_tip_contacts"),
        "total_body_contacts": summary.get("total_body_contacts"),
        "planning_duration_s": summary.get("planning_duration_s"),
        "log_defer_mentions": len(re.findall(r"defer", log, flags=re.I)),
    })

passed = sum(1 for r in runs if r["suite_ok"])
ok = passed >= 9 and len(runs) == int(${runs@Q})
agg = {
    "runs": runs,
    "suite_pass_count": passed,
    "suite_pass_rate": f"{passed}/{len(runs)}",
    "budget_passed": ok,
    "budget": {
        "min_suite_passes": 9,
        "max_target_failures_per_reported_run": 0,
        "max_failed_episodes_per_reported_run": 0,
    },
    "total_planning_failures_all_runs": sum(
        int(r.get("total_planning_failures") or 0) for r in runs
    ),
}
(out / "aggregate.json").write_text(json.dumps(agg, indent=2, sort_keys=True) + "\\n")
lines = [
    "# Phase 1.1 Option B unseeded 2×20 robustness",
    "",
    f"Runs: {len(runs)}; suite passes: {passed}/{len(runs)}; budget: {'PASS' if ok else 'FAIL'}",
    f"Total planning failures (retries absorbed): {agg['total_planning_failures_all_runs']}",
    "",
    "| run | shell_exit | success_rate | plan_fails | tip | body | plan p50 |",
    "|-----|-----------:|-------------:|-----------:|----:|-----:|---------:|",
]
for r in runs:
    p50 = (r.get("planning_duration_s") or {}).get("p50")
    p50_s = f"{p50:.3f}" if isinstance(p50, (int, float)) else "-"
    lines.append(
        f"| {r['run']} | {r.get('shell_exit')} | {r.get('success_rate')} | "
        f"{r.get('total_planning_failures')} | {r.get('total_tip_contacts')} | "
        f"{r.get('total_body_contacts')} | {p50_s} |"
    )
(out / "aggregate.md").write_text("\\n".join(lines) + "\\n")
print("\\n".join(lines))
raise SystemExit(0 if ok else 2)
PY
