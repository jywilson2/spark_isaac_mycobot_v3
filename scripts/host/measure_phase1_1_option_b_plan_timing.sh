#!/usr/bin/env bash
# Compare per-leg planning time: scaffolding vs Option B dual overlay.
# Same suite (integration 2×5), seed, and planner profile. Host-only.
#
#   ./scripts/host/measure_phase1_1_option_b_plan_timing.sh
#   # optional: ROOT_SEED=4242
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck source=./env.isaac_host.sh
# shellcheck disable=SC1091
source "${root}/scripts/host/env.isaac_host.sh"
spark_host_require_native_shell
spark_host_apply_env

root_seed="${ROOT_SEED:-4242}"
out_dir="${root}/artifacts/reports/phase1_1_option_b_timing"
mkdir -p "${out_dir}"

scaffold_report="${out_dir}/integration_2x5_scaffolding.json"
scaffold_bundle="${out_dir}/integration_2x5_scaffolding.bundle.json"
dual_report="${out_dir}/integration_2x5_option_b.json"
dual_bundle="${out_dir}/integration_2x5_option_b.bundle.json"
aggregate="${out_dir}/aggregate.json"
aggregate_md="${out_dir}/aggregate.md"

echo "=== scaffolding baseline (root_seed=${root_seed}) ==="
SPARK_PHASE7_2_REPORT="${scaffold_report}" \
SPARK_PHASE7_2_BUNDLE="${scaffold_bundle}" \
  bash "${root}/scripts/host/smoke_phase7_2_integration_2x5.sh" \
    --headless --auto-exit --root-seed "${root_seed}"

echo "=== Option B dual overlay (root_seed=${root_seed}) ==="
SPARK_PHASE7_2_REPORT="${dual_report}" \
SPARK_PHASE7_2_BUNDLE="${dual_bundle}" \
  bash "${root}/scripts/host/smoke_phase7_2_integration_2x5_option_b.sh" \
    --headless --auto-exit --root-seed "${root_seed}"

"${ISAACSIM_PYTHON_EXE}" - <<PY
import json
from pathlib import Path

out = Path(${out_dir@Q})
scaffold = json.loads((out / "integration_2x5_scaffolding.json").read_text())
dual = json.loads((out / "integration_2x5_option_b.json").read_text())
s = scaffold["summary"]["planning_duration_s"]
d = dual["summary"]["planning_duration_s"]
ratio_p50 = float(d["p50"]) / float(s["p50"]) if float(s["p50"]) > 0 else float("inf")
ratio_p95 = float(d["p95"]) / float(s["p95"]) if float(s["p95"]) > 0 else float("inf")
# Host (DGX Spark) deployment budget for Option B re-arming review: dual must
# stay within 1.50× scaffolding p50 and 2.00× scaffolding p95 on this suite.
budget_p50 = 1.50
budget_p95 = 2.00
passed = ratio_p50 <= budget_p50 and ratio_p95 <= budget_p95
agg = {
    "suite": "phase7_2_multi_target_integration_2x5",
    "root_seed": int(${root_seed@Q}),
    "scaffolding": s,
    "option_b_dual": d,
    "ratio_dual_over_scaffolding": {"p50": ratio_p50, "p95": ratio_p95},
    "host_budget": {"max_p50_ratio": budget_p50, "max_p95_ratio": budget_p95},
    "host_budget_passed": passed,
    "scaffold_success_rate": scaffold["summary"]["success_rate"],
    "dual_success_rate": dual["summary"]["success_rate"],
    "scaffold_planning_failures": scaffold["summary"]["total_planning_failures"],
    "dual_planning_failures": dual["summary"]["total_planning_failures"],
}
(out / "aggregate.json").write_text(json.dumps(agg, indent=2, sort_keys=True) + "\\n")
md = f"""# Phase 1.1 Option B planning-time evidence

Suite: integration 2×5, root_seed={agg['root_seed']}, host DGX Spark.

| Config | plan p50 (s) | plan p95 (s) | success_rate | planning_failures |
|--------|-------------:|-------------:|-------------:|------------------:|
| scaffolding (32) | {s['p50']:.3f} | {s['p95']:.3f} | {agg['scaffold_success_rate']} | {agg['scaffold_planning_failures']} |
| Option B dual (32+1012) | {d['p50']:.3f} | {d['p95']:.3f} | {agg['dual_success_rate']} | {agg['dual_planning_failures']} |

**Ratios (dual / scaffolding):** p50 = {ratio_p50:.3f}×, p95 = {ratio_p95:.3f}×

**Host budget (declared for DGX Spark re-arming review):** p50 ≤ {budget_p50:.2f}×, p95 ≤ {budget_p95:.2f}× → **{'PASS' if passed else 'FAIL'}**

Orin AGX / embedded device calibration is not in this measurement; device
claims require device runs (spec §8 Phase 1.1 planning-time criterion).
"""
(out / "aggregate.md").write_text(md)
print(md)
if not passed:
    raise SystemExit(2)
PY
