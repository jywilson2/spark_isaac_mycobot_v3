# Phase 6 — Randomized workspace benchmark

## Scope and safety boundary

Phase 6 measures the existing cuRobo `plan_grasp` → independent validation
path. It does not add another planner, Isaac/ROS/hardware dependencies, or
physical commands. The AABBs in `config/benchmark_workspace.yml` are honestly
declared as conservative, unmeasured candidate regions near the Phase 3/4
reachable interior; they are not a measured dexterous-workspace claim.

## Reproducibility and replay

`sample_benchmark_cases` uses NumPy's deterministic generator from one root
seed. A frozen case records region and normal-bin labels, target position and
normal in `g_base`, explicit start joints in radians, roll policy,
pre-approach distance in meters, scene revision, and planner seed.

The planner's Phase 3 invariant requires
`request.random_seed == profile.random_seed`. The runtime therefore uses
`dataclasses.replace(base_profile, random_seed=case_seed)` and constructs a
fresh `NominalPlanner` for every attempt. Reports preserve the raw planner
status and embed the complete serialized `PlanningRequest` for every failure.
`scripts/plan_single_target.py` replays either a request JSON object or one
failed request selected from a benchmark report.

## Stages and reports

- smoke: 20 frozen cases (`root_seed=6006`);
- regression: 100 frozen parameter-only cases (`root_seed=60100`);
- exploratory: at least 1,000 generated cases (not claimed as executed here).

Run:

```bash
python3 scripts/benchmark_random_targets.py --stage smoke --root-seed 6006
python3 scripts/plan_single_target.py artifacts/benchmarks/phase6_smoke_seed_6006.json
```

Matching JSON and Markdown reports are written under `artifacts/benchmarks/`
by default. JSON retains all case outcomes and failed replay requests.
Aggregation counts every submitted case. Optional zero-residual replay runs
only after validation; an execution rejection is reported separately and is
never relabeled as a planning failure.

## Failure taxonomy

The stable categories are `no_reachable_ik`, `collision_infeasibility`,
`trajectory_optimization_failure`, `terminal_line_validation_failure`,
`orientation_validation_failure`, `numerical_failure`, and
`configuration_model_failure`. Planner failures map from their structured
category/reason/status while retaining raw status. Validation failures map
from all violation metric names using deterministic precedence.

## Verification

Unit tests cover root-seed determinism, request round-trip replay, taxonomy,
aggregation, report output, and fixture integrity. The GPU-marked integration
loads the frozen 20-case smoke fixture, executes a short dual-run subset under
the fresh-backend/warmup lifecycle, checks zero disagreement, and writes both
report formats to an ownership-safe temporary directory when cuRobo/CUDA is
available. The full 20-case smoke stage remains a CLI command.

Container CI passed 97 unit tests and Ruff lint/format. Host GPU verification
passed all six integrations, including the Phase 6 dual-run smoke subset. The
previously recorded GB10 PyTorch compute-capability warning remains visible.

No exploratory 1,000-case result or physical accuracy claim is made by this
implementation report.
