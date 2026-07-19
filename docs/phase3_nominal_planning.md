# Phase 3 — cuRobo nominal planning

## Scope and safety boundary

Phase 3 converts a validated surface target and named current joint state into
cuRobo's free-space approach and constrained terminal trajectories. It does not
authorize execution: every `NominalPlan` is created with
`validation_status="not_evaluated"` and `executable=False`. Phase 4 owns
independent waypoint validation.

cuRobo is the exclusive global and local motion planner. No retry, fallback,
learned policy, simulator, integration, or execution adapter may generate a
replacement path. The documented two-call fallback may vary the pinned cuRobo
API sequence, but both calls remain cuRobo-owned.

## Public API path

The adapter uses cuRobo v0.8.0 public APIs:

- `MotionPlannerCfg.create(...)` for planner configuration;
- `MotionPlanner.plan_grasp(...)` for goal-set selection, free-space approach,
  and approach-to-target terminal motion;
- `GoalToolPose` and `JointState` for typed inputs.

The configured pre-approach distance is converted to a signed offset from the
TCP approach-axis convention. `plan_grasp_to_lift` is disabled. Padded samples
past cuRobo's valid-last-timestep metadata are never included.

## v0.8.0 planner lifecycle decision

GPU investigation found that a successful `plan_grasp` call mutates internal
optimizer/tool-criteria state. A second call on the same planner could fail or
return a shortened trajectory, despite identical input and seed. Resetting the
seed and calling public warmup before or after planning did not reliably restore
the original behavior.

The selected Phase 3 policy is therefore:

1. `NominalPlanner` receives an application-owned backend factory.
2. Every `plan_grasp` attempt constructs a fresh `MotionPlanner`.
3. The adapter resets the seed, runs configured public warmup, resets the seed
   again, then issues exactly one `plan_grasp` call.
4. Retries after infeasibility also construct a fresh planner and repeat the
   warmup sequence.
5. Construction and warmup cost is included in adapter wall time.
6. Reuse remains prohibited for the pinned v0.8.0 runtime.

Phase 4 endpoint validation later showed that an unwarmed fresh planner could
report success while remaining at the pre-approach pose; mandatory warmup is
therefore part of this lifecycle, not optional. This is a reliability-first
tradeoff. It costs more GPU latency than warmed reuse, but avoids depending on
mutated private state. A future cuRobo upgrade may remove this policy only
after the repeated-call and endpoint GPU regressions pass.

## Result and failure contracts

A successful result records:

- selected goal index and corresponding roll angle in radians;
- finite approach, terminal, and concatenated joint trajectories;
- planner status and backend/adapter timings;
- scene revision, planner profile, seed, and cuRobo version.

Normal infeasibility returns a `PlanningFailure` rather than raising. Missing,
non-finite, discontinuous, or index-invalid backend data fails closed as
`invalid_planner_result`. Runtime/backend exceptions are structured as
`backend_error`.

## Acceptance evidence

Executed on 2026-07-19:

- `python3 -m pytest tests/unit` — **61 passed**.
- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 <Isaac Sim Python> -m pytest -m gpu
  tests/integration/test_phase3_nominal_planning_gpu.py -vv` — **1 passed** on
  the DGX Spark host.

The strengthened GPU regression constructs distinct warmed planners for
identical seeded requests, verifies both trajectory segments and selected roll,
compares reproduced approach trajectories and terminal FK poses, checks the
target-normal line, and requires the measured endpoint to reach the target
within configured planner tolerance. Phase 3 and Phase 4 GPU tests pass with
the mandatory lifecycle. This is simulation/FK evidence, not physical-robot
accuracy evidence.

## Review recommendation

Review the planner-construction latency during Phase 6 benchmarking. Do not
restore backend reuse solely for performance; require a pinned-version upgrade
and passing lifecycle regression first.
