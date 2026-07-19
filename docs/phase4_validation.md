# Phase 4 — Independent trajectory validation

## Scope and safety boundary

Phase 4 treats every Phase 3 planner result as untrusted until a separate
validator recomputes its safety and task metrics. A valid report grants
execution eligibility; any violation or unevaluated required metric fails
closed. Simulation thresholds are not physical MyCobot accuracy claims.
Validation may accept or reject a cuRobo path but never generate, repair, or
replace one; cuRobo remains the exclusive global and local motion planner.

## Public contracts

`src/mycobot_curobo/validation.py` provides:

- `ValidationProfile` — SI thresholds loaded from
  `config/validation_profiles.yml`;
- `KinematicCollisionBatch` — ordered FK/collision evaluator output;
- `ValidationViolation` / `ValidationMetrics` / `ValidationReport`;
- `ValidatedPlan` — nominal plan plus eligibility;
- `CuroboTrajectoryEvaluator` — cuRobo FK and self-collision sphere pairs;
- `validate_nominal_plan(...)` — fail-closed entrypoint.

Roll error is the angle between the projected selected-roll tangent and the
actual TCP tangent about the approach axis. Joint-limit and dynamics checks
run before FK so limit failures become structured violations rather than
exceptions.

## Required checks

For each terminal waypoint:

1. finite FK and trajectory values;
2. lateral distance from the target-normal line;
3. configured signed TCP-axis alignment;
4. selected-roll alignment when roll is constrained;
5. monotonic progress toward the target;
6. joint position margin, velocity, acceleration, and available jerk;
7. self-collision sphere clearance;
8. world clearance — empty worlds are evaluated explicitly; unsupported
   non-empty worlds remain unevaluated and fail closed;
9. terminal position and orientation error;
10. approach/terminal boundary continuity.

## Simulation profile

`config/validation_profiles.yml` carries the specification's initial
simulation thresholds: 5 mm lateral and terminal-position error, 0.05236 rad
approach-axis and terminal-orientation error, 1 mm progress regression, and
0.02 rad joint-limit margin. It additionally configures 0.05236 rad selected
roll error, zero-meter minimum self/world clearances, and a 1e-6 rad segment
boundary tolerance. These are simulation starting values, not hardware
accuracy or safety-margin claims.

## cuRobo v0.8.0 lifecycle correction

The first GPU endpoint gate exposed a Phase 3 issue that the earlier line-only
test could not detect. A fresh but unwarmed `MotionPlanner` reported success
while its terminal trajectory remained approximately one full pre-approach
offset from the target. Generic warmup before the first `plan_grasp` call
restored the expected approach-to-target motion.

The enforced lifecycle is now:

1. construct a fresh planner for each attempt;
2. reset its seed;
3. run configured public warmup;
4. reset the seed again;
5. issue exactly one `plan_grasp` call.

No planner instance is reused. Construction and warmup remain part of request
wall time.

## Acceptance evidence

The synthetic suite covers a valid normal-line path, curved lateral deviation,
reversed progress, TCP misorientation, joint limit/dynamics violations, self
collision, non-finite evaluator output, and unevaluated world collision. A
prior local suite contained 68 tests before final strengthening; that
historical count is not presented as the final suite total.

The GPU test independently evaluates a real cuRobo plan using cuRobo FK and
configured self-collision sphere pairs in an explicitly empty world. These are
simulation/model results, not physical-robot accuracy evidence. Phase 3 and
Phase 4 GPU tests pass with the mandatory warmup lifecycle.

## Review recommendation

The accepted scene is empty. Non-empty scenes intentionally fail closed until a
supported world-distance adapter and obstacle regression are added. The
`hardware_placeholder` profile exists only as a named stub for Phase 9/10.
