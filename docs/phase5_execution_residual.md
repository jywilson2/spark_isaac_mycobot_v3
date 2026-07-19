# Phase 5 — Execution abstraction and zero-residual seam

## Scope and safety boundary

Phase 5 separates independently validated planning output from future execution
integrations. It does not add hardware motion, learned behavior, or another
planner. The only supplied corrector is `ZeroResidualCorrector`; therefore the
dry-run joint command stream is exactly the validated cuRobo trajectory.

Non-zero Cartesian residuals can be inspected by `SafetyProjector`, but the
Phase 5 executor rejects them because no bounded Cartesian-to-joint correction
mapping has been accepted yet. This prevents the seam from becoming an
implicit IK solver or replacement trajectory generator.

## Public contracts

- `residual.py`: `CartesianResidual`, `ResidualObservation`,
  `ResidualCorrector`, and `ZeroResidualCorrector`.
- `safety.py`: `ResidualSafetyProfile`, `SafetyDecision`, and the deterministic
  `SafetyProjector`.
- `execution.py`: `TrajectorySource`, `RobotStateProvider`,
  `ReplayRobotStateProvider`, `TcpPoseEvaluator`, `TrajectoryExecutor`, and
  `InMemoryCommandAdapter`.
- `config/residual_safety.yml`: explicit SI limits for translation, rotation,
  lateral corridor, joint margin, state age, and watchdog timeout.

The safety projector reports clipping explicitly. It rejects non-executable
plans, stale or future state, watchdog expiry, invalid joint-vector shape,
joint-envelope violations, and corrected TCP positions outside the
target-normal corridor.

## Execution sequence

For every waypoint, `TrajectoryExecutor`:

1. requires a `ValidatedPlan` whose report is valid and executable;
2. samples the unchanged combined nominal trajectory;
3. obtains a timestamped measured state from the injected provider;
4. evaluates the nominal TCP pose without invoking a planner;
5. requests a typed Cartesian residual;
6. projects and rechecks freshness, joint feasibility, and corridor bounds;
7. rejects any non-zero Phase 5 correction; and
8. emits the unchanged command through the in-memory adapter.

Any failure stops before the offending command and returns a structured
`ExecutionResult`.

## Acceptance evidence

The Phase 5 unit suite verifies:

- zero residual reproduces the nominal command matrix exactly;
- oversized translation and rotation are explicitly clipped;
- an off-axis correction is rejected by a tighter synthetic terminal corridor;
- stale state, watchdog timeout, invalid plans, and joint-envelope violations
  fail closed;
- non-zero residuals cannot generate replacement joint paths; and
- the Phase 5 modules do not import physical drivers, ROS, Isaac Sim, or Isaac
  Lab.

`./scripts/run_verification.sh ci` passes with 90 unit tests and Ruff
lint/format checks. The cache paths used by pytest and Ruff are explicitly
writable in root-squashed containers; no warning is suppressed.

`./scripts/run_verification.sh spark --with-gpu` also passes all five existing
GPU integrations through Isaac Sim's host `python.sh`. The previously recorded
PyTorch warning that GB10 compute capability 12.1 is newer than the wheel's
advertised maximum 12.0 remains visible.

## Remaining boundary

Phase 8 may add a non-zero residual corrector only behind these contracts and
only with a separately accepted bounded local mapping followed by independent
validation. Hardware adapters remain out of scope until Phase 9 and must stay
disabled by default.
