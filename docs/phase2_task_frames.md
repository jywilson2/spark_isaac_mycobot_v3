# Phase 2 — Surface targets and task frames

Date: 2026-07-18  
Branch: `wip_phase2`

## Result

**PASS.** Validated base-frame surface targets now produce deterministic,
right-handed task-frame candidates and public cuRobo `GoalToolPose` goal sets.

## Contracts

- `SurfaceTarget`: finite position, normalized outward normal, optional tangent,
  fixed roll or finite bounded candidate tuple, pre-approach distance, explicit
  `tcp_link`, and non-empty target ID.
- `TaskFrameConfig`: configured TCP axis/sign and whether motion approaches
  against the outward surface normal.
- `TaskFrameCandidate`: immutable target position, base-from-tool rotation,
  scalar-first `wxyz`, approach direction, and normalized roll.
- `SurfaceGoalSet`: ordered candidates plus exact goal-index-to-roll mapping.

Defaults are loaded from `config/app.yml`:

- approach axis/sign: tool Z / -1;
- desired approach: `-surface_normal`;
- unconstrained rolls: 0°, 45°, …, 315°;
- pre-approach range: 0.01–0.15 m.

## Geometry

The configured signed TCP axis is aligned with the desired approach direction.
A tangent hint is projected into the normal plane. If it is degenerate, the
world basis axis least aligned with the approach direction is projected
instead. Cyclic tool columns and a cross product produce a right-handed
rotation. Each roll is applied about the approach direction, preserving the
target point and signed approach-axis alignment.

Each candidate is checked for finite values, orthonormal axes, determinant +1,
and unit quaternion. No invalid vector or matrix is silently clamped.

## Acceptance evidence

```text
python3 -m pytest tests/unit/test_targets_and_frames.py -q -p no:cacheprovider
22 passed

python.sh -m pytest tests/integration/test_phase2_goal_set_gpu.py -q \
  -p no:cacheprovider
1 passed
```

The seeded property test covers 512 normals, including six nearly axis-aligned
cases and nearly parallel tangent hints. Tests cover all x/y/z approach axes
and both signs. Candidate ordering is byte-for-byte deterministic for repeated
inputs. The GPU test creates a three-roll public cuRoboV2 `GoalToolPose` with
shape `[1, 1, 1, 3, 3/4]`.

The existing GB10/PyTorch compute-capability warning remains visible and is
not suppressed.
