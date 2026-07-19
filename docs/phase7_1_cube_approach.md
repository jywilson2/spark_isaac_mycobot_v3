# Phase 7.1 — Unknown-start normal-approach cube visualization

## Status

Requirements finalized; implementation pending on `wip_phase7_1`.

Authoritative acceptance criteria are in [`spec.md`](../spec.md) §8.

## Purpose

Run a configurable, replayable set of cuRobo-planned episodes in which the
circular bare-flange face approaches a small cube along the cube-face normal
and configured signed tool axis. Stream readable results while the suite runs,
then write matching machine-readable aggregate evidence.

Phase 7.1 extends, but does not replace:

- Phase 6 deterministic sampling, failure taxonomy, and replay;
- Phase 4 independent terminal/collision validation; and
- Phase 7 Isaac validated-plan playback.

## Fixed requirement decisions

- Branch: `wip_phase7_1`.
- Default episode count: **5**.
- Default cube edge: **14 mm**.
- Cube derivation: a square face covering 25% of a circular 31 mm flange face,
  `s = d * sqrt(pi) / 4 ≈ 13.7 mm`, rounded to 14 mm.
- The 31 mm diameter is an unverified design assumption pending physical
  measurement in Phase 9.
- Default modes: A independent unknown starts and D 3D goal diversity.
- Optional ordinary-run modes: B chained starts and C relocate then approach.
- Acceptance testing must exercise A, B, C, and D.
- The cube is world-collision geometry in cuRobo and Isaac; Phase 7.1 stops at
  a positive configurable standoff and does not command physical contact.
- Isaac tip metrics remain null and `not_evaluated`.

## Mode contract

### A — Independent unknown start (default)

Select seeded valid joint starts from a diverse bank or configured continuous
joint ranges. Reject and count self-colliding, out-of-limit, or malformed
states. Freeze every exact start for replay; a zeros-only suite is invalid.

### B — Chained start (optional)

Begin each episode from the prior successful endpoint. Failure handling must be
explicit: use the last successful state or terminate according to
configuration.

### C — Relocate then approach (optional)

Use cuRobo for an unknown-start-to-safe-nest segment, then cuRobo
`plan_grasp` for the cube approach. Both segments must be collision-safe and
eligible for execution. Terminal line/orientation metrics apply to the
approach.

### D — 3D goal diversity (default)

Sample cube positions in a declared conservative `g_base` AABB and normals
from labeled bins. The region is not a measured dexterous-workspace claim.

## Reporting and acceptance

Each live console result includes episode index/count, start mode/label, cube
position/normal, planner and validation states, lateral/axis errors,
self/world collision results/clearances, Isaac prohibited-contact count,
failure category, and timing. The final console summary and JSON artifact
include pass count/rate, failure counts, p50/p95 lateral and axis errors, root
seed, and complete frozen requests.

An episode passes only when cuRobo is the sole planner, independent validation
marks it executable, terminal geometry and progress pass, limits/dynamics pass,
and no self- or arm-to-cube/environment collision is detected. Unevaluated
non-empty-world clearance fails closed. Zero prohibited Isaac contact events
is required as separate simulation evidence but does not authorize a plan.
Simulation thresholds are not physical accuracy evidence.

Phase 7.1 does not create or enable a tip tool. Joint playback may succeed
while Isaac tip position/orientation remain null and `not_evaluated`.
