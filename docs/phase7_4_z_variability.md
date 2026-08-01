# Phase 7.4 — Extended Z variability and Z-aware EE-clearance spacing

**Status:** Specified; implementation pending. **Branch:** `wip_phase7_4`.
Normative rules live in [`spec.md`](../spec.md) §8 Phase 7.4; this document
carries design rationale and will grow into the phase report during
implementation.

## Problem

Phase 7.2/7.3 generated fields place cube centres in a fixed mid-Z band of
width `0.5 * arm_z_motion_range_m` (`GRID_Z_VARIABILITY_FRACTION` in
`target_placement.py`). The approach-plane EE-clearance floor
(`target_edge_m + flange_diameter_assumption_m + ee_approach_clearance_m`)
was derived assuming near-coplanar cube tops. Widening Z variation breaks
that assumption:

- The **free-space approach segment** is unaffected in principle: every
  remaining cube is a cuboid obstacle in the cuRobo world (omit-active
  invariant), and trajectory optimization routes over taller neighbors
  without heuristic lift waypoints. The residual risk is local-minimum
  failures, mitigated by graph-seeded attempts.
- The **linear terminal descent** of `plan_grasp` cannot curve. A neighbor at
  the minimum XY separation but sitting much higher can intrude into the
  descent corridor of a shorter target, turning legal placements into
  systematic `planning_infeasible` retries.

## Design

Two configuration keys, both preserving existing fields at their defaults:

| Key | Default | Meaning |
|-----|---------|---------|
| `z_band_fraction` | `0.5` | Fraction of `arm_z_motion_range_m` used as the generated-centre Z band width, clamped to `field_aabb` Z |
| `z_separation_gain` | `1.0` | Slope of the Z-aware separation floor per metre of top-face delta, clamped at `pre_approach_distance_m` |

Pairwise floor (see spec for the normative statement):

    edge + flange + ee_approach_clearance
      + z_separation_gain * min(Δz_top_faces, pre_approach_distance_m)

Gain 1.0 gives a 45° escape cone from the lower cube's descent corridor. The
additive term clamps at `pre_approach_distance_m` because only the linear
segment below the standoff needs the widened corridor; the base floor
guarantees corridor width at every height when `Δz` exceeds the clamp
("canyon start" standoffs are permitted but raise expected retries).

## Open items for implementation

- Confirm `plan_grasp` retry attempts benefit from graph-seeded warmup
  (`enable_graph_warmup`) on the plan path, not only during warmup.
- Record planning-duration distributions at `z_band_fraction` 0.5 vs the
  widened setting (sim timings only; no Orin SLA claim).
- Decide whether any suite needs an unclamped gain option; the spec ships the
  clamped rule only.
