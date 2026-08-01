# Phase 7.4 — Extended Z variability and Z-aware EE-clearance spacing

**Status:** Implemented (2026-08-01) on `wip_phase7_4`.  
Normative rules: [`spec.md`](../spec.md) §8 Phase 7.4.

## Problem

Phase 7.2/7.3 generated fields place cube centres in a mid-Z band of width
`0.5 * arm_z_motion_range_m`. The approach-plane EE-clearance floor
(`target_edge_m + flange_diameter_assumption_m + ee_approach_clearance_m`)
assumed near-coplanar tops. Widening Z variation breaks that assumption for
the linear `plan_grasp` terminal descent.

## Design (landed)

| Key | Default | Meaning |
|-----|---------|---------|
| `z_band_fraction` | `0.5` | Band width = fraction × `arm_z_motion_range_m`. Must be `> 0`; **not** upper-clamped. |
| `delta_z_m` | unset | Optional absolute **full** band width (metres). Overrides the fraction when set. **Not** clamped to `field_aabb` Z or to the arm reach envelope. |
| `z_separation_gain` | `1.0` | Slope of the Z-aware separation floor per metre of top-face Δz; must be `≥ 1.0`. Additive term clamps at `pre_approach_distance_m`. |

Pairwise floor:

```text
edge + flange + ee_approach_clearance
  + z_separation_gain * min(Δz_top_faces, pre_approach_distance_m)
```

(`min_center_separation_m` may raise the constant part only.)

### Arm-reach validation and substitute retries

Generated centres (`grid`, `random`, `layout`) that fall outside the declared
tip-reach envelope (Z within `arm_z_motion_range_m` centered on field mid-Z;
optional `max_target_radial_m` rim) are **discarded** and a substitute is
sampled (same XY, Z in band ∩ reach). After **3** failed substitutes for one
target, suite target generation fails closed (`ConfigurationError`). Manual
lists fail closed on reach violations without substitution.

`delta_z_m` / `z_band_fraction` are intentionally unclamped: oversized bands
are accepted at config load; unreachable samples are rejected by the reach
check and retry budget.

### Testing defaults

- Default Z variability used in testing is **~50%** of
  `arm_z_motion_range_m` (`z_band_fraction: 0.5`).
- Example widened/explicit-delta suite:
  [`config/phase7_4_multi_target_widened_z.yml`](../config/phase7_4_multi_target_widened_z.yml)
  (`delta_z_m: 0.14`).
- Densest GUI smoke remains standard 2×20
  (`scripts/host/smoke_phase7_2_standard_2x20.sh`).

## Evidence / gates

- Unit: `tests/unit/test_phase7_4_z_variability.py` (band defaults, unclamped
  `delta_z_m`, Z-aware floor, gain&lt;1 fail-closed, ROM retry exhaustion,
  widened config sample, 2×20 pack under Z-aware floor).
- Field AABBs for default grid / `--targets 10` fallback widened so nearest
  XY spacing clears `0.076 + pre_approach` when Δz &gt; 0.
