# Phase 7.4 — Extended Z variability and Z-aware EE-clearance spacing

**Status:** **PARTIALLY FUNCTIONAL — closed in this state by decision
2026-08-02.** Implemented (2026-08-01) on `wip_phase7_4`, including the
2026-08-01 dexterous-reach screening amendment (suite-wide
reject-and-regenerate) and the goal-feasibility amendment (world-aware
tip-IK screen, `order: z_desc`, goal-set rolls, regen on
`targets_unplanned`). Default/mid-width band suites pass; wide-band
fixed-count acceptance evidence is partial (one 3×15 `delta_z_m: 0.30`
headless pass at 2/3 with `max_failed_episodes: 1`; GUI and 2×15 / 2×20
unproven) and is accepted as-is — the wide-band stress goal moves to
Phase 7.5
([`docs/phase7_5_variable_target_stress.md`](phase7_5_variable_target_stress.md)).
No further Phase 7.4 remediation is planned.  
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
| `max_reach_rejections` | `target_count × episode_count` | Suite-wide budget of dexterous-reach rejections during target generation. Exceeding it fails suite generation closed. Dense wide-band stress suites may raise this explicitly. |
| `require_tip_ik` | `false` | When true, host planning supplies a tip-IK pre-screen at placement. Densest 2×20 stress suite sets `true`. Empty-world today; **world-aware** (accepted cubes as obstacles) after the pending goal-feasibility amendment. |
| `max_ik_rejections` | `target_count` (per episode) | Tip-IK reject-and-regenerate budget during placement. Densest 2×20 stress suite overrides to `20000` (match reach-budget pressure). |
| `max_field_regenerations` | `3` | Suite-wide regenerations of an episode field after consecutive-unplanned abort or `targets_unplanned`. |
| `order: z_desc` | *(pending)* | New `OrderPolicy` value from the goal-feasibility amendment: contact order by descending top-face Z, id tiebreak; `order_seed` recorded but unused. |
| `roll_candidates_deg` | existing key | Goal-set rolls threaded to the `plan_grasp` goal bank; the amendment requires wide-band (`delta_z_m`) suites to set 8 evenly spaced rolls instead of `fixed_roll_rad: 0.0`. |
| `dexterous_reach.*` | URDF-calibrated defaults | Declared wrist-sphere parameters (`shoulder_height_m`, `L_wrist_to_tcp_m`, `R_wrist_max_m`, `reach_margin_m`). |

Pairwise floor:

```text
edge + flange + ee_approach_clearance
  + z_separation_gain * min(Δz_top_faces, pre_approach_distance_m)
```

(`min_center_separation_m` may raise the constant part only.)

### Dexterous-reach screening: reject-and-regenerate (amended 2026-08-01)

*Supersedes the original substitute-retry rules. See `spec.md` §8 Phase 7.4
for the normative text.*

The original screen (Z within `arm_z_motion_range_m` centered on field
mid-Z; optional `max_target_radial_m` XY rim) is axis-separable and accepts
centres whose height and radial extent are individually in range but jointly
unreachable for a flange-normal descent. Evidence: the 2×20
`delta_z_m: 0.40` suite samples z ∈ [0.08, 0.48] m while the measured
tip-contact map (`artifacts/workspace/tip_contact_workspace_v1.json`)
records 93% planning success at z = 0.10 m, 58% at z = 0.22 m, and no
measurements above 0.22 m.

Implemented semantics:

- **Wrist-sphere reach model.** A centre is inside dexterous reach iff the
  required wrist point `W = p_face + L_wrist_to_tcp_m · n̂` lies within
  `R_wrist_max_m − reach_margin_m` of the J2 shoulder centre
  `S = (0, 0, shoulder_height_m)`. Defaults from
  `assets/mycobot_280_m5/urdf/mycobot_280_m5_kinematics.urdf`:
  `shoulder_height_m = 0.13156`; `L_wrist_to_tcp_m = 0.11878`
  (= 0.07318 + 0.0456); `R_wrist_max_m = 0.36` (≥ farthest measured-success
  ≈ 0.3116 m; raised so densest Phase 7.2 grid/--targets packs remain
  feasible under the Z-aware EE floor).
- **Reject and regenerate.** Random Z is drawn from the **full** requested
  band. An out-of-reach centre is rejected (one target-placement error) and
  generation keeps drawing until the field holds the full `target_count`
  targets; fields are never delivered short.
- **Suite-wide budget.** Rejections accumulate across all episode fields of
  one suite invocation; exceeding `max_reach_rejections` (default
  `target_count × episode_count`) fails generation closed, naming the
  episode, centre, and running count. Rejected centres and generation
  duration are recorded in the plan bundle (`placement_generation`).
- Console streaming: `phase7_4_placement:` accept/reject lines plus
  `suite target generation completed duration_s=…` (also via
  `scripts/host/generate_phase7_4_placement.sh` for a placement-only stream).
- Manual lists still fail closed on reach violations without regeneration;
  separation / keep-out / rim resampling is unchanged and does not count
  against the budget.
- **Tip-IK pre-screen (optional).** Host `plan_multi_target_suite` with
  `require_tip_ik: true` rejects centres that lack empty-world tip IK
  (`CuroboTipIkScreen`); per-episode `max_ik_rejections` defaults to
  `target_count`. CPU placement-only generation skips the screen.
- **Field regeneration after consecutive-unplanned.** When
  `max_consecutive_unplanned_targets` is a positive limit and planning hits
  it, the runner may rebuild that episode's field (fresh seeds) up to
  `max_field_regenerations` before aborting the suite. Default **0** means
  no maximum (no consecutive abort / regen trigger from this path).
- **Densest stress planner.** The `delta_z_m=0.30` 3×15 / 2×20 suites use
  `planner_profile: planning_high_effort` (solution 3) after tip-IK
  pre-screen + field regen still left multi-obstacle legs failing under
  `benchmark_reproducible`. The 3×15 suite sets `max_failed_episodes: 1`.
- Upward inter-target motion remains an emergent property of cuRobo
  optimization; no lift waypoints are injected.

### Goal-feasibility amendment (2026-08-01 — specified, implementation pending)

*Normative text: `spec.md` §8 Phase 7.4, "Goal-feasibility amendment".*

**Root cause.** The 3×15 `delta_z_m: 0.30` headless smoke (root seed
72152210, `/tmp/smoke_3x15_headless.log`) failed 0/3 with
`targets_unplanned`, but the episodes failed for different reasons:

- Episode 1 (9/15 planned): six failures with **no** collision-state
  messages, concentrated at z ≥ 0.22 m or radius ≥ 0.26 m — genuine
  optimization difficulty near the reach shell (the measured workspace
  artifact records 58% success at z = 0.22 m and nothing above, while the
  wrist-sphere screen admits samples up to z ≈ 0.35 m).
- Episodes 2 and 3 (12/15 and 0/15): every failed attempt logged cuRobo
  **"Start or End state in collision"** (48 and 588 lines). Episode 2
  isolates the failing end: from the identical home start, target 12
  planned while targets 15/13/1 failed in-collision — the **goal state**
  collided with neighboring cubes. Zero-configuration FK clears the nearest
  generated cube by ≈ 63 mm, ruling out the start. A goal-in-collision leg
  cannot be solved by any planner; the empty-world tip-IK screen could not
  catch it because the colliding bodies are the *other* cubes.

**Changes (all four are placement/goal-selection level; cuRobo remains the
exclusive planner):**

1. **World-aware tip-IK screen.** `CuroboTipIkScreen` solves the
   flange-normal tip IK against a caller-supplied obstacle set.
   Draw-order packing screens against already-accepted cubes; after a full
   pack, placement revalidates each centre with **omit-self** (all other
   cubes), matching the planner's omit-active per-leg world, and redraws
   the field under `max_ik_rejections` on failure. `TipIkScreen` signature:
   `__call__(center_m, accepted_centers_m) -> bool`.
2. **`order: z_desc`.** Tallest-first contact order (top-face Z descending,
   ascending numeric-id tiebreak; non-numeric ids fail closed with
   `ConfigurationError`). With `retain_targets_after_contact: false`,
   removing tall cubes first monotonically un-shadows lower descent
   corridors — proactively, instead of the reactive deferral/reconsider
   path.
3. **Goal-set rolls.** Wide-band suites replace `fixed_roll_rad: 0.0` with
   `roll_candidates_deg: [0, 45, 90, 135, 180, 225, 270, 315]` (existing
   key, existing Mode-D goal-bank plumbing). cuRobo then selects among 8
   goal postures per target rather than retrying one fixed goal that may be
   in collision.
4. **Regeneration on `targets_unplanned`.** The suite runner's field-regen
   path currently triggers only on
   `max_consecutive_unplanned_targets_exceeded`, which is unreachable with
   the default limit 0 (tracking off) — the 3×15 report shows
   `field_regenerations: 0` despite a 0/15 episode. `targets_unplanned`
   becomes a second trigger under the same suite-wide
   `max_field_regenerations` budget (default 3).
   `max_reconsider_passes_exceeded` stays non-triggering.

**Config updates (landed):**
`config/phase7_4_multi_target_standard_2x15_delta_z_0_30.yml`,
`…_3x15_delta_z_0_30.yml`, `…_2x20_delta_z_0_30.yml` use
`order: z_desc` and `roll_candidates_deg` (8 rolls).

**Considered and deferred** (not part of this amendment):

- Raising the Z-aware floor clamp — the additive term
  `gain × min(Δz, pre_approach_distance_m)` caps at 1 cm for
  `pre_approach_distance_m: 0.01`, so a 30 cm taller neighbor earns only
  1 cm of extra XY separation. A descent-corridor check or a wrist-scale
  clamp remains a candidate follow-up if goal-in-collision rejections stay
  high after the world-aware screen.
- Recalibrating `R_wrist_max_m` (0.36) against the measured success surface
  (episode-1-style high-Z failures). High-Z samples remain intentional
  stress content for now.

**Planned tests** (`tests/unit/test_phase7_4_z_variability.py` /
`tests/unit/test_multi_target.py`): world-aware screen rejects only with
the accepted neighbor present; `z_desc` order + tiebreak + round-trip;
stress YAMLs load with roll goal sets; regen on `targets_unplanned` within
budget, fail-closed when exhausted.

### PhysX contact policy (playback)

Unchanged from Phase 7.2 and required for Phase 7.4 GUI/playback smokes:

- **Allowed tip contact** (`joint6_flange` vs active target) is success
  evidence for a contacted target.
- **Any PhysX-detected prohibited body–target collision** (non-tip link vs
  any target) **fails the episode immediately** (`body_contact`) and, with
  the default `max_failed_episodes: 0`, fails suite acceptance. Tip contact
  on the same leg does not waive the body-contact failure.

Original (superseded) behaviour, kept for history: out-of-reach generated
centres were discarded and substituted (same XY, Z in band ∩ reach) with ≤3
substitutes per target before failing closed.

`delta_z_m` / `z_band_fraction` are intentionally unclamped: oversized bands
are accepted at config load, including bands exceeding the dexterous space;
unreachable samples are governed by the screening and budget above.

### Testing defaults

- Default Z variability used in testing is **~50%** of
  `arm_z_motion_range_m` (`z_band_fraction: 0.5`).
- Example widened/explicit-delta suite:
  [`config/phase7_4_multi_target_widened_z.yml`](../config/phase7_4_multi_target_widened_z.yml)
  (`delta_z_m: 0.14`).
- Densest GUI smoke remains standard 2×20
  (`scripts/host/smoke_phase7_2_standard_2x20.sh`).
- Regenerated 2×20 with `delta_z_m: 0.30` (current stress suite):
  `config/phase7_4_multi_target_standard_2x20_delta_z_0_30.yml` /
  `scripts/host/smoke_phase7_4_standard_2x20_delta_z_0_30.sh`
  (`arm_z_motion_range_m: 0.28`, `max_reach_rejections: 20000`, rim 0.28 m).
- Prior regenerated 2×20 with `delta_z_m: 0.40` (original envelope screen
  evidence): `config/phase7_4_multi_target_standard_2x20_delta_z.yml` /
  `scripts/host/smoke_phase7_4_standard_2x20_delta_z.sh` — host GUI
  2026-08-01 showed pervasive `plan_failed` legs under the pre-amendment
  screen.

## Evidence / gates

- Unit: `tests/unit/test_phase7_4_z_variability.py` — band defaults, unclamped
  `delta_z_m`, Z-aware floor, gain&lt;1 fail-closed, widened config sample,
  densest 2×20 pack under Z-aware floor, full-count regenerated fields with
  all centres in-reach, suite-wide rejection budget fail-closed, seed-
  reproducible rejection sequences, manual-list fail-closed, and the
  workspace-artifact calibration cross-check.
- Field AABBs for default grid / `--targets 10` fallback widened so nearest
  XY spacing clears `0.076 + pre_approach` when Δz &gt; 0.
