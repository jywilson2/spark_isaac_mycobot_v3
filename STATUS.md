# STATUS — MyCobot 280 M5 Constrained Approach Planner

Last updated: **2026-08-01**

## Current phase

**Phase 7.2 — Multi-target tip-contact clearance suite: COMPLETE**
Including deferral / reconsider after tip-removals, playback in plan-creation
order, and episode FAIL if any target remains unplanned
(`targets_unplanned`). Approach-plane EE clearance floor
`edge + flange + ee_approach_clearance_m` (default clearance = flange),
optional `max_target_radial_m` rim guard, right-reading viewport digit labels
(`AddRotateZOp(180)`), dimmer default suite lighting (dome 400 /
distant 1000), widened forward-biased integration 2×5 AABB + base keep-out,
content-aware GUI framing (`compute_viewport_framing`), TrajOpt collision
activation `0.01` m on benchmark/high-effort, tip-classify hardening,
measured +Z tip-contact workspace artifact, `planning_high_effort` IK seeds
**32**, and integration 2×5 **flange-face containment** with flange-sized cubes on a
multi-quadrant open arc, plus flange-rim anti-graze (Ø31 mm flange spheres +
neighbor clearance). Host plan/smoke use `--root-seed N` for reproducible
layouts; omitting it draws an independent random seed per episode. Default
YAML field AABB packs `--targets 10` grid fallback. Integration stays on
`benchmark_reproducible`. See `spec.md` §8 Phase 7.2.

**Phase 7.3 — Controllable target-block placement: IMPLEMENTED**
`random` / `layout` (`rows`, `arc`) placement with keep-outs and approach-plane
EE-clearance separation floor; CI bootstrap; labels / grid Z. See
[`docs/phase7_3_target_placement.md`](docs/phase7_3_target_placement.md).

**Phase 7.4 — Extended Z variability & Z-aware EE-clearance spacing:
COMPLETE (`wip_phase7_4`)**
Default testing Z band ≈ 50% of `arm_z_motion_range_m`
(`z_band_fraction: 0.5`); optional unclamped `delta_z_m`; Z-aware EE floor
(`z_separation_gain` default 1.0); dexterous-reach wrist-sphere screening
with suite-wide reject-and-regenerate (`max_reach_rejections`, default
`target_count × episode_count`); bands may exceed the dexterous space by
design. PhysX prohibited body–target contact fails the episode/suite
(Phase 7.2 policy, documented for 7.4). See [`spec.md`](spec.md) §8 Phase
7.4 and [`docs/phase7_4_z_variability.md`](docs/phase7_4_z_variability.md).

**Phase 1.1 — Target-scale collision-sphere coverage: COMPLETE / OPTION B
ARMED (`wip_phase1_1b`)**
Default robot YAML loads dual overlay (32 scaffolding + 1012 world-cover).
Acceptance gate passed 2026-08-01 (GPU self-clear/body-clip, 7.1/7.2,
integration 2×5 headless+GUI, host plan-time budget, unseeded 2×20 10/10).
Orin AGX timing not measured. See [`spec.md`](spec.md) §8 Phase 1.1 and
[`docs/phase1_1_target_scale_collision_spheres.md`](docs/phase1_1_target_scale_collision_spheres.md).

Roadmap: [`docs/implementation_phases.md`](docs/implementation_phases.md)  
Authoritative criteria: [`spec.md`](spec.md) §8 (Phases 0–11)  
Phase 7.2 design: [`docs/phase7_2_multi_target_contact.md`](docs/phase7_2_multi_target_contact.md)

This status is initialized for v3. No v2 completion metrics, GUI results,
planning-success claims, or hardware-readiness claims carry forward.

## Phase roadmap (summary)

| Phase | Focus | Status |
|-------|-------|--------|
| 0 | Env / version guard | **Complete** |
| 1 | Robot model + spheres | **Complete** |
| 1.1 | Target-scale collision-sphere coverage | **Complete (Option B armed)** |
| 2 | Task frames / roll goals | **Complete** |
| 3 | `plan_grasp` nominal planning | **Complete** |
| 4 | Independent validation | **Complete** |
| 5 | Execution + zero residual seam | **Complete** |
| 6 | Randomized benchmark | **Complete** |
| 7 | Isaac Sim closed-loop viz/validation | **Complete** |
| 7.1 | Unknown-start cube approach visualization | **Complete** |
| 7.2 | Multi-target tip-contact clearance suite | **Complete** |
| 7.3 | Controllable target-block placement (+ CI fixes) | **Complete** |
| 7.4 | Extended Z variability + Z-aware EE-clearance spacing | **Complete** |
| 8 | Bounded residual RL (sim only) | Planned |
| 9 | Fabricated contact test tool | Requirements finalized |
| 9.1 | Contact test tool evaluation | Requirements finalized |
| 10 | Hardware interface + dry-run | Planned |
| 11 | Physical MyCobot 280 M5 validation | Planned |

## Implemented

- Phase 1.1 **complete (Option B):** dual-role overlay armed by default —
  scaffolding (32) self-collision + Option A cover (1012) on `*_world_cover`
  for world checks; shared omit-active `planning_world`; suite rejects
  `target_edge_m < E`; adapter strips project-only keys.
- Phase 7.4: `z_band_fraction` / unclamped `delta_z_m`, Z-aware EE floor,
  arm-reach discard + ≤3 substitutes; example
  `config/phase7_4_multi_target_widened_z.yml`.
- Phase 7.3: `placement: random` / `layout` (`rows`, `arc`) with
  `min_center_separation_m`, `keep_outs`, episode-diverse seeds; example
  configs `config/phase7_3_*.yml`; module `mycobot_curobo.target_placement`.
  Also: GitHub Actions CI bootstrap; viewport ID labels; contact highlights;
  tip collision vs non-contact targets; grid mid-Z variability.
- Phase 7.2 multi-target tip-contact suite: `TargetField`,
  `MultiTargetEpisodeRunner`, failure budgets
  (`max_planning_failure_per_target` default 3,
  `max_consecutive_unplanned_targets` default 3 — aborts suite planning,
  `max_failed_episodes` default 0), tip contact required only for
  successfully planned targets, plan/play split, host smoke with
  `--targets` / `--episodes`, and `--no-auto-exit` continuous episode replay.
- Python 3.10+ `src/` project layout.
- Exact cuRobo v0.8.0 Git-tag dependency declaration.
- Phase 0–7.1 complete (see prior STATUS history and phase reports).
- Phase 7.1 cube suite: validated config, FK-aligned Mode D goal bank,
  cube-world clearance, Mode C `plan_cspace` relocation, illuminated Isaac
  plan/playback process split, drive-target motion, PhysX prohibited-contact
  evidence, and null/`not_evaluated` tip metrics.

## Acceptance checklist (Phase 7.2)

- [x] Parameterized `target_count` / `episode_count`; grid and manual;
      shuffle and listed; retain and remove-after-contact; seeded replay.
- [x] Flange-normal tip/EE contact; body–target contact fails closed.
- [x] Per-target planning retries; target/episode/suite budgets as landed.
- [x] Landed: tip contact not required for planning-failed targets; tip miss
      after a successful plan aborts the episode.
- [x] Spec revision landed: deferral + reconsider; planning world =
      remaining after tip-removals; playback = plan-creation order; FAIL if
      any target remains unplanned (exercised live by 2×20 seed-4242 GUI run:
      `start→1` deferred, replanned via reconsider as final leg `7→1`).
- [x] Dual console/JSON timing; host plan/play split; smoke gates wired.
- [x] Container CI (unit tests + Ruff) green for the landed change set.
- [x] Host GUI evidence: seed 123, `--targets 10 --episodes 1`, suite accepted
      `1/1`, tip contacts on non-failed targets, zero body contacts, replay under
      `--no-auto-exit`.
- [x] No physical command, alternate planner, or physical-accuracy claim.

## Next step / resume (2026-08-01)

**Where we left off:** Phase 7.4 dexterous-reach screening is implemented on
`wip_phase7_4` (wrist-sphere model, suite-wide `max_reach_rejections`,
placement timing/stream logs). New stress suite
`config/phase7_4_multi_target_standard_2x20_delta_z_0_30.yml`
(`delta_z_m: 0.30`).

**Next steps:**

1. Host GUI loop for the `delta_z_m: 0.30` 2×20 suite after CI gates.
2. Phase 8 (bounded residual RL, sim only) on a new `wip_phase8` branch.
   Entry criteria verified 2026-07-31. **Note:** residual policies trained
   under scaffolding-only spheres may need retraining under the denser
   world-cover set now armed by default.


## 2026-07-20 compliance note

Phase 7.1 sources were re-audited against newly added Cursor `python` /
`bash` / `clean-code` rules; chemistry/PyTorch and C++ packs were treated as
non-applicable. Cleanup committed on `wip_phase7_1` after CI/GPU/GUI retest.

## 2026-07-20 GUI / lighting fix

Kit auto light-rig is disabled before `open_stage` so opening the dark robot
USD no longer posts **No lights found… applying 'Default'** or hides UsdLux
prims. Players force stage lighting via `SetLightingMenuModeCommand` (and
again after `World.reset` / GUI settle). Use `--gui --no-auto-exit` on the
host desktop to confirm the Lighting menu shows Stage with no warning toast.
