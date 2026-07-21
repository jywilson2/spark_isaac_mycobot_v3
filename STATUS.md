# STATUS — MyCobot 280 M5 Constrained Approach Planner

Last updated: **2026-07-20**

## Current phase

**Phase 7.2 — Multi-target tip-contact clearance suite: COMPLETE**

**Phase 7.3 — Controllable target-block placement: UNDER CONSIDERATION**
(brainstorm / specification on `wip_phase7_3`; also scoped to fix GitHub
Actions CI execution). See
[`docs/phase7_3_target_placement.md`](docs/phase7_3_target_placement.md).

**Phase 1.1 — Target-scale collision-sphere coverage: IMPLEMENTED**
Sparse mesh-constrained spheres (128 total for `E = 0.014 m`) via overlay
`config/robots/mycobot_280_m5_phase1_1_spheres.yml`. See [`spec.md`](spec.md)
§8 Phase 1.1 and
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
| 1.1 | Target-scale collision-sphere coverage | **Complete** |
| 2 | Task frames / roll goals | **Complete** |
| 3 | `plan_grasp` nominal planning | **Complete** |
| 4 | Independent validation | **Complete** |
| 5 | Execution + zero residual seam | **Complete** |
| 6 | Randomized benchmark | **Complete** |
| 7 | Isaac Sim closed-loop viz/validation | **Complete** |
| 7.1 | Unknown-start cube approach visualization | **Complete** |
| 7.2 | Multi-target tip-contact clearance suite | **Complete** |
| 7.3 | Controllable target-block placement (+ CI fixes) | **Under consideration** |
| 8 | Bounded residual RL (sim only) | Planned |
| 9 | Fabricated contact test tool | Requirements finalized |
| 9.1 | Contact test tool evaluation | Requirements finalized |
| 10 | Hardware interface + dry-run | Planned |
| 11 | Physical MyCobot 280 M5 validation | Planned |

## Implemented

- Phase 1.1 target-scale collision spheres: DAE mesh cover overlay (128 spheres
  for `E = 0.014 m`); suite rejects `target_edge_m < E`.
- Phase 7.3 (partial, on `wip_phase7_3`): GitHub Actions CI interpreter/deps
  fix; viewport-visible multi-target ID labels (red 7-segment geometry with
  parent-local Z offset); yellow/green/red contact-state cube highlights; tip
  collision left enabled vs non-contact targets; grid mid-Z variability over
  `0.5 * arm_z_motion_range_m`. Broader placement APIs remain under
  consideration.
- Phase 7.2 multi-target tip-contact suite: `TargetField`,
  `MultiTargetEpisodeRunner`, three-tier failure budgets
  (`max_planning_failure_per_target` default 5, `max_target_failures` default
  3, `max_failed_episodes` default 0), tip contact required only for
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
- [x] Per-target planning retries; target/episode/suite budgets as specified.
- [x] Tip contact not required for planning-failed targets; tip miss after a
      successful plan aborts the episode.
- [x] Dual console/JSON timing; host plan/play split; smoke gates wired.
- [x] Container CI (unit tests + Ruff) green for the landed change set.
- [x] Host GUI evidence: seed 123, `--targets 10 --episodes 1`, suite accepted
      `1/1`, tip contacts on non-failed targets, zero body contacts, replay under
      `--no-auto-exit`.
- [x] No physical command, alternate planner, or physical-accuracy claim.

## Next step / resume after delay (2026-07-20)

**Open investigation (do this next):**

1. **Phase 1.1 collision spheres may not be effective in Isaac GUI smoke.**
   Near-blocker / body-contact behavior still looked wrong; confirm the
   planning process actually loads the overlay
   (`config/robots/mycobot_280_m5_phase1_1_spheres.yml`, 128 spheres,
   `min_detectable_obstacle_edge_m: 0.014`) via `load_curobo_robot_config`.
2. **Planning console messages were missing or atypical** in the last GUI
   run — verify the host plan step ran (`plan_multi_target_suite` / smoke
   script) and that tip/body / plan_failed lines still print.
3. **Existing sphere / clearance tests to consult first:**
   - Unit: `tests/unit/test_collision_sphere_cover.py`,
     `tests/unit/test_robot_model.py` (overlay count / `E`),
     `tests/unit/test_inspect_robot_model.py`
   - GPU / headless-style: `tests/integration/test_phase7_1_cube_suite_gpu.py`
     (`test_phase7_1_cube_scene_plan_validates_with_evaluated_world_clearance`)
   - GPU multi-target: `tests/integration/test_phase7_2_multi_target_gpu.py`
   There is **no** dedicated headless Isaac test yet that asserts Phase 1.1
   spheres reject a body-clipping trajectory against a target-sized cuboid;
   add one if GPU gates still pass while GUI body-contacts persist.

Continue Phase 7.3 placement brainstorm on `wip_phase7_3` only after the
sphere / planning-message investigation. Preserve Phase 7 / 7.1 / 7.2 smoke
gates for Isaac-path changes.

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
