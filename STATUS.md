# STATUS — MyCobot 280 M5 Constrained Approach Planner

Last updated: **2026-07-23**

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

**Phase 1.1 — Target-scale collision-sphere coverage: OPTION A (DISARMED)**
Thickness-capped overlay (1012 spheres, radii ≤ `E`) regenerated; GPU
self-clear + body-clip detectability pass under trial enable, but arming
regresses Phase 7.1 / 7.2 GPU planning. Default robot uses scaffolding (32).
See [`spec.md`](spec.md) §8 Phase 1.1.

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
| 1.1 | Target-scale collision-sphere coverage | **Option A (disarmed)** |
| 2 | Task frames / roll goals | **Complete** |
| 3 | `plan_grasp` nominal planning | **Complete** |
| 4 | Independent validation | **Complete** |
| 5 | Execution + zero residual seam | **Complete** |
| 6 | Randomized benchmark | **Complete** |
| 7 | Isaac Sim closed-loop viz/validation | **Complete** |
| 7.1 | Unknown-start cube approach visualization | **Complete** |
| 7.2 | Multi-target tip-contact clearance suite | **Complete** |
| 7.3 | Controllable target-block placement (+ CI fixes) | **Complete** |
| 8 | Bounded residual RL (sim only) | Planned |
| 9 | Fabricated contact test tool | Requirements finalized |
| 9.1 | Contact test tool evaluation | Requirements finalized |
| 10 | Hardware interface + dry-run | Planned |
| 11 | Physical MyCobot 280 M5 validation | Planned |

## Implemented

- Phase 1.1 (partial): regenerator + overlay candidate (128 / `E=0.014 m`);
  suite rejects `target_edge_m < E`; adapter strips project-only keys. Overlay
  **not** loaded by default (self-collision infeasible).
- Phase 7.3: `placement: random` / `layout` (`rows`, `arc`) with
  `min_center_separation_m`, `keep_outs`, episode-diverse seeds; example
  configs `config/phase7_3_*.yml`; module `mycobot_curobo.target_placement`.
  Also: GitHub Actions CI bootstrap; viewport ID labels; contact highlights;
  tip collision vs non-contact targets; grid mid-Z variability.
- Phase 7.2 multi-target tip-contact suite: `TargetField`,
  `MultiTargetEpisodeRunner`, three-tier failure budgets
  (`max_planning_failure_per_target` default 3, `max_target_failures` default
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

## Next step / resume (2026-07-23)

**Where we left off:** the named-suite family is complete and documented —
standard 2×10 (open arc, flange-sized cubes) and densest 2×20 (two-ring
manual field, 14 mm cubes; GUI evidence seed 4242: exit 0, 2/2 episodes,
40/40 tip contacts, 0 body contacts, deferral/reconsider exercised live).
Phase 7.2 report gained decision guidance: dedicated suite vs
`--targets`/`--episodes`, base-suite selection for reduced counts, the
wrapper-gate table, and `manual` placement terminology. README carries a
timestamped project-size + AI-context snapshot. `wip_phase7_3` committed,
rebased onto `origin/main`, and pushed.

**Next steps:**

1. Review `wip_phase7_3` and fast-forward `main` to the tested commit per
   branch policy (no merge commits into phase branches).
2. Build unseeded 2×20 failure-rate evidence (repeat runs without
   `--root-seed`); nudge ring radii (inner 0.15 / outer 0.23) only if
   deferrals accumulate on close-in or far legs.
3. Phase 1.1 cover-option decision (below) remains the gate for denser
   collision spheres; integration 2×5 must stay green when re-arming.
4. Then Phase 8 (bounded residual RL, sim only) per the roadmap.

**Phase 1.1 — awaiting approval of revised cover approach** (see `spec.md`
§8 Phase 1.1 “Proposed revision”). Headless findings:

1. **Fixed:** adapter stripped project-only keys so cuRobo can construct a
   planner when an overlay is enabled.
2. **Blocked:** greedy 128-sphere cover self-collides at every tested posture
   (including zero); scaffolding (32) is self-clear. Overlay path commented out
   in `mycobot_280_m5.yml`.
3. **Proposed options in spec:** A thickness-capped cover (recommended), B dual
   self/world sets, C distal-only densify, D scene-side keep-outs.

Do **not** re-enable the overlay or iterate cover radii until an option is
chosen. Phase 7.3 placement APIs are available with scaffolding spheres.

**Integration smoke (opt-in final gate):** `smoke_phase7_2_integration_2x5.sh`
— 2 episodes × 5 targets. Enable with
`./scripts/run_verification.sh spark --with-integration-smoke`.
Playback tip-face evidence uses terminal joint snap + FK/USD proximity
(15 mm) so short headless holds do not drop tip contact after PhysX push-out.

**Standard denser suite:** `smoke_phase7_2_standard_2x10.sh` — 2 episodes ×
10 targets (dedicated open-arc YAML). Not part of the default spark gate.

**Densest suite:** `smoke_phase7_2_standard_2x20.sh` — 2 episodes × 20
targets (two-ring manual field, 14 mm cubes). Not part of the default spark
gate.

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
