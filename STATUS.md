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

**Phase 1.1 — Target-scale collision-sphere coverage: OPTION A (DISARMED);
OPTION B PROPOSED (2026-08-01)**
Thickness-capped overlay (1012 spheres, radii ≤ `E`) regenerated; GPU
self-clear + body-clip detectability pass under trial enable, but arming
regresses Phase 7.1 / 7.2 GPU planning. Default robot uses scaffolding (32).
A dual-role split (scaffolding = self-collision only, Option A cover =
world-only) is drafted in spec §8 Phase 1.1 "Proposed revision: Option B";
development branch `wip_phase1_1b`. See [`spec.md`](spec.md) §8 Phase 1.1.

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
| 1.1 | Target-scale collision-sphere coverage | **Option A (disarmed); Option B proposed** |
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

## Next step / resume (2026-08-01)

**Where we left off:** Phase 7.3 is complete and landed; `main` is
fast-forwarded to the `wip_phase7_3` tip. The named-suite family (standard
2×10, densest 2×20) is documented with a demo video (README inline player
via `user-attachments`; repo copies under `docs/videos/`), and smoke
wrappers accept `--record FILE.mp4` for GUI captures.
**Unseeded 2×20 robustness evidence is recorded (2026-07-31, 10 headless
runs):** 9/10 pass (the one failure was an NVIDIA Vulkan driver segfault at
playback startup, not a planning failure); zero target failures / failed
episodes in all reported runs; 45 planning retries + 6 deferrals all
absorbed by budgets/reconsider, clustering on inner-ring destinations as
expected. **Verdict: no ring-radius nudge.** Details in the phase 7.2
report ("Standard smoke 2×20") and
`artifacts/reports/phase7_2_unseeded_2x20/` (local).

**Next steps:**

1. Phase 8 (bounded residual RL, sim only) on a new `wip_phase8` branch.
   Entry criteria verified 2026-07-31: Phase 7.2 accepted, Phase 5 seam
   stable, Phase 6 baselines recorded, remote CI green, Isaac Lab 0.54.4
   present on host. First milestone: training-env contract + zero-residual
   pass-through reproducing Phase 6 baseline metrics.
2. Phase 1.1 cover-option decision (below) remains the parallel gate for
   denser collision spheres; integration 2×5 must stay green when
   re-arming. An **Option B (dual-role split) revision is drafted**
   (2026-08-01, spec §8 Phase 1.1) on `wip_phase1_1b`; its first task is
   diagnosing the exact mechanism of the armed-Option-A regression. Note:
   a residual policy trained under scaffolding spheres may need retraining
   if Phase 1.1 later re-arms a denser set.
3. Watch the host NVIDIA driver flake (580.173.02 Vulkan segfault at Kit
   startup, ~1-in-10 headless playback launches on 2026-07-31); if it
   recurs, investigate driver/Kit versions rather than suite code.

**Phase 1.1 — Option A chosen and implemented; overlay disarmed** (see
`spec.md` §8 Phase 1.1 “Chosen revision: Option A”). History and state:

1. **Fixed:** adapter stripped project-only keys so cuRobo can construct a
   planner when an overlay is enabled.
2. **First cover rejected:** greedy 128-sphere cover self-collides at every
   tested posture (including zero); scaffolding (32) is self-clear. Overlay
   path commented out in `mycobot_280_m5.yml`.
3. **Option A landed (B dual self/world sets, C distal-only densify, D
   scene-side keep-outs: not selected):** thickness-capped cover regenerated
   (1012 spheres, radii ≤ `E`); GPU self-clear and body-clip detectability
   pass under trial enable, but arming the default YAML regresses
   Phase 7.1 / 7.2 GPU planning (cuRobo reports start/end state in
   collision against target cubes).

Do **not** re-arm the overlay until the spec's Option A acceptance gate
passes (non-negative self-clearance at the gate postures, body-clip
detectability retained, and Phase 7.1 / 7.2 GPU planning suites green).
The gate also includes a planning-time criterion (spec §8 Phase 1.1, added
2026-08-01): measured overlay-vs-scaffolding plan p50/p95 on the host, a
device calibration run if an embedded planner target (e.g. Jetson Orin AGX)
is in scope, and review against a budget declared for the deployment
target. Phase 7.3 placement APIs are available with scaffolding spheres.

**Proposed revision (2026-08-01): Option B — dual-role sphere split**, spec
§8 Phase 1.1. Scaffolding (32) stays the only self-collision participant;
the Option A cover (1012) is reused as world-only, so self-collision cost
and semantics are unchanged and the planning-time question reduces to the
linear world term. Preconditions and gates: diagnose the armed-Option-A
regression first (mechanism is not established — Phase 7.2 already omits
the active cube), verify cuRobo v0.8.0 ignore-map granularity from source,
centralize world construction in one shared helper (active-target and
just-contacted exclusions; no contact carve-outs), add a config-time
dense-sphere-vs-neighbor-face check per named suite, then pass the Option A
gate + timing criterion + an armed unseeded 2×20 batch within existing
budgets. Development on `wip_phase1_1b`; not yet accepted or implemented.

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
