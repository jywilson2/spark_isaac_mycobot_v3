# CHANGES — MyCobot 280 M5 Constrained Approach Planner

## 2026-07-23 — `--record FILE.mp4` GUI video capture on Phase 7.2 smokes

1. `scripts/host/smoke_phase7_2_multi_target.sh`: new `--record FILE.mp4`
   option (GUI only; named wrappers forward it). Waits for the Isaac Sim Kit
   window on `$DISPLAY` via `xwininfo` (skipping the mutter decoration
   frame), then records exactly that region with `ffmpeg` x11grab at 30 fps;
   SIGINT on playback exit finalizes the mp4 (H.264 yuv420p, `+faststart`).
2. ffmpeg resolution: system binary when present, else the static build
   bundled with Isaac Sim's python (`imageio_ffmpeg` wheel) — no host
   install and no sudo required.
3. Fail-closed: `--record` with `--headless` exits 2; missing `xwininfo` or
   ffmpeg exits 2 with a clear error. A Kit window that never appears only
   prints a warning; recording never changes the suite exit status and is
   not a verification gate.
4. Recorder hardening: the background capture subshell relaxes inherited
   `set -e` / `pipefail` (a `head`-induced SIGPIPE in the window poll
   silently killed the first implementation).
5. Docs: phase 7.2 report Host CLI overrides, README Phase 7.2 section,
   REFERENCES host-CLI notes, spec §9 host entry points.
6. Wiring assertions added to `tests/unit/test_isaac_viz_smoke.py`.

### Verification

- `pytest tests/unit` — 202 passed; Ruff check/format clean.
- Host GUI: `smoke_phase7_2_multi_target.sh --gui --auto-exit --root-seed 123
  --record /tmp/phase7_2_record_demo.mp4` — exit 0, 1/1 episodes, 2/2 tip
  contacts; wrote a 20.6 s 2880×1800 30 fps H.264 mp4 (1.5 MB) whose frames
  show the viewport playback.

---

## 2026-07-23 — Densest 2×20 multi-target suite (two-ring manual field)

1. Added `config/phase7_2_multi_target_standard_2x20.yml` and
   `scripts/host/smoke_phase7_2_standard_2x20.sh` (2 episodes × 20 targets).
2. Two-ring manual field: outer r=0.23 m (11 targets, 0.45 rad step), inner
   r=0.15 m (9 targets, 0.5625 rad step), z=0.16; 0.08 m radial ring gap ≥
   the 0.076 m approach-plane EE floor for any angle pairing.
3. 14 mm cube regime (flange-sized cubes cannot pack 20 reachable centres at
   the 0.093 m floor); `require_flange_face_containment` off. Keep-out
   shrunk to ±0.10 m so the inner ring clears it. Anti-graze world clearance
   0.006 m retained.
4. Manual placement: field identical across episodes; shuffle order and
   planner seeds vary per episode.
5. Unit tests: two-ring packing above the EE floor (`test_target_placement`),
   episode sampling shares placement but varies order/seeds
   (`test_multi_target`), wrapper wiring (`test_isaac_viz_smoke`).
6. Ruff format applied to `isaac_sim/plan_multi_target_suite.py` (pre-existing
   drift) and `tests/unit/test_multi_target.py`.

### Review recommended

- Inner ring r=0.15 close-in tip contacts and outer ring r=0.23 reach: watch
  GUI planning failures/deferrals; radii may need a nudge after evidence.

### Verification

- `pytest tests/unit` — 202 passed; Ruff check/format clean.
- Host GUI: `smoke_phase7_2_standard_2x20.sh --gui --auto-exit --root-seed 4242`
  — exit 0, 2/2 episodes, 40/40 tip contacts, 0 body contacts, 3 planning
  failures (ep1 `start→1` validation ×3 → deferred, replanned via reconsider
  as final leg `7→1`), plan p50/p95 = 4.44 s / 7.25 s.

---

## 2026-07-23 — Document dedicated-suite vs CLI-override decision

1. `docs/phase7_2_multi_target_contact.md`: new subsection "When to create a
   dedicated suite vs `--targets` / `--episodes`" — overrides change counts
   only; placement fails closed when N cannot pack the unchanged field;
   manual YAMLs silently fall back to grid when the list is shorter than N.
   Dedicated YAML + pinned wrapper required for recurring named sizes,
   retuned field geometry, or any non-count parameter change.
2. `README.md`: short pointer to the new subsection. Docs-only change.
3. Follow-up: "Choosing the base suite for a lower `--targets` count" —
   select the base YAML by its non-count regime (field/arc geometry, cube
   size, containment, clearances), not by native count. Reduced counts run
   through the generic smoke with `--config <named suite YAML>`; such runs
   are not evidence for the named gate.
4. Follow-up: "Which named wrappers run as gates" — table mirroring
   `run_verification.sh spark`: Phase 7 / 7.1 / 7.2-default GUI smokes are
   required gates, integration 2×5 is opt-in (`--with-integration-smoke`),
   standard 2×10 is on-demand only (not wired into verification).
5. Follow-up: Placement terminology note — `manual` means the centres are
   declared in YAML (author-supplied, validated fail-closed, identical every
   episode), not that cubes are positioned by hand at runtime; computed
   policies (`grid`/`layout`/`random`) sample centres per episode instead.
6. `README.md`: timestamped project-size snapshot (179 tracked files,
   42,874 lines excluding third_party/assets/artifacts) plus an AI context
   utilization note — corpus ≈ 400k tokens ≈ 2× a ~200k-token window;
   complex cross-cutting turns run ~50–80% of the window, worst case one
   full window with summarization.

---

## 2026-07-23 — Standard 2×10 multi-target suite

1. Added dedicated `config/phase7_2_multi_target_standard_2x10.yml` and
   `scripts/host/smoke_phase7_2_standard_2x10.sh` (2 episodes × 10 targets).
2. Open arc at r=0.22 m / span 4.5 rad with flange-sized cubes and face
   containment (same tip-contact policy as integration 2×5, denser field).
3. Documented as a named standard size (not a retune of the default 2-target
   YAML). GUI visual smoke used for validation.

### Review recommended

- Confirm GUI 2×10 tip contacts and transit clearance look clean under shuffle.

### Verification

- Unit packing/smoke wiring tests.
- Host GUI: `smoke_phase7_2_standard_2x10.sh --gui --auto-exit --root-seed 4242`
  — exit 0, 2/2, 20 tip contacts, 0 plan fails, 0 body contacts.

---

## 2026-07-22 — Fix `--targets 10` pack + per-episode random seeds

1. Default `phase7_2_multi_target.yml` `field_aabb` was too small for a 10-target
   EE-floor grid (`--targets 10` failed at placement). Expanded to pack a 3×4
   lattice at ~0.076 m separation.
2. Omitting `--root-seed` now draws an independent random seed for **each**
   episode (not one shared suite seed). `--root-seed N` keeps deterministic
   `episode_seed = N + 1009*(i+1)`.
3. Plan logs `episode_seed` / `order_seed` per episode; bundle adds `seed_mode`
   and `episode_seeds`. Docs updated.

### Review recommended

- Confirm GUI `--targets 10 --episodes 3 --no-auto-exit` places and plays.

### Verification

- Unit: 10-target pack + independent episode seeds; smoke CLI wiring.

---

## 2026-07-22 — CLI `--root-seed` with varying default

1. Host plan/smoke accept `--root-seed N` (non-negative integer) to fix suite
   placement and episode planner seeds.
2. When omitted, each invocation draws a fresh seed in `[0, 2**31)` so layouts
   vary for coverage; YAML `root_seed` is library/API default only.
3. Effective seed logged as `phase7_2_plan: root_seed=N (cli|random)` and stored
   in the plan bundle. Documented in `spec.md`, phase 7.2 design, and README.

### Review recommended

- None beyond confirming operators use `--root-seed N` when reproducing a run.

### Verification

- Unit tests for `resolve_invocation_root_seed` and smoke CLI wiring.

---

## 2026-07-22 — Flange-rim anti-graze on transit

1. Root cause: `joint6_flange` collision spheres were r=0.008 while the assumed
   flange is Ø31 mm, so ~7.5 mm of rim was invisible to cuRobo and could skim
   neighbor cubes during tip-to-tip transit.
2. Flange spheres: central r=0.014 (+ `collision_sphere_buffer` 0.003 ≈ Ø31 mm
   envelope) with three rim helpers; count stays 32.
3. Multi-target validation: `flange_disk_cube_clearance_m` fails closed on
   flange-sphere penetration of non-contact cubes (`flange_neighbor_clearance`).
4. Planner activation `0.01 → 0.012` m; integration world clearance `0.006` m.

### Review recommended

- Confirm GUI transit no longer shows flange-edge skims on remaining cubes.
- If graze returns on denser packs, prefer more spacing over further sphere growth.

### Verification

- `./scripts/run_verification.sh ci` — 195 passed.
- Headless + GUI integration 2×5 — exit 0, 2/2, 10 tip contacts, 0 plan fails.

---

## 2026-07-22 — Surround open-arc integration 2×5 (multi-quadrant)

1. Root cause of “one quadrant” clustering: integration `field_aabb` was
   X≥0-only with a rectangular grid phase — targets could never leave the
   forward half.
2. Integration 2×5 now uses `placement: layout` / `arc` with
   `radius_m: 0.20`, `span_rad: 4.2` (~240° about `g_base`), keep-out ±0.12,
   and field `[-0.24,-0.24]…[0.24,0.24]`. Centres span ±X and ±Y (including
   −X). A full closed ring left a brittle rear pose; the open arc keeps
   multi-quadrant coverage with reliable clearance.
3. Host A/B: open-arc shuffle and listed both 2/2 with **0** plan fails.

### Review recommended

- Closed full ring (span≈2π·4/5) still fails some shuffle seeds on a rear
  target; revisit after a flange-sized tip-contact workspace remasure.

### Verification

- `./scripts/run_verification.sh ci` — unit/ruff green after test updates.
- Headless integration 2×5 — exit 0, 2/2, 10 tip contacts, 0 plan fails.
- GUI integration 2×5 — exit 0, 2/2, 10 tip contacts, `framed=True`.

---

## 2026-07-22 — Flange-face containment validation; flange-sized integration cubes

1. Added CPU flange-disk vs contact-face overhang metric
   (`flange_disk_face_overhang_m` / `flange_disk_collides_contact_face`) and
   suite flag `require_flange_face_containment` wired through multi-target
   validation (`flange_face_containment` violation). Tolerance
   `flange_face_overhang_tolerance_m` (default `0.005`) absorbs planner
   lateral IK error when edge ≈ flange Ø.
2. Integration 2×5: `target_edge_m: 0.031` (≥ flange), containment **on**,
   field `[0.02,-0.22]…[0.30,0.22]`, rim `0.36`, EE floor `0.093`. No more
   expected flange-edge clip on undersized 14 mm faces.
3. Unit tests: `tests/unit/test_flange_face_containment.py`.

### Review recommended

- Default (non-integration) suites still use 14 mm cubes without containment;
  enable the flag only when packing allows edge ≥ flange.
- GUI smoke still saw a few deferred plan retries (3) before full clearance.

### Verification

- `./scripts/run_verification.sh ci` — 194 passed.
- Headless integration 2×5 — exit 0, 2/2, 10 tip contacts, 0 plan fails.
- GUI integration 2×5 — exit 0, 2/2, 10 tip contacts, `framed=True`.

---

## 2026-07-22 — Flange tip classify; workspace map; high-effort IK-seed fix

1. Tip/body classification: path-prefix target match + tip-link segment match;
   `merge_contact_events(..., active_target_id=...)` so flange overhang on the
   **active** contact cube stays `ALLOWED_TIP_CONTACT` (Isaac solid cube still
   present; cuRobo already omits it). Documented: with `target_edge_m` (14 mm)
   < flange Ø (31 mm) tip contact implies ~8.5 mm face overhang — not a cube
   grow this pass (would break 2×5 packing).
2. Measured +Z tip-contact workspace sampler:
   `mycobot_curobo.tip_contact_workspace` +
   `scripts/host/measure_tip_contact_workspace.py` →
   `artifacts/workspace/tip_contact_workspace_v1.json` (candidate region;
   **not** a full dexterous claim). Integration AABB **not** expanded from map.
3. High-effort one-knob bisect on packing-safe 1×2: sole regressor is
   `num_ik_seeds: 64` (grasp segment `plan_failed`). Fixed
   `planning_high_effort` to keep IK seeds at **32**, retain trajopt 8 /
   attempts 4 / orient tol 0.05. Confirmed PASS; integration stays on
   `benchmark_reproducible` until a deliberate 2×5 re-enable.

### Review recommended

- Consume success AABB from the v1 workspace artifact before any field expand.
- Optional later: suite mode with `target_edge_m >= flange_diameter_assumption_m`
  once spacing allows.

### Verification

- Host GPU bisect: `num_ik_seeds=64` FAIL; trajopt/attempts OK; fixed
  `planning_high_effort` 1×2 PASS.
- Host GPU workspace measure: 86/114 (75.4%) →
  `artifacts/workspace/tip_contact_workspace_v1.json`.
- `./scripts/run_verification.sh ci` — 190 passed.
- GUI integration 2×5 smoke — exit 0, 2/2, 10 tip contacts, `framed=True`.

---

## 2026-07-22 — Diagnose high-effort; widen field; content frame; anti-graze

1. Host one-knob A/B (1×2): baseline PASS; rolls PASS; `pre_approach 0.025`
   FAIL (`plan_failed`); `planning_high_effort` FAIL at orient tol `0.08` and
   `0.05` (cuRobo infeasibility, not validation_failed). Kept high-effort
   orient tol at **0.05** (≤ Phase 4) with unit guard.
2. Integration 2×5: forward-biased widened AABB `[0.0,-0.17]…[0.24,0.17]`
   (full ±Y under rim; X≥0 avoids home start-collision from rear cubes),
   base keep-out ±0.08, `arm_z_motion_range_m: 0.28`; grid seed-offset retry
   when keep-out rejects a phase; `minimum_world_collision_clearance_m: 0.004`;
   pre-approach stays `0.01`.
3. Kit-free `compute_viewport_framing` / `content_aabb_from_field`; multi-target
   GUI settle frames arm ∪ targets (defaults remain fallback).
4. `optimizer_collision_activation_distance_m: 0.001 → 0.01` on
   `benchmark_reproducible` and `planning_high_effort`. Suite clearances now
   override the validation profile in `plan_multi_target_suite.py`.

### Review recommended

- High-effort still fails on this field even with tol `0.05` — separate
  diagnosis before re-enabling on integration.
- Visual: content-aware framing logged `framed=True` on GUI 2×5 smoke
  (2/2 tip contacts); confirm paths climb rather than skim cube tops.

### Verification

- `./scripts/run_verification.sh ci` — 184 passed.
- GUI integration 2×5 smoke — exit 0, 2/2 episodes, 10 tip contacts.
- Doc sync: README current-phase / high-effort / integration smoke wording;
  Phase 7.3 report EE-floor + keep-out retry note.

---

## 2026-07-22 — Fail episode when a pass defers all with no tip progress

1. `MultiTargetEpisodeRunner`: if a pass leaves deferred targets and made no
   tip-contact progress (including the first pass), fail immediately with
   `targets_unplanned` instead of burning `max_reconsider_passes` on the same
   obstacle field.
2. Spec / phase-report wording and unit coverage updated.

---

## 2026-07-22 — Zoom viewport on arm; keep packing-safe 2×5 field

1. `frame_viewport_on_arm()` in `scene_setup` (eye≈`(0.28,0.55,0.32)` →
   target≈`(0.14,-0.08,0.14)`); GUI play paths call it after viewport settle.
2. Integration 2×5 keeps packing-safe grid AABB + rim `0.28` and
   `benchmark_reproducible`. Trials of `planning_high_effort`,
   `pre_approach_distance_m: 0.025`, roll goalsets, and +X/−Y layout packs
   made every start→target plan fail closed — left as profile/docs only.

### Review recommended

- Confirm GUI framing is close enough; tweak `DEFAULT_VIEWPORT_*` if needed.
- Diagnose high-effort / roll / longer pre-approach regressions on this field.

---

## 2026-07-22 — Add planning_high_effort profile; propose placement easing

1. New planner profile `planning_high_effort` (64 IK / 8 trajopt seeds,
   orientation tol 0.08 rad, 4 `max_plan_grasp_attempts`) — less search-
   constrained than `benchmark_reproducible`. Integration 2×5 suite now uses it.
2. Documented placement easing proposal (later applied in the entry above).

---

## 2026-07-22 — Implement approach-plane EE clearance, labels, lighting

1. Placement uses **approach-plane** centre separation (⊥ `outward_normal_base`)
   with floor `edge + flange + ee_approach_clearance_m` (default clearance =
   flange → **0.076 m**); optional `max_target_radial_m` rim guard.
2. Integration 2×5 / Phase 7.3 example AABBs widened so grids pack at the new
   floor; integration sets `max_target_radial_m: 0.36`.
3. Viewport digit labels: `AddRotateZOp(180)` so glyphs are right-reading from
   the default +Y camera.
4. Default / suite lighting dimmed to dome **400** / distant **1000**.

### Review recommended

- Integration 2×5 AABB was tightened under `max_target_radial_m: 0.28` after the
  first GUI run left well-spaced but out-of-reach centres (~0.33 m). Re-check
  tip-contact success with the straddling field near ±Y.

---

## 2026-07-22 — Spec: stronger EE clearance + label facing note

1. Phase 7.2/7.3 placement: EE clearance uses **approach-plane** separation
   (not 3D alone) with floor
   `edge + flange + ee_approach_clearance_m` (default
   `ee_approach_clearance_m = flange`); optional rim guard. Aimed at mutual
   proximal deadlock (e.g. integration ep1 leftovers `1`/`2`).
2. Viewport digit labels must be right-reading from the primary camera
   (current fixed local +Y face can appear backward).

---

## 2026-07-22 — Fix planning-failure budget off-by-one

1. Defer when `current_count_planning_failure_per_target >=
   max_planning_failure_per_target` (was `>`), so a budget of 3 means exactly
   three failed attempts before moving to the next target.
2. Spec/phase-report wording: count **reaches** the limit.

---

## 2026-07-22 — Default max_planning_failure_per_target = 3

1. Suite loader / episode deserialize default for
   `max_planning_failure_per_target` is now **3** (was 5). Reaching the
   budget **defers** the target and continues to the next unfinished id.
2. Spec, phase report, README/STATUS, and configs/tests updated.

---

## 2026-07-22 — Implement EE-clearance target spacing

1. `ee_clearance_min_center_separation_m` helper; suite load defaults
   `min_center_separation_m` to `target_edge_m + flange_diameter_assumption_m`
   and rejects explicit values below that floor.
2. Phase 7.3 example configs updated above the floor; unit tests cover default,
   floor reject, and grid field validation.
3. Spec/docs already describe the rule (prior entry).

### Review recommended

- Re-run integration 2×5 smoke; packing already sat above the 0.045 m floor but
  mutual deadlock may still need a larger practical margin.

---

## 2026-07-22 — Spec: EE-clearance spacing for generated targets

1. `spec.md` §8 Phase 7.2 placement (grid / Z-band): generated centres must keep
   pairwise distance ≥ `target_edge_m + flange_diameter_assumption_m` (or a
   stricter `min_center_separation_m`) so tip/EE approach is not mutually
   deadlocked by adjacent remaining cubes.
2. Phase 7.3 `min_center_separation_m` default/floor updated to the same
   EE-clearance rule (legacy `2 * target_edge_m` alone is insufficient when
   flange diameter > edge). Synced phase 7.2 / 7.3 design notes.

---

## 2026-07-21 — Implement Phase 7.2 deferral / reconsider + Option A spheres

1. `MultiTargetEpisodeRunner`: defer after per-target planning budget; reconsider
   deferred targets after tip-removals (`max_reconsider_passes`); FAIL with
   `targets_unplanned` / `max_reconsider_passes_exceeded` if any target remains
   unplanned. Playback keeps plan-creation order of validated legs.
2. Unit tests for defer→remove→replan success and unplanned FAIL; play loader
   deserializes `deferred_target_ids` / `planned_target_ids`.
3. Phase 1.1 **Option A**: thickness-capped cover in
   `collision_sphere_cover.py`; regenerated overlay **1012** spheres
   (`radii ≤ E=0.014 m`). Trial GPU self-clear + body-clip detectability pass;
   arming still regresses Phase 7.1/7.2 GPU planning — overlay left commented out.
4. Spec/docs/STATUS updated: 7.2 deferral/reconsider implemented; Option A chosen
   but disarmed pending planning reconciliation.

### Review recommended

- Tune Option A (or suite fixtures) until 7.1/7.2 GPU + integration 2×5 pass
  with overlay armed; then uncomment `collision_sphere_overlay_path`.

---

## 2026-07-21 — Spec: Phase 7.2 deferral / reconsider / all-planned

1. `spec.md` §8 Phase 7.2: planning world uses only obstacles remaining after
   tip-contact removals (active contact cube still omitted for tip feasibility).
2. Exceeding `max_planning_failure_per_target` **defers** a target; after
   removals, deferred targets are **reconsidered** with the reduced field.
3. Playback must follow **plan-creation order**.
4. Episode PASS requires **every** target to end with a successful validated
   plan; any unplanned target → `targets_unplanned` FAIL. Deprecates
   `max_target_failures` as an episode-PASS escape hatch; introduces
   `max_reconsider_passes` (default `target_count`).
5. Implemented on `wip_phase7_3` (see entry above). Synced
   `docs/phase7_2_multi_target_contact.md`.

---

## 2026-07-21 — Phase 7.3 controllable placement implemented

1. Finalized `spec.md` §8 Phase 7.3: `random` and `layout` (`rows` / `arc`)
   with `min_center_separation_m`, `keep_outs`, fail-closed sampling.
2. Added `mycobot_curobo.target_placement` and wired
   `multi_target.build_target_field` / suite config load.
3. Example configs `config/phase7_3_multi_target_{random,layout_rows,layout_arc}.yml`.
4. Unit tests `tests/unit/test_target_placement.py`; phase report + STATUS.
5. Fixed Phase 3/4 GPU fixtures for tip-face `tool_approach_sign=+1`: known
   reachable poses now use outward normal `-TCP_Z` so `plan_grasp` recovers
   the FK goal (tests still used the pre-tip-face `+TCP_Z` normal).
6. Hardened Phase 7.2 playback tip-face evidence: snap to terminal joints
   (including after the hold so PhysX push-out cannot drop tip evidence),
   FK+USD tip checks, 15 mm tolerance, longer headless hold; integration 2×5
   grid `arm_z_motion_range_m` reduced so Z stays in the field AABB.
7. Integration 2×5 final gate green: headless + GUI `success_rate=1.0`
   (2/2 episodes, tip=7, body=0).

### Review recommended

- Host smoke a Phase 7.3 config under GUI if reviewing layouts visually.
- Integration 2×5 smoke remains the opt-in final host gate.

---

## 2026-07-21 — Integration smoke 2×5 + Phase 1.1 acceptance wiring

1. Config `phase7_2_multi_target_integration_2x5.yml` and host smoke
   `smoke_phase7_2_integration_2x5.sh` (2 episodes × 5 targets).
2. Grid placement accepts `placement_seed` so episodes get distinct fields;
   sampling passes `episode_seed` for grid suites.
3. `run_verification.sh spark --with-integration-smoke` (or
   `SPARK_RUN_INTEGRATION_SMOKE=1`) runs that smoke headless then GUI; not in
   the default spark gate. CI notes/skips Isaac integration smoke.
4. `spec.md` Phase 1.1 acceptance names this integration smoke for
   self-collision + unremoved-target evidence before re-arming overlays.

---

## 2026-07-21 — Phase 1.1 acceptance: headless+GUI self/unremoved-target gates

1. `spec.md` §8 Phase 1.1 acceptance: before re-arming any overlay, host
   headless **and** GUI smokes must evidence self-collision hard-gate behavior
   and fail-closed world collision vs **unremoved** non-contact targets.
2. Clarified relationship text: collision spheres originate in **Phase 1**;
   Phase 1.1 revises coverage; Phase 7.3 is placement only (not sphere intro).

---

## 2026-07-21 — Phase 1.1 headless verify: adapter fix + cover revision proposal

1. **Bug fixed:** `load_curobo_robot_config` stripped project-only keys after
   overlay merge so cuRobo `KinematicsLoaderCfg` can construct a planner.
2. Restored the 128-sphere overlay after a bad regenerate (~17k mm-scale
   centres). Regenerator refuses `|center| > 0.5 m` or `total > 512`.
3. **Cover blocked:** greedy Phase 1.1 spheres self-collide at the zero pose
   (and all sampled postures); scaffolding (32) is self-clear. Overlay path
   commented out in `mycobot_280_m5.yml`.
4. **`spec.md` §8 Phase 1.1** marked needs revision; proposed options A–D
   (recommend thickness-capped cover). Awaiting approval before further cover
   work.
5. GPU: scaffolding loads + zero pose self-clear; enabling overlay fails the
   self-collision hard gate.

### Review recommended

- Approve a Phase 1.1 revision option (A–D) in `spec.md` before re-arming
  any overlay.

---

## 2026-07-20 — STATUS resume note + push Phase 1.1 / 7.2 viz work

1. `STATUS.md` Next step documents the open investigation: Phase 1.1 spheres
   may not be biting in GUI smoke; planning messages looked missing; lists
   existing unit/GPU sphere tests and the gap (no headless Isaac test that
   asserts Phase 1.1 rejects body-clipping paths).
2. Bundles uncommitted Phase 7.3-branch work: CI pytest, red ID labels, tip
   clearance vs other targets, highlight colors, Phase 1.1 sphere overlay.

### Review recommended

- On resume: run Phase 7.2 GPU / host smoke and confirm overlay load + console
  plan lines; add a headless sphere-vs-cuboid regression if needed.

---

## 2026-07-20 — Red target ID labels

1. Multi-target viewport digit labels use bright red
   (`LABEL_COLOR_RGBA`) instead of white for higher contrast on blue/yellow
   cubes.

---

## 2026-07-20 — Phase 1.1 target-scale collision spheres implemented

1. Kept [`spec.md`](spec.md) §8 Phase 1.1 wording; marked implemented.
2. Added `mycobot_curobo.collision_sphere_cover` (COLLADA positions + unit scale,
   sparse greedy cover for obstacle edge `E`) and host regenerator
   `scripts/host/regenerate_target_scale_collision_spheres.py`.
3. Committed overlay
   `config/robots/mycobot_280_m5_phase1_1_spheres.yml` (**128** spheres for
   `E = 0.014 m`; was 32 Phase 1 scaffolding). Robot YAML declares
   `min_detectable_obstacle_edge_m` + overlay path; load merges into cuRobo
   config.
4. Multi-target suite load fails closed when `target_edge_m < E`.
5. Unit tests: cover invariants, overlay counts, edge mismatch.

### Review recommended

- Host GUI smoke: fewer false-clear body contacts vs near blockers; watch
  planning time with denser spheres.

---

## 2026-07-20 — Target highlight colors + tip clearance vs blockers

1. Playback highlights: yellow pending current target, green on tip contact,
   red on tip-miss / body contact; white 7-segment ID labels for contrast.
2. Multi-target `plan_grasp` no longer globally disables tip/flange world
   collision; only the active contact cube is stripped from the planning world
   so other remaining targets force tip detours around near blockers.
3. Spec / phase docs updated for highlight colors and collision policy.

### Review recommended

- Host GUI smoke: yellow→green/red transitions; tip routes around a near
  high-Z blocker toward a farther target (expect more plan skips if the field
  is dense).

---

## 2026-07-20 — Fix viewport label double-transform

1. `add_target_label` now applies a **parent-local** Z offset
   (`label_parent_local_offset_m`) under the translated cube prim. The prior
   world `center_m` translate on the child double-counted the parent pose and
   placed digits far from the blocks (looked “missing” in GUI smoke).
2. Unit coverage for the local-offset contract in `test_target_labels.py`.
3. Tip/world collision policy unchanged: Phase 7.2 still disables
   `joint6_flange` vs all world cuboids so tip contact remains feasible.

### Review recommended

- Re-run Phase 7.2 Isaac GUI smoke and confirm yellow IDs sit above each cube.

---

## 2026-07-20 — Grid mid-Z variability (50% of arm Z range)

1. Grid placement spaces target Z evenly in a band of width
   `0.5 * arm_z_motion_range_m` centered on the field AABB mid-Z (XY lattice
   unchanged). Band is not clipped to the thin AABB Z span.
2. Suites must declare `arm_z_motion_range_m` (configs use vendor
   `working_radius_m` = 0.28 m as the declared envelope magnitude).
3. Unit coverage in `test_grid_z_varies_across_half_arm_range`; docs/spec
   updated.

### Review recommended

- Confirm host planning success rates with taller Z spread (`--targets` grid
  fallback and `phase7_2_multi_target_grid.yml`).

---

## 2026-07-20 — CI pytest fix + viewport target IDs (Phase 7.3 branch)

1. GitHub Actions [`.github/workflows/pytest.yml`](.github/workflows/pytest.yml):
   install CPU-safe deps + `--no-deps -e .`; set `SPARK_PYTEST_PYTHON` to the
   setup-python interpreter so `run_verification.sh ci` finds pytest (was
   defaulting to system `/usr/bin/python3`).
2. `add_target_label` now spawns high-contrast 7-segment digit geometry above
   each multi-target cube (metadata-only Xform was invisible in the viewport).
3. Documented Phase 7.2 `grid` as an **XY mid-Z** lattice (not a 3D volume);
   volumetric layouts remain Phase 7.3 brainstorm.
4. Unit tests: `tests/unit/test_target_labels.py` + viz/CI contract asserts.

### Review recommended

- Confirm digit size/contrast in Isaac GUI smoke on DGX Spark.
- Confirm GitHub Actions matrix (3.10 / 3.12) goes green after push.

---

## 2026-07-20 — Land Phase 7.2; open Phase 7.3 under consideration

1. Documented Phase 7.2 completion (three-tier failures, tip-contact rule,
   `--no-auto-exit` continuous replay, host GUI evidence) across STATUS /
   README / phase report / `spec.md`.
2. Added Phase 7.3 placeholder (controllable target-block placement + GitHub
   Actions CI fixes) as **under consideration / brainstorm** on
   `wip_phase7_3`: `docs/phase7_3_target_placement.md`, roadmap, REFERENCES.
3. Synced landing docs with unpushed Phase 7.2 implementation commits.

---

## 2026-07-20 — max_target_failures default 3 + indefinite episode replay

1. Changed `max_target_failures` default from `floor(target_count / 2)` to a
   fixed **`3`** (`--targets` no longer rescales it).
2. Playback no longer skips plan-failed episodes that still have validated
   trajectories (so tip-contact motion is visible for review).
3. With `--no-auto-exit`, episodes **replay continuously** until the Kit window
   closes or Ctrl+C (not a frozen hold of the last frame).
4. Closing the Kit window mid-replay stops cleanly (no articulation teardown
   traceback) and still writes the first-pass report.

---

## 2026-07-20 — Hold GUI indefinitely with --no-auto-exit

1. `play_multi_target_suite.py` keeps stepping the Kit world until the window
   closes or Ctrl+C when `--no-auto-exit` is set.
2. `smoke_phase7_2_multi_target.sh` no longer aborts before playback when the
   planner exits non-zero (bundle still required), so GUI review can run.
3. With `--no-auto-exit`, the smoke wrapper skips the hard “fully succeeded”
   gate so the held session is not torn down solely for incomplete clearance;
   plan/play exit codes are still returned.

---

## 2026-07-20 — Tip contact required only for planned targets

1. Episode PASS no longer requires tip contact on planning-failed targets
   (motion was never attempted).
2. Tip miss after successful plan/validation aborts the episode immediately
   (`tip_contact_missed`).
3. Playback expects tip contact only for `contact_order_ids - failed_target_ids`.
4. Updated `spec.md`, phase report, unit tests.

### Review recommended

- Confirm that the prior 10-target run (7 tip / 3 planning failures within
  budget) would now PASS under this rule.

---

## 2026-07-20 — Default max_target_failures = floor(target_count / 2)

1. Changed the episode target-failure budget default from `target_count` to
   **`floor(target_count / 2)`** (`default_max_target_failures`).
2. `--targets N` continues to recompute that default when the prior value was
   the previous half-default.
3. Updated YAML comments/examples, unit tests, and Phase 7.2 docs/`spec.md`.

### Review recommended

- Odd `target_count` values floor (e.g. 5 → 2). Confirm that is intended vs
  ceil/round.

---

## 2026-07-20 — Phase 7.2 three-tier failure model

1. Replaced `intra_episode_plan_failures` / `max_intra_episode_plan_failures` /
   `max_failed_plans` with:
   - `max_planning_failure_per_target` (default **5**) +
     `current_count_planning_failure_per_target`;
   - `max_target_failures` (default `floor(target_count / 2)`);
   - `max_failed_episodes` (default **0**).
2. Runner marks a **target** failed when per-target planning failures exceed
   the budget, an **episode** failed when target failures exceed theirs, and
   suite acceptance when `failed_episodes > max_failed_episodes`.
3. Updated `spec.md`, phase report, configs, unit/GPU tests, and plan suite
   reporting.
4. Container CI: 142 unit tests + Ruff passed after the change.

### Review recommended

- Confirm default `max_target_failures == floor(target_count / 2)` with early
  abort when `target_failure_count > max`.

---

## 2026-07-20 — Phase 7.2 smoke `--episodes` override

1. Added `--episodes N` to `smoke_phase7_2_multi_target.sh`, forwarding to
   `plan_multi_target_suite.py` (already supported).
2. Artifact tags use `epM` or `NxM` when episode/target overrides are set.
3. Documented in `spec.md` §8/§9, README, and the Phase 7.2 report.

## 2026-07-20 — Default max_intra_episode_plan_failures = 5

1. Changed the within-episode retry ceiling default from 10 to **5** in loader,
   deserialize fallback, tests, and Phase 7.2 docs/`spec.md`.

## 2026-07-20 — Implement Phase 7.2 two-scope plan-failure counting

1. Wired `max_intra_episode_plan_failures` (default 5) as the within-episode
   retry ceiling and `intra_episode_plan_failures` as the observed metric.
2. Suite aggregation counts planning-failed **episodes**
   (`suite_planning_failed_episodes` / `total_failed_plans`); acceptance uses
   `max_failed_plans` via `suite_acceptance_passed`.
3. Episode taxonomy on budget exhaustion is
   `max_intra_episode_plan_failures_exceeded`.
4. Added `scripts/host/run_phase7_2_gpu.sh` for focused host GPU coverage.

# CHANGES — MyCobot 280 M5 Constrained Approach Planner

## 2026-07-20 — Phase 7.2 plan-failure counting (spec)

1. Added observed metric **`intra_episode_plan_failures`** (starts at `0` each
   episode) for within-episode planning/validation retry attempts.
2. Added config **`max_intra_episode_plan_failures`** (default **`5`**):
   within-episode retry ceiling. Exceeding it fails the episode and counts as
   **exactly one** suite planning failure.
3. Clarified **`max_failed_plans`** (default `target_count`) as the suite /
   acceptance budget on the number of planning-failed **episodes**.
4. Updated `spec.md` §8 Phase 7.2,
   [`docs/phase7_2_multi_target_contact.md`](docs/phase7_2_multi_target_contact.md),
   and `docs/implementation_phases.md`. Spec-only; no implementation in this
   change set.

### Review recommended

- Confirm `max_failed_plans` as suite-level episode budget.

---

## 2026-07-20 — Phase 7.2 requirements and API design

1. Added Phase 7.2 (multi-target tip-contact clearance) to `spec.md` §8,
   `docs/implementation_phases.md`, and
   [`docs/phase7_2_multi_target_contact.md`](docs/phase7_2_multi_target_contact.md).
2. Documented core Isaac-free multi-target API: `TargetField` with
   `placement` (`grid`|`manual`), `order` (`shuffle`|`listed`),
   `retain_targets_after_contact`, same-leg retry, and
   `max_failed_plans == target_count` by default.
3. Specified flange-normal tip/EE allow-list contact vs arm-body fail-closed
   policy, dual console timing, and hardware-transfer surfaces
   (`ContactDetector`, `TargetPoseSource`, scene revision, `MotionGate`) for
   Phases 10–11.
4. Locked docstring and design-doc conventions: concise public headers;
   call/control flow in the phase report; thin README pointer.
5. Updated README/STATUS/REFERENCES; branch `wip_phase7_2`. No implementation
   code in this change set.

### Review recommended

- Confirm acceptance defaults (`grid`, `shuffle`, remove-on-contact) vs
  suggested HW defaults (`manual`, `listed`, retain) in the phase report.

---

## 2026-07-20 — Fix Kit "No lights found" / stage lighting warning

1. Opening the prepared robot USD (no `LightAPI` prims) with Kit
   `autoLightRig.enabled=true` posted **No lights found in stage, applying
   lighting: 'Default'** and applied a light rig that hides later UsdLux
   prims — the viewport warning persisted even after `lighting_ready=true`.
2. Added `configure_kit_for_stage_lighting()` (disable auto light-rig +
   suppress the menubar notification) and call it **before** `open_stage` in
   both Phase 7 / 7.1 players.
3. `enable_viewport_stage_lighting()` now prefers
   `SetLightingMenuModeCommand(lighting_mode="stage")` with an explicit
   UsdContext (works before a viewport exists); re-assert after GUI settle.
4. `stage_lighting_mode_active()` also checks the menubar `lightingMode`
   setting so a Default rig no longer reports as stage mode.

### Review recommended

- Host GUI: `./scripts/host/run_phase7_1_chained_gui.sh --GUI --episodes 5`
  and confirm no "No lights found" toast; viewport Lighting menu shows Stage.

---

## 2026-07-20 — Host Mode B chained GUI runner

1. Added `scripts/host/run_phase7_1_chained_gui.sh` for host-native Mode B
   chained cube GUI (default 20 episodes, `--no-auto-exit`). Planner gains
   `--chained` (force modes B+D) and `--episodes`.
2. Added explicit `--GUI`/`--gui`/`--headless` flags; GUI mode resolves
   `DISPLAY`/`XAUTHORITY` via `spark_require_gui_display` before Kit launch.
3. Documented host and `spark_host_exec` invocation; unit-wired in
   `test_isaac_viz_smoke.py` / `test_cube_suite.py`.

### Review recommended

- Host: `./scripts/host/run_phase7_1_chained_gui.sh --GUI --episodes 20` and
  confirm the Isaac window appears and logs show `B/chained_last_success`
  after the first success.

---

## 2026-07-20 — Flange tip face leads cube contact

1. Mode D had placed the cube on the tip’s **−Z** side while
   `tool_approach_sign: -1`, so the wrist/back of the bare flange led into
   contact. GUI evidence: wrong side of the EE hit the cube.
2. Set `tool_approach_sign: +1` (app default + `TaskFrameConfig`) so tool **+Z**
   (flange tip) aligns with the approach direction into the workpiece.
3. Mode D now stores outward normal as **−tool_+Z** (cube on the tip-face side)
   and expands the conservative goal AABBs so the existing goal-joint bank still
   samples inside declared regions.
4. Unit regression: tip +Z, planned approach axis, and tip→cube direction agree
   for seed-123 episodes.

### Review recommended

- Re-run Phase 7.1 GUI (`--gui --no-auto-exit`) and confirm the flange tip face
  approaches the cube. Host GPU smoke after the sign flip.

---

## 2026-07-20 — Isaac GUI visibility + stage lighting mode

1. Creating UsdLux dome/distant lights alone left the Kit viewport on
   camera/rig lighting, which **hides** stage `LightAPI` prims — the UI showed
   stage lighting disabled and the scene stayed dark.
2. Added `enable_viewport_stage_lighting()` /
   `prepare_illuminated_stage()` in `isaac_sim/scene_setup.py` to call
   `set_lighting_mode_stage` and clear `/rtx/useViewLightingMode`, then re-apply
   after `World.reset()` in both Phase 7 and 7.1 players.
3. GUI path: explicit window size, DISPLAY banner, viewport settle frames,
   default `--hold-s 2` when `--gui`, and clearer `--no-auto-exit` hold message.
   Exit status is written into the report before `app.close()` so Kit shutdown
   cannot mask failures. Light setup is idempotent so re-applying after
   `World.reset` does not stack `xformOp:rotateXYZ`.
4. Unit coverage for the new helpers; argparse `--gui`/`--headless` now share a
   single `gui` dest (headless default).

### Review recommended

- Confirm on the Spark desktop (`DISPLAY=:1`) that the Kit window is lit and
  the viewport lighting menu shows **Stage**. Interactive check:
  `./scripts/host/spark_host_exec.sh ./scripts/host/smoke_isaac_viz.sh --gui --no-auto-exit`

---

## 2026-07-20 — Phase 7.1 rule-compliance cleanup

1. Audited Phase 7.1 sources against newly added Cursor rules (`python`,
   `bash`, `clean-code`). Skipped chemistry/PyTorch and C++ rule packs as
   out of scope for this package.
2. Removed unused `_sample_normal` from `cube_suite.py`, named the Mode D
   rejection budget (`GOAL_REGION_SAMPLE_ATTEMPTS`), and split Kit playback
   helpers in `play_cube_suite.py` (`STAGE_SETTLE_UPDATES`).
3. Refactored `smoke_phase7_1_cube_suite.sh` to a `main` with `local` vars,
   `printf`, and shellcheck-clean sourcing.
4. Added the new rule files under `.cursor/rules/` and a unit check for the
   named sample budget. Retested: CI 120 unit tests, host GPU 8/8, Phase 7
   GUI smoke, Phase 7.1 GUI `--all-modes` exit 0.

### Review recommended

- Broader repo bash scripts still use `echo` (pre-existing). Only the Phase
  7.1 smoke was brought to the new bash template in this change.

---

## 2026-07-19 — Phase 7.1 complete (host acceptance)

1. Landed the full Phase 7.1 cube-approach suite: validated config (14 mm cube,
   0.08 m standoff, FK-aligned Mode D goal bank), cube clearance, Mode C
   `plan_cspace`, illuminated Isaac plan/playback process split, drive-target
   motion, and PhysX prohibited-contact evidence with null tip metrics.
2. Restored empty-scene handling in `create_curobo_planner` (`cuboid: {}` is
   empty) and kept `scene_config_path` as the Phase 0–6 path alias.
3. Host evidence: CI 119 unit tests; GPU 8/8; Phase 7 GUI smoke lit; Phase 7.1
   headless 5/5 with 0 contacts; GUI `--all-modes` 4/5 + 1 structured failure,
   0 contacts on played episodes. Simulation metrics only.

### Review recommended

- Optional: reduce Mode B chained trajopt failures under `--all-modes` without
  weakening thresholds; not required for Phase 7.1 landing.

---

## 2026-07-19 — Phase 7.1 and contact-tool requirements

### Enumerated changes

1. Inserted Phase 7.1 on `wip_phase7_1`: a configurable normal-approach cube
   visualization suite with a default of five episodes, 14 mm cube, live
   console/JSON reporting, and exact replay.
2. Defined Mode A independent unknown starts and Mode D diverse 3D goals as
   defaults; Mode B chained starts and Mode C relocate-then-approach are
   optional at runtime but all A–D modes are required for acceptance.
3. Required the cube as cuRobo/Isaac collision geometry, a positive
   configurable standoff, fail-closed non-empty-world clearance, zero
   prohibited Isaac arm/cube/environment contact events, independent
   lateral/axis/terminal/collision validation, and null/`not_evaluated` Isaac
   tip position/orientation throughout Phase 7.1.
4. Inserted Phase 9 for a fabricated flange contact tool, including physical
   flange measurement, parameterized millimetre OpenSCAD source, matching
   manifold/watertight printable STL, deterministic regeneration, print/fit
   documentation, and optional explicit TCP/visual/collision profiles.
5. Inserted Phase 9.1 for unpowered tool evaluation: dimensional inspection,
   calibration uncertainty, remounting repeatability, independent FK,
   collision-model checks, and seeded tool-profile cube episodes.
6. Renumbered hardware dry-run and physical validation to Phases 10 and 11,
   and documented decimal branch names `wip_phase7_1` / `wip_phase9_1`.
7. Added dedicated Phase 7.1, 9, and 9.1 requirement reports and synchronized
   `spec.md`, roadmap, README, references, status, project rule, and prompt
   history.
8. Passed the existing container CI gate: 108 unit tests, Ruff lint, and Ruff
   format. GPU/Isaac implementation gates are deferred until Phase 7.1 code
   exists.

### Review recommended

- **Flange dimension:** the 31 mm diameter used to derive the 14 mm cube is an
  explicit unverified assumption. Phase 9 must measure and record the physical
  flange before finalizing the tool.
- **Thresholds:** Phase 9.1 must collect repeatability/calibration evidence
  before proposing hardware gates; no measurement threshold is invented here.
- **Implementation status:** these changes finalize requirements only. The
  Phase 7.1 acceptance checklist remains pending.

---

## 2026-07-19 — Phase 7 Isaac Sim validated-plan playback

### Enumerated changes

1. Added a versioned, typed, Isaac-neutral playback JSON contract generated
   from `ValidatedPlan`, including exact joints/units/target metadata and a
   fail-closed executable-plan gate.
2. Added a compact six-waypoint executable fixture plus tests for round-trip
   loading, invalid execution status, joint ordering, and non-finite values.
3. Added NumPy-only tip position/orientation metrics and exact required-joint
   to articulation-DOF mapping helpers.
4. Added an Isaac Sim 6.x standalone player that opens the prepared USD,
   discovers its articulation, applies every waypoint, and writes separate sim
   metrics. Missing `tcp_link` pose data stays null/unevaluated.
5. Replaced the host smoke placeholder with prerequisite, vendor asset, USD
   conversion, and validated-plan playback orchestration for headless/GUI use.
6. Made the Phase 7 GUI smoke a mandatory `run_verification.sh spark` gate,
   delegating through `spark_host_exec.sh` from the container with no bypass.
7. Added Phase 7 wiring tests and synchronized the specification, roadmap,
   README, references, status, phase report, and prompt history.
8. Passed container CI (108 tests plus Ruff), host prerequisites/conversion,
   and both headless and GUI auto-exit smokes. Each smoke played all six
   waypoints and exited zero.
9. Fast-forwarded `main` to the tested Phase 7 tip after the activated spark
   GUI gate passed, preserving `wip_phase7` as the historical phase snapshot.

### Review recommended

- **Isaac warnings:** host runs retain visible audio-device and duplicate
  protobuf-registration warnings. Stage loading and playback still completed;
  the warnings were not suppressed.
- **Synthetic fixture:** the committed near-zero trajectory proves playback
  wiring only; it is not planning quality or physical-accuracy evidence.
- **TCP metrics:** review the prepared USD hierarchy if `tcp_link` remains
  unavailable. Null/unevaluated metrics are intentional until an exact prim is
  present.

---

## 2026-07-19 — Phase 6 randomized workspace benchmark

### Enumerated changes

1. Added a validated benchmark YAML declaring conservative, unmeasured `g_base`
   candidate AABBs, labeled normal bins, explicit start states, roll and
   pre-approach policies, planner seed sweep, repeat count, and minimum
   20/100/1000 stage sizes.
2. Added immutable benchmark cases/results/summaries, deterministic root-seed
   sampling, complete request serialization/deserialization, seven-category
   planning/validation failure mapping, raw planner-status retention, and
   all-case aggregation.
3. Added plan → independent validation orchestration with injected planner and
   validator boundaries. Optional Phase 5 zero-residual execution replay is
   post-validation and its rejection is never counted as a planning failure.
4. Preserved the Phase 3 request/profile seed invariant by copying
   `PlannerProfile` with each sweep seed and constructing fresh planners.
5. Added JSON and Markdown writers under `artifacts/benchmarks/`, the benchmark
   and single-request replay scripts, and the app benchmark-config path.
6. Added frozen 20-case smoke and 100-case regression parameter fixtures,
   deterministic unit coverage, and a GPU-marked dual-run smoke integration.
7. Added the Phase 6 report and synchronized specification, roadmap, README,
   references, status, and prompt history.
8. Passed container CI with 97 unit tests plus Ruff lint/format. Host GPU
   verification passes all six integrations, including a two-case dual-run
   Phase 6 smoke subset with zero disagreement. Host pytest now uses an
   ownership-safe basetemp, and the Phase 6 GPU test creates its own report
   directory. Full 20-case smoke and exploratory 1,000-case stages are
   available via CLI and are not claimed as executed here.

### Review recommended

- **Workspace evidence:** configured AABBs are deliberately labeled unmeasured
  candidate regions. Review host smoke outcomes before changing their bounds;
  do not relabel them as a measured dexterous workspace.
- **Exploratory evidence:** the implementation supports 1,000 cases, but this
  change does not claim that exploratory stage was executed.
- **Full smoke stage:** the GPU gate intentionally uses a short dual-run
  subset under the fresh-backend/warmup lifecycle; run the 20-case CLI stage
  when recording a baseline metrics report.

---

## 2026-07-19 — Phase 5 execution and zero-residual seam

### Enumerated changes

1. Added typed `CartesianResidual`, `ResidualObservation`, and
   `ZeroResidualCorrector` contracts without introducing a learned policy,
   hardware driver, or alternate planner.
2. Added configured `ResidualSafetyProfile` loading and deterministic
   `SafetyProjector` decisions for residual magnitude, terminal corridor,
   joint feasibility, state freshness, and watchdog expiry.
3. Added `TrajectorySource`, deterministic replay state, independent TCP pose
   evaluation, structured execution results, and an in-memory-only command
   adapter.
4. Kept Phase 5 execution fail closed: only valid executable plans enter the
   seam, every waypoint is rechecked, and projected non-zero residuals are
   rejected before they can become joint commands.
5. Added negative and identity tests covering unsafe corrections, stale state,
   invalid plans, replacement-path prevention, and forbidden runtime
   dependencies.
6. Added `config/residual_safety.yml`, the Phase 5 report, public exports, and
   synchronized specification, roadmap, README, references, and status.
7. Updated verification caches for root-squashed workspaces without suppressing
   warnings. The CI gate passes 90 unit tests plus Ruff lint/format; all five
   host GPU integrations also pass with the recorded GB10 warning visible.
8. Corrected container-to-host GPU verification to delegate a repository shell
   script instead of incorrectly asking the script-only host wrapper to execute
   the Python binary through `bash`; native host GPU tests now use Isaac Sim's
   `python.sh`, where the pinned cuRobo/CUDA stack is installed, with unrelated
   ROS pytest entry-point plugins disabled to avoid undeclared plugin imports.

### Review recommended

- **Future non-zero mapping:** Phase 8 must specify a bounded local
  Cartesian-to-joint correction and independently validate it. The current
  executor intentionally rejects all non-zero residuals.
- **Hardware timing:** Phase 5 timestamps are deterministic replay values.
  Phase 9 must validate real clock source, stale-state, and watchdog behavior
  before any gated hardware adapter can emit motion.

---

## 2026-07-19 — Container Ruff bootstrap

### Enumerated changes

1. Added always-on Cursor rule `.cursor/rules/40-container-dev-tools.mdc`
   directing agents to install Ruff in the Isaac ROS / Cursor container for CI
   gates without installing cuRobo, CUDA PyTorch, or Isaac Kit.
2. Added `scripts/ensure_container_dev_tools.sh` to create a Ruff-only venv
   (project `.venv`, cache, or `/tmp` fallback) when the workspace is not
   writable by the container UID.
3. Updated `scripts/run_verification.sh` to auto-bootstrap Ruff, run lint via
   the Ruff interpreter, and keep unit tests on the system/container Python
   that already provides NumPy/PyYAML. Pytest cache output defaults to a
   writable `/tmp` path, with `SPARK_PYTEST_CACHE_DIR` available for explicit
   overrides, so root-squashed workspace ownership does not emit cache-write
   warnings. Ruff uses the equivalent writable cache policy through
   `SPARK_RUFF_CACHE_DIR`.
4. Added unit coverage for the bootstrap/verification policy and synchronized
   workflow rule, README, status, and references.

### Review recommended

- **Workspace ownership:** prefer fixing bind-mount UID/GID so project `.venv`
  is writable; `/tmp` fallback works but is session-local.
- **Full host install:** on DGX Spark, continue using `pip install -e '.[dev,cuda*]'`
  when a complete planning environment is required.

---

## 2026-07-19 — cuRobo-exclusive planning policy

### Enumerated changes

1. Made cuRobo v0.8.0 the explicit exclusive global and local motion planner,
   rather than merely the primary planning dependency.
2. Prohibited non-cuRobo planning through retries, fallbacks, learned policies,
   simulators, ROS/hardware adapters, external packages, and any runtime or
   configuration switch.
3. Limited any future CPU planning to a capability supplied by the pinned
   cuRobo implementation and covered by explicit project validation.
4. Clarified that independent validation and bounded residual execution
   corrections are not planners: they may reject or locally correct a cuRobo
   plan but may not generate replacement trajectories or full pose-to-joint
   solutions.
5. Synchronized the cuRobo Cursor rule, specification, README, references,
   status, implementation roadmap, and Phase 3–4 reports.
6. Verified 76 unit tests pass, documentation diffs have no whitespace errors,
   and edited files have no IDE lint diagnostics. The unified CI wrapper could
   not run Ruff initially because the module was unavailable in this container;
   pytest also retained its existing cache-directory permission warnings.
   Follow-up: container Ruff bootstrap now lands in a later change set.

### Review recommended

- **Phase 5/8 enforcement:** when those phases are implemented, add tests that
  reject residual or adapter outputs representing replacement trajectories or
  target-pose-to-full-joint solutions.
- **Future cuRobo upgrades:** retain planner exclusivity and revalidate any CPU
  execution capability before enabling it.

---

## 2026-07-19 — Prior-project retirement / V3 isolation

### Enumerated changes

1. Added V3-only `spark_isaac_mycobot_v3.code-workspace` and retirement docs
   (`docs/v2_retirement.md`, `docs/legacy/`).
2. Added Cursor rules `05-v2-retirement.mdc` and `30-workflow-and-isaac.mdc`;
   corrected `10-curobo-v080.mdc` to the fresh-planner-per-call v0.8.0 policy.
3. Migrated Phase 7 scaffolding tests (`test_urdf_utils`, `test_joint_drives`),
   Phase 8 Isaac Lab host bootstrap (`isaac_lab/`, install/verify scripts),
   `scripts/run_verification.sh`, CI workflow, and secondary docs push helper.
4. Added Phase 7 `smoke_isaac_viz.sh` placeholder and fixed dangling host-script
   examples that advertised nonexistent commands.
5. Archived the prior project's final uncommitted docs/metrics under
   `docs/legacy/v2_archive/` (historical only; not V3 acceptance evidence).
6. Retired prior-tree agent access via that tree's `.cursorignore`, `RETIRED.md`,
   and workspace redirect away from its own sources.

### Review recommended

- **Workspace reopen:** reload Cursor on the V3-only workspace and start a new
  agent chat so multi-root prior-project context is discarded.
- **Isaac Lab pin:** `isaac_lab/versions.env` still defaults to `develop`; pin
  an exact revision before Phase 8 reproducibility claims.
- **Phase 7 player:** implement a V3-native `NominalPlan` player; do not revive
  the prior IK/recovery viz stack.

---

## 2026-07-19 — Phase 4 validation

### Enumerated changes

1. Added `validation.py` with typed `ValidationProfile`,
   `KinematicCollisionBatch`, violations, metrics, reports, `ValidatedPlan`,
   `CuroboTrajectoryEvaluator`, and fail-closed `validate_nominal_plan`.
2. Added `config/validation_profiles.yml` with the specification's simulation
   thresholds plus roll, self/world clearance, segment-boundary limits, and a
   non-authoritative `hardware_placeholder` stub for later hardware work.
3. Added `CuroboTrajectoryEvaluator` for independent cuRobo FK and configured
   self-collision sphere-pair clearance; explicitly empty worlds are evaluated
   while unsupported non-empty worlds fail closed as unevaluated.
4. Enforced fresh backend → reset seed → configured public warmup → reset seed
   → exactly one `plan_grasp` after GPU evidence showed an unwarmed v0.8.0
   planner could stop at the pre-approach pose while reporting success.
5. Strengthened the Phase 3 GPU regression to require the measured terminal FK
   endpoint to reach the target within the configured planner position
   tolerance.
6. Added synthetic coverage for valid, curved, reversed-progress, misoriented,
   unevaluated-world, limit/dynamics, self-collision, and non-finite cases.
   Added a DGX Spark GPU eligibility regression using real cuRobo FK and
   self-clearance in an explicitly empty world.
7. Added `docs/phase4_validation.md` and synchronized STATUS, README,
   REFERENCES, specification, roadmap, Phase 3 lifecycle notes, and change
   inventory.

### Review recommended

- **World clearance:** empty-scene evaluation is accepted; non-empty worlds
  still fail closed until a supported distance adapter and obstacle regression
  land.
- **Hardware thresholds:** `hardware_placeholder` is a stub only. Do not use it
  for physical MyCobot claims before Phase 9/10 measurement.
- **Clearance policy:** review zero-meter simulation thresholds and collision
  sphere coverage before hardware work; these are not hardware safety margins.
- **Planner latency:** benchmark fresh construction plus warmup in Phase 6
  without weakening the one-call lifecycle.

---

## 2026-07-19 — Phase 3 nominal planning

### Enumerated changes

1. Added typed planning requests, named joint states, planner profiles, nominal
   plans, structured failures, and fail-closed outcomes.
2. Added the public cuRobo v0.8.0 `MotionPlanner.plan_grasp` adapter with
   approach-only options and signed TCP-axis pre-approach offsets.
3. Added valid-last-timestep trajectory extraction, finite checks, segment
   continuity enforcement, concatenation, and stable selected-roll mapping.
4. Added YAML planner profiles and an empty deterministic planning scene.
5. Adopted the user-selected reliability policy of constructing a fresh
   `MotionPlanner` for every `plan_grasp` call and retry after GPU tests showed
   unsafe state mutation when a v0.8.0 instance was reused.
6. Added CPU orchestration/error tests and a DGX Spark GPU regression covering
   two-segment planning, distinct backend instances, seeded reproducibility,
   endpoint FK, and the target-normal line constraint.
7. Added `docs/phase3_nominal_planning.md`, updated the authoritative lifecycle
   in `spec.md`, and synchronized README, references, status, roadmap, exports,
   change inventory, and prompt history.

### Review recommended

- **Planner latency:** fresh construction is intentionally slower than warmed
  reuse. Measure it in Phase 6, but do not restore reuse without a future
  pinned cuRobo version passing the lifecycle regression.
- **Validation boundary:** Phase 3 plans remain non-executable. Review Phase 4
  geometry, collision, limits, and smoothness validation before execution.

---

## 2026-07-18 — Phase 2 task frames

### Enumerated changes

1. Added immutable `SurfaceTarget` validation with explicit units/frames,
   finite checks, normal normalization, pre-approach bounds, mutually exclusive
   fixed/candidate rolls, and duplicate-angle rejection.
2. Added configurable x/y/z signed TCP-axis task-frame construction, projected
   tangent handling, deterministic least-aligned fallback, rotation validation,
   and scalar-first quaternion conversion.
3. Added ordered `SurfaceGoalSet` with stable goal-index-to-roll mapping and
   public cuRoboV2 `GoalToolPose` conversion.
4. Added typed `AppConfig` and `config/app.yml` so approach sign, axis, roll
   density, bounds, paths, profiles, seed, and logging are startup-validated.
5. Added tests for invalid inputs, all six axis/sign conventions, degeneracy,
   fixed roll, index bounds, 512 seeded randomized normals, and GPU cuRobo goal
   conversion.
6. Added `docs/phase2_task_frames.md` and updated all project documentation.

### Review recommended

- Confirm the physical tool's signed approach-axis convention visually in
  Phase 7 and again before hardware use. Phase 2 proves the configured
  mathematics; it does not calibrate a physical tool.

---

## 2026-07-18 — Phase 1 robot model

### Enumerated changes

1. Added `config/robots/mycobot_280_m5.yml` in cuRobo v0.8.0 format 2.0
   with exact URDF joint order, explicit bare-flange `tcp_link`, conservative
   acceleration/jerk assumptions, 32 static collision spheres, and
   self-collision configuration.
2. Pinned Elephant Robotics asset provenance to `mycobot_ros2` `humble`
   commit `3999e2cda7460d61f4fd2ffaa31049f000eae7a8` and retained its
   BSD-2-Clause license.
3. Documented the derived cuRobo URDF: vendor transforms/position limits are
   retained while zero velocity placeholders are replaced with the published
   160 deg/s maximum.
4. Added `mycobot_curobo.robot_model` with typed metadata, strict config
   validation, explicit named-state reordering, independent CPU FK, and a
   cuRobo adapter that resolves external paths deterministically.
5. Added five FK regression fixtures, negative order/limit/config tests,
   an inspection CLI + host wrapper, and a GPU integration test.
6. Corrected CUDA dependency installation: project extras are `cuda12` /
   `cuda13`, and the host installer uses cuRobo's `cu13` extra without
   replacing Isaac Sim's CUDA-enabled PyTorch.
7. Updated `spec.md` for the verified v0.8.0 external-config behavior and
   updated all project documentation with Phase 1 evidence.

### Review recommended

- **Collision geometry:** visually review the reduced four-sphere-per-link set
  against every vendor mesh before hardware use; increase density if coverage
  is incomplete.
- **TCP:** the identity transform is correct only for the bare flange. Any
  attached tool requires measured calibration.
- **Runtime:** continue monitoring the visible GB10 compute-capability warning
  and pre-existing Isaac Lab package-version conflicts.

---

## 2026-07-18 — Phase 0 completion

### Enumerated changes

1. Completed the DGX Spark environment gate with cuRobo v0.8.0, public
   cuRoboV2 imports, CUDA allocation on NVIDIA GB10, and a machine-readable
   valid report.
2. Replaced the host's stale `nvidia-curobo 0.0.0` editable installation with
   the exact v0.8.0 tag using `scripts/host/install_curobo.sh`.
3. Formatted all Python sources and fixed the environment CLI import block;
   `ruff check` and `ruff format --check` now pass.
4. Added [`docs/phase0_environment.md`](docs/phase0_environment.md) with
   runtime versions, test evidence, the recorded GB10/PyTorch warning, and the
   Phase 0 boundary.
5. Added the persistent phase-branch / rebase / fast-forward-main policy to
   `spec.md` §13.1 and `.cursor/rules/00-project-core.mdc`.
6. Updated `README.md`, `STATUS.md`, `REFERENCES.md`, and prompt history for
   the completed phase.

### Review recommended

- **Review in Phase 1:** PyTorch 2.10.0+cu130 advertises compute capability
  through 12.0 while the GB10 reports 12.1. CUDA allocation succeeds, but
  planner kernel execution must be verified before Phase 1 acceptance.

---

## 2026-07-18 — Phase roadmap + Isaac Sim scaffolding

### Enumerated changes

1. Added [`docs/implementation_phases.md`](docs/implementation_phases.md) defining
   Phases 0–10 (initial planner 0–6; Isaac Sim 7; residual RL 8; hardware
   dry-run 9; physical MyCobot validation 10).
2. Expanded [`spec.md`](spec.md) §2, §7 layout, §8 (Phases 7–10), and §14 so
   residual RL and physical testing are first-class planned phases while keeping
   Phases 0–6 as the initial-project definition of done.
3. Copied/adapted Isaac Sim host resources from v2 into v3:
   `scripts/isaac_sim_env.sh`, `scripts/download_mycobot_ros2.sh`,
   `scripts/convert_urdf_to_usd.sh`, `scripts/host/*`, `isaac_sim/{urdf_utils,
   convert_urdf_to_usd, urdf_import, joint_drives}.py`.
4. Staged `assets/mycobot_280_m5/urdf/{kinematics,curobo}.urdf` from v2 with
   provenance READMEs; obtained vendor `mycobot_ros2` via local sibling symlink
   under `third_party/` (gitignored).
5. Pinned `scripts/host/install_curobo.sh` default to **v0.8.0**.
6. Updated `.cursor/rules/00-project-core.mdc`, `STATUS.md`, `REFERENCES.md`,
   and `README.md` for the extended roadmap.
7. Deliberately omitted v2 `run_ik_viz.py`, residual IK recovery, ROS packages,
   and e2e learned-IK training stacks.

### Review recommended

- Confirm Phase 1 will re-validate staging URDFs (limits, TCP, license) before
  treating them as authoritative.
- Confirm residual RL Phase 8 observation/action units against Phase 5 contracts
  before any training code lands.
- Hardware Phases 9–10 enable-flag naming should match any future CI secrets /
  operator checklist before live motion.

---

## 2026-07-18 — Phase 0 bootstrap

### Enumerated changes

1. Preserved v3's pre-existing [`spec.md`](spec.md) and
   [`.cursor/rules/`](.cursor/rules/) as the primary requirements.
2. Adapted the v2 Apache-2.0 project license for v3 contributors.
3. Adapted v2's generic Python packaging and ignore-list concepts into a new
   cuRoboV2-specific `pyproject.toml` and `.gitignore`.
4. Added `mycobot_curobo.version_guard` with typed runtime/report contracts,
   exact cuRobo v0.8.0 checks, required-public-API checks, CUDA diagnostics,
   GPU allocation verification, and JSON output.
5. Added a Phase 0 environment CLI, lightweight unit tests, and a separately
   marked GPU integration import smoke test.
6. Created fresh v3 `README.md`, `STATUS.md`, `CHANGES.md`, and
   `REFERENCES.md`; no v2 status or performance metrics were copied.
7. Initialized generated-artifact directories with committed placeholders.

### Deliberately omitted from v2 (bootstrap)

- Isaac viz player / recovery / MotionGen stacks (later: scaffolding only);
- ROS 2 packages, hardware scripts, `pymycobot`, and vendor ROS checkouts in git;
- supervised/RL residual training code and checkpoints;
- v2 configuration, tests, metrics, logs, and current-status documentation.

### Review recommended

- Verify the CUDA 12/PyTorch resolver choice on the target host.
- Confirm the exact MyCobot 280 M5 vendor asset source and license before
  Phase 1 import.
- Confirm that installed cuRobo metadata reports version `0.8.0` for the pinned
  Git tag; update only the metadata adapter, not the required baseline, if its
  packaging format differs.
