# CHANGES — MyCobot 280 M5 Constrained Approach Planner

## 2026-08-02 — Phase 7.5: geometric-full primary stop + populate metrics

Branch `wip_phase7_5`. Dense random packing was truncated by a short
consecutive plan-failure streak (`n3-0-5` under streak=5).

1. **Primary stop** = `geometric_full` (cannot draw a legal candidate
   within `max_placement_attempts`).
2. **Secondary timeouts** = `max_total_target_failures` (default **25**)
   and optional `max_consecutive_target_failures` (**0** = off). At least
   one timeout must be > 0.
3. Example YAML: `max_consecutive_target_failures: 0`,
   `max_total_target_failures: 25`.
4. Per-episode `tip_contacts` + `populate_s` / `populate_duration_s` in
   populate DONE lines, suite table, bundle extras, replay DONE, and
   `phase7_5_episode_metrics` lines.

### Evidence

- CI: **275** passed, Ruff clean.
- Headless smoke seed-4242: `suite_accepted: true`, artifact
  `phase7_5-variable_dz0_30_n6-4-11_seed4242`, tip=21, body=0, 3/3
  episodes; stop=`total_failures` (25) each episode (not yet geometric
  full under this budget). Populate wall: 660.0 / 687.6 / 695.0 s.

### Needs review

- Episodes still hit the secondary total-failure timeout before
  geometric fullness; raise `max_total_target_failures` later if denser
  packing is required. GUI smoke deferred.

## 2026-08-02 — Phase 7.5 playback wall-time: cache FK / tip sampling

Branch `wip_phase7_5`. GUI/headless motion wall times were ~10–22× planned
duration (Phase 7.2 was ~1×). Cause: incremental mid-path tip checks called
`forward_kinematics()` every physics step, and the default path reloaded
robot YAML + reparsed the URDF (~100 ms/call).

1. Cache `load_robot_model_spec` / URDF parse (mtime-keyed); expose
   `clear_robot_model_caches()`.
2. Playback loads the robot spec once; mid-path tip prefers USD, then
   cached-spec joint FK; planned-trajectory FK fallback also uses the cache.
3. Unit: `test_uncached_forward_kinematics_uses_spec_cache`.

### Needs review

- Headless `n3-0-5` replay after the fix: full suite ~20 s wall (was
  ~849 s); per-leg `motion_s` ~0.6–0.9 s (CPU-stepped, `render=False`)
  vs prior ~60–150 s. GUI will track closer to planned ~5–8 s/leg when
  rendering; confirm visually if desired.

## 2026-08-02 — Phase 7.5 remediation host gates closed (`n3-0-5`)

Branch `wip_phase7_5`. Spec tasks 6–9 + host re-evidence complete.

1. **Retreat distance** — suite default / example YAML
   `retreat_distance_m: 0.10` (0.02 m and 0.05 m left Option B sphere
   penetration on some accepts; fail-closed start clearance).
2. **Suite tolerance** — example YAML `max_failed_episodes: 1` so one
   empty capacity episode does not reject the stress suite.
3. **Playback tip FK fallback** — when mid-path tip samples miss under PD
   lag on retreated legs, classify tip contact from planned-trajectory FK
   tip-at-face (imports hoisted for Ruff I001).
4. **Host headless** (`smoke_phase7_5_variable_dz_0_30.sh --headless
   --root-seed 4242`): `suite_accepted: true`, achieved **`n3-0-5`**,
   tip=8, body=0, `candidate_failures` lengths `[10, 5, 14]`, corridor
   counters `[2, 0, 4]`; artifact
   `phase7_5-variable_dz0_30_n3-0-5_seed4242`.
5. **Host GUI** replay of that frozen bundle: tip=8, body=0,
   `lighting_ready`, `joint_playback_completed`, exit 0.
6. **CI** — `./scripts/run_verification.sh ci` → **272 passed**, Ruff
   clean. Docs (`spec.md`, `STATUS.md`, phase report, README,
   `implementation_phases.md`) mark Phase 7.5 complete.

### Needs review

- Two-call retreat (vs one-shot `plan_grasp_to_lift`) remains a
  documented cuRobo-owned deviation from the original retract-segment
  wording.
- Corridor FK cost grows with accepted legs × waypoints (observed
  acceptable on `n3-0-5`).
- Empty episodes are capacity noise; `max_failed_episodes: 1` is
  intentional for this stress suite.

## 2026-08-02 — Phase 7.5 remediation implemented (retreat / maze / failures)

Branch `wip_phase7_5`. Implements spec tasks 6–9 for the `n1-1-1` defect.

1. **Post-contact retreat** — incremental legs use two fresh `plan_grasp`
   calls (contact with `plan_grasp_to_lift=False`, then contact-start
   retreat with `plan_approach_to_grasp=False` and signed
   `retreat_distance_m`). Host-tuned default became **0.10 m** (see
   closure entry above). Fail-closed FK start-clearance before every
   post-acceptance plan. Fixed-mode stays approach-only. One-shot
   `plan_grasp_to_lift=True` failed systematically on host GPU.
2. **Corridor (maze) pre-filter** — accepted-leg FK sphere clouds retained;
   candidates that intersect any prior corridor are non-counting geometric
   rejects (`GeometricRejectCounts.corridor`). Host wires
   `waypoint_spheres_fn` via cuRobo `compute_kinematics`.
3. **Candidate failure records** —
   `incremental_episodes[*].candidate_failures` with category/reason/
   planner status/timing/streak; populate FAIL lines render
   `(category: reason)`.
4. **Playback** — incremental mode accepts mid-trajectory tip evidence
   (PhysX or geometric face reach) because legs end retreated.
5. **Config / validation** — `retreat_distance_m` on suite config and
   example YAML; `validate_retreat_segment` for joint limits + world
   clearance on the retract path (active cube excluded).
6. **Tests** — remediation unit coverage; `./scripts/run_verification.sh
   ci` → **272 passed**, Ruff clean.

## 2026-08-02 — Phase 7.5 reopened: `n1-1-1` defect diagnosed, remediation specified (docs only)

Branch `wip_phase7_5`. Documentation-only change set; no code modified.
Resolves the open "capacity is low" review item from the entry below.

1. **Diagnosis** (from `phase7_5-variable_dz0_30_n1-1-1_seed4242.bundle.json`
   extras): accepted legs ended at the tip-contact pose while the contacted
   cube stayed in the world, so every post-acceptance plan started at zero
   clearance to a retained obstacle — below the 0.006 m world-clearance
   floor and inside the 0.01 m TrajOpt activation distance. Post-acceptance
   `plan_grasp` failed 15/15 (11–22.7 s each) vs 3/5 from the home start;
   every episode stopped at exactly `streak 5/5`.
2. **`spec.md` §8 Phase 7.5 amendments** — status Reopened; new normative
   sections: *Post-contact retreat* (pinned `plan_grasp` retract segment,
   `plan_grasp_to_lift=True`, new `retreat_distance_m` key default 0.02 m,
   fail-closed FK start-clearance verification), *Retained obstacles and
   in-order navigability* (targets are **never removed** and never
   excluded from the planning world; the growing field must remain a
   navigable maze — the EE must be able to revisit each block in
   acceptance order, so new cubes must not intersect previously recorded
   leg corridors; corridor clearance pre-filter, non-counting `corridor`
   reject counter), and *Candidate failure records*
   (`incremental_episodes[*].candidate_failures` with category, reason,
   planner status, timing, streak; specific reasons required on populate
   FAIL lines). Tasks 6–9 and remediation acceptance criteria added; the
   "no lift waypoints" criterion reworded to permit only `plan_grasp`'s
   documented approach/retract segments.
3. **`docs/phase7_5_variable_target_stress.md`** — status Reopened; new
   "Defect: only the first target plans" section with the per-episode
   evidence table, root cause, diagnostic-gap note, and the remediation
   list; new "maze framing" subsection explaining both directions of the
   navigability invariant; config table and sampling example updated.
4. **`STATUS.md`, `README.md`, `docs/implementation_phases.md`** — Phase
   7.5 status Reopened with the remediation summary; STATUS next steps now
   lead with spec tasks 6–9 and the remediated smoke rerun before Phase 8.
5. `docs/last_prompt.md` updated (this prompt and the 10:55 investigation
   prompt).
6. **`.cursor/rules/10-curobo-v080.mdc`** — clarifying note under the
   approach-only bullet: Phase 7.5 incremental legs are approach+retreat
   and set `plan_grasp_to_lift=True` (the documented retract segment);
   fixed-mode Phase 7.2–7.4 legs stay approach-only. (File was root-owned
   from a container-side edit; replaced in place, now owned by the host
   user like its siblings.)

### Needs review

- `plan_grasp_to_lift=True` for incremental-mode legs intentionally varies
  the approach-only guidance in `.cursor/rules/10-curobo-v080.mdc`; the
  rule now documents the exception explicitly (item 6). Phase 7.5 legs are
  approach+retreat, not approach-only, so this stays cuRobo-owned.
- `retreat_distance_m` default 0.02 m is a documented estimate (2× the
  0.01 m pre-approach). Whether it clears the Option B world-cover sphere
  model by ≥ 0.006 m must be confirmed by the fail-closed FK check on the
  first remediated GPU run.
- The corridor pre-filter cost grows with recorded legs × waypoints;
  acceptable as deterministic CPU FK, but worth timing on the remediated
  smoke.

## 2026-08-02 — Phase 7.5 implemented: incremental Z-density stress suite

Branch `wip_phase7_5`. Adds `target_population: incremental` so suites
measure how many targets can be placed and tip-contacted under a designated
Z-density instead of requiring a fixed-count field.

1. **Config fail-closed matrix** in `load_multi_target_suite_config`:
   incremental forbids `target_count` / `order` / deferral / regen / tip-IK
   keys; requires `placement: random` and `retain_targets_after_contact:
   true`; new keys `max_consecutive_target_failures` (default 5),
   `max_total_target_failures`, `max_targets_per_episode`,
   `min_targets_per_episode`.
2. **`IncrementalPopulationRunner`**
   (`mycobot_curobo.incremental_population`): sample → geometric pre-filters
   → one `plan_grasp`+validate → accept (retain cube, advance joints) or
   count a streak failure; stops on consecutive/total/geometric-full/max
   caps; `insufficient_targets` below the min floor.
3. **Normative console tags** `phase7_5_populate:` / `_sampling:` /
   `_episode:` / `_suite:` / `_replay:`; artifact naming
   `{prefix}_dz{w}_n{N1-N2-…}_seed{S}`; bundles marked
   `target_population: incremental`.
4. Example `config/phase7_5_variable_targets_dz_0_30.yml` and host smoke
   `scripts/host/smoke_phase7_5_variable_dz_0_30.sh`.
5. Unit tests: `tests/unit/test_phase7_5_variable_targets.py` (22 passed);
   `./scripts/run_verification.sh ci` → 265 unit + Ruff clean.
6. Host headless smoke (`--root-seed 4242`): suite accepted `3/3`, achieved
   counts `n1-1-1`, tip=3, body=0, artifact
   `phase7_5-variable_dz0_30_n1-1-1_seed4242`; GUI replay of the frozen
   bundle exit 0 (`lighting_ready`, tip=3, body=0).

### Needs review

- First `dz0_30` host smoke achieved only **1 target per episode** before
  five consecutive planner failures. Mechanically correct (oracle + streak
  stop), but capacity is low vs the motivational example (`n14-11-16`).
  Worth a follow-up look at candidate distribution / advisory reach
  prefilter aggressiveness — not a gate failure.

## 2026-08-02 — fix(phase1.1): repair two pre-existing unit test failures

Branch `wip_phase1_1_test_fixes` (from `main` @ `516c93d`). Both tests
failed identically at `origin/main` before this change; neither failure
was introduced by the Phase 7.4 / 7.5 landings.

1. `load_robot_model_spec` now validates `format_version` and
   `joint_names` **before** merging the collision sphere overlay. Overlay
   merging reads a sibling file, so config copies (pytest tmp dirs)
   previously died with "collision sphere overlay not found" instead of
   the specific schema error. Fixes
   `test_config_rejects_silent_joint_reordering`.
2. `scripts/host/write_option_b_trial_app.py` falls back to an absolute
   `robot_config_path` when the trial robot output lives outside the repo
   root (`Path.relative_to` cannot express such paths;
   `load_app_config`'s repo-root join leaves absolute paths untouched).
   Fixes `test_write_option_b_trial_app_arms_dual_overlay`.

### Verification

- `pytest tests/unit -q`: 243 passed (previously 241 passed, 2 failed).
- `ruff check .` and `ruff format --check .`: clean.

## 2026-08-02 — Phase 7.4 closed as partially functional; Phase 7.5 approved (docs only)

Decision 2026-08-02: **Phase 7.4 is left in its current state — partially
functional — with no further remediation planned.**

1. What works: Z band configuration, Z-aware EE floor, dexterous-reach
   screening, and the full goal-feasibility amendment (world-aware tip-IK
   screen, `order: z_desc`, goal-set rolls, regen on `targets_unplanned`),
   all unit-tested; default/mid-width band suites pass.
2. Evidence gap accepted as-is: one post-amendment 3×15 `delta_z_m: 0.30`
   headless run reached configured acceptance (2/3 episodes,
   `max_failed_episodes: 1`, 5 field regens; `place_wall_s` median ≈ 3.9 s).
   GUI **play** of that accepted bundle exited 0 (`lighting_ready`,
   `joint_playback_completed`); full `--gui` plan+play was SIGKILL'd mid
   tip-IK. The 2×15 / 2×20 variants remain unproven and per-episode results
   stay lottery-like. The amendment acceptance criterion requiring those
   suites to pass is **waived** in `spec.md` §8 Phase 7.4; the wide-band
   stress goal moves to Phase 7.5.
3. All remaining Phase 7.5 review items approved (failure-threshold
   default 5, `min_targets_per_episode` 1, one-attempt-per-candidate,
   bundle-authoritative determinism, artifact naming, sample-σ
   statistics). Phase 7.5 entry criteria are satisfied; implementation may
   begin on `wip_phase7_5`.
4. Updated: `spec.md` (§8 Phase 7.4 status/boundary + waived criterion),
   `docs/phase7_4_z_variability.md`, `docs/implementation_phases.md`
   (7.4/7.5 rows, section statuses, 7.5 entry criteria), `README.md`,
   `STATUS.md` (headline, roadmap table, next steps), `CHANGES.md`,
   `docs/last_prompt.md`.

### Verification

- Docs-only change set; no code, config, or test changes.

**Follow-up (2026-08-02):** added an explicit accuracy note to `STATUS.md`
("2026-08-02 accuracy note (Phase 7.4 closure)") stating that "partially
functional" describes the wide-band **evidence gap** (GUI and 2×15 / 2×20
unproven; results lottery-like across seeds), not broken machinery — the
implemented behavior is unit-tested and the 3×15 headless smoke did reach
configured acceptance once.

---

## 2026-08-02 — Phase 7.5 specified: variable-target-count Z-density stress suite (docs only)

New phase specification (`spec.md` §8 Phase 7.5, new report
`docs/phase7_5_variable_target_stress.md`, roadmap/README/STATUS updated).
Implementation pending on `wip_phase7_5`.

1. **Incremental population** (`target_population: incremental`): targets
   are created, verified, and planned one at a time; each candidate gets
   exactly **one** `plan_grasp` attempt (plus Phase 4 validation) from the
   arm's current pose against all previously accepted cubes — the plan
   attempt is the sole feasibility authority. Accepted cubes are retained
   (`retain_targets_after_contact: true` mandatory); geometric pre-filters
   are advisory and non-counting.
2. **Configurable stop threshold:** `max_consecutive_target_failures`
   default **5** (~2 min worst-case stop tail at ~22 s per failed
   high-effort attempt; ≤ 3% premature-stop probability at ≥ 50% marginal
   feasibility). Optional `max_total_target_failures`,
   `max_targets_per_episode`; acceptance floor `min_targets_per_episode`
   (default 1, `insufficient_targets` on violation).
3. **Fail-closed key matrix:** in incremental mode `target_count`, `order`,
   deferral/reconsider/consecutive-unplanned budgets, field regen, tip-IK
   screen, and reach budgets must be absent; `placement` must be `random`.
4. **Suite naming:** configs carry the Z-density label
   (`phase7_5_variable_targets_dz_0_30.yml`); generated artifacts embed
   achieved counts and seed
   (`{prefix}_dz{width}_n{N1-N2-...}_seed{root_seed}`).
5. **Normative human-readable console output:** fixed `phase7_5_*:` tags;
   per-candidate lines show progress (`accepted N`) and stop proximity
   (`streak k/K` + plain-English remaining count); aggregated sampler
   stats; episode/suite summaries with stop reason, Z range, wall time,
   and mean s/target; no raw dict dumps as primary output.
6. **Determinism exception documented:** the accepted field depends on
   planner outcomes, so the frozen bundle (marked
   `target_population: incremental`) is the replay authority; a seed
   reproduces the candidate stream, not necessarily the accepted field.
7. **Playback order pinned (2026-08-02 follow-up):** playback replays the
   frozen bundle's legs in acceptance order with the recorded
   trajectories; legs are kinematically chained, so reordering is
   structurally impossible.
8. **Timing/statistics output pinned (2026-08-02 follow-up):** population
   and replay output must state the Z-distribution (type, band width,
   bounds) per episode, the total accepted target count, the per-target
   planning time (recorded per leg in the bundle), and the mean + sample
   standard deviation of accepted-target planning times per episode and
   suite-wide (`σ = n/a` below two targets); replay restates recorded
   values only and never re-plans. New `phase7_5_replay:` tag.

### Review recommended

- Default `max_consecutive_target_failures = 5` and acceptance floor
  `min_targets_per_episode = 1` (stress suites should set explicit
  floors).
- One-attempt-per-candidate policy (no per-candidate retry) — fresh draws
  replace retries by design.
- Phase 7.5 measures greedy packing capacity, not adversarial fixed-field
  solvability; Phase 7.4 fixed-count suites remain the instrument for the
  latter.

### Verification

- Docs-only change set: `spec.md`, `docs/phase7_5_variable_target_stress.md`
  (new), `docs/implementation_phases.md`, `README.md`, `STATUS.md`,
  `CHANGES.md`, `docs/last_prompt.md`. No code, config, or test changes.

---

## 2026-08-02 — z_desc numeric-id fail-closed rule pinned (docs only)

`spec.md` §8 Phase 7.4 and `docs/phase7_4_z_variability.md`: under
`order: z_desc`, target ids must parse as integers for the tiebreak; a
non-numeric id (possible only in manual lists) is a `ConfigurationError`,
never a silent fallback to string ordering. Added to the amendment's
planned unit tests.

---

## 2026-08-01 — Phase 7.4 goal-feasibility amendment implemented

1. World-aware tip-IK screen (`CuroboTipIkScreen(center, accepted_centers)`).
2. After packing, full-field **omit-self** tip-IK revalidation (matches
   planner omit-active); failed fields redraw under `max_ik_rejections`.
3. `OrderPolicy.Z_DESC` / `order: z_desc` (tallest-first, id tiebreak).
4. Densest `delta_z_m: 0.30` YAMLs use 8-roll goal sets + `z_desc`.
5. Field regen also triggers on `targets_unplanned`.
6. `max_consecutive_unplanned_targets` default
   `max(3, ceil(target_count/3))` (`0` still disables tracking).
7. Placement logs `tip_ik_wall_s` and `place_wall_s` per accepted target.

### Verification

- Unit: world-aware accepted-centres arg, omit-self redraw, z_desc order,
  regen on `targets_unplanned`, scaled consecutive default, stress YAML rolls.
- Host headless 3×15 densest: **suite_accepted** (2/3, tip=43, body=0,
  `field_regenerations: 5`); `place_wall_s` median ≈ 3.9 s / mean ≈ 6.7 s
  (core <60 s). Densest YAMLs set `max_consecutive_unplanned_targets: 10`
  and `max_field_regenerations: 5`.
- Host GUI: play of the accepted headless bundle exited 0
  (`lighting_ready`, `joint_playback_completed`). Full `--gui` plan+play
  was SIGKILL'd mid tip-IK in this environment.

---

## 2026-08-01 — Phase 7.4 goal-feasibility amendment specified (docs only)

Root-cause analysis of the failing `delta_z_m: 0.30` smokes (3×15 headless,
root seed 72152210): episodes 2 and 3 failed with cuRobo "Start or End
state in collision" on every failed attempt — **goal states** colliding
with neighbor cubes (episode 2 planned one target from the same home start
that others failed from; zero-config FK clears the nearest cube by ≈63 mm).
Episode 1's failures were genuine high-Z optimization difficulty
(z ≥ 0.22 m, at/above the measured success ceiling). The empty-world tip-IK
screen cannot catch goal-vs-neighbor collisions, and the field-regen path
was unreachable with `max_consecutive_unplanned_targets = 0`
(`field_regenerations: 0`).

Amendment specified in `spec.md` §8 Phase 7.4 ("Goal-feasibility
amendment") and `docs/phase7_4_z_variability.md`; **implementation
pending**:

1. World-aware tip-IK pre-screen (accepted cubes as obstacles;
   `TipIkScreen.__call__(center_m, accepted_centers_m)`).
2. New `OrderPolicy` value `z_desc` (tallest-first contact order, id
   tiebreak) for wide-band suites.
3. Wide-band suites must use goal-set rolls
   (`roll_candidates_deg: [0, 45, …, 315]`, existing key) instead of
   `fixed_roll_rad: 0.0`.
4. Field regeneration also triggers on `targets_unplanned` (same
   `max_field_regenerations` budget).

Deferred, documented in the phase report: Z-aware floor clamp revision
(additive term caps at `pre_approach_distance_m` = 1 cm) and
`R_wrist_max_m` recalibration against the measured success surface.

Also fixed a stale README bullet that still described the superseded
arm-reach substitute retries as current and the first (dexterous-reach)
amendment as unimplemented.

### Review recommended

- Confirm the `z_desc` determinism rule (ascending numeric id tiebreak;
  `order_seed` recorded but unused) before implementation.
- Confirm 8 rolls is the intended goal-set size for the `delta_z_m: 0.30`
  YAMLs (IK/goal-bank cost grows with the set).

### Verification

- Docs-only change set: `spec.md`, `docs/phase7_4_z_variability.md`,
  `README.md`, `STATUS.md`, `CHANGES.md`, `docs/last_prompt.md`. No code,
  config, or test changes; smokes unchanged.

---

## 2026-08-01 — Consecutive-unplanned default 0; densest 3×15 suite

1. `max_consecutive_unplanned_targets` default is **`0`** (no maximum /
   tracking off). Negative values are rejected; positive values keep abort +
   optional field regen.
2. New densest suite `phase7_4_multi_target_standard_3x15_delta_z_0_30.yml`
   (3 episodes × 15 targets) with `max_failed_episodes: 1`.

### Verification

- Unit: default loads as 0; disabled path uses `targets_unplanned`.
- Host headless 3×15 densest: **0/3** (`targets_unplanned` each);
  `suite_accepted=false` (3 failures > `max_failed_episodes=1`). GUI skipped.

---

## 2026-08-01 — Densest suites use planning_high_effort (solution 3)

1. `phase7_4_multi_target_standard_2x15_delta_z_0_30.yml` and
   `…_2x20_delta_z_0_30.yml` switch `planner_profile` from
   `benchmark_reproducible` to `planning_high_effort`.

### Verification

- Host headless 2×15 densest + `planning_high_effort`: placement OK; all 3
  field regens used; still **0/2**
  (`max_consecutive_unplanned_targets_exceeded`). Slightly more tip contacts
  than benchmark (4 vs 2 on final attempt) but not suite-passing.

---

## 2026-08-01 — Tip-IK placement screen + field regen on consecutive-unplanned

1. **Tip-IK pre-screen** (`tip_ik_screen.CuroboTipIkScreen`): empty-world
   flange-normal tip IK via cuRobo `ik_solver.solve_pose`. Suite keys
   `require_tip_ik` (default false) and `max_ik_rejections` (default
   **`target_count` per episode**). Host `plan_multi_target_suite` enables
   the screen when `require_tip_ik` is true; CPU placement-only generation
   skips it with a log line.
2. **Field regeneration** after `max_consecutive_unplanned_targets`:
   suite-wide `max_field_regenerations` (default **3**) rebuilds the failed
   episode field with fresh seeds and retries before aborting remaining
   suite planning.
3. Densest 2×20 `delta_z_m=0.30` YAML sets `require_tip_ik: true`,
   `max_ik_rejections: 20000` (match reach-budget pressure; library default
   remains `target_count`), and `max_field_regenerations: 3`.

### Review recommended

- 2×20 placement remained stuck near 19/20 under tip-IK; retest path is the
  new 2×15 densest suite (`delta_z_m=0.30`, `max_ik_rejections: 20000`).
  Geometry easing (`delta_z_m`) still held.

### Verification

- Unit: IK budget default, require_tip_ik fail-closed, field regen retry.
- Host 2×15 headless (`delta_z_m=0.30`, tip-IK on): placement OK (~47s,
  110 IK rejects); field regen used all 3 attempts; suite still failed
  (`max_consecutive_unplanned_targets_exceeded`, 0/2).

---

## 2026-08-01 — Consecutive unplanned-target suite abort

1. Added `max_consecutive_unplanned_targets` (default **3**): after that many
   consecutive deferred targets (each exhausted
   `max_planning_failure_per_target`) without an intervening tip success,
   episode planning fails with
   `max_consecutive_unplanned_targets_exceeded` and remaining suite episodes
   are not planned.
2. Tip contact success resets the consecutive counter.

### Review recommended

- Confirm dense 2×20 smokes abort early rather than burning full reconsider
  budgets when the first three targets all defer.

### Verification

- Unit: consecutive deferrals abort suite; remaining episode skipped.

---

## 2026-08-01 — Phase 7.4 dexterous-reach screening implemented + delta_z=0.30 suite

1. **Wrist-sphere dexterous-reach model** in `target_placement.py`
   (`DexterousReachModel`, defaults from URDF +
   `R_wrist_max_m=0.36` ≥ farthest measured-success ‖W−S‖ in
   `artifacts/workspace/tip_contact_workspace_v1.json`; raised above the
   minimal calibrated bound so densest 2×20 packs stay feasible under
   Z-aware spacing).
2. **Reject-and-regenerate** for `random` / `grid` / `layout`: Z drawn from the
   full requested band; suite-wide `max_reach_rejections` (default
   `target_count × episode_count`) fails generation closed with episode +
   centre; rejections + `generation_duration_s` recorded in the plan bundle
   (`placement_generation`) and streamed as `phase7_4_placement:` lines.
3. Placement-only streamer:
   `scripts/host/generate_phase7_4_placement.sh` /
   `scripts/generate_multi_target_placement.py`.
4. New 2×20 stress suite `delta_z_m: 0.30`:
   `config/phase7_4_multi_target_standard_2x20_delta_z_0_30.yml` /
   `scripts/host/smoke_phase7_4_standard_2x20_delta_z_0_30.sh`
   (`arm_z_motion_range_m: 0.28`, `max_reach_rejections: 20000`).
5. Docs: PhysX prohibited **body–target** contact already fails the episode
   (Phase 7.2); restated in `docs/phase7_4_z_variability.md` for 7.4 smokes.
6. Unit coverage updated in `tests/unit/test_phase7_4_z_variability.py`.
7. Smoke script
   `scripts/host/smoke_phase7_4_standard_2x20_delta_z_0_30.sh` pins its own
   `SPARK_PHASE7_2_{REPORT,BUNDLE}` paths so a prior loop’s env cannot
   redirect artifacts (e.g. leftover `dz40_gui_loop` names).

**Needs review:** Wide bands still reject many high-Z draws (upper band can be
geometrically unreachable under the wrist model once the base keep-out is
applied); authors must size `max_reach_rejections` for stress suites.
`R_wrist_max_m=0.36` is intentionally looser than the minimal measured
farthest-success bound (~0.3116 m) for densest-pack feasibility.

---

## 2026-08-01 — Phase 7.4 spec amended: dexterous-reach screening (docs only)

Root cause analysis of the `delta_z_m: 0.40` 2×20 GUI loop (pervasive
`plan_failed` legs, including from the home start) showed the axis-separable
arm-reach screen admits jointly unreachable targets: the band samples
z ∈ [0.08, 0.48] m while the measured tip-contact workspace
(`artifacts/workspace/tip_contact_workspace_v1.json`) records 93% success at
z = 0.10 m, 58% at z = 0.22 m, nothing above.

1. **Spec (`spec.md` §8 Phase 7.4):** "Arm-reach validation and substitute
   retries" superseded by "Dexterous-reach screening and target
   regeneration": wrist-sphere reach model
   (`‖W − S‖ ≤ R_wrist_max_m − reach_margin_m`, wrist point
   `W = p_face + L_wrist_to_tcp_m·n̂`, shoulder `S` at `shoulder_height_m`)
   with URDF-declared `dexterous_reach` parameters and a calibration unit
   test against the measured workspace artifact (no measured-success sample
   may be rejected).
2. **Reject-and-regenerate:** wide bands stay unclamped and may exceed the
   dexterous space; random Z samples the full band; out-of-reach centres are
   rejected and regenerated until each field holds the full `target_count`.
   Suite-wide budget `max_reach_rejections` (default
   `target_count × episode_count`) fails generation closed when exceeded;
   rejections are recorded in suite records for replay.
3. **Considered and rejected:** a via-home `plan_cspace` relocation retry —
   upward inter-target motion is expected to remain an emergent property of
   cuRobo optimization; no lift waypoints are injected.
4. Docs updated: `spec.md`, `docs/phase7_4_z_variability.md`, `README.md`,
   `STATUS.md`. Implementation landed in the following CHANGES entry.

---

## 2026-08-01 — Phase 7.4 Z variability + Z-aware spacing implemented

1. Suite keys: `z_band_fraction` (default 0.5 ≈ 50% of `arm_z_motion_range_m`),
   optional unclamped `delta_z_m` (full band width), `z_separation_gain`
   (default 1.0, must be ≥ 1.0).
2. Pairwise Z-aware approach-plane floor:
   `edge+flange+clearance + gain*min(Δz_top, pre_approach)`.
3. Arm-reach validation: discard out-of-reach / infeasible centres and
   substitute (≤3 retries per target) then fail suite generation.
4. Example `config/phase7_4_multi_target_widened_z.yml`; unit tests
   `tests/unit/test_phase7_4_z_variability.py`.
5. Widened default grid / `--targets 10` field AABBs for Z-aware nearest-neighbour
   spacing.

**Host evidence:** CI 224 passed + Ruff; densest standard 2×20 GUI
(`--root-seed 4242 --auto-exit`) suite_status 0 — 40 tip / 0 body / 0 plan
fails.

---

## 2026-08-01 — Phase 7.4 specified: extended Z variability (spec only)

1. Spec section drafted on `wip_phase7_4` (later implemented; see entry above).
2. Design notes `docs/phase7_4_z_variability.md`; implementation_phases map.

---

## 2026-08-01 — Phase 1.1 Option B complete; default YAML armed

1. **Re-armed** `config/robots/mycobot_280_m5.yml` with
   `collision_sphere_overlay_path` + `collision_sphere_overlay_role: dual`
   (32 scaffolding + 1012 world-cover spheres).
2. **Planning-time evidence** (host DGX Spark, integration 2×5 seed 4242):
   scaffolding p50/p95 = 4.282 / 6.244 s; dual = 5.000 / 6.993 s; ratios
   **1.168× / 1.120×** vs declared budget ≤ 1.50× / 2.00× → PASS. Script:
   `scripts/host/measure_phase1_1_option_b_plan_timing.sh`.
3. **Armed unseeded 2×20:** 10/10 suite passes; 40 tip / 0 body per run;
   33 planning retries absorbed. Scripts:
   `smoke_phase7_2_standard_2x20_option_b.sh`,
   `run_phase1_1_option_b_unseeded_2x20.sh`, `write_option_b_trial_app.py`.
4. Unit tests updated for armed default (robot_model, inspect, cover,
   GPU integration defaults). Spec / STATUS / phase report / REFERENCES /
   implementation_phases mark Option B complete.
5. Prior gate evidence retained: dual merge, shared `planning_world`,
   integration 2×5 headless+GUI (seed 4242), GPU self-clear/body-clip.
6. Post-re-arm confirmation: headless integration 2×5 with default YAML
   (no trial wrapper, seed 4242) exit 0 — 10 tip / 0 body / 0 plan fails.
   CI 214 passed + Ruff; host GPU Phase 1.1b suite 8 passed.

**Review needed:** Orin AGX plan-time calibration if embedded planning is
in scope; residual RL (Phase 8) may need retraining under denser world spheres.

---

## 2026-08-01 — Option B armed integration 2×5 headless + GUI green

1. Added `--app-config` to `plan_multi_target_suite.py` /
   `smoke_phase7_2_multi_target.sh` so trial robot YAMLs can be selected
   without editing the default robot YAML.
2. New wrapper `scripts/host/smoke_phase7_2_integration_2x5_option_b.sh`
   writes a temporary dual-armed robot + app.yml, runs integration 2×5,
   and cleans up trial files.
3. Host evidence (`--root-seed 4242`): headless and GUI both exit 0 —
   2/2 episodes, 10/10 tip contacts, 0 body contacts, 0 planning failures
   (plan p50 ≈ 4.9–5.1 s). Reports under
   `artifacts/reports/phase7_2_multi_target_integration_2x5_option_b.*`.

---

## 2026-08-01 — Phase 1.1 Option B implementation started

Accepted the dual-role proposal and began implementation on `wip_phase1_1b`.

1. **Diagnosis** (`scripts/host/diagnose_phase1_1_option_a_regression.sh`):
   with Option A `replace`, Phase 7.1 start is clear but tip-contact
   `plan_grasp` fails (`Start or End state in collision` / unreachable goal
   set) because the active cube remains in the planning world. cuRobo
   v0.8.0 `self_collision_ignore` confirmed per-link.
2. **Dual-role merge** in `robot_model.apply_collision_sphere_overlay`
   (default role `dual`): scaffolding spheres stay on real links; dense
   cover attaches to FIXED `*_world_cover` children fully ignored for
   self-collision; active self-pair count preserved. Role `replace` kept
   for diagnosis. Strip `collision_sphere_overlay_role` before cuRobo.
3. **Shared world builder** `mycobot_curobo.planning_world` omits the
   active contact cube; wired into `MultiTargetEpisodeRunner` and Phase
   7.1 tip-contact plan/validate (start preflight and Mode C still keep
   the cube).
4. Unit tests: Option B dual merge, planning-world invariant, named-suite
   dense-radius packing check. GPU Phase 1.1 tests updated for dual role;
   Phase 7.1 GPU tip path uses omit-active.

---

## 2026-08-01 — Phase 1.1 Option B revision drafted (spec proposal; docs only)

Drafted on the new `wip_phase1_1b` branch (cut from `main` after Phase 7.3
completion; spec's earlier note that Phase 1.1 work may share `wip_phase7_3`
is updated accordingly).

1. `spec.md` §8 Phase 1.1: new section **"Proposed revision (2026-08-01):
   Option B — dual-role sphere split"** (status: proposed, not accepted or
   implemented). Scaffolding (32 spheres) remains the only self-collision
   participant; the Option A cover (1012 spheres) is reused unchanged as a
   world-only set, keeping self-collision semantics/cost byte-identical and
   reducing the planning-time question to the linear world-checking term.
   Normative design if accepted: (1) combined sphere list with extended
   ignore map and a fail-closed unit test asserting the active self-pair
   count; (2) cuRobo v0.8.0 ignore-granularity verification from source
   before implementation (virtual child frames as the per-link fallback);
   (3) a single shared world-builder helper enforcing active-target and
   just-contacted exclusions across 7.1/7.2/validation/fixture paths — no
   per-sphere disables or contact carve-outs; (4) a config-time
   dense-sphere-vs-neighbor-face clearance check per named suite.
   Precondition: diagnose the armed-Option-A regression mechanism first
   (not established; Phase 7.2 already omits the active cube). Acceptance
   gate: Option A gate + planning-time criterion + armed unseeded 2×20
   batch within existing (unrelaxed) budgets. Options table row B now
   reads "Proposed 2026-08-01"; status paragraph and the Phase 7.3
   relationship bullet (branch note) updated.
2. `docs/phase1_1_target_scale_collision_spheres.md`: status header points
   to the proposal and branch.
3. `STATUS.md`: Phase 1.1 headline, roadmap row, next-steps item 2, and the
   Phase 1.1 block gain the proposal summary.
4. `REFERENCES.md`: Phase 1.1 entry notes the proposed revision.

**Review needed:** the Option B section is a proposal; accepting it (and
starting implementation) is a separate decision. First implementation task
is the regression diagnosis, not sphere-list changes.

---

## 2026-08-01 — Planning-failure behavior narrative completed (docs only)

1. `docs/phase7_2_multi_target_contact.md` § "Failures — planning, target,
   episode" verified against the design narrative and extended with the
   pieces it lacked in place: (a) a lead-in stating that expected
   infeasibility is a structured result — never an exception, fallback
   planner, or motion — with every tier failing closed; (b) the rationale
   for why deferral works (the world shrinks between passes as contacted
   cubes are removed), citing the unseeded 2×20 batch as the measured
   example; (c) a note that arming a denser Phase 1.1 cover is expected to
   raise planning-failure rates and that this is the correction of false
   negatives (body-clip plans), not a suite regression.
2. Budgets, tier definitions, and normative spec cross-references were
   already accurate and are unchanged.

---

## 2026-08-01 — Planning-time criterion added to the Phase 1.1 re-arming gate

1. `spec.md` §8 Phase 1.1: the Option A acceptance gate now also requires
   measured planning-time evidence before re-arming — (a) overlay-vs-
   scaffolding per-leg plan p50/p95 ratio on the host (same suite, seeds,
   profile), (b) a device calibration run when an embedded planning target
   (e.g. Jetson Orin AGX) is in scope for Phase 10+ (device claims require
   device measurements; host extrapolation must be labeled an estimate),
   and (c) review against a planning-time budget explicitly declared for
   the intended deployment target. No numeric budget is invented ahead of
   the measurements, per project rules.
2. Rationale recorded in the spec: the 1012-sphere cover multiplies
   collision-cost evaluation inside the planner (≈32× spheres for world
   checks; quadratic growth in candidate self-collision pairs), which an
   Orin-class GPU cannot absorb the way the DGX Spark can. The spec also
   names the cost-mitigation alternatives reviewable with the evidence:
   dense cover only in Phase 4 independent validation, or Option B's
   world-only overlay split.
3. `docs/phase1_1_target_scale_collision_spheres.md` verification-gate list
   gains the timing item (marked not yet measured); STATUS Phase 1.1 block
   updated to match.

---

## 2026-08-01 — STATUS Phase 1.1 block aligned with spec (docs only)

1. The lower STATUS Phase 1.1 block still carried pre-decision wording
   (“awaiting approval of revised cover approach”, “Proposed options”,
   “until an option is chosen”), lagging the spec’s
   “Chosen revision: Option A — Chosen / implemented”. Found while
   verifying that conversation answers matched project documentation.
2. Rewritten to the current state: Option A chosen and implemented
   (B/C/D not selected); overlay disarmed because arming regresses
   Phase 7.1 / 7.2 GPU planning; re-arming is gated by the spec’s
   Option A acceptance gate, not by an options decision.
3. No behavior, config, or spec change; STATUS top summary was already
   correct.

---

## 2026-08-01 — Unseeded 2×20 robustness evidence recorded (no code change)

1. Ran the densest 2×20 suite 10× headless, unseeded (each episode drew an
   independent random seed), sequentially on the DGX Spark host
   (2026-07-31 20:32–21:22 PDT). Documentation-only change set: the
   evidence is summarized in the phase 7.2 report ("Standard smoke 2×20"),
   REFERENCES, and STATUS; run artifacts stay local under
   `artifacts/reports/phase7_2_unseeded_2x20/` per the `artifacts/reports/`
   gitignore convention.
2. Results: 9/10 pass; `success_rate` 1.0 with zero target failures and
   zero failed episodes in every reported run; 45 planning retries and
   6 deferrals all absorbed by per-target budgets and reconsider;
   clustering on inner-ring destinations (39/45, worst t17/t20) with
   target 2 the only outer-ring offender (approached from inner starts).
3. Verdict recorded: **no ring-radius nudge warranted** — closes the
   2×20 "review recommended" follow-up and STATUS next-step 2.
4. run_05's exit 1 was an NVIDIA Vulkan driver segfault
   (580.173.02) at headless playback startup after planning succeeded —
   logged in STATUS as a host infrastructure flake to watch, with
   per-episode seeds and frozen bundle recorded for reproduction.
5. STATUS resume section updated: Phase 8 entry criteria verified
   (Isaac Lab 0.54.4 on host, remote CI green); next step is Phase 8 on
   `wip_phase8`.

---

## 2026-07-23 — README streams the demo via a user-attachments inline player

1. The demo video was uploaded through the GitHub web editor, producing the
   permanent asset `github.com/user-attachments/assets/e1632486-…`; the
   accompanying web-editor commit (`749d36d`, direct to `main`) left the
   bare URL above the README title. `wip_phase7_3` was fast-forwarded onto
   that commit to keep the fast-forward-only branch policy intact.
2. README: stray top-of-file URL removed; the Phase 7.2 suite section now
   embeds the asset URL on its own line, which GitHub renders as an inline
   streaming player for the full 1:58 video (replacing the 18 s GIF embed;
   the pasted signed `private-user-images…?jwt=…` variant expires within
   minutes and must not be committed — the canonical asset URL redirects to
   a fresh signed `video/mp4` URL on every view, verified unauthenticated
   with `curl -I`).
3. `docs/videos/` (mp4, GIF excerpt, poster) is retained and documented as
   the canonical offline copy for forks and non-GitHub renderers; the
   Phase 7.3 report records the asset URL and upload date.

---

## 2026-07-23 — Autoplaying GIF preview replaces static poster in README

1. Diagnosis of "click does not start playback": GitHub serves committed
   repository mp4s as `application/octet-stream` with
   `X-Content-Type-Options: nosniff` (verified with `curl -I`) and its
   README sanitizer only renders inline `<video>` players for manually
   uploaded `user-attachments` assets — so no repo-committed link can
   start playback in the browser.
2. New `docs/videos/mycobot_280_m5_2x20_preview.gif` (18 s excerpt,
   t=30–48 s, 720 px @ 10 fps, palette-optimized, 0.76 MB): autoplays
   inline in the README, showing active-target highlight, arm motion, and
   cube removal. Clicking it still opens the full mp4 file.
3. README wording now states the mp4 click offers the file rather than
   streaming; the Phase 7.3 report's incorrect "GitHub renders the mp4
   inline on the blob page" claim is corrected and documents the two
   manual upgrade paths (user-attachments upload or GitHub Pages).
4. Poster jpg retained as a still-frame companion asset.

---

## 2026-07-23 — README demo video as clickable poster image

1. New `docs/videos/mycobot_280_m5_2x20_poster.jpg` (1280-wide frame from
   the demo mp4, ~101 KB) extracted with the Isaac-bundled ffmpeg.
2. README Phase 7.2 suite section now shows the poster as a clickable image
   (`[![…](poster.jpg)](….mp4)`); clicking it opens the mp4 on GitHub,
   which plays it inline on the blob page. The Phase 7 header link points
   to the poster location.
3. `docs/last_prompt.md`: removed two duplicated `# Old prompts:` headers
   introduced near the top by recent turns.

---

## 2026-07-23 — Phase 7.3 demo video committed under docs/videos

1. New `docs/videos/` directory with `mycobot_280_m5_2x20.mp4` (1:58,
   2880×1800 @ 30 fps, H.264, 6.4 MB): GUI playback of the densest standard
   2×20 suite recorded on the DGX Spark host with the smoke wrapper's
   `--record` option.
2. Presented as the completed-Phase 7.3 demonstration in
   `docs/phase7_3_target_placement.md` (new "Demo video" section listing the
   Phase 7.3 machinery visible in the clip: fail-closed placement, shuffle
   order/seeds, ID labels and contact highlights, deferral/reconsider).
3. README: video linked from the Phase 7 header, the implemented-features
   list, and the Phase 7.2 suite-commands section; stale "Not implemented —
   Phase 7.3 (under consideration)" bullet moved to the implemented list.
4. REFERENCES: demo-video entry.
5. Note: the source file was supplied as `~/Videos/mycobot_280_m5_2x10.mp4`,
   but frame and console inspection shows the 2×20 two-ring suite (target
   IDs up to 20), so the committed copy is renamed `…_2x20.mp4` to keep the
   documentation accurate.

---

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
