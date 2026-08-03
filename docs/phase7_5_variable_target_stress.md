# Phase 7.5 — Variable-target-count Z-density stress suite

**Status:** **COMPLETE** (2026-08-02) — remediation + geometric-full
primary stop host-re-evidenced (`n6-4-11`, tip=21, body=0). Playback
inter-episode clear + PhysX console tags amended 2026-08-03 (GUI loop
re-evidence after clear fix still open). See
[Defect: only the first target plans](#defect-only-the-first-target-plans-2026-08-02).
**Branch:** `wip_phase7_5`.
Normative rules: [`spec.md`](../spec.md) §8 Phase 7.5.
Log keys: [`console_log_keys.md`](console_log_keys.md).

## Implementation

| Piece | Location |
|-------|----------|
| Config key + fail-closed matrix | `mycobot_curobo.multi_target.load_multi_target_suite_config` |
| Candidate sampler | `mycobot_curobo.target_placement.draw_incremental_candidate` |
| Population runner + console/naming | `mycobot_curobo.incremental_population` |
| Host plan/play | `isaac_sim/plan_multi_target_suite.py`, `play_multi_target_suite.py` |
| Example config | `config/phase7_5_variable_targets_dz_0_30.yml` |
| Host smoke | `scripts/host/smoke_phase7_5_variable_dz_0_30.sh` |
| Unit tests | `tests/unit/test_phase7_5_variable_targets.py` |

## Problem

Phase 7.2–7.4 suites draw a **fixed-count** field up front and require every
target to be planned and contacted. At wide Z bands (`delta_z_m: 0.30`)
that made feasibility a whole-field lottery:

- Placement screens (wrist-sphere reach, tip-IK, Z-aware floor) try to
  **predict** the planner; every residual mismatch surfaces at plan time as
  3 × ~22 s failed attempts per target.
- A field with even one unplannable target fails the episode
  (`targets_unplanned`) and triggers whole-field regeneration — a fresh
  draw from the same distribution with roughly the same failure
  probability. Observed cost on the 3×15 `dz0_30` smoke (2026-08-02):
  initial placement 2,156 s with 3,418 tip-IK rejections, failed attempts
  averaging 22.4 s, one episode reaching regeneration 4 of 5 — worst-case
  suite wall time in hours.

## Design (specified)

Invert the flow: **create, verify, and plan targets one at a time** until a
configurable failure threshold says the dexterous space at the requested
Z-density is exhausted. The achieved count is the measurement. The single
`plan_grasp` attempt plus Phase 4 validation is the feasibility oracle;
placement geometry (separation floor, keep-outs, rim, optional reach model)
is demoted to cheap advisory pre-filtering.

| Key | Default | Meaning |
|-----|---------|---------|
| `target_population: incremental` | `fixed` | Selects this suite type; `fixed` keeps Phase 7.2–7.4 behaviour. |
| `max_consecutive_target_failures` | `0` (off) | Optional consecutive plan-failure streak; secondary timeout only. |
| `max_total_target_failures` | `25` | Secondary absolute plan-failure budget (timeout). |
| `max_targets_per_episode` | `0` (off) | Optional acceptance cap. |
| `min_targets_per_episode` | `1` | Acceptance floor; fewer accepted targets ⇒ `insufficient_targets` episode failure. |
| `max_failed_episodes` | `1` (example YAML) | Suite budget; capacity suites may draw one empty episode. Core default remains `0`. |
| `retreat_distance_m` | `0.10` | Post-contact retreat along the outward normal (second `plan_grasp` from contact); next leg starts from the retreated state. |

**Stop policy:** primary stop is **geometric fullness** (random placement
keeps packing until separation/keep-out/corridor/reach cannot yield a
legal candidate). Plan-failure budgets are secondary timeouts so
unplannable-but-legal draws cannot run forever. Per-episode logs/reports
emit `tip_contacts` and `populate_s`.

Loop per episode: sample candidate (XY in field, Z from the designated
Z-density) → geometric pre-filters (non-counting, bounded by
`max_placement_attempts`; exhaustion = geometrically full, the primary
stop), including the corridor clearance check against all recorded leg
corridors → **one** plan attempt from the arm's current (retreated) pose
against all previously accepted cubes → accept (arm advances to the
retreated terminal state, cube retained as a permanent obstacle) or count
one failure with a persisted failure record (arm stays).
`retain_targets_after_contact: true` is mandatory and targets are **never
removed**: retained cubes are obstacles in every later plan, which is the
density stress being measured. No deferral, reconsider, per-target
retries, tip-IK screen, or field regeneration in this mode; the
incremental oracle makes them redundant. Forbidden keys fail closed —
see the spec table.

### The maze framing

Incremental population is essentially the construction of a **navigable
maze**: each accepted cube is a wall segment added to the field, and the
suite is only meaningful if the end effector can still visit every block,
in acceptance order, once the maze is finished. Two hazards follow, one in
each temporal direction:

- **Old blocks must not strand the arm** — every new candidate is planned
  with all previously accepted cubes in the world, so this direction holds
  by construction (an unreachable candidate just fails its plan attempt).
- **New blocks must not cut off old corridors** — playback replays leg `k`
  in the *final* field, but leg `k` was planned against only cubes
  `1..k−1`. A later cube dropped into leg `k`'s swept corridor would turn
  a recorded, validated trajectory into a prohibited body contact at
  playback. The corridor clearance pre-filter (spec: in-order navigability)
  rejects such candidates before they spend a plan attempt, by FK-sweeping
  every recorded leg's collision spheres against the candidate cuboid at
  the `minimum_world_collision_clearance_m` floor. Corridor rejections are
  geometric (non-counting) and appear as the `corridor` counter in
  `phase7_5_sampling:` lines.

Playback contacts exactly the accepted targets in acceptance order, from
the recorded trajectories in the frozen bundle; legs are chained (each
starts at the previous leg's terminal joint state), so the order is
structurally fixed, not a replay option.

### Inter-episode clear and PhysX console (playback)

Each episode is populated from a **clear field**. Isaac playback must
delete **all** prims under `/World/Phase7_2/Targets` before spawning the
next episode (and before each `--no-auto-exit` loop pass). Clearing only
the next episode's ids is wrong: candidate serials are disjoint (e.g.
`n6-4-11` uses ids `{1,5,8,…}` then `{6,7,15,22}`), so leftovers remain
and produce PhysX overlaps that look like “targets not removed between
episodes.”

GUI and headless smokes must keep PhysX tip/body monitoring fail-closed
and print evidence on the streamed console as `phase7_2_physx:` lines.
See [`console_log_keys.md`](console_log_keys.md) and Cursor rule
`.cursor/rules/35-isaac-smoke-physx-and-logs.mdc`.

### What this measures (and what it does not)

- Measures: how many targets the planner can place **and** contact under a
  designated Z-density, per episode — a capacity measurement of the
  dexterous field.
- Does not measure: whether a *given* adversarial fixed field is solvable.
  Greedy construction only builds fields the planner can solve, so
  shadowing traps are avoided rather than confronted. Phase 7.4
  fixed-count suites remain the instrument for that question.
- Determinism shifts: the candidate stream is seed-deterministic, but the
  accepted field depends on planner outcomes (GPU non-bit-stable), so the
  frozen bundle — not the seed — is the replay authority. Bundles are
  marked `target_population: incremental`.

### Suite naming

Config names carry the Z-density label only (count is an outcome):
`config/phase7_5_variable_targets_dz_0_30.yml`. Generated artifacts embed
label, achieved counts, and seed:

```text
{scene_revision_prefix}_dz{width}_n{N1-N2-...}_seed{root_seed}
e.g. phase7_5-variable_dz0_30_n14-11-16_seed4242.bundle.json
```

The console suite summary echoes the same base name.

### Debug output

Normative formats live in the spec; the intent is that any single line
answers "how far along, and how close to stopping":

```text
phase7_5_populate: ep 2/3 BEGIN | z-dist uniform dz=0.30 band 0.08–0.38 m | threshold 5
phase7_5_populate: ep 2/3 | accepted 12 | cand 15 z=0.264 r=0.151 | plan OK 6.4s | streak 0/5
phase7_5_populate: ep 2/3 | accepted 12 | cand 18 z=0.331 r=0.204 | plan FAIL 22.1s (plan_failed) | streak 3/5 — 2 more failures end episode
phase7_5_sampling: ep 2/3 | draws 240 | geometric rejects 198 (separation 118, keep_out 40, rim 36, corridor 4) | planned 42
phase7_5_episode: ep 2/3 DONE | accepted 14 | stop consecutive_failures 5/5 | fails 9 total | z 0.084–0.331 | plan µ=7.1s σ=2.3s | wall 411s | min 1: PASS
```

Bundle replay restates the recorded population facts without re-planning —
per episode: a Z-distribution + total-target header, one line per leg with
the recorded planning time, and a footer with the planning-time mean and
sample standard deviation:

```text
phase7_5_replay: ep 2/3 BEGIN | z-dist uniform dz=0.30 band 0.08–0.38 m | targets 14
phase7_5_replay: ep 2/3 | leg 3/14 target 3 | recorded plan 6.4s | contact allowed_tip_contact
phase7_5_replay: ep 2/3 DONE | targets 14 contacted 14 | plan µ=7.1s σ=2.3s (recorded)
```

Rules: fixed `phase7_5_*:` tags; each episode (population and replay)
opens with the designated Z-distribution (type, band width, bounds);
streak always rendered `k/K` with a plain-English remaining count when
k > 0; geometric rejects aggregated (every ≤ 25 draws and at episode end),
never per-line at default verbosity; planning-time µ/σ computed over
accepted targets only (sample standard deviation, `n/a` below two
targets), reported per episode and suite-wide, with per-leg durations
frozen in the bundle so replay restates them without re-planning; the
episode summary reports mean seconds per accepted target; suite summary is
an aligned table with a totals row plus the artifact base name, with the
machine JSON line last — no raw dict dumps as the primary human output.

## Defect: only the first target plans (2026-08-02)

The first host smoke passed its configured gate (`3/3` episodes ≥
`min_targets_per_episode: 1`) but achieved only **one target per episode**
(`n1-1-1`), far below the motivational example (`n14-11-16`). The frozen
bundle's `incremental_episodes` extras
(`phase7_5-variable_dz0_30_n1-1-1_seed4242.bundle.json`) localize the
failure precisely:

| Episode | Home-start attempts | Post-acceptance attempts | Accepted plan | Failed plans |
|---------|--------------------|--------------------------|---------------|--------------|
| 1 | 1 (OK) | 5 (all FAIL) | 8.4 s | 13–22 s |
| 2 | 3 (FAIL, FAIL, OK) | 5 (all FAIL) | 6.5 s | 11–22.7 s |
| 3 | 1 (OK) | 5 (all FAIL) | 4.8 s | 13–22.4 s |

From the home start, `plan_grasp` succeeded on 3 of 5 candidates. After
the first acceptance it failed on **15 of 15**, each attempt burning
11–22.7 s of the `planning_high_effort` budget across the 8-roll goal set.
At the home-start failure rate (~40%), fifteen consecutive failures have
probability ≈ 4 × 10⁻⁷ — systematic, not hard geometry.

**Root cause.** An accepted leg ended at the tip-contact pose, and the
runner advanced the arm state to that terminal configuration while the
just-contacted cube stayed in the world (retention is mandatory). Every
subsequent plan therefore started with the flange at **zero clearance** to
a retained obstacle — below the `minimum_world_collision_clearance_m`
(0.006 m) validation floor and inside the TrajOpt collision activation
distance (0.01 m) — so the planner could not produce an acceptable
trajectory from that start state. The episode then always ended at exactly
`streak 5/5`. (`leg_world_geometries` has an `exclude_names` escape hatch
for exactly this start-pose collision, but excluding the cube would let
the planner sweep through it and merely move the failure to the PhysX
prohibited-contact gate at playback — it is now explicitly forbidden in
incremental mode.)

**Diagnostic gap.** Failed legs are filtered out of `results[*].legs` and
the extras stored only durations, so no failure category or reason
survived into the artifacts; the diagnosis required correlating timings.
The console log (`/tmp/phase7_5_headless_smoke.log`) had the per-candidate
lines but was transient.

### Remediation (implemented 2026-08-02)

1. **Post-contact retreat** — accepted legs are planned in two fresh
   `plan_grasp` calls: (a) approach+linear contact with
   `plan_grasp_to_lift=False`, then (b) from the contact joint state, a
   second `plan_grasp` with `plan_approach_to_grasp=False` and
   `grasp_approach_offset` = signed `retreat_distance_m` (default **0.10 m**)
   so the tip withdraws to the offset pose along the tool approach axis /
   outward normal. Host smoke showed one-shot `plan_grasp_to_lift=True`
   reaching grasp then failing the linear lift segment systematically;
   0.02 m / 0.05 m retreat left Option B sphere penetration on some
   accepts. The arm advances to the retreated terminal configuration;
   start clearance is FK-verified before each post-acceptance plan.
   Targets remain retained forever. Fixed-mode (7.2–7.4) legs stay
   approach-only.
2. **Candidate failure records** — every counted failure persists
   candidate serial, centre, failure category, reason, planner status,
   plan wall time, and post-failure streak in
   `incremental_episodes[*].candidate_failures`; populate FAIL lines must
   show the specific category/reason.
3. **In-order navigability (maze invariant)** — corridor clearance
   pre-filter so new cubes never intersect previously recorded leg
   corridors (see "The maze framing" above).
4. **Playback tip evidence** — incremental legs end retreated, so tip-face
   geometry at the terminal waypoint is expected to miss. Playback accepts
   tip contact observed mid-trajectory (PhysX or geometric face reach),
   and falls back to planned-trajectory FK tip-at-face when live samples
   miss under PD lag, when no prohibited body contact occurred. Mid-path
   geometric checks must use a **cached** robot spec / USD tip — calling
   default `forward_kinematics()` every physics step reloads YAML+URDF
   (~100 ms/call) and made 7.5 motion wall time ~10–22× planned duration
   vs ~1× on Phase 7.2.

Normative statements live in `spec.md` §8 Phase 7.5 (post-contact retreat,
retained obstacles and in-order navigability, candidate failure records,
`retreat_distance_m` config row, amended tasks 6–9 and acceptance
criteria).

## Evidence / gates

- Unit remediation coverage (corridor non-counting, failure-record
  round-trip, retreated next-start, fail-closed start clearance, retreat
  segment in `NominalPlanner`) plus prior incremental tests: full
  `pytest tests/unit` **272 passed**, Ruff clean
  (`./scripts/run_verification.sh ci`, 2026-08-02).
- **Superseded defect smoke** (`n1-1-1`, pre-remediation): suite accepted
  `3/3` under the old home-start-only behaviour; tip=3, body=0. Kept only
  as the defect signature, not as a Z-density measurement.
- **Remediated headless** (`smoke_phase7_5_variable_dz_0_30.sh --headless
  --root-seed 4242`, 2026-08-02): `suite_accepted: true` with
  `max_failed_episodes: 1` (2/3 episodes; ep2 `insufficient_targets` with
  0 accepts), achieved counts **`n3-0-5`**, tip=8, body=0, artifact
  `phase7_5-variable_dz0_30_n3-0-5_seed4242`. Episodes 1 and 3 each accepted
  ≥ 2 targets (retreated post-contact starts exercised). Bundle
  `incremental_episodes[*].candidate_failures` lengths `[10, 5, 14]`;
  corridor counters `[2, 0, 4]` in sampling lines; populate FAIL lines
  include `(category: reason)`.
- **Remediated GUI replay** of that frozen bundle (`--gui --auto-exit`):
  `lighting_ready`, `joint_playback_completed`, tip=8, body=0, exit 0
  (`artifacts/reports/phase7_5_variable_targets_dz_0_30.gui.json`).
- **Geometric-full primary stop headless** (same smoke script, later
  2026-08-02): `max_consecutive_target_failures: 0`,
  `max_total_target_failures: 25`; artifact
  `phase7_5-variable_dz0_30_n6-4-11_seed4242`; suite accepted 3/3; tip=21,
  body=0. Per-episode `tip_contacts` / `populate_s`: 6 / 660.0 s, 4 /
  687.6 s, 11 / 695.0 s (each stopped on `total_failures`).
- **Inter-episode clear fix (2026-08-03):** playback now clears all
  `/World/Phase7_2/Targets` children before each episode; tip/body
  contacts stream as `phase7_2_physx:`. Re-run GUI loop of the frozen
  `n6-4-11` bundle to close visual evidence after the clear fix.

## Relation to Phase 7.4

Phase 7.4 fixed-count suites and their goal-feasibility amendment remain
valid and unchanged; Phase 7.5 adds a second suite type rather than
replacing them. The wrist-sphere reach model and Z-aware floor are reused
as pre-filters; the tip-IK screen, reach budget, field regeneration, and
deferral machinery simply do not run under `target_population:
incremental`.
