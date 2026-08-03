# Console and report log key glossary

**Authority:** normative tags and report keys emitted by Phase 7.x plan /
playback / population hosts. Update this file in the **same change set**
whenever a console line tag, key=value token, or JSON report field is
added, renamed, removed, or changes meaning. Cursor rule
[`.cursor/rules/35-isaac-smoke-physx-and-logs.mdc`](../.cursor/rules/35-isaac-smoke-physx-and-logs.mdc)
mandates that update.

Units: meters (m), radians (rad), seconds (s) unless noted. Quaternions
are scalar-first `wxyz` when present in JSON.

---

## Tag families (stdout / smoke logs)

| Tag prefix | Producer | Meaning |
|------------|----------|---------|
| `phase7_2_plan:` | `isaac_sim/plan_multi_target_suite.py`, core runner | Host planning / placement progress for fixed or incremental suites. |
| `phase7_2_playback:` | `isaac_sim/play_multi_target_suite.py` | Kit lifecycle, lighting, viewport, inter-episode field clear, replay-loop control. |
| `phase7_2_physx:` | `isaac_sim/play_multi_target_suite.py` | **Normative** PhysX tip/body contact evidence on the streamed smoke console. Must appear for tip and prohibited body contacts. |
| `phase7_5_populate:` | `mycobot_curobo.incremental_population` | Incremental candidate create/plan progress (BEGIN / per-candidate / mode banner). |
| `phase7_5_sampling:` | `mycobot_curobo.incremental_population` | Geometric pre-filter reject counters for one episode. |
| `phase7_5_episode:` | `mycobot_curobo.incremental_population` | Population episode DONE summary. |
| `phase7_5_suite:` | `mycobot_curobo.incremental_population` | Multi-episode table + artifact base name. |
| `phase7_5_replay:` | playback + format helpers | Frozen-bundle replay BEGIN / per-leg / DONE. |
| `phase7_5_physx_regen:` | host plan pipeline (post-episode PhysX gate) | Discard / retry / exhausted lines for PhysX-failed episodes; must carry sphere-cover diagnostic fields. |
| `phase7_5_episode_metrics:` | plan + play hosts | Compact per-episode `tip_contacts` / `populate_s` echo. |
| `[ep i/n] from->to …` | `format_leg_console_row` | Per-leg plan/valid/contact timings (Phase 7.2 style). |
| `[i/n] targets=…` | `format_episode_console_row` | Per-episode aggregate success row. |
| `Phase 7.2: …` | `format_suite_summary` | Suite success rate + tip/body contact totals. |

---

## `phase7_2_playback:` keys / phrases

| Token / phrase | Meaning |
|----------------|---------|
| `opening Isaac Sim GUI on DISPLAY=…` | GUI mode launch (host DISPLAY). |
| `kit ready` | SimulationApp / world ready. |
| `lighting_ready stage_lighting_mode=…` | Illumination prims + Kit stage-lighting mode check. |
| `GUI viewport settled …` | Framing eye/target after content AABB. |
| `cleared N prior target prim(s) before episode i/n` | Inter-episode / inter-replay full field clear under `/World/Phase7_2/Targets`. |
| `skip episode i (no validated trajectories)` | Episode has no playable legs. |
| `replaying episodes (replay-k)` | `--no-auto-exit` loop pass. |
| `Kit window closed; stopping…` | User closed Kit; exit replay loop. |
| `contact_diag …` | Optional `SPARK_PHASE7_2_CONTACT_DIAG=1` verbose contact dump. |

---

## `phase7_2_physx:` keys / phrases

| Token / phrase | Meaning |
|----------------|---------|
| `TIP CONTACT a->b plan_s=… motion_s=… ttc_s=…` | Allowed tip/EE contact on active target `b`; episode continues (retain or remove per config). |
| `BODY CONTACT a->b plan_s=… motion_s=…` | Prohibited non-tip body–target contact; episode **FAIL** (`body_contact`). Suite fails when `failed_episodes > max_failed_episodes`. |
| `SELF COLLISION DETECTED a->b links=x↔y` | Mid-path robot–robot PhysX contact (non-adjacent links); motion aborts. |
| `SELF COLLISION a->b links=x↔y plan_s=… motion_s=…` | Leg failure for self-collision (`self_collision` / `prohibited_self_collision`). |
| `smoke FAIL self_collisions=N` | Playback exit code forced to 1 whenever any self-collision leg was observed (hard safety; not waived by `max_failed_episodes`). |

Absence of `phase7_2_physx:` lines during a GUI/headless play that reports tip contacts is a monitoring defect. Kit-native PhysX overlap/collide engine messages (spawn-time penetrating statics) must also fail the smoke when detected; see the Cursor rule and `spec.md` §8 Phase 7.2 / 7.5.

Adjacent kinematic pairs from the robot YAML `self_collision_ignore` map are ignored so connected-link proximity does not false-trigger.

---

## `phase7_5_physx_regen:` keys / phrases

Emitted by the **post-episode PhysX acceptance** loop (host plan
pipeline). Normative field table lives in `spec.md` §8 Phase 7.5.

| Token / phrase | Meaning |
|----------------|---------|
| `ep i/n regen a/b DISCARD` | Episode `i` PhysX-failed; discard attempt `a` of budget `b` (`max_physx_regenerations`). |
| `category self_collision` / `body_contact` / `physx_overlap` | Hard-error class that triggered discard. |
| `leg from->to request=…` | Failing leg ids when motion-time. |
| `links x↔y` | Canonical robot link pair (self-collision). |
| `target_id=…` | Body–target contact id when applicable. |
| `waypoint i/N u=… t_s=…` | First offending trajectory sample. |
| `q_rad=[…]` | Joint state at that sample (`JOINT_NAMES` order). |
| `sphere_clearance_m=… sphere_pair=…` | Optional host-side cuRobo sphere clearance at `q_rad` (required when evaluable without Kit). |
| `reason …` | One-line summary for sphere-cover tickets. |
| `RETRY \| next_episode_seed=…` | Regenerating population with advanced seed. |
| `EXHAUSTED` / `physx_regeneration_exhausted` | Budget spent; suite fails closed. |
| `ACCEPT \| physx_regen_attempts=K` | Episode kept after `K` prior discards (may be 0). |
| `launching headless gate … exe=… timeout_s=…` | Gate child launch: resolved Isaac python (`python.sh`, never nested Kit `sys.executable`) and kill budget. |
| `gate play finished exit=… report_exists=…` | Gate child completed; report presence gates the pass path. |
| `gate timeout after Ns; discarding episode (fail closed)` | Gate child exceeded `timeout_s`; process group killed; discard reason `gate_timeout`. |

Bundle mirror: `incremental_episodes[*].physx_discards[]` and
`physx_acceptance` / `physx_regen_attempts` on kept episodes. A discard
line without `links` (or `target_id`), `waypoint`, and `q_rad` is
non-compliant.

---

## `phase7_5_populate:` / sampling / episode / suite

| Line / key | Meaning |
|------------|---------|
| `ep i/n BEGIN` | Start of incremental population for episode `i`. |
| `z-dist uniform dz=… band lo–hi m` | Designated Z-density width and band. |
| `primary_stop geometric_full` | Population ends when geometry cannot yield a legal candidate. |
| `secondary consecutive=… total_fails=…` | Optional plan-failure timeouts (`off` when 0). |
| `accepted N` | Cubes accepted so far in the episode. |
| `cand K` | Candidate serial within the episode. |
| `z=… r=…` | Candidate centre height (m) and radial XY distance (m). |
| `plan OK T s` / `plan FAIL T s (category: reason)` | Single `plan_grasp` (+ validation) outcome; FAIL must show specific category/reason. |
| `streak a/b` | Consecutive failure streak vs threshold (`consecutive stop off` when disabled). |
| `draws` | Geometric sampler draw count. |
| `geometric rejects` | Non-counting pre-filter rejects: `separation`, `keep_out`, `rim`, optional `reach` / `aabb` / `corridor`. |
| `planned` | Candidates that reached the planner. |
| `DONE \| tip_contacts … \| populate_s …` | Episode population complete; tip_contacts = accepted count; populate_s = wall seconds. |
| `stop …` | `geometric_full`, `total_failures N`, `consecutive_failures a/b`, or cap. |
| `fails N total` | Counted plan/validation failures. |
| `min M: PASS\|FAIL` | Floor check vs `min_targets_per_episode`. |
| Suite table columns `ep`, `tip_contacts`, `populate_s`, `stop`, `fails`, `plan_mu`, `plan_sigma` | Per-episode + total row; `artifact` base name. |

---

## `phase7_5_replay:` / `phase7_5_episode_metrics:`

| Key | Meaning |
|-----|---------|
| `targets N` | Accepted field size for the frozen episode. |
| `leg i/n target id` | Replay of accepted leg `i` toward `id`. |
| `recorded plan T s` | Planning duration stored in the bundle (not live replan). |
| `contact …` | Playback contact kind (`allowed_tip_contact`, `prohibited_body_contact`, `none`, …). |
| `tip_contacts` | Tip contacts observed/credited this episode. |
| `populate_s` | Population wall time from the frozen extras (`n/a` if missing). |
| `(recorded)` on µ/σ | Mean/σ of **recorded** plan durations, not live motion. |

---

## Leg / episode console row keys (`format_*_console_row`)

| Key | Meaning |
|-----|---------|
| `plan` / `valid` | Planning succeeded / Phase 4 validation passed. |
| `contact` | `ContactKind` value or `None`. |
| `plan_s` / `motion_s` / `ttc_s` | Plan wall, playback motion, time-to-contact (s). |
| `attempt` | Attempt index within the target’s budget (fixed mode). |
| `failure` | `MultiTargetFailureCategory` value when failed. |
| `targets` / `contacted` / `removed` | Field size; tip-contacted ids; removed ids (0 when retain). |
| `plan_fails` / `target_fails` | Episode planning / target failure counts. |
| `succeeded` | Episode PASS/FAIL boolean. |
| `episode_s` | Episode wall duration when measured. |

---

## Suite summary keys (`format_suite_summary` / JSON `summary`)

| Key | Meaning |
|-----|---------|
| `successes` / `total_episodes` / `success_rate` | Episode pass counts. |
| `tip` / `total_tip_contacts` | Suite tip-contact total. |
| `body` / `total_body_contacts` | Suite prohibited body-contact total (**must be 0** for acceptance under default budgets). |
| `self` / `total_self_collisions` | Suite PhysX robot–robot self-collision leg count (**must be 0**; smoke exit fails if > 0). |
| `failed_episodes` | Episodes that failed. |
| `plan_fails` / `total_planning_failures` | Aggregated planning failures. |
| `target_fails` / `total_target_failures` | Aggregated target failures. |
| `plan_p50` / `plan_p95` | Planning duration percentiles (s). |
| `failure_category_counts` | Histogram of failure categories. |

---

## Frozen bundle / report JSON (selected keys)

| Key | Meaning |
|-----|---------|
| `schema_version` | Report/bundle schema integer. |
| `target_population` | `fixed` or `incremental`. |
| `retain_targets_after_contact` | Within-episode retain policy (`true` mandatory for incremental). |
| `root_seed` / `episode_seeds` / `seed_mode` | Reproducibility metadata. |
| `artifact_base_name` | e.g. `phase7_5-variable_dz0_30_n6-4-11_seed4242`. |
| `suite_accepted` / `fully_succeeded` / `max_failed_episodes` | Acceptance outcome and budget. |
| `incremental_episodes` | Per-episode population extras (stop reason, failures, Z band, `candidate_failures`, `populate_duration_s` / `wall_duration_s`). |
| `trajectories` | Map of `request_id` → joint trajectory for playback. |
| `results[*].episode` | Frozen episode request (field, order, start state). |
| `results[*].legs` | Accepted (and failure) leg records for playback. |
| `results[*].contacted_ids` / `removed_ids` | Playback contact / removal evidence. |
| `z_density` | Designated Z band metadata for incremental suites. |
| `lighting_ready` / `joint_playback_completed` | Playback report booleans. |

---

## Maintenance checklist

When amending log output:

1. Update this glossary (tag table + key rows).
2. Update `spec.md` if the tag or key is normative.
3. Update the applicable `docs/phaseN_*.md` evidence/format notes.
4. Add/adjust unit tests that assert the new format strings.
5. Mention the glossary touch in `CHANGES.md`.
