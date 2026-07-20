# CHANGES — MyCobot 280 M5 Constrained Approach Planner

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
