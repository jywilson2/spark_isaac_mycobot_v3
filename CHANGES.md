# CHANGES — MyCobot 280 M5 Constrained Approach Planner

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
