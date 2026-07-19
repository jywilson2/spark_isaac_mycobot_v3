# CHANGES — MyCobot 280 M5 Constrained Approach Planner

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
