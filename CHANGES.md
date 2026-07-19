# CHANGES — MyCobot 280 M5 Constrained Approach Planner

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

### Deliberately omitted from v2

- all `isaac_sim/`, Isaac host scripts, USD assets, and GUI smoke machinery;
- ROS 2 packages, hardware scripts, `pymycobot`, and vendor ROS checkouts;
- supervised/RL residual code, datasets, checkpoints, and training reports;
- v2 numerical IK, distance-dependent planner recovery, moving-marker contact
  behavior, and legacy cuRobo `MotionGen` integration;
- v2 configuration, tests, metrics, logs, prompt history, and current-status
  documentation;
- v2 URDF derivatives and collision spheres pending the Phase 1 authoritative
  model/provenance review.

### Review recommended

- Verify the CUDA 12/PyTorch resolver choice on the target host.
- Confirm the exact MyCobot 280 M5 vendor asset source and license before
  Phase 1 import.
- Confirm that installed cuRobo metadata reports version `0.8.0` for the pinned
  Git tag; update only the metadata adapter, not the required baseline, if its
  packaging format differs.

