# Prior-project retirement

Date: **2026-07-19**

## Goal

Stop agent access to the retired prior project tree
(`../spark_isaac_mycobot_v2`) and keep V3 self-contained for Phases 0–11,
including decimal Phases 7.1 and 9.1.

## Access controls

1. Open only [`spark_isaac_mycobot_v3.code-workspace`](../spark_isaac_mycobot_v3.code-workspace).
2. Cursor rule [`.cursor/rules/05-v2-retirement.mdc`](../.cursor/rules/05-v2-retirement.mdc)
   forbids reading or searching the retired tree.
3. The retired tree has a `.cursorignore` that ignores all files if opened by mistake.

Reload the window / reopen the V3-only workspace and start a new agent chat so
the old multi-root context is discarded.

## Migrated into V3

Already present before retirement:

- Isaac host helpers under `scripts/host/` and `scripts/isaac_sim_env.sh`
- URDF / USD import scaffolding under `isaac_sim/`
- Vendor obtain script and staged URDFs under `assets/mycobot_280_m5/`
- Static collision spheres in `config/robots/mycobot_280_m5.yml`

Added at retirement:

- `tests/unit/test_urdf_utils.py`, `tests/unit/test_joint_drives.py`
- `isaac_lab/` detect + versions env, host install/verify scripts
- `scripts/run_verification.sh`, `.github/workflows/pytest.yml`
- `scripts/git_secondary_docs_push.sh`
- `scripts/host/smoke_isaac_viz.sh` Phase 7 placeholder
- Workflow / Isaac host Cursor rules
- `docs/legacy/` historical notes and a final docs/metrics archive

Vendor meshes remain at the independent checkout
`/workspaces/isaac_ros-dev/src/mycobot_ros2`
(revision `3999e2cda7460d61f4fd2ffaa31049f000eae7a8`), linked from
`third_party/mycobot_ros2`.

## Intentionally not migrated

- Residual IK package, ROS 2 workspace, MotionGen / distance-switching recovery
- Prior IK viz player (`run_ik_viz.py`) and tip-omit contact heuristics
- Supervised / SAC training stacks, datasets, and checkpoints
- Prior completion metrics as V3 acceptance claims

Phase 7 must implement a V3-native `NominalPlan` player rather than revive the
prior recovery player.
