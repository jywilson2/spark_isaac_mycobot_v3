# REFERENCES

References are pinned to the v3 specification's cuRoboV2 baseline. Older
`curobo.org` examples commonly use v0.7.x `MotionGen` APIs and are not
implementation authority for this project.

## Phase 0 implementation libraries

- **NVIDIA cuRobo v0.8.0 tag**  
  <https://github.com/NVlabs/curobo/tree/v0.8.0>  
  Exact planning/runtime baseline.
- **cuRobo v0.8.0 package metadata**  
  <https://github.com/NVlabs/curobo/blob/v0.8.0/pyproject.toml>  
  Confirms distribution name `nvidia-curobo`, Python `>=3.10`, CUDA optional
  dependency sets, and PyTorch `>=2.5` for CUDA 12.
- **Public motion-planner module**  
  <https://github.com/NVlabs/curobo/blob/v0.8.0/curobo/motion_planner.py>  
  Public `MotionPlanner` and `MotionPlannerCfg` imports.
- **Public types module**  
  <https://github.com/NVlabs/curobo/blob/v0.8.0/curobo/types.py>  
  Public `GoalToolPose`, `JointState`, and scalar-first `wxyz` `Pose`.
- **cuRobo v0.8.0 changelog**  
  <https://github.com/NVlabs/curobo/blob/v0.8.0/CHANGELOG.md>
- **PyTorch CUDA semantics**  
  <https://pytorch.org/docs/stable/cuda.html>
- **Python packaging specification**  
  <https://packaging.python.org/en/latest/specifications/pyproject-toml/>
- **pytest markers**  
  <https://docs.pytest.org/en/stable/example/markers.html>
- **Ruff configuration**  
  <https://docs.astral.sh/ruff/configuration/>

Tested Phase 0 host baseline (2026-07-18): Isaac Sim Python 3.12.13,
`nvidia-curobo==0.8.0`, PyTorch 2.10.0+cu130, CUDA runtime 13.0, NVIDIA GB10.
See [`docs/phase0_environment.md`](docs/phase0_environment.md). The observed
compute-capability warning is retained in that report and is not suppressed.

## Phase 1 asset sources

- **Elephant Robotics `mycobot_ros2`**  
  <https://github.com/elephantrobotics/mycobot_ros2>  
  Vendor URDF + meshes. Obtained locally via `scripts/download_mycobot_ros2.sh`
  into gitignored `third_party/`. Staging kinematics/cuRobo URDFs under
  `assets/mycobot_280_m5/urdf/` still require Phase 1 provenance review.

## Phase 7 Isaac Sim libraries / host tooling

- **Isaac Sim host `python.sh` resolution** — `scripts/isaac_sim_env.sh`
- **Container → host delegation** — `scripts/host/spark_host_exec.sh`
- **URDF import workarounds** — `isaac_sim/urdf_utils.py` (package://, COLLADA GUID)
- **Joint drive gains (sim)** — `config/robots/joint_drives.yaml`

## Phase 8 residual RL (planned)

- Training only in Isaac Lab / Isaac Sim; residual must use Phase 5
  `ResidualCorrector` + `SafetyProjector` contracts in `spec.md` §4.6 / §6.5.
- No e2e pose→joints primary policy.

## Phases 9–10 hardware (planned)

- Physical MyCobot 280 M5 via gated adapter; default dry-run; enable flag required
  for live motion. Do not claim hardware accuracy from sim metrics.

