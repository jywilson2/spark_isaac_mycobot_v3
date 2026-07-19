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
  Vendor URDF + meshes pinned for Phase 1 to `humble` commit
  `3999e2cda7460d61f4fd2ffaa31049f000eae7a8`. Obtained locally via
  `scripts/download_mycobot_ros2.sh` into gitignored `third_party/`.
- **Elephant Robotics `mycobot_description` BSD-2-Clause license**
  Retained at `assets/mycobot_280_m5/VENDOR_LICENSE`.
- **MyCobot 280 M5 published specifications**
  <https://www.elephantrobotics.com/en/mycobot-280-m5-2023-specificatons-en/>
  Source for the 160 deg/s family maximum. Acceleration and jerk remain
  explicitly documented conservative assumptions.
- **cuRobo v0.8.0 robot format-2.0 example**
  <https://github.com/NVlabs/curobo/blob/v0.8.0/curobo/content/configs/robot/franka.yml>
  Authority for `robot_cfg.kinematics`, c-space, tool-frame, extra-link,
  collision sphere, and self-collision fields.
- **cuRobo v0.8.0 CUDA extras**
  <https://github.com/NVlabs/curobo/blob/v0.8.0/pyproject.toml>
  `cu12` / `cu13` install `cuda.core`; host install intentionally omits the
  `-torch` extra to preserve Isaac Sim's CUDA-enabled wheel.

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

