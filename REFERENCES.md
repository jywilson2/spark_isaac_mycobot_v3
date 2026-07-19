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

## Future Phase 1 asset sources

- **Elephant Robotics `mycobot_ros2`**  
  <https://github.com/elephantrobotics/mycobot_ros2>  
  Candidate source for MyCobot 280 M5 URDF and meshes. No asset is copied into
  v3 until the exact model, revision, license, joint limits, and TCP transform
  are reviewed and recorded.

