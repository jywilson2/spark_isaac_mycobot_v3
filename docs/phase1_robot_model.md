# Phase 1 — MyCobot 280 M5 robot model report

Date: 2026-07-18  
Branch: `wip_phase1`

## Result

**PASS.** The project now has a validated cuRobo v0.8.0 format-2.0 robot
configuration, explicit frame/joint contracts, independent CPU FK, static
collision spheres for every collision link, and a public-API GPU warmup test.
No physical robot command path exists.

## Model and provenance

- Robot: Elephant Robotics MyCobot 280 M5, bare flange
- Vendor repository: <https://github.com/elephantrobotics/mycobot_ros2>
- Vendor branch / commit: `humble` /
  `3999e2cda7460d61f4fd2ffaa31049f000eae7a8`
- Vendor package version: `mycobot_description` 0.4.0
- Vendor license: BSD-2-Clause, retained in
  `assets/mycobot_280_m5/VENDOR_LICENSE`
- Base / flange / TCP: `g_base` / `joint6_flange` / `tcp_link`
- TCP: identity fixed transform at the bare flange. This is explicit and must
  be replaced by measured calibration when hardware carries a tool.

The vendor URDF publishes zero velocity for all six revolute joints. The
committed `mycobot_280_m5_curobo.urdf` replaces only those placeholders with
the published MyCobot 280 maximum speed, 160 deg/s (2.792527 rad/s). Vendor
position limits and transforms are unchanged.

Acceleration (3 rad/s²) and jerk (30 rad/s³) are conservative planning
assumptions because authoritative values were not found. They are marked in
the robot YAML and must not be presented as manufacturer ratings.

## Explicit joint order

1. `joint2_to_joint1`
2. `joint3_to_joint2`
3. `joint4_to_joint3`
4. `joint5_to_joint4`
5. `joint6_to_joint5`
6. `joint6output_to_joint6`

`reorder_joint_state` is the only supported path for reordering named input.
Missing, duplicate, unknown, and non-finite values fail closed.

## Collision geometry

`config/robots/mycobot_280_m5.yml` contains 32 static spheres (four for each
of `g_base`, `joint1` … `joint6`, and `joint6_flange`) derived from v2's
cuRobo mesh fit. Adjacent links are explicitly ignored for self-collision;
`grasp_contact_link_names` is empty.

The reduced sphere set is sufficient for the Phase 1 loader/warmup and ensures
all moving links are represented. **Further review is recommended before
hardware:** visually compare sphere coverage with every vendor mesh and
increase density where needed. Spheres must remain static; they are never
moved to shape a path.

## cuRobo v0.8.0 discrepancy

Passing a project-relative robot YAML string directly to
`MotionPlannerCfg.create` is not portable in v0.8.0: cuRobo resolves it against
its installed `content/configs/robot` directory. The supported project adapter
loads YAML, validates it, resolves `asset_root_path` / `urdf_path` to absolute
paths, then calls:

```python
MotionPlannerCfg.create(robot=load_curobo_robot_config(...), ...)
```

This uses public planner APIs and keeps external path handling deterministic.

## Acceptance evidence

```text
python3 -m pytest tests/unit -q -p no:cacheprovider
29 passed

python.sh -m pytest -m gpu tests/integration -q -p no:cacheprovider
2 passed
```

The GPU test:

- creates `MotionPlannerCfg` with self-collision enabled;
- constructs `MotionPlanner`;
- verifies exact joint/tool frame order;
- warms the planner for one iteration;
- obtains a self-collision configuration with 336 evaluated pairs;
- compares cuRobo default-state TCP FK to independent CPU FK within 1 µm
  numerical tolerance (simulation/model consistency only).

The existing PyTorch warning about GB10 compute capability 12.1 versus the
wheel's advertised maximum 12.0 remains visible. Planner warmup and CUDA
kinematics execute successfully; the warning is not suppressed.

## Runtime dependency correction

cuRobo v0.8.0 requires `cuda.core`. The host installer now selects cuRobo's
`cu13` extra without `-torch`, preserving Isaac Sim's CUDA-enabled PyTorch.
Selecting `cu13-torch` from an unconfigured PyPI index can replace it with a
CPU wheel; this failure was reproduced, repaired, and documented.
