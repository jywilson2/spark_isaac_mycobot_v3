# REFERENCES

References are pinned to the v3 specification's cuRoboV2 baseline. Older
`curobo.org` examples commonly use v0.7.x `MotionGen` APIs and are not
implementation authority for this project.

cuRobo v0.8.0 is the project's exclusive global and local motion planner.
References for simulators, learned residuals, ROS, hardware adapters, or
validation do not authorize those components to generate replacement paths.

## Phase 0 implementation libraries

## Phase 7.1 implementation references

- **cuRobo v0.8.0 motion-planner result source**
  <https://github.com/NVlabs/curobo/blob/v0.8.0/curobo/_src/motion/motion_planner_result.py>
  Used for the public-result-compatible `interpolated_trajectory` and
  `interpolated_last_tstep` extraction used by the Mode C cuRobo relocation
  adapter.

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
- **Container Ruff bootstrap**
  `scripts/ensure_container_dev_tools.sh` and Cursor rule
  `.cursor/rules/40-container-dev-tools.mdc`. Installs a Ruff-only venv in the
  Isaac ROS / Cursor container without pulling cuRobo/CUDA/Isaac Kit.
  `scripts/run_verification.sh` auto-bootstraps and keeps pytest on the
  system/container Python.
Tested Phase 0 host baseline (2026-07-18): Isaac Sim Python 3.12.13,
`nvidia-curobo==0.8.0`, PyTorch 2.10.0+cu130, CUDA runtime 13.0, NVIDIA GB10.
See [`docs/phase0_environment.md`](docs/phase0_environment.md). The observed
compute-capability warning is retained in that report and is not suppressed.

## Phase 1 asset sources

- **Phase 1.1 target-scale sphere coverage**  
  [`docs/phase1_1_target_scale_collision_spheres.md`](docs/phase1_1_target_scale_collision_spheres.md)
  — Option A thickness-capped cover implemented; disarmed pending 7.1/7.2
  planning reconciliation (see `spec.md` §8 Phase 1.1). Overlay file:
  `config/robots/mycobot_280_m5_phase1_1_spheres.yml`.

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

## Phase 2 task-frame APIs

- **cuRobo v0.8.0 public types**
  <https://github.com/NVlabs/curobo/blob/v0.8.0/curobo/types.py>
  Authority for public `Pose` and `GoalToolPose` imports.
- **cuRoboV2 `GoalToolPose.from_poses`**
  <https://github.com/NVlabs/curobo/blob/v0.8.0/curobo/_src/types/tool_pose.py>
  Defines `[batch, horizon, links, goalset, 3/4]` shape and ordered tool-frame
  conversion. The project calls the class through `curobo.types`.

## Phase 3 nominal-planning APIs

- **cuRobo v0.8.0 `MotionPlanner.plan_grasp`**
  <https://github.com/NVlabs/curobo/blob/v0.8.0/curobo/motion_planner.py>
  Authority for approach-only arguments, separate approach/grasp trajectories,
  goal-set selection, and valid-last-timestep fields.
- **cuRobo v0.8.0 tool-pose criteria**
  <https://github.com/NVlabs/curobo/blob/v0.8.0/curobo/_src/wrap/reacher/tool_pose_reacher.py>
  Internal implementation reference used to diagnose planner-state mutation;
  project code does not import this private module.
- **Project compatibility decision**
  [`docs/phase3_nominal_planning.md`](docs/phase3_nominal_planning.md)
  Records the observed repeated-call and unwarmed-endpoint failures and the
  selected policy: fresh backend, seed reset, configured public warmup, seed
  reset, then exactly one `plan_grasp`, including retries.

## Phase 4 validation APIs

- **cuRobo v0.8.0 `MotionPlanner.compute_kinematics`**
  <https://github.com/NVlabs/curobo/blob/v0.8.0/curobo/motion_planner.py>
  Public FK entry point used by `CuroboTrajectoryEvaluator`.
- **cuRobo v0.8.0 robot kinematics implementation**
  <https://github.com/NVlabs/curobo/blob/v0.8.0/curobo/_src/cuda_robot_model/cuda_robot_model.py>
  Implementation reference for returned tool poses and robot collision spheres;
  project code accesses these through the planner result.
- **Project Phase 4 validation report**
  [`docs/phase4_validation.md`](docs/phase4_validation.md)
  Documents typed reports, configured thresholds, synthetic/GPU coverage, and
  the fail-closed boundary for non-empty-world collision clearance.
- **NumPy**
  <https://numpy.org/doc/stable/>
  Deterministic terminal geometry, rotation, limits, dynamics, and clearance
  metric calculations.
- **PyYAML**
  <https://pyyaml.org/wiki/PyYAMLDocumentation>
  Named validation-profile loading from `config/validation_profiles.yml`.

## Phase 5 execution and residual-safety contracts

- **Python typing protocols**
  <https://docs.python.org/3/library/typing.html#typing.Protocol>
  Structural interfaces for state providers, residual correctors, pose
  evaluators, and command adapters.
- **Project Phase 5 execution report**
  [`docs/phase5_execution_residual.md`](docs/phase5_execution_residual.md)
  Documents the zero-output implementation, deterministic projector,
  fail-closed stop behavior, and no-hardware boundary.
- **NumPy**
  <https://numpy.org/doc/stable/>
  Deterministic residual norm projection, corridor geometry, and command-stream
  identity checks.
- **PyYAML**
  <https://pyyaml.org/wiki/PyYAMLDocumentation>
  Named residual safety profiles in `config/residual_safety.yml`.

## Phase 6 benchmark contracts

- **NumPy random generator**
  <https://numpy.org/doc/stable/reference/random/generator.html>
  Root-seeded deterministic target, normal-cone, start-state, roll-policy, and
  pre-approach sampling.
- **Python dataclasses**
  <https://docs.python.org/3/library/dataclasses.html>
  Immutable benchmark contracts and `dataclasses.replace` for the per-case
  planner seed required by the request/profile invariant.
- **JSON**
  <https://www.rfc-editor.org/rfc/rfc8259>
  Machine-readable reports, frozen compact parameters, and exact failed-case
  replay requests.
- **Project Phase 6 benchmark report**
  [`docs/phase6_benchmark.md`](docs/phase6_benchmark.md)
  Declared unmeasured candidate workspace, stable taxonomy, report semantics,
  replay command, and verification boundary.

## Phase 7 Isaac Sim libraries / host tooling

- **Isaac Sim standalone Python environment**
  <https://docs.isaacsim.omniverse.nvidia.com/latest/python_scripting/manual_standalone_python.html>
  Authority for creating `SimulationApp` before importing Kit-dependent APIs.
- **Isaac Sim Core API**
  <https://docs.isaacsim.omniverse.nvidia.com/latest/py/source/extensions/isaacsim.core.api/docs/index.html>
  `World` and articulation playback runtime used by the host player.
- **OpenUSD physics articulation schema**
  <https://openusd.org/release/api/class_usd_physics_articulation_root_a_p_i.html>
  Used to discover the imported articulation root without assuming its path.
- **Isaac Sim host `python.sh` resolution** — `scripts/isaac_sim_env.sh`
- **Container → host delegation** — `scripts/host/spark_host_exec.sh`
- **URDF import workarounds** — `isaac_sim/urdf_utils.py` (package://, COLLADA GUID)
- **Joint drive gains (sim)** — `config/robots/joint_drives.yaml`
- **Project Phase 7 report** —
  [`docs/phase7_isaac_sim.md`](docs/phase7_isaac_sim.md)

## Phase 7.1 cube-suite contracts

- **Project Phase 7.1 report (implemented)** —
  [`docs/phase7_1_cube_approach.md`](docs/phase7_1_cube_approach.md)
- Reuses Phase 4 geometric/collision metrics, Phase 6 deterministic sampling
  and replay, and Phase 7 playback. Isaac tip metrics deliberately remain
  null/`not_evaluated`.
- Host process split: `isaac_sim/plan_cube_suite.py` (cuRobo only) then
  `isaac_sim/play_cube_suite.py` (Kit only). Sharing cuRobo/Warp with
  `SimulationApp` in one process breaks Kit extension startup on this stack.
- Implemented terminal standoff default `0.08 m` keeps flange collision spheres
  clear of the cube at host-feasible grasp poses (simulation clearance only).
- The 31 mm flange diameter used to derive the default 14 mm cube is an
  unverified design assumption, not a vendor-backed dimensional reference.
  Phase 9 must replace or confirm it with a recorded physical measurement.
- **OpenUSD lighting schemas** —
  <https://openusd.org/release/api/usd_lux_page_front.html>
  Authority for `UsdLux.DomeLight` and `UsdLux.DistantLight` scene lighting.
- **Kit viewport stage lighting mode** —
  `omni.kit.viewport.menubar.lighting` command `SetLightingMenuModeCommand`
  (`lighting_mode="stage"`), action `set_lighting_mode_stage`, and
  `/rtx/useViewLightingMode=false`. Disable
  `/persistent/exts/omni.kit.viewport.menubar.lighting/autoLightRig/enabled`
  **before** `open_stage` when the USD has no lights yet; otherwise Kit posts
  "No lights found" and applies the Default rig that hides later UsdLux prims.
- **OpenUSD collision schema** —
  <https://openusd.org/release/api/class_usd_physics_collision_a_p_i.html>
  Authority for static cube collision geometry; PhysX contact reporting is
  host-only evidence and remains separate from cuRobo validation.

## Phase 7.2 multi-target contracts

- **Project Phase 7.2 report** —
  [`docs/phase7_2_multi_target_contact.md`](docs/phase7_2_multi_target_contact.md)
- Normative acceptance: [`spec.md`](spec.md) §8 Phase 7.2.
- Core orchestration stays Isaac-free (`TargetField`, order/retain policies,
  `MultiTargetEpisodeRunner`, `ContactDetector` protocol). Isaac host code
  implements visualization and PhysX tip/body classification only
  (`isaac_sim/tip_body_contact.py`).
- Failure model: planning failures (per-target retry budget) → target
  failures (episode budget) → episode failures (suite `max_failed_episodes`).
- Hardware transfer surfaces align with remaining adapters: perception as
  `TargetPoseSource`, force/current as `ContactDetector`, online scene update
  as scene-revision feed (Phases 10–11).
- Planning latency logged in 7.2 is sim-host evidence only; Orin AGX budgets
  remain Phases 10–11.
- High-effort bisect helper:
  `scripts/host/bisect_high_effort_profile.py` (one-knob profile variants;
  does not rewrite repo YAML).
- Host CLI seed: `--root-seed N` on `plan_multi_target_suite.py` /
  `smoke_phase7_2_multi_target.sh` (and integration 2×5 / standard 2×10
  wrappers). Omit for an independent random seed per episode; see phase 7.2
  report Host CLI overrides.
- Standard denser suite: `config/phase7_2_multi_target_standard_2x10.yml` via
  `scripts/host/smoke_phase7_2_standard_2x10.sh` (2 episodes × 10 targets).
- Densest suite: `config/phase7_2_multi_target_standard_2x20.yml` via
  `scripts/host/smoke_phase7_2_standard_2x20.sh` (2 episodes × 20 targets;
  two-ring manual field, 14 mm cubes).

## Phase 7.3 target placement

- Design: [`docs/phase7_3_target_placement.md`](docs/phase7_3_target_placement.md)
- Spec: [`spec.md`](spec.md) §8 Phase 7.3 (`random` / `layout` / keep-outs).
- Branch: `wip_phase7_3`.
- CI bootstrap: CPU-safe deps + `SPARK_PYTEST_PYTHON` in
  `.github/workflows/pytest.yml`
  landed on this branch).
- Measured +Z tip-contact workspace map (candidate region, not a dexterous
  claim): `src/mycobot_curobo/tip_contact_workspace.py`,
  `scripts/host/measure_tip_contact_workspace.py`, artifact
  `artifacts/workspace/tip_contact_workspace_v1.json`.
- Flange-face containment: `flange_disk_face_overhang_m` in
  `src/mycobot_curobo/cube_scene.py`; suite keys
  `require_flange_face_containment` /
  `flange_face_overhang_tolerance_m`.

## Phase 8 residual RL (planned)

- Training only in Isaac Lab / Isaac Sim; residual must use Phase 5
  `ResidualCorrector` + `SafetyProjector` contracts in `spec.md` §4.6 / §6.5.
- Residuals are bounded local execution corrections, not planners; no
  replacement trajectory or end-to-end pose→joints policy is permitted.

## Phase 9/9.1 contact-tool sources (planned)

- **OpenSCAD documentation**
  <https://openscad.org/documentation.html>
  Authority for the parameterized source and reproducible STL export workflow.
- **Project Phase 9 requirement report** —
  [`docs/phase9_contact_tool.md`](docs/phase9_contact_tool.md)
- **Project Phase 9.1 requirement report** —
  [`docs/phase9_1_tool_evaluation.md`](docs/phase9_1_tool_evaluation.md)
- Physical flange measurements, fabrication settings, calibration equipment,
  and uncertainty records must be added when those phases are implemented;
  the present documentation does not claim a verified flange dimension.

## Phases 10–11 hardware (planned)

- Physical MyCobot 280 M5 via gated adapter; default dry-run; enable flag required
  for live motion. Do not claim hardware accuracy from sim metrics.

