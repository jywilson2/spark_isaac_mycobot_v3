   # MyCobot 280 M5 Constrained Approach Planner

## Cursor Implementation Specification

**Document status:** Initial implementation specification  
**Primary robot:** Elephant Robotics MyCobot 280 M5  
**Exclusive motion planner and motion-planning dependency:** NVIDIA cuRobo **v0.8.0 (cuRoboV2)**\
**Scope:** Deterministic, collision-aware motion planning with a controlled surface-normal approach. The architecture must expose safe extension points for residual reinforcement learning and hardware integration. Phases 0–6 implement the initial planner; Phases 7–10 cover Isaac Sim validation, bounded residual RL, and physical MyCobot 280 M5 testing (see §8 and `docs/implementation_phases.md`).

---

## 1. Purpose

Build a reusable Python project that plans and verifies MyCobot 280 M5 end-effector motion to a target pose while controlling the final approach direction and tool orientation.

The key behavior is:

1. Move from the current joint state through free space to a pre-approach pose.
2. Approach the target along the target surface normal.
3. Maintain the required end-effector orientation during the terminal approach.
4. Reject any trajectory that violates geometric, kinematic, collision, smoothness, or numerical constraints.
5. Preserve a bounded correction interface for a future residual RL policy without allowing that policy to invalidate the nominal cuRobo plan.

This project must replace obstacle-based path shaping and distance-dependent IK switching with explicit task-frame construction, cuRobo trajectory optimization, terminal linear-motion criteria, and independent trajectory validation.

---

## 2. Non-goals for the initial implementation (Phases 0–6)

Do **not** implement the following inside the core `mycobot_curobo` planner path during Phases 0–6. Host-side Isaac scaffolding (scripts/assets) may be staged early for Phase 7, but must remain optional and unused by Phase 0–6 acceptance tests:

- ROS 2 integration
- Isaac Sim or Isaac Lab **runtime** integration in the core package
- Physical MyCobot serial or network control
- `pymycobot`
- Camera, depth, force, tactile, or motor-current sensing
- Reinforcement-learning frameworks or trained policies
- Online learning
- Dynamic obstacle tracking
- Contact-rich manipulation

Phases 7–10 explicitly schedule Isaac Sim, residual RL, hardware dry-run, and physical validation.
- A graphical user interface
- A general task planner

The initial project plans and validates trajectories. It does not command physical hardware.

---

## 3. Fixed technology baseline

### 3.1 cuRobo version

Pin cuRobo to the exact Git tag:

```text
v0.8.0
```

Do not develop against an unpinned `main` branch. Record the resolved package version and source revision at runtime.

cuRobo v0.8.0 is cuRoboV2 and is a major API rewrite. Do not copy code written for the older v0.7.x API.

### 3.2 Required public cuRobo APIs

Prefer stable public imports such as:

```python
from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
from curobo.types import GoalToolPose, JointState, Pose
```

Use these core operations:

- `MotionPlannerCfg.create(...)`
- `MotionPlanner(...)`
- `MotionPlanner.warmup(...)`
- `MotionPlanner.plan_pose(...)`
- `MotionPlanner.plan_grasp(...)`
- `MotionPlanner.compute_kinematics(...)`
- `GoalToolPose.from_poses(...)`
- `JointState.from_position(...)`
- `TrajOptSolverResult.get_interpolated_plan()` where applicable

Do not use these legacy v0.7 patterns in project code:

- `MotionGen`
- `MotionGenConfig`
- `MotionGenPlanConfig`
- `PoseCostMetric.create_grasp_approach_metric`
- imports under `curobo.wrap.reacher`

### 3.3 Python, CUDA, and PyTorch

The cuRobo v0.8.0 package declares Python `>=3.10`. Its CUDA 12 + PyTorch optional dependency set requires PyTorch `>=2.5`.

The project shall:

- use Python 3.10 or newer;
- use an NVIDIA CUDA-capable environment;
- select a PyTorch build compatible with the installed CUDA runtime and GPU;
- fail fast with an actionable diagnostic if CUDA, PyTorch, or cuRobo is unavailable;
- do not fall back to CPU or to another motion planner. A later phase may add
  CPU planning only if the pinned cuRobo implementation itself supports it and
  the project explicitly validates it; no phase may add a non-cuRobo planner.

### 3.4 Dependency policy

Initial application dependencies are limited to:

- cuRobo v0.8.0 and its required dependencies;
- PyTorch/CUDA required by cuRobo;
- NumPy;
- PyYAML;
- pytest for tests;
- ruff for formatting and linting.

Do not add a package merely for convenience. Use the Python standard library where practical.

---

## 4. Design principles

### 4.1 cuRobo exclusively owns motion planning

cuRobo is responsible for:

- inverse kinematics;
- collision checking;
- graph-seeded planning where needed;
- trajectory optimization;
- joint-space feasibility;
- velocity, acceleration, and jerk-aware smoothing;
- selecting among candidate target roll orientations.

No application code, learned policy, simulator, ROS integration, hardware
adapter, external package, or fallback may generate a motion path with a
planner or planning algorithm other than cuRobo. This prohibition applies to
both global and local motion planning. IK used to generate a motion path must
also be cuRobo-owned; independent FK, validation, control, and bounded residual
execution correction are not motion planning and remain permitted only within
their explicit contracts below.

### 4.2 The target is a full task frame, not only a position

Every target shall include:

- target position in the robot base frame;
- unit surface normal in the robot base frame;
- desired tool approach axis;
- a tangent reference or explicit roll angle;
- target-frame orientation;
- pre-approach distance;
- allowed roll candidates, when roll about the normal is not fixed.

### 4.3 The final approach is planned and then independently verified

A cuRobo success result is necessary but not sufficient. Every returned terminal trajectory must be independently checked with forward kinematics and explicit geometric metrics.

### 4.4 No moving collision spheres for path shaping

Collision spheres describe robot geometry and safety margins. They must not be moved dynamically to force a desired approach path.

### 4.5 No planner switching

The same deterministic cuRobo planning pipeline must work across randomized
valid targets. No target, phase, failure mode, runtime condition, or explicit
configuration may switch to another motion planner or path-generating IK
method. Profiles, constraints, seeds, and supported cuRobo APIs may vary, but
cuRobo remains the exclusive motion planner.

### 4.6 Future RL is residual and bounded

The future learning policy will correct execution error around a verified
nominal path. A residual is a bounded local execution correction, not a
replacement trajectory or planning fallback. It must not generate a motion
path, solve target pose to full joint configurations, replace cuRobo planning,
or bypass deterministic safety checks.

---

## 5. Coordinate-frame conventions

All frame conventions must be explicit, documented, and covered by tests.

### 5.1 Required frames

Use at least these conceptual frames:

- `base_link`: robot planning reference frame;
- `flange_link`: final modeled robot flange;
- `tcp_link`: calibrated tool center point used as the cuRobo tool frame;
- `target_frame`: task frame at the desired target;
- `surface_frame`: optional source frame supplied by perception later.

The cuRobo robot configuration must expose `tcp_link` as a tool frame. Do not plan to a visual mesh frame unless it is exactly the calibrated TCP.

### 5.2 Units and quaternion order

- Position: meters
- Joint angles: radians
- Linear velocity: meters/second where Cartesian; radians/second where joint-space
- Quaternion: scalar-first `wxyz`, matching cuRobo examples

### 5.3 Approach-axis convention

The project configuration shall define one of the TCP axes as the physical approach axis, for example:

```yaml
tool_approach_axis: z
tool_approach_sign: -1
```

The sign and axis must be verified by a unit test and a visual or numerical sanity check. Never assume that positive tool Z points toward the target.

### 5.4 Target frame construction

Given target position `p`, outward surface normal `n`, and tangent hint `r`:

1. Normalize `n`.
2. Define the desired approach direction `a = -n`, unless task configuration says otherwise.
3. Align the configured TCP approach axis with `a`.
4. Project the tangent hint onto the plane orthogonal to `a`.
5. Normalize the projected tangent.
6. Form an orthonormal right-handed rotation matrix.
7. Convert it to a normalized `wxyz` quaternion.

If the tangent hint is nearly parallel to the normal, use a deterministic fallback basis axis with the smallest absolute dot product with the normal.

Reject zero-length normals, non-finite values, invalid quaternions, and non-orthonormal rotations.

---

## 6. Core data contracts

Use typed dataclasses or immutable value objects. Do not pass unstructured dictionaries through the core domain layer.

### 6.1 `SurfaceTarget`

Required fields:

```text
position_base_m: vector[3]
surface_normal_base: unit vector[3]
tangent_hint_base: optional vector[3]
fixed_roll_rad: optional float
roll_candidates_rad: tuple[float, ...]
pre_approach_distance_m: float
tool_frame: str
target_id: str
```

Validation requirements:

- all values finite;
- normal magnitude greater than a configured epsilon before normalization;
- pre-approach distance positive and within configured bounds;
- fixed roll and roll candidates mutually exclusive;
- no duplicate roll candidates after angle normalization.

### 6.2 `PlanningRequest`

Required fields:

```text
current_joint_state
surface_target
scene_revision
planner_profile
random_seed
```

### 6.3 `NominalPlan`

Required fields:

```text
selected_goal_index
selected_roll_rad
approach_trajectory
terminal_trajectory
combined_trajectory
planner_status
planner_timings
curobo_version
scene_revision
```

### 6.4 `ConstraintReport`

Include at least:

```text
valid
failure_reasons
max_lateral_error_m
max_orientation_error_rad
max_line_progress_regression_m
terminal_position_error_m
terminal_orientation_error_rad
min_joint_limit_margin_rad
max_joint_velocity_ratio
max_joint_acceleration_ratio
max_joint_jerk_ratio
minimum_collision_clearance_m
sample_count
```

Where a metric cannot yet be computed from an available public API, return `not_evaluated` rather than inventing a result. A plan cannot be marked hardware-ready when required metrics are not evaluated.

### 6.5 Future residual-correction contract

Define an interface but provide only a zero-output implementation:

```python
class ResidualCorrector(Protocol):
    def correction(self, observation: ResidualObservation) -> CartesianResidual:
        ...
```

The initial `ZeroResidualCorrector` always returns zero.

The interface exists to prevent future RL code from being coupled directly into cuRobo planning classes.

---

## 7. Proposed repository layout

```text
.
├── spec.md
├── README.md
├── pyproject.toml
├── .cursorrules
├── .cursor/
│   └── rules/
│       ├── 00-project-core.mdc
│       ├── 10-curobo-v080.mdc
│       └── 20-safety-and-testing.mdc
├── config/
│   ├── app.yml
│   ├── planner_profiles.yml
│   ├── scenes/
│   │   └── empty.yml
│   └── robots/
│       └── mycobot_280_m5.yml
├── assets/
│   ├── mycobot_280_m5/
│   │   ├── urdf/
│   │   └── meshes/
│   ├── usd/                    # generated; gitignored
│   └── logs/isaac_host/        # host Isaac logs; gitignored content
├── isaac_sim/                  # Phase 7+ host helpers (optional for Phases 0–6)
├── src/
│   └── mycobot_curobo/
│       ├── __init__.py
│       ├── version_guard.py
│       ├── config.py
│       ├── errors.py
│       ├── frames.py
│       ├── targets.py
│       ├── goal_set.py
│       ├── robot_model.py
│       ├── planner.py
│       ├── trajectory.py
│       ├── validation.py
│       ├── safety.py
│       ├── residual.py
│       ├── benchmark.py
│       └── cli.py
├── tests/
│   ├── unit/
│   ├── integration/
│   ├── regression/
│   └── data/
├── scripts/
│   ├── verify_environment.py
│   ├── inspect_robot_model.py
│   ├── plan_single_target.py
│   ├── benchmark_random_targets.py
│   ├── isaac_sim_env.sh
│   ├── download_mycobot_ros2.sh
│   ├── convert_urdf_to_usd.sh
│   └── host/                   # DGX Spark / native Isaac Sim launchers
├── docs/
│   └── implementation_phases.md
└── artifacts/
    ├── plans/
    ├── reports/
    └── benchmarks/
```

Generated artifacts must not be committed unless explicitly designated as regression fixtures.

---

## 8. Implementation phases

## Phase 0 — Repository bootstrap and environment verification

### Objective

Create a minimal, reproducible Python project that verifies the required runtime before any planning code is written.

### Tasks

1. Create `pyproject.toml` with a `src/` layout.
2. Pin the project integration to cuRobo v0.8.0.
3. Add pytest and ruff development configuration.
4. Implement `version_guard.py`.
5. Implement `scripts/verify_environment.py`.
6. Print and record:
   - Python version;
   - cuRobo version;
   - PyTorch version;
   - CUDA runtime version visible to PyTorch;
   - GPU name;
   - CUDA availability;
   - selected device and dtype.
7. Fail when:
   - cuRobo is not exactly compatible with v0.8.0;
   - a legacy cuRobo API is detected;
   - CUDA is unavailable;
   - tensors cannot be allocated on the GPU.
8. Add a smoke test that imports `MotionPlanner`, `MotionPlannerCfg`, `GoalToolPose`, `JointState`, and `Pose` from public modules.

### Acceptance criteria

- `pytest` passes without constructing a planner.
- `ruff check .` passes.
- The environment script produces a machine-readable JSON report.
- No ROS, Isaac, MyCobot hardware, or RL dependency is present.

---

## Phase 1 — MyCobot 280 M5 model and cuRobo robot configuration

### Objective

Create a validated cuRobo v0.8.0 robot configuration for the exact MyCobot 280 M5 model and tool configuration.

### Tasks

1. Import or supply the authoritative URDF and mesh assets.
2. Preserve source asset licensing and provenance.
3. Confirm the actuated joint list and order.
4. Confirm joint position, velocity, and acceleration limits from authoritative robot data.
5. Add conservative jerk limits when authoritative values are unavailable; mark them as assumptions.
6. Define:
   - base link;
   - flange link;
   - calibrated TCP link;
   - tool frames;
   - collision link names;
   - mesh link names;
   - default joint position;
   - c-space weights;
   - self-collision buffers;
   - self-collision ignore map;
   - collision spheres for every moving link.
7. Use cuRobo robot-config format version `2.0`.
8. Keep collision-sphere definitions static and version controlled.
9. Set `grasp_contact_link_names` to an empty list unless a later contact task explicitly requires collision disabling.
10. Add an inspection script that prints:
    - active joints in cuRobo order;
    - tool frames;
    - default state;
    - FK pose for the default state;
    - collision-sphere count by link.
11. Add tests for joint-name ordering and limit consistency.
12. Add FK regression fixtures for at least five known joint configurations.

### Design constraints

- Never infer joint order from dictionary order.
- Never silently clamp an input state to make it valid.
- Reject joint states with missing, duplicate, unknown, or reordered names unless an explicit reorder function is called.
- TCP calibration must be represented as a fixed transform and must not be buried in target coordinates.
- Collision buffers must be configurable but not changed dynamically to shape paths.

### Acceptance criteria

- Loading `config/robots/mycobot_280_m5.yml` through the project adapter and
  passing the validated mapping to `MotionPlannerCfg.create(robot=..., ...)`
  succeeds. cuRobo v0.8.0 resolves a relative string against its installed
  content directory, so project-owned paths must be resolved explicitly (see
  `docs/phase1_robot_model.md`).
- The planner can warm up on the GPU.
- FK results are repeatable and within the test tolerance.
- Self-collision checking can be executed for the default configuration.
- No physical robot command is issued.

---

## Phase 2 — Surface target and task-frame generation

### Objective

Convert a target point, surface normal, and optional tangent or roll constraint into one or more valid cuRobo `GoalToolPose` candidates.

### Tasks

1. Implement robust vector normalization and finite-value checks.
2. Implement target rotation construction for each permitted TCP approach axis and sign.
3. Implement deterministic tangent fallback.
4. Implement rotation-matrix validation.
5. Implement quaternion conversion and normalization in `wxyz` order.
6. Implement roll candidate generation around the surface normal.
7. Convert candidate poses to `GoalToolPose` with `num_goalset > 1` when multiple rolls are allowed.
8. Store the mapping from goal-set index to roll angle.
9. Add tests over randomized normals, including near-axis and nearly degenerate cases.
10. Add property tests without introducing another testing dependency; use seeded pytest parameterization and NumPy random generation.

### Default roll strategy

If roll is unconstrained, begin with:

```text
0, 45, 90, 135, 180, 225, 270, 315 degrees
```

Make this configurable. Do not generate an unbounded number of candidates.

### Acceptance criteria

For every generated candidate:

- the TCP approach axis aligns with the desired approach direction within numerical tolerance;
- the rotation determinant is approximately `+1`;
- axes are mutually orthogonal;
- quaternion norm is approximately `1`;
- the target position is unchanged by roll generation;
- candidate ordering is deterministic for a fixed input.

---

## Phase 3 — cuRobo nominal planning

### Objective

Plan a free-space segment to a pre-approach pose and a constrained linear terminal segment to the target.

### Required cuRobo v0.8.0 method

Use `MotionPlanner.plan_grasp(...)` as the required high-level primitive,
configured as an approach-only operation:

```python
result = planner.plan_grasp(
    grasp_poses=goal_set,
    current_state=current_state,
    grasp_approach_axis=config.tool_approach_axis,
    grasp_approach_offset=signed_pre_approach_offset,
    grasp_approach_in_tool_frame=True,
    plan_approach_to_grasp=True,
    plan_grasp_to_lift=False,
    disable_collision_links=[],
)
```

The sign of `signed_pre_approach_offset` must be derived from the configured TCP axis convention and tested. Never copy an offset sign from an unrelated robot example.

In cuRobo v0.8.0, `plan_grasp`:

1. evaluates the supplied grasp goal set;
2. selects a reachable target candidate;
3. constructs an approach pose by applying the configured offset;
4. plans from the current state to the approach pose;
5. applies `ToolPoseCriteria.linear_motion(...)` for the approach-to-target segment;
6. returns separate approach and grasp trajectories.

### Planner creation

Start from a profile similar to:

```python
planner_cfg = MotionPlannerCfg.create(
    robot=robot_config_path,
    scene_model=scene_config_path,
    self_collision_check=True,
    num_ik_seeds=32,
    num_trajopt_seeds=4,
    position_tolerance=0.005,
    orientation_tolerance=0.05,
    use_cuda_graph=True,
    random_seed=project_seed,
    optimizer_collision_activation_distance=0.01,
    max_batch_size=1,
    multi_env=False,
    max_goalset=max_roll_candidates,
)
```

Treat these as an initial profile, not universal constants. Store values in YAML and record them with every plan result.

### Planner lifecycle

- Construct planners through one application-owned factory.
- For the pinned cuRobo v0.8.0 runtime, create a fresh `MotionPlanner` for every
  `plan_grasp` call, including retries. Phase 3 GPU regression testing found
  that reusing one instance mutates optimizer/tool-criteria state and can
  produce shortened trajectories or later failures.
- Before that single call, reset the seed, run configured public
  `MotionPlanner.warmup(...)`, and reset the seed again. Phase 4 endpoint
  validation found that an unwarmed fresh planner could report success while
  remaining at the pre-approach pose.
- Treat planner construction/warmup latency as part of request wall time.
- Do not reuse a planner merely because robot and scene configuration are
  unchanged. Revisit this reliability-first exception only after a future
  pinned cuRobo version passes the same repeated-call regression.
- Use a fixed random seed for reproducible tests.
- Permit a separate seed sweep in benchmarking.

### Result handling

A planning operation is not successful unless:

- the result object exists;
- `result.success` contains a true value;
- `approach_interpolated_trajectory` exists;
- `grasp_interpolated_trajectory` exists;
- the selected goal-set index is valid;
- all required trajectories contain finite values;
- independent validation passes.

Do not concatenate padded or invalid trailing samples. Respect any valid-last-timestep metadata returned by cuRobo.

### cuRobo-only fallback method

Implement a two-call fallback only if `plan_grasp` cannot satisfy a documented
robot-specific case. Both calls must remain inside the pinned cuRobo API:

1. `plan_pose()` to the pre-approach pose;
2. a narrowly scoped terminal plan using cuRoboV2 tool-pose criteria.

Do not reintroduce legacy `PoseCostMetric` APIs or invoke a non-cuRobo planner.
The fallback must pass the same validator and be tagged in the output as a
fallback plan.

### Acceptance criteria

- At least one reachable test target produces both segments.
- Goal-set selection returns the associated roll candidate.
- The terminal segment remains near the target-normal line.
- Repeated calls with the same seed and input are reproducible within defined tolerance.
- Regression evidence confirms each `plan_grasp` call uses a distinct planner instance.
- Planning failures return structured reasons rather than exceptions from normal infeasibility.

---

## Phase 4 — Independent trajectory verification

### Objective

Prove that the terminal segment meets the application’s approach constraints before it is eligible for execution.

### Implemented validation contracts

`validation.py` owns typed `ValidationProfile`, `KinematicCollisionBatch`,
`ValidationViolation`, `ValidationMetrics`, `ValidationReport`, and
`ValidatedPlan` contracts. `CuroboTrajectoryEvaluator` supplies cuRobo FK and
self-clearance data to backend-neutral `validate_nominal_plan`.

### Required checks

For each sampled waypoint in the interpolated terminal trajectory:

1. Compute FK for `tcp_link`.
2. Calculate lateral distance from the ideal normal-approach line.
3. Calculate angular error between actual TCP approach axis and desired approach direction.
4. Calculate roll error when roll is constrained.
5. Verify monotonic progress toward the target, allowing only a small numerical tolerance.
6. Verify joint position limits.
7. Verify joint velocity limits.
8. Verify joint acceleration limits.
9. Verify joint jerk limits when available.
10. Evaluate self-collision and world-collision clearance using a supported cuRobo collision API.
11. Check terminal position and orientation error.
12. Check continuity at the approach/terminal segment boundary.

### Geometric metrics

Let `a` be the unit approach direction, `p_goal` the target point, and `p_i` a TCP position.

Line-lateral error:

```text
|| (I - a a^T) (p_i - p_goal) ||
```

Signed distance to goal along the approach axis:

```text
a^T (p_i - p_goal)
```

Approach-axis angular error:

```text
acos(clamp(signed_tcp_axis_i dot a, -1, 1))
```

Use the configured TCP axis and sign rather than always using TCP Z.

Roll error about the approach axis when roll is constrained:

```text
acos(clamp(projected_actual_tangent_i dot desired_tangent, -1, 1))
```

where the tangent is the next cyclic TCP axis after the configured approach
axis, projected into the plane perpendicular to `a`.

### Initial validation thresholds

Use separate simulation and hardware profiles in
`config/validation_profiles.yml`. For simulation-only initial acceptance:

```yaml
max_lateral_error_m: 0.005
max_approach_axis_error_rad: 0.05236  # 3 degrees
max_roll_error_rad: 0.05236
max_terminal_position_error_m: 0.005
max_terminal_orientation_error_rad: 0.05236
max_progress_regression_m: 0.001
minimum_joint_limit_margin_rad: 0.02
minimum_self_collision_clearance_m: 0.0
minimum_world_collision_clearance_m: 0.0
boundary_position_tolerance_rad: 1.0e-6
```

Empty worlds may report infinite world clearance. Unsupported non-empty world
distance evaluation must remain unevaluated and fail closed. These are starting
values. Do not claim physical MyCobot accuracy from simulation thresholds. The
`hardware_placeholder` profile is a named stub only until Phase 9/10 measures
hardware-specific thresholds.

### Fail-closed behavior

- Any non-finite value invalidates the plan.
- Any unevaluated required safety metric invalidates hardware eligibility.
- Any threshold violation invalidates the plan.
- A failed plan must include machine-readable reasons and the offending waypoint index.

### Acceptance criteria

- Synthetic trajectory deviations are detected by unit tests.
- A deliberately curved terminal path fails lateral validation.
- A reversed-progress path fails monotonicity validation.
- A misoriented TCP fails orientation validation.
- Joint position/dynamics, self-collision, non-finite output, and unevaluated
  world collision failures identify the offending waypoint and fail closed.
- A valid nominal path passes.
- A GPU test validates real cuRobo FK and self-collision clearance in an
  explicitly empty world.
- Non-empty-world collision clearance remains unevaluated and non-executable
  until a supported adapter and obstacle regression are available.

---

## Phase 5 — Execution abstraction and residual-correction seam

### Objective

Create interfaces that allow later hardware and RL integrations without coupling them to the planner.

### Components

#### `TrajectorySource`

Samples the verified nominal trajectory by time or waypoint index.

#### `RobotStateProvider`

Returns measured joint state and timestamps. Initial implementation is a deterministic replay provider.

#### `ResidualCorrector`

Returns a small Cartesian correction. Initial implementation is `ZeroResidualCorrector`.

#### `SafetyProjector`

Projects or rejects residual corrections so that they remain inside a configured Cartesian corridor and joint-space feasibility envelope.

#### `TrajectoryExecutor`

Consumes a verified plan and produces commands through an injected output adapter. Initial adapter writes commands to an in-memory log only.

The Phase 5 executor supports only `ZeroResidualCorrector`. `SafetyProjector`
can explicitly clip or reject synthetic Cartesian residuals for contract
testing, but any non-zero residual that survives projection is rejected before
command emission because no bounded Cartesian-to-joint mapping is accepted in
this phase. A later phase must specify and independently validate such a
mapping; it may not become an IK solver or replacement path generator.

### Hard constraints for future RL

The future residual policy shall not:

- alter the world model;
- modify collision spheres;
- change joint limits;
- disable collision checks;
- choose arbitrary global joint configurations;
- exceed configured translation, rotation, velocity, or acceleration corrections;
- run when the nominal plan is invalid;
- continue after watchdog timeout, stale state, collision risk, or corridor violation.

The safety projector must remain deterministic and outside the learned policy.

### Acceptance criteria

- Replaying with `ZeroResidualCorrector` reproduces the nominal command stream.
- Unsafe synthetic corrections are clipped or rejected.
- A correction cannot move the command outside the terminal approach corridor.
- No physical driver dependency is present.

---

## Phase 6 — Randomized workspace benchmark

### Objective

Measure whether the planning method works consistently when targets are randomized within a declared dexterous workspace.

### Benchmark input

Define a configurable target-sampling region that excludes clearly unreachable or unsafe geometry. Do not label the entire geometric reach envelope as dexterous workspace without measurement.

Randomize:

- target position;
- surface normal;
- target roll constraints;
- start joint configuration from a validated set;
- pre-approach distance within safe bounds;
- planner seed in a controlled seed sweep.

### Benchmark stages

1. A small deterministic smoke set of at least 20 cases.
2. A regression set of at least 100 fixed cases stored as seeds and parameters.
3. An exploratory set of at least 1,000 generated cases.

### Metrics

Report:

- planning success rate;
- validation pass rate;
- success rate by workspace region;
- success rate by surface-normal direction;
- failure category counts;
- selected roll distribution;
- IK/planning time distributions;
- maximum and percentile lateral error;
- maximum and percentile orientation error;
- minimum clearance distribution;
- deterministic-repeat disagreement rate.

Distinguish:

- no reachable IK;
- collision infeasibility;
- trajectory optimization failure;
- terminal-line validation failure;
- orientation validation failure;
- numerical failure;
- configuration/model failure.

### Acceptance criteria

- The benchmark is reproducible from a root seed.
- Every failed case can be replayed from a serialized request.
- Reports are written in JSON and Markdown.
- The benchmark never suppresses failures to inflate success rate.

---

## Phase 7 — Isaac Sim closed-loop visualization and sim validation

### Objective

Visualize and closed-loop-validate Phase 3–6 plans in Isaac Sim on the DGX Spark host without moving planning authority out of cuRobo.

### Tasks

1. Obtain vendor URDF + meshes (`scripts/download_mycobot_ros2.sh`).
2. Convert/import MyCobot into USD using `isaac_sim/` helpers (Isaac 6.x workarounds retained).
3. Play validated `NominalPlan` joint trajectories in headless and GUI modes.
4. Report sim tip / orientation metrics separately from cuRobo planning metrics.
5. Keep Kit/python.sh resolution on the host (`scripts/isaac_sim_env.sh`, `scripts/host/*`).

### Design constraints

- Core `mycobot_curobo` modules must not import Isaac Kit APIs.
- Prefer host execution; container may only delegate via `spark_host_exec.sh`.
- Simulation thresholds are sim metrics only — never claim physical accuracy.

### Acceptance criteria

- Host prereq check finds Isaac Sim `python.sh` and vendor URDF.
- Headless smoke loads the robot and plays at least one validated plan.
- GUI smoke exits 0 when required by the project verification gate for that change set.
- Failures remain fail-closed: invalid plans are not marked executable because Isaac played them.

---

## Phase 8 — Bounded residual RL (Isaac Lab / Isaac Sim)

### Objective

Train and evaluate a residual policy that improves approach metrics under
model mismatch while preserving cuRobo as the exclusive motion planner.

Residual RL is in scope because this architecture already separates nominal planning from a bounded correction seam (Phase 5 / §4.6 / §6.5). End-to-end learned IK is not an acceptable substitute.

### Tasks

1. Implement a non-zero `ResidualCorrector` behind the Phase 5 protocol.
2. Train only in Isaac Lab / Isaac Sim; never command the physical arm during training.
3. Clamp corrections through `SafetyProjector` (Cartesian and/or small joint residual as configured).
4. Compare residual vs `ZeroResidualCorrector` on Phase 6 scenes in simulation.
5. On any validation failure after correction: fall back to nominal plan or no motion.

### Hard constraints

- Deployed path remains: nominal cuRobo plan → optional bounded local execution
  correction → independent validation.
- A residual may not generate a replacement trajectory, invoke a planner, or
  map target pose → full 6-DOF joint solutions in any hardware or simulation
  path.
- No failure or rejection may trigger a non-cuRobo planner; the only permitted
  outcomes are a validated cuRobo nominal plan with an optional accepted
  residual correction, the validated nominal plan alone, or no motion.
- Checkpoints are advisory; deterministic validation has final authority.

### Acceptance criteria

- Training and eval scripts run only in sim.
- Residual bounds are configuration-driven and tested.
- Benchmark reports show residual-on vs residual-off with clear sim-only labeling.
- Physical hardware is never moved by the training loop.

---

## Phase 9 — Hardware interface and dry-run execution

### Objective

Expose a MyCobot 280 M5 state/command adapter that consumes validated plans with motion disabled by default.

### Tasks

1. Define hardware adapter interfaces that depend on domain types only.
2. Implement dry-run logging of intended joint commands without serial/network motion.
3. Gate any live command path behind an explicit environment flag (e.g. `ENABLE_MYCOBOT_HARDWARE_TESTS=1`).
4. Carry planning timestamps; reject stale state.
5. Document e-stop and robot-side limit assumptions next to configuration.

### Acceptance criteria

- Default tests and launches never move the arm.
- Dry-run tests cover command formatting, joint-name ordering, and stale-state rejection.
- Core planner/validator do not import `pymycobot` or serial stacks.

---

## Phase 10 — Physical MyCobot 280 M5 validation

### Objective

Run gated on-robot trials that measure approach success and safe failure behavior on a real MyCobot 280 M5.

### Tasks

1. Publish a hardware test plan (workspace envelope, reduced speeds, operator presence).
2. Execute zero-residual baseline trials before any residual-enabled trials.
3. Log target, plan status, validation report, residual mode, and measured tip error when instrumented.
4. Separate hardware metrics from Phase 6/7 sim reports in all documentation.
5. Optionally evaluate Phase 8 residuals only after the zero-residual baseline is recorded.

### Acceptance criteria

- Live motion requires the explicit hardware enable flag and operator acknowledgment.
- No unsupervised online RL on the physical arm.
- Documentation does not claim sub-millimeter accuracy without measured hardware evidence.
- Validation failures produce no motion (or a documented safe abort), never unsafe residuals.

---

## Remaining future adapters (not yet phased)

Document interfaces only until a later change set schedules them:

- ROS 2 joint trajectory execution
- perception adapter (point, normal, confidence)
- force/current-based terminal contact adapter
- online scene update adapter

Each adapter must depend on core domain interfaces. The core planner and validator must not import these external systems.

---

## 9. Configuration requirements

All behavior-changing parameters must be configuration-driven and validated at startup.

### `config/app.yml`

Include:

- robot config path;
- scene config path;
- tool frame;
- approach axis and sign;
- planner profile name;
- validation profile name;
- output path;
- project random seed;
- logging level.

### `config/planner_profiles.yml`

Include named profiles such as:

- `development_fast`;
- `validation_strict`;
- `benchmark_reproducible`.

Each profile records cuRobo seed counts, tolerances, graph settings, CUDA graph settings, collision activation distance, and goal-set size.

Do not hard-code planner tolerances in application logic.

---

## 10. Error handling and observability

Use structured exceptions for invalid configuration and environment errors. Use result objects for expected planning infeasibility.

Every plan attempt shall log:

- request ID;
- target ID;
- current joint state hash;
- scene revision;
- planner profile;
- cuRobo version;
- GPU and runtime information;
- goal-set candidate count;
- selected goal index and roll;
- planner status;
- timings;
- validation report;
- serialized replay data.

Never log only `planning failed`. Preserve the most specific available reason.

---

## 11. Testing strategy

### Unit tests

Cover:

- frame construction;
- normal and tangent degeneracy;
- quaternion convention;
- roll candidate generation;
- joint-state name reordering;
- configuration validation;
- lateral and orientation metrics;
- progress monotonicity;
- residual safety projection.

### Integration tests

Cover:

- cuRobo import and version guard;
- robot configuration load;
- planner warmup;
- FK calculation;
- empty-scene planning;
- obstacle-scene planning;
- goal-set planning;
- `plan_grasp` approach-only planning;
- interpolated trajectory validation.

Mark GPU tests explicitly so lightweight unit tests can run separately.

### Regression tests

Persist only compact inputs and expected metric bounds. Do not commit large generated trajectory dumps unless needed to reproduce a defect.

### Test commands

The project should support:

```bash
pytest tests/unit
pytest -m gpu tests/integration
ruff check .
ruff format --check .
```

---

## 12. Safety requirements

Even before physical hardware is connected, the project shall be designed as safety-critical motion software.

1. Invalid or unverified trajectories are never marked executable.
2. Joint names and units are never inferred silently.
3. Planning state timestamps are carried through future execution interfaces.
4. Stale state invalidates execution.
5. Planner collision checks are not disabled globally.
6. Contact-link collision disabling must be explicit, narrowly scoped, and off by default.
7. Terminal approach speed limits will be lower than free-space limits when hardware is added.
8. Future physical execution requires independent emergency stop and robot-side limits.
9. RL output is advisory and bounded; deterministic safety has final authority.
10. All assumptions about robot limits, calibration, payload, and tool geometry are documented next to configuration values.

---

## 13. Cursor implementation workflow

Cursor shall implement one phase at a time.

For each phase:

1. Read this specification and applicable `.cursor/rules/*.mdc` files.
2. Create or continue the phase branch named exactly `wip_phaseN`.
3. Create or update a phase checklist in the pull request or working notes.
4. Inspect the exact cuRobo v0.8.0 source or official documentation before using an unfamiliar API.
5. Add tests before or alongside implementation.
6. Run the narrow tests for the modified module.
7. Run the complete unit suite.
8. Run GPU integration tests when the environment supports them.
9. Update all project documentation and the phase report after tested behavior exists.
10. Do not begin the next phase while acceptance criteria for the current phase are failing.
11. Do not hide incomplete behavior behind a successful exit code.

### 13.1 Phase branch and `main` landing policy

Each completed phase must remain available as a historical repository state:

1. Develop Phase N only on `wip_phaseN`.
2. After all applicable acceptance gates pass, commit and push `wip_phaseN`.
3. Fetch the latest remote refs and rebase `wip_phaseN` onto `origin/main`.
   For Phase 0, which initializes `main`, this step is not applicable until
   `main` exists.
4. If rebase changes the commit, re-run affected gates and update
   `wip_phaseN` with a normal push when fast-forwardable. Rewriting a published
   phase branch requires explicit user authorization.
5. Fast-forward `main` to the exact tested phase commit and push `main`.
6. Create `wip_phaseN+1` from the updated `main`; never merge `main` into a
   phase branch.
7. Preserve every completed `wip_phaseN` branch. `main` represents the most
   current completed functionality.

The phase landing documentation set is: `spec.md`, `README.md`,
`REFERENCES.md`, `STATUS.md`, `CHANGES.md`, `docs/last_prompt.md`, and the
applicable `docs/phaseN_*.md` report.

When cuRobo behavior differs from this document, record the discrepancy and prefer the pinned v0.8.0 source code as the technical authority. Do not silently substitute a v0.7.x example.

---

## 14. Definition of done for the initial project

The **initial project** (Phases 0–6) is complete when:

- cuRobo v0.8.0 is version-checked and reproducible;
- the MyCobot 280 M5 robot model loads correctly;
- task frames are robustly generated from randomized target normals;
- multiple roll candidates can be supplied as a goal set;
- `plan_grasp` produces free-space and terminal approach segments;
- the final segment is independently validated against the surface-normal line and orientation constraints;
- failures are replayable and categorized;
- randomized benchmarks produce JSON and Markdown reports;
- the zero-residual execution seam is tested;
- the core `mycobot_curobo` package has no ROS, Isaac Kit, hardware-control, or RL runtime dependency;
- all unit tests, required GPU tests, linting, and formatting checks pass.

Phases 7–10 are complete only when their own acceptance criteria in §8 are met. Host-side Isaac scaffolding may exist earlier without counting as Phase 7 completion.

---

## 15. Official references

Use pinned v0.8.0 sources wherever possible.

### cuRobo release and API baseline

- cuRobo releases: https://github.com/NVlabs/curobo/releases
- cuRobo v0.8.0 tag: https://github.com/NVlabs/curobo/tree/v0.8.0
- cuRobo v0.8.0 README: https://github.com/NVlabs/curobo/blob/v0.8.0/README.md
- cuRobo v0.8.0 changelog: https://github.com/NVlabs/curobo/blob/v0.8.0/CHANGELOG.md
- cuRoboV2 paper: https://arxiv.org/abs/2603.05493

### Motion planning

- v0.8.0 motion-planning example: https://github.com/NVlabs/curobo/blob/v0.8.0/curobo/examples/getting_started/motion_planning.py
- Public motion-planner module: https://github.com/NVlabs/curobo/blob/v0.8.0/curobo/motion_planner.py
- MotionPlanner implementation: https://github.com/NVlabs/curobo/blob/v0.8.0/curobo/_src/motion/motion_planner.py
- MotionPlannerCfg implementation: https://github.com/NVlabs/curobo/blob/v0.8.0/curobo/_src/motion/motion_planner_cfg.py
- Motion-planner result types: https://github.com/NVlabs/curobo/blob/v0.8.0/curobo/_src/motion/motion_planner_result.py
- Tool-pose criteria: https://github.com/NVlabs/curobo/blob/v0.8.0/curobo/_src/cost/tool_pose_criteria.py

### Robot model and configuration

- v0.8.0 Franka robot configuration example: https://github.com/NVlabs/curobo/blob/v0.8.0/curobo/content/configs/robot/franka.yml
- Robot-model example: https://github.com/NVlabs/curobo/blob/v0.8.0/curobo/examples/getting_started/build_robot_model.py
- Public types: https://github.com/NVlabs/curobo/blob/v0.8.0/curobo/types.py
- Robot configuration internals: https://github.com/NVlabs/curobo/blob/v0.8.0/curobo/_src/types/robot.py

### Package requirements

- v0.8.0 `pyproject.toml`: https://github.com/NVlabs/curobo/blob/v0.8.0/pyproject.toml

### Cursor rules

- Current Cursor Rules documentation: https://cursor.com/docs/rules

### Legacy documentation warning

The pages at https://curobo.org/ primarily document the v0.7.x generation and are useful for concepts, but they are not the implementation authority for this project. When an example uses `MotionGen` or `curobo.wrap.reacher`, treat it as legacy unless a v0.8.0 source confirms the same API.
