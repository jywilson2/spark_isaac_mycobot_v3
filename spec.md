   # MyCobot 280 M5 Constrained Approach Planner

## Cursor Implementation Specification

**Document status:** Initial implementation specification  
**Primary robot:** Elephant Robotics MyCobot 280 M5  
**Exclusive motion planner and motion-planning dependency:** NVIDIA cuRobo **v0.8.0 (cuRoboV2)**\
**Scope:** Deterministic, collision-aware motion planning with a controlled surface-normal approach. The architecture must expose safe extension points for residual reinforcement learning and hardware integration. Phases 0–6 implement the initial planner; Phases 7–11 cover Isaac Sim validation, unknown-start approach visualization,
multi-target tip-contact clearance, optional finer target placement (Phase 7.3,
under consideration), target-scale collision-sphere coverage (Phase 1.1,
specified for review), bounded residual RL, contact-tool
development/evaluation, and physical MyCobot 280 M5 testing (see §8 and
`docs/implementation_phases.md`).

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

Phases 7–11 explicitly schedule Isaac Sim, unknown-start visualization,
multi-target tip-contact clearance, residual RL, contact-tool
development/evaluation, hardware dry-run, and physical validation.
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

Collision spheres describe robot geometry and safety margins. They must not be
moved dynamically to force a desired approach path. Coverage density may be
regenerated offline (see Phase 1.1) so that world obstacles at least as large
as the configured multi-target cube are detectable in cuRobo; regeneration is
not path shaping.

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
tool_approach_sign: 1
```

The signed approach axis is tool Z with ``+1`` so the bare-flange tip face leads
into the workpiece. Sign ``-1`` pointed the wrist/back of the flange at the
target. The sign and axis must be verified by a unit test and a visual or
numerical sanity check. Never assume an unverified sign from another robot.

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

## Phase 1.1 — Target-scale collision-sphere coverage

**Status:** **Option A implemented; overlay disarmed.** Self-clear and
body-clip detectability GPU checks pass under a trial-enabled overlay, but
arming it in the default robot YAML regresses Phase 7.1 / 7.2 GPU planning
(cuRobo reports start/end state in collision against target cubes). Keep
scaffolding until cover/suite fixtures are reconciled. Design notes:
[`docs/phase1_1_target_scale_collision_spheres.md`](docs/phase1_1_target_scale_collision_spheres.md).

### Headless verification finding (2026-07-21) — first cover rejected

Headless cuRobo checks found two defects in the first Phase 1.1 landing:

1. **Adapter bug (fixed):** `load_curobo_robot_config` forwarded project-only
   keys into cuRobo `KinematicsLoaderCfg`, so `MotionPlanner` construction
   failed and the overlay never loaded. The adapter now merges the overlay then
   strips those keys.
2. **Cover incompatible with self-collision (rejected):** the first greedy mesh
   cover (128 spheres, radii up to `2E` ≈ 28 mm plus `collision_sphere_buffer`)
   yielded **negative self-collision clearance** at the zero configuration.
   Phase 1 scaffolding (32 spheres) remains self-clear.

### Chosen revision: Option A (thickness-capped cover)

**Goal unchanged:** detect axis-aligned cubes of edge ≥ `E` (default 0.014 m)
in planning/world clearance **without** destroying self-collision feasibility
or expanding ignore maps silently.

| Option | Idea | Status |
|--------|------|--------|
| **A. Thickness-capped cover** | Mesh-constrained offline cover; each sphere radius capped by **local link thickness / medial radius** and `≤ E`; densify for detectability | **Chosen / implemented** |
| **B. Dual role split** | Self spheres vs separate world-only overlay | Not selected |
| **C. Distal densify only** | Densify only distal links | Not selected |
| **D. Scene-side keep-outs** | Inflate world cuboids; leave robot scaffolding | Not selected |

**Option A acceptance gate (before re-arming default YAML):**
`validate_start_state` at the zero configuration and at least one mid-reach
seeded posture must report **non-negative** self-collision clearance with the
cover, **and** a body-clip cube of edge `E` must still fail world clearance
where scaffolding would falsely clear. Phase 7.1 / 7.2 GPU planning suites and
host headless + GUI integration 2×5 smoke must also pass with the overlay
armed (see Acceptance criteria). Default robot YAML keeps
`collision_sphere_overlay_path` commented out until those gates pass.

### Objective

Replace the Phase 1 reduced (four-spheres-per-link) set with a **static,
version-controlled, mesh-constrained** collision-sphere cover that lets cuRobo
planning and independent world-clearance validation detect axis-aligned cuboid
obstacles of edge length **at least** the Phase 7.2 target size
(`target_edge_m`, default **0.014 m**), while keeping the sphere set **as sparse
as that obstacle size and link geometry allow**.

PhysX contact remains playback evidence only. It must not become the planner’s
collision oracle (plan/play split unchanged).

### Motivation

Sparse spheres can report non-negative clearance against a target-sized cube
while Isaac mesh–cube body contact still occurs. Phase 1.1 closes that gap for
obstacles ≥ the declared target edge without requiring a dense sphere cloud.

### Normative inputs

| Symbol | Meaning | Default / source |
|--------|---------|------------------|
| `E` / `min_detectable_obstacle_edge_m` | Smallest AABB edge that must be detectable | Suite `target_edge_m` (Phase 7.2 default **0.014 m**) |
| Link meshes | Vendor collision / visual meshes used for covering | Authoritative MyCobot assets already in-tree |
| `collision_sphere_buffer` | Existing cuRobo buffer on spheres | Keep configurable; do not use buffer alone as the detection guarantee |

When a multi-target suite declares `target_edge_m`, the committed robot sphere
set used for that suite must satisfy Phase 1.1 for
`min_detectable_obstacle_edge_m <= target_edge_m` (typically equality).

### Covering and sparsity rules

1. **Offline generation only.** Produce spheres with a host/offline procedure
   from link meshes; commit the result under `config/robots/`. Do not fit,
   move, or densify spheres at planning time to shape a path (§4.4).
2. **Mesh-constrained.** Every sphere centre must lie on or inside a documented
   mesh-derived envelope for its link (fit / medial / surface cover). Spheres
   must not be placed in free space solely to block an approach corridor.
3. **Detectability (primary guarantee).** For every collision link, the sphere
   set (including `collision_sphere_buffer`) must be such that **no**
   axis-aligned cube of edge `E` can intersect that link’s mesh envelope
   without also having **non-positive** signed clearance against at least one
   of that link’s collision spheres when evaluated with the project’s
   sphere–AABB clearance helper (same math as independent world clearance).
4. **Sparsity (secondary objective).** Minimize the number of spheres per link
   subject to (3). Prefer fewer, larger spheres whose spacing and radii are
   dictated by `E` and the link’s local thickness/curvature — not a uniform
   fine grid. Neighboring sphere **surface** gaps along the cover must not
   leave a pocket that admits an edge-`E` cube against the mesh without a
   sphere hit (rule 3).
5. **Self-collision policy unchanged (hard gate).** Keep explicit adjacent-link
   ignore maps and self-collision buffers; regenerating world-cover spheres must
   not silently alter self-collision semantics without an explicit YAML change
   and tests. A candidate cover is **rejected** if the zero configuration (and
   agreed mid-reach seeds) fail self-collision clearance under the same
   thresholds used by Phase 4 / suite validation.
6. **Still format 2.0.** Output remains cuRobo robot-config `collision_spheres`
   with per-link lists; inspection script continues to report counts by link.

### Non-goals

- Using PhysX contacts inside `plan_grasp` or as a substitute for spheres.
- Claiming hardware safety margins or sub-millimetre real-world accuracy from
  sim sphere covers.
- Inflating spheres at runtime, animating spheres, or disabling tip links to
  invent clearance.
- Guaranteeing detection of obstacles **smaller** than `E`.

### Tasks (when implementation is authorized)

1. Document the covering algorithm and parameters in
   `docs/phase1_1_target_scale_collision_spheres.md`.
2. Add a host regeneration script (deterministic seed / inputs) that writes
   updated `collision_spheres` into the robot YAML (or a layered override file
   loaded by the adapter). The adapter must merge the overlay then **strip
   project-only keys** (`min_detectable_obstacle_edge_m`,
   `collision_sphere_overlay_path`) before calling cuRobo
   `MotionPlannerCfg` / `KinematicsLoaderCfg` (unknown kwargs abort construction).
3. Set `min_detectable_obstacle_edge_m` explicitly in robot or suite config;
   fail closed if a suite `target_edge_m` is smaller than the committed cover’s
   `E`.
4. Unit tests: packing/gap invariants for synthetic link envelopes; rejection
   when `target_edge_m < E`.
5. GPU / host regression: at least one fixture where a cube of edge `E`
   intersecting a link envelope yields planning or validation world-collision
   failure with the new spheres, and passes falsely with the Phase 1 reduced
   set (or an explicit under-covered baseline).
6. Re-run Phase 7.2 unit gates; host headless and GUI **integration 2×5**
   smokes must satisfy the self-collision and unremoved-target acceptance
   bullet below (not optional review-only evidence; not the default 2-target
   spark gate).

### Acceptance criteria

- Committed spheres satisfy detectability for `E` equal to the default Phase
  7.2 `target_edge_m` unless a different `E` is explicitly declared.
- Sphere count is justified by sparsity under that `E` (report before/after
  counts by link in the phase report): Phase 1 scaffolding **32** → Phase 1.1
  Option A cover **1012** spheres for `E = 0.014 m` (thickness-capped; radii
  `≤ E`).
- Suite configs fail closed when `target_edge_m < min_detectable_obstacle_edge_m`.
- Core package remains Isaac-free; PhysX is not imported into planning.
- No physical robot command is issued.
- Before re-arming any Phase 1.1 overlay in the default robot YAML, host
  **headless and GUI** runs of the Phase 7.2 **integration smoke**
  (`scripts/host/smoke_phase7_2_integration_2x5.sh`, config
  `config/phase7_2_multi_target_integration_2x5.yml`: **2 episodes × 5
  targets**, grid placement with distinct per-episode placement and planner
  seeds) must both pass. This smoke is **integration-only** (not part of the
  default spark GUI gate); enable with
  `./scripts/run_verification.sh spark --with-integration-smoke` or
  `SPARK_RUN_INTEGRATION_SMOKE=1`. Evidence required:
  1. **Self-collision:** the armed cover keeps the zero configuration and the
     agreed mid-reach seeds self-clear under suite self-collision thresholds,
     and planning/validation rejects a deliberate self-colliding fixture (or
     equivalent fail-closed case); and
  2. **Unremoved targets:** with only the active contact cube stripped from the
     planning world (Phase 7.2 policy), a body-clip against an edge-`E`
     **unremoved** non-contact target yields planning or independent
     world-collision failure—including cases where Phase 1 scaffolding would
     falsely report non-negative clearance.
  Episodes in that smoke must differ in target placement **and** planned paths
  (distinct `placement_seed` / `episode_seed`).

### Relationship to other phases

- **Phase 1:** introduced the authoritative robot YAML and static collision
  spheres (scaffolding; four per collision link). Phase 1.1 revises only that
  sphere coverage for target-scale world obstacles.
- **Phase 7.2:** continues to strip only the active contact cube from the
  planning world and keep tip collision enabled vs other targets; better
  spheres improve body (and tip) detection of those cuboids.
- **Phase 7.3:** controllable target-block placement (under consideration). It
  does **not** introduce collision spheres; placement keep-outs remain optional
  and complementary and do not replace Phase 1.1 coverage. Phase 1.1 work may
  share the `wip_phase7_3` branch without becoming part of Phase 7.3.

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
`hardware_placeholder` profile is a named stub only until Phase 10/11 measures
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

The Phase 6 implementation uses immutable `BenchmarkConfig`, `BenchmarkCase`,
`BenchmarkResult`, and `BenchmarkSummary` domain objects. The declared `g_base`
AABBs are labeled unmeasured conservative candidate regions. Every case stores
all request parameters, and every failed result embeds a complete serialized
`PlanningRequest` for exact replay.

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

Planner seed sweeps preserve the Phase 3 request/profile seed invariant by
constructing a fresh `PlannerProfile` copy with the case seed and a fresh
planner. Optional zero-residual execution replay occurs only after validation;
its rejection is reported separately and does not alter planning taxonomy.

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
6. Cross the core/simulator boundary through a versioned, typed playback JSON
   with exact joint names, SI units, scalar-first `wxyz`, and explicit
   executable/validation status. Reject non-executable input before Kit starts.

### Design constraints

- Core `mycobot_curobo` modules must not import Isaac Kit APIs.
- Prefer host execution; container may only delegate via `spark_host_exec.sh`.
- Simulation thresholds are sim metrics only — never claim physical accuracy.
- A missing simulator TCP prim may leave tip metrics unevaluated after joint
  playback; it must never cause invented pose values.

### Acceptance criteria

- Host prereq check finds Isaac Sim `python.sh` and vendor URDF.
- Headless smoke loads the robot and plays at least one validated plan.
- GUI smoke exits 0 when required by the project verification gate for that change set.
- Failures remain fail-closed: invalid plans are not marked executable because Isaac played them.

---

## Phase 7.1 — Unknown-start normal-approach cube visualization

### Objective

Visually and numerically exercise cuRobo-planned motion that brings the
circular bare-flange contact face toward a small cube along the cube-face
surface normal and configured TCP approach axis. Run a configurable number of
episodes from diverse starting joint states and 3D goals while streaming
well-formatted results to the console in real time.

Phase 7.1 uses branch `wip_phase7_1`. It reuses the Phase 3–7 planning,
validation, benchmark/replay, and Isaac playback contracts; it does not replace
the Phase 6 randomized benchmark or Phase 7 smoke.

### Configuration and scene

- `episode_count` is a positive configurable integer with default `5`.
- The default cube edge is `0.014 m`. This is the rounded result of
  `s = d * sqrt(pi) / 4`, where a square cube face covers 25% of an assumed
  circular flange face with diameter `d = 0.031 m`.
- The 31 mm flange diameter is an explicit unverified design assumption.
  Phase 9 shall measure the flange and revise the tool/cube basis if needed.
- Cube positions are sampled in a declared conservative reachable AABB in
  `g_base`; the declaration is not a dexterous-workspace claim.
- The cube is represented consistently as world-collision geometry in cuRobo
  and Isaac. The contact-ready endpoint uses a positive configurable standoff
  from the cube face in Phase 7.1; physical contact is not commanded.
- Isaac playback creates the configured dome and distant lights before the
  first world reset. The static cube carries collision and contact-report APIs;
  its arm-contact event count is an explicit per-episode acceptance metric.
- Cube-face normals are sampled from labeled bins. The desired terminal motion
  is opposite the outward face normal and aligned to the configured signed TCP
  approach axis.
- Motion remains free-space to pre-approach followed by the constrained linear
  terminal segment produced by cuRobo `plan_grasp`.

### Start and goal modes

All modes are required in Phase 7.1 validation testing:

- **Mode A — independent unknown start (default):** sample a seeded continuous
  joint state within configured limits or select from an expanded start-state
  bank. Reject and report invalid, out-of-limit, or self-colliding starts.
  Record the exact state for replay. A zeros-only suite is prohibited.
- **Mode B — chained start (optional runtime mode):** episode `k+1` begins at
  episode `k`'s successful final joint state. A failed episode uses the last
  successful state or terminates according to explicit configuration.
- **Mode C — relocate then approach (optional runtime mode):** use cuRobo to
  plan from the unknown start to a configured safe nest, then use
  `plan_grasp` from the nest to the cube. Both segments must pass collision and
  execution eligibility checks; terminal geometry metrics apply to the
  approach segment.
- **Mode D — 3D goal diversity (default):** sample cube positions across the
  configured AABB and normals across the configured labeled bins.

Modes A and D are enabled by default. Modes B and C are opt-in for ordinary
runs, but the Phase 7.1 acceptance gate must exercise A, B, C, and D.
Teleporting is permitted only as an explicitly labeled simulator-reset mode
and cannot satisfy a planned-motion acceptance result.

### Per-episode acceptance and metrics

An episode passes only when:

1. cuRobo `plan_grasp` is the sole approach planner and returns the required
   segments;
2. independent validation marks the plan executable;
3. terminal `max_lateral_error_m` and
   `max_approach_axis_error_rad` satisfy the active simulation profile;
4. terminal position/orientation, progress, limits, dynamics, and available
   collision-clearance checks pass;
5. no arm self-collision or arm-to-cube/environment collision is detected,
   and the configured minimum self/world clearances are met;
6. every required metric is finite and evaluated, except the explicitly
   excluded Isaac tip metrics below.

Phase 7.1 must provide supported collision evaluation for its cube scene.
Unsupported or unevaluated non-empty-world clearance fails the episode closed.
Isaac collision/contact events are reported separately as simulation evidence;
zero prohibited events is required but does not replace cuRobo planning or
independent validation.

Line-lateral error and approach-axis angular error are the primary
hardware-transferable geometric metrics. Simulation thresholds remain
simulation-only and cannot establish physical accuracy.

### Isaac tip-metric boundary

Phase 7.1 shall keep Isaac tip position and orientation metrics null and
`not_evaluated`, even when joint playback succeeds. It shall not invent a tip
pose, add a pointer prim, or enable the future optional tool profile. The
contact test tool is deferred to Phase 9 and evaluated separately in
Phase 9.1.

### Live console and artifacts

- Print a concise live result as each episode completes: index/count, start
  mode/label, cube position and normal bin, planning/validation status,
  lateral error, approach-axis error, self/world collision results and
  clearances, Isaac prohibited-contact count, failure category, and timing.
- Print running and final summaries with pass count/rate, failure-category
  counts, and lateral/axis p50 and p95.
- Write a matching machine-readable JSON report containing the root seed and a
  complete frozen request for every episode. Expected infeasibility remains a
  structured result and all failures count in aggregation.

### Acceptance criteria

- Default execution runs five episodes with Modes A and D enabled.
- Dedicated validation runs exercise all four modes A–D, including chained and
  relocate-then-approach behavior.
- Results stream during execution and the final console/JSON aggregates agree.
- Every case, including invalid starts and failures, is exactly replayable.
- Isaac tip metrics remain null/`not_evaluated`.
- No physical hardware command, alternate planner, or simulation-derived
  physical-accuracy claim is introduced.

---

## Phase 7.2 — Multi-target tip-contact clearance suite

### Objective

Exercise sequenced flange-normal tip/EE contact against a numbered multi-target
field in Isaac Sim while cuRobo remains the sole path planner. Support a typed
multi-target orchestration API that is reusable by later hardware adapters.
Phase 7.1 remains the single-cube standoff suite; Phase 7.2 does not replace it.

Phase 7.2 uses branch `wip_phase7_2`. Design detail and call-flow documentation
live in `docs/phase7_2_multi_target_contact.md`.

### Multi-target API (core, Isaac-free)

`plan_grasp` remains one target per call. Multi-target behavior is orchestration
over successive independently validated plans with an explicit world revision.

- **`TargetField`:** numbered `SurfaceTarget` set with placement and order policy.
- **`placement: grid | manual`:** deterministic evenly spaced **XY** grid in a
  declared `g_base` AABB, with Z centres spaced evenly in a band of width
  `0.5 * arm_z_motion_range_m` centered on the AABB mid-Z (declared arm
  vertical envelope; typically vendor `working_radius_m`), or a
  caller-supplied list of numbered targets (no position sampling when manual).
  **EE clearance spacing (normative; prevents mutual proximal deadlock):**
  when centres are generated (`grid`, and Phase 7.3 `random` / `layout`),
  pairwise separation must leave enough tip/EE approach room so two remaining
  neighbors cannot mutually deadlock tip planning (each needs the other
  removed first). Rules:

  1. **Approach-plane metric.** Separation is measured in the plane
     perpendicular to the suite `outward_normal_base` (for the default
     upward normal `[0,0,1]`, that is **XY**). Do **not** use 3D Euclidean
     distance alone: Z-band offsets must not inflate apparent clearance.
  2. **Floor.** Minimum approach-plane centre distance defaults to
     **`target_edge_m + flange_diameter_assumption_m + ee_approach_clearance_m`**,
     i.e. face-to-face gap at least
     `flange_diameter_assumption_m + ee_approach_clearance_m`.
     **`ee_approach_clearance_m`** defaults to
     **`flange_diameter_assumption_m`** (extra flange-width of approach
     corridor beyond the geometric flange diameter). With Phase 7.2 defaults
     (`edge=0.014`, `flange=0.031`) the floor is **0.076 m** on the approach
     plane.
  3. Suites may raise the effective minimum via `min_center_separation_m`
     but must not set it below that floor. Manual lists fail closed if they
     violate it. If a generated field cannot satisfy the floor inside
     `field_aabb`, fail closed (`ConfigurationError`) rather than packing
     tighter.
  4. **Optional rim guard (recommended for grid/random/layout):** reject a
     generated centre when
     `hypot(x, y) + 0.5 * target_edge_m`
     exceeds a declared working envelope
     (`working_radius_m` or suite `max_target_radial_m`) minus a tip margin
     (default `flange_diameter_assumption_m`), so tip-face goals are not
     placed on the workspace rim where goalset IK is brittle.
- **`order: shuffle | listed`:** seeded permutation of active target ids
  (replayable), or preserve enumeration / list order.
- **`retain_targets_after_contact` (bool, default `false`):** when `false`,
  allowed tip/EE contact removes the target from the scene and cuRobo world
  geometry; when `true`, the target is marked contacted, recolored, and left in
  place as an obstacle for subsequent plans. Acceptance and integration smokes
  use `false` so tip-contacted cubes leave the planning world.
- **`MultiTargetEpisodeRunner`:** multi-pass orchestration (see **Clearance,
  deferral, and reconsider** below): for each next id, build the world from
  **remaining** obstacles only, call `plan_grasp` with flange-normal approach,
  independently validate, then consume a `ContactDetector` result.
- **`ContactDetector` protocol:** reports `allowed_tip_contact`,
  `prohibited_body_contact`, or `none`. Isaac PhysX and later force/current or
  operator-ack adapters implement the same contract.
- Planning / deferral / suite budgets:
  - **`max_planning_failure_per_target`** (default **`3`**): each failed
    planning/validation attempt for the current target increments
    `current_count_planning_failure_per_target`. When that count **reaches**
    this limit, the target is **deferred** and the next unfinished target is
    processed (skipped for this pass), not permanently written off for the
    episode. The attempt that hits the budget is the last try (exactly
    `max_planning_failure_per_target` failures, not one more).
  - **`max_reconsider_passes`** (default **`target_count`**): after a pass
    that tip-contacts and (when retain is false) removes one or more targets,
    deferred targets are reconsidered with the reduced obstacle set. If a
    pass produces **no tip-contact progress** while deferred targets remain
    (including the **first** pass that defers every target), the episode
    fails immediately (`targets_unplanned`) — do not keep replanning the same
    unchanged obstacle field. If reconsider passes are exhausted with
    deferred targets still unplanned, fail (`max_reconsider_passes_exceeded`
    or `targets_unplanned`).
  - **`max_failed_episodes`** (default **`0`**): suite / acceptance budget;
    the number of failed episodes must not exceed this value.
  - **`max_target_failures`:** **deprecated for episode PASS**. Historical
    configs may still declare it; it must not allow an episode to PASS with
    any target left unplanned. Prefer `max_reconsider_passes`.
- Observed counters: `current_count_planning_failure_per_target` (resets each
  target attempt window), episode `planning_failure_count`,
  `deferred_target_ids`, `planned_target_ids`, and suite `failed_episodes`.

### Scene and episode model

- Configure positive integers `target_count` and `episode_count`.
- Host planning/smoke may override `target_count` with CLI `--targets N`
  (positive integer). When the selected YAML uses `placement: manual` and the
  listed poses are fewer than N, the override switches to `placement: grid`
  inside the declared field AABB; when the list is long enough, it is truncated
  to the first N ids. When `--targets` overrides the count,
  `max_reconsider_passes` defaults to the effective `target_count` unless set
  explicitly.
- Host planning/smoke may override `episode_count` with CLI `--episodes N`
  (positive integer).
- Host planning/smoke may set the suite root seed with CLI `--root-seed N`
  (non-negative integer). When set, episode planner/placement seeds derive
  deterministically as `episode_seed = N + 1009*(i+1)` (and a distinct
  `order_seed`). When omitted, each episode draws its own independent random
  seed in `[0, 2**31)` so multi-episode runs maximize coverage across layouts.
  YAML `root_seed` remains the library/API default when sampling without an
  explicit seed; host CLI does not read YAML `root_seed` unless the operator
  passes that value via `--root-seed`. Seeds are logged as
  `phase7_2_plan: episode=i episode_seed=…` (and `root_seed=N (cli)` when the
  flag is set) and stored in the plan bundle (`seed_mode`, `episode_seeds`).
- `max_planning_failure_per_target` (default **`3`**) and
  `max_failed_episodes` (default **`0`**) do not auto-follow `target_count`.
  `max_reconsider_passes` defaults to `target_count` when omitted.
- An **episode** is one full clearance sequence over the field (including
  deferral/reconsider passes), or an early fail.
- Contact / attempt order follows `shuffle` or `listed` as configured for the
  first pass; reconsider passes revisit deferred ids in that same relative
  order among remaining deferred targets.
- Terminal approach is opposite the outward face normal and aligned to the
  configured signed TCP approach axis (flange tip / tool approach policy).
- On structured planning or validation failure for the current target
  (`from_id → to_id`), **retry the same target** until success or
  `current_count_planning_failure_per_target` reaches
  `max_planning_failure_per_target` (default **`3`**); then **defer** that
  target and continue to the next unfinished id.
- Robot self-collision remains a planning/validation failure.

### Clearance, deferral, and reconsider (normative)

1. **Planning world = remaining tip-contact obstacles only.**
   Each `plan_grasp` / independent clearance check builds the cuRobo world
   from targets that have **not** yet been removed by allowed tip/EE contact
   (when `retain_targets_after_contact` is `false`). Tip-contacted and removed
   cubes are absent. The **active** contact target is also omitted from that
   world so tip-face occupancy remains feasible. Do **not** globally disable
   tip/flange links (`disable_collision_links` stays empty).
   `tip_allow_link_names` is for Isaac tip-vs-body classification only.
   When retain is `true`, contacted targets remain in the world as marked
   obstacles (no removal); reconsider still applies to deferred plans.

2. **Defer after per-target planning retries.** Exceeding
   `max_planning_failure_per_target` does **not** permanently fail the
   episode by itself. The target is recorded in `deferred_target_ids` and
   skipped for the remainder of the current pass so other reachable targets
   can be tip-contacted and (when retain is false) removed.

3. **Reconsider deferred targets.** After a pass tip-contacts at least one
   target (and removes it when retain is false), run further passes over any
   still-deferred / unplanned targets using the **updated remaining obstacle
   set**. Reset `current_count_planning_failure_per_target` when a deferred
   target is reconsidered. Stop reconsider when every target has a successful
   plan, or when a pass produces no new successful plan while unplanned
   targets remain (`targets_unplanned`), or when `max_reconsider_passes` is
   exceeded.

4. **Playback order = plan-creation order.** Host plan/play split must replay
   validated trajectories in the **same order the successful plans were
   created** during the episode (including legs planned on reconsider
   passes). Playback must not reshuffle or reorder relative to that creation
   sequence.

5. **All targets must end planned.** After clearance, deferral, reconsider,
   and recording of successful plans, **every** target id in the episode
   field must appear in `planned_target_ids` with a validated trajectory.
   If any target remains unplanned, the episode **FAIL**s
   (`targets_unplanned`). Tip contact remains required for every planned leg
   that is played (tip miss → `tip_contact_missed`).

### Contact policy (hard requirement)

- **Allowed:** configured tip/flange (EE allow-list) contact with a target →
  record success for that id; recolor; post viewport and console messages with
  timings; remove or retain per `retain_targets_after_contact`.
- **Prohibited:** any other robot link (arm body) versus any target → episode
  **FAIL** immediately (`body_contact`), even if tip contact also occurs.
- Zero prohibited body–target contacts is required simulation evidence and does
  not replace cuRobo planning or independent validation.
- **Flange overhang (geometry):** when `target_edge_m < flange_diameter_assumption_m`,
  tip-face contact on a cube top necessarily overhangs the face (defaults:
  14 mm edge vs 31 mm flange ≈ 8.5 mm). The active contact cuboid is omitted
  from the cuRobo planning world so tip occupancy stays feasible; Isaac PhysX
  still simulates the solid cube, so flange→edge/corner contacts on the
  **active** tip-contact target classify as allowed tip (path/name matching,
  including child meshes), not surprise body failures.
- **Flange-face containment (optional validation):** when
  `require_flange_face_containment` is true, terminal tip TCP must keep the
  flange disk inside the contact face within
  `flange_face_overhang_tolerance_m` (default `0.005` m). Suites that enable
  this should use `target_edge_m >= flange_diameter_assumption_m` (integration
  2×5 uses `0.031` m) so containment is geometrically achievable.
- **Flange-rim transit clearance:** collision spheres on `joint6_flange` must
  cover the assumed flange diameter so TrajOpt sees the rim. Multi-target
  validation may also reject plans whose flange-radius TCP sphere penetrates a
  non-contact target (`flange_neighbor_clearance`).

### Success and failure

An episode **PASS** only when:

1. **Every** target in the field has a successful, independently validated
   cuRobo plan (`planned_target_ids` equals the full field id set);
2. Every planned leg that is played achieves allowed tip/EE contact (and is
   removed when retain is false);
3. Playback order matches plan-creation order;
4. Zero prohibited body–target contacts;
5. Required timing and identity fields are finite and logged.

Otherwise **FAIL**. After a successful plan/validation, a tip-contact miss
aborts the episode immediately (`tip_contact_missed`). Taxonomy includes at
least `plan_failed`, `validation_failed`, `body_contact`, `tip_contact_missed`,
`max_planning_failure_per_target_exceeded` (deferral event),
`targets_unplanned`, `max_reconsider_passes_exceeded`, and
`targets_incomplete`. Failed plan attempts must identify the leg as
`from_id → to_id` (use `start` when leaving the episode start state).

**Suite acceptance:** `failed_episodes <= max_failed_episodes` (default **`0`**).
Report planning-attempt totals, deferred-then-recovered counts, and failed
episodes. An episode that finishes with any unplanned target increments
`failed_episodes` by one.

### Timing, visualization, and latency labeling

- Per leg: `planning_duration_s`, `motion_duration_s`, `time_to_contact_s`
  (plan + motion through first allowed tip contact); when a leg attempt fails,
  log the attempt against `current_count_planning_failure_per_target`.
- Per episode: `episode_duration_s`, success/fail outcome, contact counts,
  `planning_failure_count`, `deferred_target_ids`, `planned_target_ids`,
  plan-creation order of successful legs.
- Suite summary: episode pass/fail counts; `failed_episodes`,
  `total_planning_failures`, unplanned-episode counts; acceptance requires
  `failed_episodes <= max_failed_episodes`.
- Emit matching live lines to the host terminal and the Isaac Sim console.
- Numbered viewport labels must match `target_id` in logs and JSON (bright red
  7-segment digits for contrast on colored cubes). Digits must be
  **right-reading from the primary viewport camera** (not mirrored / facing
  the wrong side). Fixed local glyph orientation that appears backward from
  the default playback camera is a defect; prefer a yaw about world/local Z
  so the glyph faces the camera, or a camera-facing billboard, without
  changing `target_id` semantics.
- **Target cube highlight colors (playback):**
  - Non-current remaining targets: default blue until selected as current.
  - Current target with contact pending (validated leg about to move): yellow.
  - Successful allowed tip contact: green.
  - Tip contact missed after a successful plan: red.
  - Prohibited body contact: red.
- Planning latency recorded in Phase 7.2 is **simulation host evidence only**.
  It does not establish Orin AGX real-time budgets (Phases 10–11). An optional
  advisory `warn_planning_duration_s` may warn without failing the suite.

### Hardware transfer surfaces

Phase 7.2 shall document and type the following so Phases 10–11 can reuse the
same runner with swapped adapters (see also Remaining future adapters below):

- `MultiTargetEpisodeRunner`, `TargetField`, contact-order policy, retain flag,
  deferral/reconsider policy, `max_planning_failure_per_target` /
  `max_reconsider_passes` / `max_failed_episodes` budgets;
- `ContactDetector`, optional `TargetPoseSource`, scene-revision / obstacle set,
  `MotionGate`, and leg/episode report schema;
- Phase 5 execution seam (`RobotStateProvider`, command adapter,
  `SafetyProjector`, residual corrector) remains the only command path for
  physical motion.

Suggested physical defaults when hardware work lands: `placement=manual`,
`order=listed`, `retain_targets_after_contact=true`. Isaac-only visualization
(prim recolor, Kit messages, PhysX subscription details, USD spawn/despawn)
must not enter the core package.

### Documentation conventions

Public modules, classes, and methods use concise one-line docstrings.
Google-style `Args` / `Returns` blocks are reserved for non-obvious public
contracts. Detailed call/control flow belongs in
`docs/phase7_2_multi_target_contact.md`; `README.md` carries only a short
pointer (and optional summary diagram).

### Acceptance criteria

- Parameterized `target_count` and `episode_count`; `grid` and `manual`
  placement; `shuffle` and
  `listed` order; retain and remove-after-contact modes; seeded exact replay.
- Generated target centres (and validating manual lists) enforce
  approach-plane EE-clearance spacing:
  ≥ `target_edge_m + flange_diameter_assumption_m + ee_approach_clearance_m`
  (default `ee_approach_clearance_m = flange_diameter_assumption_m`) so tip
  approach is not mutually deadlocked by adjacent remaining cubes; optional
  rim guard against workspace-edge centres.
- Flange-normal tip/EE contact; body–target contact fails closed.
- Planning world uses only obstacles that remain after tip-contact removals
  (plus retain-mode marked contacts when configured); active contact cube
  omitted for tip feasibility.
- Per-target planning retries until `max_planning_failure_per_target`, then
  defer; reconsider deferred targets after removals; episode FAIL if any
  target remains unplanned (`targets_unplanned`).
- Playback order equals plan-creation order of successful legs.
- Suite acceptance: `failed_episodes <= max_failed_episodes` (default **`0`**);
  dual console/JSON timing.
- Phase 7 and Phase 7.1 smoke gates remain mandatory for Isaac-path changes.
- No physical hardware command, alternate planner, Orin SLA claim from sim
  timings, or simulation-derived physical-accuracy claim is introduced.

**Implementation status:** the deferral/reconsider / all-targets-planned
policy above is **implemented** on `wip_phase7_3` (supersedes the prior
“planning-failed targets need no tip contact / `max_target_failures` may leave
targets unplanned” episode PASS rule).

---

## Phase 7.3 — Controllable target-block placement

**Status:** Implemented on `wip_phase7_3`. Design notes:
[`docs/phase7_3_target_placement.md`](docs/phase7_3_target_placement.md).

### Objective

Give suite authors finer control over numbered target-block placement used by
Phase 7.2 multi-target clearance, beyond the Phase 7.2 `grid` AABB lattice and
fully enumerated `manual` lists, while keeping fail-closed configuration errors
and the existing plan/play split. Also repair GitHub Actions CI execution on
remote runners.

### Placement policies (normative)

| `placement` | Behaviour |
|-------------|-----------|
| `manual` | Explicit `targets[]` list (Phase 7.2) |
| `grid` | XY lattice + mid-Z band (Phase 7.2); optional per-episode phase shift |
| `random` | Seeded constrained-random centres inside `field_aabb` / Z band |
| `layout` | Named parameterized layout (`rows` or `arc`) without listing centres |

All non-manual policies must respect:

1. **`min_center_separation_m`:** pairwise **approach-plane** centre distance
   (see Phase 7.2 EE clearance); fail closed if violated. **Default and floor:**
   `target_edge_m + flange_diameter_assumption_m + ee_approach_clearance_m`
   with `ee_approach_clearance_m` defaulting to
   `flange_diameter_assumption_m`. An explicit larger value is allowed; an
   explicit smaller value is a configuration error.
2. **`keep_outs`** (optional list of AABBs in `g_base`): a candidate is
   rejected if the target cube of edge `target_edge_m` would intersect a
   keep-out (base/pedestal exclusion). Phase-shifted `grid` placement retries
   deterministic seed offsets when keep-outs would otherwise reject a lattice
   phase.
3. **`max_placement_attempts`** (random only; default `1000`): fail closed with
   `ConfigurationError` if a legal field cannot be sampled.
4. **Episode diversity:** for `grid` / `random` / `layout`, each episode uses a
   distinct `placement_seed` derived from `episode_seed` so fields (and thus
   plans) differ across episodes when `episode_count > 1`.
5. Integration fields may use the full radial working envelope
   (`max_target_radial_m`) with keep-outs; the EE-clearance floor is a **lower
   bound** on centre spacing, not a target packing density (even grids widen
   pitch as `field_aabb` grows).

Layout parameters (when `placement: layout`):

- `layout.name: rows` with positive `rows` and `columns` whose product is
  ≥ `target_count` (first `target_count` cells used in row-major order).
- `layout.name: arc` with `radius_m > 0`, `span_rad > 0`, `center_xy_m`,
  optional `z_m` (default field mid-Z), optional `start_angle_rad` (default
  `-span_rad/2`). All centres must lie inside `field_aabb` (XY) or fail closed.

### Tasks

1. Document policies in `docs/phase7_3_target_placement.md`.
2. Implement `random` and `layout` placement in the core package (Isaac-free).
3. Validate separation / keep-outs for all placement modes that generate
   centres; manual lists fail closed on separation/keep-out violations too.
4. Ship example configs under `config/phase7_3_*.yml`.
5. Unit-test sampling determinism, episode diversity, and fail-closed cases.
6. Keep GitHub Actions CI bootstrap green (`pytest.yml` + `run_verification.sh ci`).
7. Preserve Phase 7 / 7.1 / 7.2 default smoke gates; integration 2×5 smoke remains
   opt-in via `--with-integration-smoke`.

### Non-goals

- Interactive GUI drag-and-drop re-placement (optional later).
- Claiming reachability of every sampled centre (planning failures remain
  structured Phase 7.2 outcomes).
- Relabeling the integration `field_aabb` as a full dexterous workspace without
  a measured tip-contact map (`artifacts/workspace/tip_contact_workspace_v1.json`
  is candidate-region evidence only).
- Hardware command or PhysX-as-planner-oracle.
- Re-arming Phase 1.1 spheres without the separate Phase 1.1 acceptance gates.

### Acceptance criteria

- `placement: random` and `placement: layout` (`rows`, `arc`) produce
  deterministic fields for a fixed seed and fail closed on infeasible layouts.
- Keep-outs and approach-plane EE-clearance `min_center_separation_m` (floor
  `target_edge_m + flange + ee_approach_clearance_m`) are enforced for
  generated and manual centres.
- Multi-episode suites with non-manual placement yield distinct centre sets
  across episodes.
- Example Phase 7.3 configs load in unit tests; Phase 7.2 default configs remain
  valid.
- Container CI (`./scripts/run_verification.sh ci`) passes.
- Host default spark GUI smokes (Phase 7 / 7.1 / 7.2) remain mandatory for
  Isaac-path changes; integration 2×5 smoke is the opt-in final gate when
  requested.
- No physical robot command; core package stays Isaac-free.

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

## Phase 9 — Fabricated contact test tool

### Objective

Design, document, and fabricate an optional, repeatable contact-test tool that
mounts rigidly to `joint6_flange`, provides a circular contact face coaxial
with the configured approach axis, and defines a measurable TCP for later
simulation and physical evaluation.

### Tasks

1. Measure and document the physical flange outside diameter, mounting pattern,
   fasteners, available clearances, and coordinate convention. Reconcile the
   Phase 7.1 31 mm assumption and 14 mm cube default without silently changing
   historical reports.
2. Define a short, stiff tool with a circular contact face and an optional
   coaxial pointer/datum of documented length. Avoid a flexible needle-like
   design.
3. Create parameterized OpenSCAD source in millimetres and generate a matching
   manifold, watertight STL suitable for 3D printing. Store both source and
   generated STL under version control.
4. Document every model parameter, mounting clearance, fastener feature,
   minimum wall thickness, print orientation, material assumption, support
   requirement, and the exact command/tool version used to regenerate the STL.
5. Validate STL manifoldness/watertightness, critical dimensions, wall
   thickness, and mounting fit. A fit-check coupon may be used before printing
   the complete tool.
6. Publish dimensioned drawings or equivalent model documentation, fabrication
   instructions, mounting procedure, and calibration procedure.
7. Add an optional robot/TCP profile with an explicit measured
   `joint6_flange` to `tcp_link` fixed transform in metres and scalar-first
   `wxyz`. The bare-flange identity TCP remains the default.
8. Add corresponding URDF/Isaac visual geometry and cuRobo collision geometry.
   The tool must participate in collision checking; any narrowly scoped
   collision exception must be explicit and independently tested.

### Acceptance criteria

- OpenSCAD source regenerates the committed STL deterministically within
  documented tool/version constraints.
- The STL passes automated or documented manifold, dimensional, and minimum
  wall-thickness checks and satisfies the mounting/contact/TCP requirements.
- A fabricated example fits the measured flange without forcing, uncontrolled
  play, or interference, and its as-built dimensions are recorded.
- The optional profile loads without changing the default bare-flange model.
- TCP calibration is explicit and never buried in target coordinates.
- Tool visual and collision models share the documented physical dimensions.
- This phase performs no powered physical-arm motion and does not retroactively
  evaluate Phase 7.1 Isaac tip metrics.

---

## Phase 9.1 — Contact test tool evaluation

### Objective

Establish whether the fabricated Phase 9 tool and optional model provide
repeatable, measurable, collision-aware TCP behavior suitable for later
hardware testing. Phase 9.1 uses branch `wip_phase9_1`.

### Tasks

1. Inspect and record contact-face diameter, tool length, mounting features,
   offsets, tolerances, and measurement uncertainty.
2. Remove and reinstall the tool for a configured multi-trial study; measure
   TCP position displacement and approach-axis angular variation after every
   installation.
3. Calibrate the `joint6_flange` to `tcp_link` transform and record method,
   date, equipment, uncertainty, position in metres, and scalar-first `wxyz`.
4. Verify independent FK against the calibrated transform and confirm that the
   tool visual and collision models are correctly placed in Isaac/curobo.
5. Run seeded normal-approach cube episodes with the optional tool profile,
   preserving the Phase 7.1 console, replay, failure-counting, and geometric
   metric contracts.
6. Compare the unpowered mounted tool against fixed reference points and
   verify compatibility with the Phase 10 dry-run interface without issuing
   commands.

### Metrics

Report at least:

- TCP positional repeatability after remounting, in metres and millimetres;
- approach-axis angular repeatability, in radians and degrees;
- TCP calibration residual and measurement uncertainty;
- CAD-to-measured critical-dimensional error;
- simulated FK-to-expected-TCP position/orientation error;
- minimum tool/arm self-collision clearance; and
- normal-approach lateral and angular errors under the optional profile.

Do not invent acceptance thresholds before collecting measurement evidence.
Phase 9.1 shall report observed distributions and propose justified thresholds
for explicit review before those thresholds become hardware gates.

### Acceptance criteria

- Fabrication inspection, calibration provenance, uncertainty, and remounting
  repeatability results are complete and reproducible.
- The optional tool profile passes FK, visual alignment, and collision-model
  tests without changing bare-flange behavior.
- Tool-enabled Isaac episodes may report evaluated TCP metrics only from the
  calibrated modeled frame; Phase 7.1 reports remain `not_evaluated`.
- Every failed episode retains exact replay data.
- No powered physical-arm motion occurs.

---

## Phase 10 — Hardware interface and dry-run execution

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

## Phase 11 — Physical MyCobot 280 M5 validation

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
- perception adapter (point, normal, confidence) — intended
  `TargetPoseSource` implementation behind the Phase 7.2 multi-target runner
- force/current-based terminal contact adapter — intended hardware
  `ContactDetector` / terminal-contact policy behind Phase 7.2
- online scene update adapter — intended scene-revision feed for retained or
  perceived multi-target obstacle sets

Each adapter must depend on core domain interfaces. The core planner and
validator must not import these external systems. Phase 7.2 defines the typed
orchestration surfaces these adapters plug into; it does not implement the
external systems.

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
- `benchmark_reproducible`;
- `planning_high_effort` (higher trajopt seed count and more
  `max_plan_grasp_attempts` than `benchmark_reproducible`; keep `num_ik_seeds`
  at the benchmark value — raising IK seeds to 64 regresses packing-safe
  multi-target `plan_grasp` on host GPU; orientation tolerance must remain ≤
  Phase 4 `simulation_initial` terminal/roll thresholds so planner “success”
  cannot be validation-rejected; not a replacement for mid-reach placement).

Each profile records cuRobo seed counts, tolerances, graph settings, CUDA graph settings, collision activation distance, and goal-set size.

Do not hard-code planner tolerances in application logic.

### Phase 7.1 suite configuration

Use a validated named YAML configuration for:

- episode count (default `5`);
- root seed and start-state sampling/bank;
- Modes A–D and explicit chained-failure behavior;
- cube edge (default `0.014 m`), world pose AABB, and normal bins;
- positive terminal standoff (implemented default `0.08 m`, chosen so robot
  collision spheres remain clear of the cube at host-feasible grasp poses);
- Mode D goal-joint bank whose FK tip defines cube centres accepted only when
  they fall inside the declared AABBs;
- planner/validation/scene profiles and explicit lighting intensities;
- console refresh/report fields and artifact path; and
- required self/world collision and Isaac prohibited-contact gates.

### Phase 7.2 suite configuration

Use a validated named YAML configuration for:

- `target_count` and `episode_count` (positive integers);
- `placement` (`grid` or `manual`) and, when manual, the explicit numbered
  target list;
- `order` (`shuffle` or `listed`);
- `retain_targets_after_contact` (default `false`);
- `max_planning_failure_per_target` defaulting to **`3`**;
- `max_reconsider_passes` defaulting to **`target_count`**;
- `max_failed_episodes` defaulting to **`0`**;
- root seed; tip/EE allow-list link names; body-contact fail-closed policy;
- planner/validation/scene profiles and lighting;
- console/report fields including planning-attempt totals, deferred/planned
  target ids, plan-creation order, artifact path, and optional
  `warn_planning_duration_s`; and
- dual host-terminal and Isaac-console timing fields.

Host entry points:

- `isaac_sim/plan_multi_target_suite.py --targets N`, `--episodes N`, and
  `--root-seed N` override YAML `target_count` / `episode_count` / invocation
  seed as defined under Scene and episode model above;
- `scripts/host/smoke_phase7_2_multi_target.sh --targets N --episodes N
  --root-seed N` forwards the same overrides into the plan process and writes
  `artifacts/reports/phase7_2_multi_target_<tag>.{bundle.json,json}` unless
  `SPARK_PHASE7_2_BUNDLE` / `SPARK_PHASE7_2_REPORT` are set. Artifact tags are
  `N` (targets only), `epM` (episodes only), or `NxM` (both). Omitting
  `--root-seed` draws an independent random seed per episode.

### Phase 9 optional tool profile

Keep measured flange/TCP transform, tool dimensions, model paths, collision
geometry, calibration provenance, and profile name together. Enabling the
profile must be explicit; absence of the profile selects the bare-flange
identity TCP.

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
- residual safety projection;
- Phase 7.1 mode/default/config validation and deterministic replay;
- Phase 7.2 multi-target field/order/retain/runner contract validation and
  deterministic replay; and
- Phase 9 OpenSCAD/STL parameter and model-manifest validation.

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
- interpolated trajectory validation;
- Phase 7.1 A–D mode runs with cube-world and Isaac prohibited-contact checks;
- Phase 7.2 multi-target tip/body contact classification, fail-budget, and
  dual console timing checks;
  and
- Phase 9.1 optional-tool FK/collision/cube-suite evaluation.

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
2. Create or continue the phase branch named exactly `wip_phaseN`. Decimal
   phases replace the decimal point with an underscore: Phase 7.1 uses
   `wip_phase7_1`, Phase 7.2 uses `wip_phase7_2`, Phase 7.3 uses
   `wip_phase7_3`, and Phase 9.1 uses
   `wip_phase9_1`.
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
6. Create the next roadmap phase branch from the updated `main`; never merge
   `main` into a phase branch. Follow the explicit roadmap order, including
   decimal phases.
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

Phases 7–11 are complete only when their own acceptance criteria in §8 are
met. Host-side Isaac scaffolding may exist earlier without counting as Phase 7
completion.

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
