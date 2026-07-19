# MyCobot 280 M5 Constrained Approach Planner

Deterministic, collision-aware surface-normal approach planning for the
Elephant Robotics MyCobot 280 M5 using NVIDIA cuRobo **v0.8.0 (cuRoboV2)**.
cuRobo is the exclusive global and local motion planner: no fallback, learned
policy, simulator, ROS integration, hardware adapter, or external package may
generate a replacement motion path.

This repository is a new design. It is not an in-place continuation of
`spark_isaac_mycobot_v2`: v2's Isaac visualization, ROS integration, learned
residual implementation, distance-dependent recovery, and legacy cuRobo
`MotionGen` code are intentionally not carried forward.

The authoritative requirements are in [`spec.md`](spec.md). Cursor guidance in
[`.cursor/rules/`](.cursor/rules/) is also authoritative.

## Current phase

**Phase 7 — Isaac Sim validated-plan playback: complete. Phase 7.1 —
unknown-start cube approach suite: complete on `wip_phase7_1`.** See
[`docs/phase7_1_cube_approach.md`](docs/phase7_1_cube_approach.md).

Full roadmap (Phases 0–11, including decimal Phases 7.1 and 9.1):
[`docs/implementation_phases.md`](docs/implementation_phases.md).

Implemented now:

- Python `src/` package layout;
- exact cuRobo v0.8.0 Git-tag dependency;
- deterministic runtime/version guard;
- CUDA tensor-allocation check;
- machine-readable environment report;
- lightweight unit tests and a separately marked GPU import smoke test;
- ruff lint/format configuration;
- validated cuRobo format-2.0 robot config with explicit joint/frame contracts;
- independent CPU FK and five known-state regression fixtures;
- static collision spheres, self-collision config, and GPU planner warmup;
- typed surface targets and deterministic configurable roll goal sets;
- public cuRoboV2 `GoalToolPose` conversion;
- structured `plan_grasp` nominal plans with finite trajectory extraction,
  selected-roll mapping, and a fresh/warmed backend for exactly one call;
- typed fail-closed validation reports covering terminal geometry, limits,
  dynamics, continuity, and available collision clearance;
- real cuRobo FK and self-collision validation in an explicitly empty world;
- typed zero-residual execution with deterministic safety projection,
  timestamp/watchdog checks, joint feasibility, and an in-memory adapter;
- deterministic randomized workspace cases, frozen smoke/regression fixtures,
  exact failed-request replay, stable failure taxonomy, and JSON/Markdown
  reports;
- versioned validated-plan playback JSON, exact articulation DOF mapping,
  NumPy pose metrics, and an Isaac Sim 6.x headless/GUI player.
- deterministic Phase 7.1 cube geometry/scene revisions, frozen episode replay,
  A–D mode sampling, JSON/console reporting, and cube sphere-AABB clearance;
- a fail-closed cube-world validation adapter and cuRobo-only joint relocation
  adapter for Mode C;
- illuminated Isaac Phase 7.1 plan/playback split (cuRobo process then Kit),
  drive-target motion, PhysX prohibited-contact evidence, and null tip metrics.

Not implemented:

- generic non-empty-world collision-clearance evaluation beyond the Phase 7.1
  cube adapter (still fails closed);
- non-zero residual correction (Phase 8);
- Phase 9/9.1 fabricated contact tool and evaluation;
- residual RL training and hardware motion (Phases 8–11).

See [`STATUS.md`](STATUS.md) for acceptance status and [`CHANGES.md`](CHANGES.md)
for the change inventory. The tested runtime evidence is in
[`docs/phase0_environment.md`](docs/phase0_environment.md).

## Install

Use Python 3.10 or newer in a CUDA-capable NVIDIA environment. The direct
dependency pins cuRobo to the exact `v0.8.0` Git tag. Select the matching CUDA
runtime extra without allowing pip to replace an existing CUDA-enabled PyTorch:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e '.[dev,cuda13]'  # DGX Spark / CUDA 13
# or: python -m pip install -e '.[dev,cuda12]'
```

In the Isaac ROS / Cursor container, install only the lightweight lint tool
when needed (do not pull cuRobo/CUDA into that bootstrap):

```bash
./scripts/ensure_container_dev_tools.sh   # Ruff-only venv
./scripts/run_verification.sh ci          # auto-bootstraps Ruff if missing
```

PyTorch must match the installed NVIDIA driver/CUDA environment. If the default
resolver selects an incompatible wheel, install the correct PyTorch CUDA wheel
first, then repeat the editable install. Do not use CPU planning or another
planner as a fallback. Future CPU planning is permitted only if the pinned
cuRobo implementation supports it and project validation covers it.

On the Isaac Sim host, use `scripts/host/install_curobo.sh`; it deliberately
installs cuRobo's `cu13` runtime extra without a Torch extra so Isaac Sim's
cu130 wheel is preserved.

## Verify Phase 0

Lightweight tests do not construct a planner and can run without a GPU:

```bash
pytest tests/unit
ruff check .
ruff format --check .
```

On the CUDA/cuRobo host:

```bash
python scripts/verify_environment.py \
  --output artifacts/reports/environment.json
pytest -m gpu tests/integration
```

The verifier records:

- Python, cuRobo, and PyTorch versions;
- cuRobo source revision when install metadata provides it;
- CUDA runtime visible to PyTorch;
- CUDA availability and GPU name;
- selected device and dtype;
- public cuRoboV2 API import status;
- GPU tensor-allocation status.

It exits nonzero if the exact cuRobo baseline, required public APIs, CUDA, or
GPU allocation are unavailable.

## Inspect Phase 1 robot model

Obtain the pinned vendor URDF/meshes, then inspect CPU metadata/FK:

```bash
./scripts/download_mycobot_ros2.sh
python3 scripts/inspect_robot_model.py
```

On the DGX Spark host, construct and warm the cuRobo planner:

```bash
./scripts/host/spark_host_exec.sh \
  ./scripts/host/inspect_robot_model.sh
```

The model uses `g_base`, `joint6_flange`, and an explicit identity
`tcp_link` for the bare flange. A fitted tool requires a measured fixed TCP
transform. See [`docs/phase1_robot_model.md`](docs/phase1_robot_model.md).

## Build Phase 2 task frames

```python
from mycobot_curobo import SurfaceTarget, build_surface_goal_set

target = SurfaceTarget.create(
    position_base_m=[0.15, 0.0, 0.20],
    surface_normal_base=[0.0, 0.0, 1.0],
    tangent_hint_base=[1.0, 0.0, 0.0],
    pre_approach_distance_m=0.05,
    target_id="example",
)
goal_set = build_surface_goal_set(target)
```

The default signed axis is tool Z × -1 and the desired motion direction is
against the outward normal. Defaults and roll angles come from
`config/app.yml`; see
[`docs/phase2_task_frames.md`](docs/phase2_task_frames.md).

## Plan a Phase 3 nominal approach

`NominalPlanner` accepts a backend factory, not a reusable backend. The pinned
cuRobo v0.8.0 runtime changed internal optimizer/tool-criteria state after
`plan_grasp`; repeated use could shorten or fail later trajectories. Phase 4
also found that an unwarmed fresh backend could report success while remaining
at the pre-approach endpoint. Every request and retry therefore constructs a
fresh backend, resets its seed, performs configured public warmup, resets the
seed again, and invokes `plan_grasp` exactly once:

```python
from mycobot_curobo import (
    NominalPlanner,
    TaskFrameConfig,
    create_curobo_planner,
    load_planner_profile,
)

profile = load_planner_profile("development_fast")
planner = NominalPlanner(
    lambda: create_curobo_planner(profile, warmup=False),
    profile,
    task_frame_config=TaskFrameConfig(),
)
outcome = planner.plan(request)
```

This reliability-first lifecycle includes construction and warmup in request
wall time. Returned nominal plans remain `executable=False` until Phase 4
independently validates every waypoint. See
[`docs/phase3_nominal_planning.md`](docs/phase3_nominal_planning.md).

### Phase 3 implementation libraries

| Library | Pinned use |
|---|---|
| NVIDIA cuRobo v0.8.0 | `MotionPlanner`, `MotionPlannerCfg`, `plan_grasp`, `JointState` |
| NumPy | fail-closed tensor conversion and trajectory validation |
| PyYAML | planner-profile and empty-scene configuration |
| pytest | CPU orchestration tests and GPU lifecycle regression |

## Validate a Phase 4 nominal plan

`validate_nominal_plan` combines a typed `ValidationProfile` with an injected
trajectory evaluator. It checks the terminal corridor, approach axis, selected
roll, target progress, endpoint pose, joint margins and dynamics, segment
boundary, and available collision clearances. Violations identify the first
offending waypoint; only a valid `ValidatedPlan` is executable.

Thresholds live in `config/validation_profiles.yml`. The GPU test uses
`CuroboTrajectoryEvaluator` for real cuRobo FK and self-collision clearances in
an explicitly empty world. Non-empty-world clearance is not yet implemented:
it is reported as unevaluated and fails closed. See
[`docs/phase4_validation.md`](docs/phase4_validation.md).

### Phase 4 implementation libraries

| Library | Pinned use |
|---|---|
| NVIDIA cuRobo v0.8.0 | terminal-waypoint FK and collision-sphere state |
| NumPy | deterministic geometry, limits, dynamics, and report metrics |
| PyYAML | named validation threshold profiles |
| pytest | synthetic violation coverage and GPU acceptance |

## Replay a Phase 5 validated plan

`TrajectoryExecutor` accepts only a valid executable `ValidatedPlan`. It samples
the unchanged nominal trajectory, uses `ReplayRobotStateProvider` for the
initial measured-state contract, invokes `ZeroResidualCorrector`, and passes
every waypoint through `SafetyProjector` before `InMemoryCommandAdapter` can
record it.

Bounds in `config/residual_safety.yml` cover residual magnitude, the terminal
corridor, joint margin, state age, and watchdog timeout. Oversized residuals
are visibly clipped; unsafe residuals are rejected. Phase 5 deliberately
rejects every non-zero residual at execution because no bounded
Cartesian-to-joint mapping has been accepted. See
[`docs/phase5_execution_residual.md`](docs/phase5_execution_residual.md).

## Run the Phase 6 workspace benchmark

The declared `g_base` regions in `config/benchmark_workspace.yml` are
conservative unmeasured candidate regions, not a measured dexterous envelope.
Run the frozen smoke stage and replay a failed serialized request with:

```bash
python3 scripts/benchmark_random_targets.py --stage smoke --root-seed 6006
python3 scripts/plan_single_target.py \
  artifacts/benchmarks/phase6_smoke_seed_6006.json --failed-index 0
```

Reports default to `artifacts/benchmarks/` in matching JSON and Markdown.
Every attempt contributes to the rates; failures are never dropped. Planner
seed sweeps create a fresh copied profile with the requested seed. Optional
zero-residual execution rejection is reported separately from planning
failures. See [`docs/phase6_benchmark.md`](docs/phase6_benchmark.md).

## Play a Phase 7 plan in Isaac Sim

Kit runs natively on the DGX Spark host. The player refuses non-executable
plans before starting `SimulationApp`, maps the exact six joint names, and
writes simulation pose metrics separately from cuRobo validation metrics.

```bash
./scripts/download_mycobot_ros2.sh          # vendor URDF + meshes (local)
./scripts/host/check_prereqs.sh             # host: Isaac python.sh + URDF
./scripts/convert_urdf_to_usd.sh            # host: URDF → USD
./scripts/host/smoke_isaac_viz.sh --headless
./scripts/host/smoke_isaac_viz.sh --gui --auto-exit
# From the Isaac ROS container:
./scripts/host/spark_host_exec.sh \
  ./scripts/host/smoke_isaac_viz.sh --gui --auto-exit
```

`./scripts/run_verification.sh spark` requires the auto-exiting GUI smoke; no
environment bypass is supported. A missing `tcp_link` in the imported USD does
not fabricate results: joint playback may complete while tip metrics are null
and marked unevaluated. See
[`docs/phase7_isaac_sim.md`](docs/phase7_isaac_sim.md).

## Phase 7.1 cube approach suite

Phase 7.1 samples **5 episodes by default** with independent unknown starts
(Mode A) and diverse 3D cube goals/normals (Mode D). Chained starts (B) and
relocate-then-approach (C) are optional runtime modes, but acceptance testing
must exercise all A–D modes.

The default cube edge is **14 mm**, derived as approximately 25% of the area of
an assumed 31 mm circular flange face. Phase 9 must measure that assumption.
The default terminal standoff is **0.08 m** so collision spheres clear the cube
at the Phase 4/7.1 grasp pose; Mode D samples FK-aligned cubes from a seeded
goal-joint bank inside declared `g_base` AABBs. Reports include lateral/axis
errors, clearances, prohibited Isaac contacts, failures, p50/p95, seed, and
frozen replay inputs. Unevaluated non-cube non-empty worlds still fail closed.
Isaac tip metrics remain null/`not_evaluated`; see
[`docs/phase7_1_cube_approach.md`](docs/phase7_1_cube_approach.md).

Host smoke plans in a cuRobo-only process, then plays in Kit with lighting,
static contact-reporting cubes, labeled resets, and drive-target motion:

```bash
./scripts/host/spark_host_exec.sh \
  ./scripts/host/smoke_phase7_1_cube_suite.sh --gui --auto-exit --all-modes
```

## Planned Phase 9/9.1 contact tool

Phase 9 will measure the flange and create a short, stiff contact tool as
parameterized OpenSCAD source plus a matching manifold/watertight printable
STL. The optional tool profile will include explicit TCP, visual, and collision
geometry while leaving the bare-flange profile as default. See
[`docs/phase9_contact_tool.md`](docs/phase9_contact_tool.md).

Phase 9.1 will characterize dimensional accuracy, calibration uncertainty,
remounting repeatability, FK, collision behavior, and seeded tool-profile cube
episodes without powered arm motion. Only this calibrated profile may enable
evaluated tool-tip metrics; Phase 7.1 remains `not_evaluated`. See
[`docs/phase9_1_tool_evaluation.md`](docs/phase9_1_tool_evaluation.md).

## Safety boundary

This project plans, validates, and dry-run replays only; it does not command a
robot. Plans are not executable until independent waypoint-by-waypoint
validation passes. Residual RL (Phase 8) remains bounded and subordinate to
deterministic safety logic. Residuals may apply local execution corrections
but may not generate replacement trajectories or full pose-to-joint solutions.
Physical motion (Phases 10–11) is gated and dry-run by default.

## Branch policy

Each phase is developed and retained on `wip_phaseN`; decimal phase names use
an underscore (`wip_phase7_1`, `wip_phase9_1`). After its acceptance gates
pass, that branch is rebased onto the latest `main` (Phase 0 initializes
`main`), pushed, and then `main` is fast-forwarded to the exact tested commit.
The next roadmap phase branches from updated `main`. This preserves historical
phase states while `main` represents the most current completed functionality.

## License

Apache-2.0. See [`LICENSE`](LICENSE).

