# MyCobot 280 M5 Constrained Approach Planner

Deterministic, collision-aware surface-normal approach planning for the
Elephant Robotics MyCobot 280 M5 using NVIDIA cuRobo **v0.8.0 (cuRoboV2)**.

This repository is a new design. It is not an in-place continuation of
`spark_isaac_mycobot_v2`: v2's Isaac visualization, ROS integration, learned
residual implementation, distance-dependent recovery, and legacy cuRobo
`MotionGen` code are intentionally not carried forward.

The authoritative requirements are in [`spec.md`](spec.md). Cursor guidance in
[`.cursor/rules/`](.cursor/rules/) is also authoritative.

## Current phase

**Phase 2 — surface target and task-frame generation: complete.**

Full roadmap (Phases 0–10, including Isaac Sim, residual RL, and physical
hardware): [`docs/implementation_phases.md`](docs/implementation_phases.md).

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
- Phase 7 Isaac Sim **scaffolding** (host scripts, URDF helpers, vendor obtain).

Not implemented through Phase 2:

- nominal planning / independent validation (Phases 3–4);
- residual seam, benchmarks (Phases 5–6);
- Isaac closed-loop player, residual RL training, hardware motion (Phases 7–10).

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

PyTorch must match the installed NVIDIA driver/CUDA environment. If the default
resolver selects an incompatible wheel, install the correct PyTorch CUDA wheel
first, then repeat the editable install. Do not use CPU fallback for planning.

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

## Isaac Sim scaffolding (Phase 7+)

Optional host tooling adapted from v2. Not required for Phase 0 unit tests.

```bash
./scripts/download_mycobot_ros2.sh          # vendor URDF + meshes (local)
./scripts/host/check_prereqs.sh             # host: Isaac python.sh + URDF
./scripts/convert_urdf_to_usd.sh            # host: URDF → USD
./scripts/host/launch_isaac_sim.sh          # host: empty-stage GUI
# From the Isaac ROS container:
./scripts/host/spark_host_exec.sh ./scripts/host/check_prereqs.sh
```

Install cuRobo **v0.8.0** into the Isaac Sim Python env when ready:

```bash
./scripts/host/install_curobo.sh
```

## Safety boundary

This project currently plans and validates only; it does not command a robot.
Plans are not executable until independent waypoint-by-waypoint validation
passes. Residual RL (Phase 8) remains bounded and subordinate to deterministic
safety logic. Physical motion (Phases 9–10) is gated and dry-run by default.

## Branch policy

Each phase is developed and retained on `wip_phaseN`. After its acceptance
gates pass, that branch is rebased onto the latest `main` (Phase 0 initializes
`main`), pushed, and then `main` is fast-forwarded to the exact tested commit.
The next phase branches from updated `main`. This preserves historical phase
states while `main` represents the most current completed functionality.

## License

Apache-2.0. See [`LICENSE`](LICENSE).

