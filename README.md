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

**Phase 0 — repository bootstrap and environment verification.**

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
- Phase 7 Isaac Sim **scaffolding** (host scripts, URDF helpers, vendor obtain).

Not implemented in Phase 0:

- robot model or collision-sphere configuration (Phase 1);
- target-frame generation / planning / validation (Phases 2–4);
- residual seam, benchmarks (Phases 5–6);
- Isaac closed-loop player, residual RL training, hardware motion (Phases 7–10).

See [`STATUS.md`](STATUS.md) for acceptance status and [`CHANGES.md`](CHANGES.md)
for the change inventory.

## Install

Use Python 3.10 or newer in a CUDA-capable NVIDIA environment. The direct
dependency in `pyproject.toml` pins cuRobo to the exact `v0.8.0` Git tag and
selects its CUDA 12 + PyTorch dependency set.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e '.[dev]'
```

PyTorch must match the installed NVIDIA driver/CUDA environment. If the default
resolver selects an incompatible wheel, install the correct PyTorch CUDA wheel
first, then repeat the editable install. Do not use CPU fallback for planning.

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

Development begins on `wip_phase0`. The `main` branch is intentionally left
without project commits until a release is ready.

## License

Apache-2.0. See [`LICENSE`](LICENSE).

