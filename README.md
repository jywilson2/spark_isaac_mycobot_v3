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

Implemented now:

- Python `src/` package layout;
- exact cuRobo v0.8.0 Git-tag dependency;
- deterministic runtime/version guard;
- CUDA tensor-allocation check;
- machine-readable environment report;
- lightweight unit tests and a separately marked GPU import smoke test;
- ruff lint/format configuration.

Not implemented in Phase 0:

- robot model or collision-sphere configuration;
- target-frame generation;
- motion planning or trajectory validation;
- ROS 2, Isaac Sim/Lab, physical hardware, sensors, or RL.

See [`STATUS.md`](STATUS.md) for acceptance status and [`CHANGES.md`](CHANGES.md)
for the bootstrap inventory.

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

## Safety boundary

This project currently plans and validates only; it does not command a robot.
Later plans are not executable until independent waypoint-by-waypoint
validation passes. Future residual corrections remain bounded and subordinate
to deterministic safety logic.

## Branch policy

Development begins on `wip_phase0`. The `main` branch is intentionally left
without project commits until a release is ready.

## License

Apache-2.0. See [`LICENSE`](LICENSE).

