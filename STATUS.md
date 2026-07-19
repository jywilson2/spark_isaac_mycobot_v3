# STATUS — MyCobot 280 M5 Constrained Approach Planner

Last updated: **2026-07-18**

## Current phase

**Phase 0 — repository bootstrap and environment verification**

This status is initialized for v3. No v2 completion metrics, GUI results,
planning-success claims, or hardware-readiness claims carry forward.

## Implemented

- Python 3.10+ `src/` project layout.
- Exact cuRobo v0.8.0 Git-tag dependency declaration.
- Phase 0 environment/version guard and JSON report writer.
- Public cuRoboV2 import smoke test, marked `gpu`.
- Lightweight deterministic unit tests.
- pytest and ruff configuration.
- Fresh README, change inventory, references, and Apache-2.0 license.

## Acceptance checklist

- [x] `pytest tests/unit` passes (16 passed).
- [ ] `ruff check .` passes.
- [ ] `ruff format --check .` passes.
- [ ] CUDA host environment report is valid.
- [ ] `pytest -m gpu tests/integration` passes (currently skipped: cuRobo absent).
- [x] Complete lightweight suite passes (16 passed, 1 GPU test skipped).
- [x] Python sources compile and `pyproject.toml` parses.
- [x] No ROS, Isaac, hardware-control, sensor, or RL dependency is present.
- [ ] `wip_phase0` is pushed; no project commit is pushed to `main`.

## Known environment status

The repository bootstrap runs in a development container that may not contain
cuRobo v0.8.0, the compatible PyTorch CUDA wheel, or direct GPU access. The
lightweight suite is expected to run without them. GPU acceptance remains
pending until `scripts/verify_environment.py` and the marked integration test
run in the intended CUDA environment.

The environment verifier was exercised here and correctly exited 2 with a JSON
report identifying missing `nvidia-curobo`, PyTorch/CUDA, public imports, GPU
name, and CUDA tensor allocation. Ruff is not installed in this container;
system-level installation was deliberately not performed during bootstrap.

## Next step

Complete all Phase 0 acceptance checks. Do not begin the Phase 1 robot model or
copy robot assets until Phase 0 passes and asset provenance is reviewed.

