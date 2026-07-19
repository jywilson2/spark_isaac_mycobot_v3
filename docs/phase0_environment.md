# Phase 0 — Environment verification report

Date: 2026-07-18  
Branch: `wip_phase0`

## Result

**PASS.** Phase 0 supplies a reproducible Python package, exact cuRobo v0.8.0
guard, machine-readable environment report, lightweight unit tests, and a
GPU-marked public-API smoke test without constructing a planner.

## DGX Spark runtime

- Python: 3.12.13 (Isaac Sim `python.sh`)
- cuRobo distribution: 0.8.0, editable checkout at tag `v0.8.0`
- PyTorch: 2.10.0+cu130
- CUDA runtime visible to PyTorch: 13.0
- GPU: NVIDIA GB10
- selected device / dtype: `cuda:0` / `float32`
- CUDA tensor allocation: PASS
- required cuRoboV2 public imports: PASS
- prohibited legacy symbols in required public modules: none

The host initially contained `nvidia-curobo 0.0.0` from an older checkout.
`scripts/host/install_curobo.sh` replaced it with the exact v0.8.0 tag before
the acceptance run.

PyTorch emits an upstream warning that GB10 compute capability 12.1 is one
minor revision newer than the wheel's advertised maximum (12.0). The required
CUDA allocation succeeds and the warning is not suppressed. This remains an
environment compatibility item to monitor before planner kernels are accepted
in Phase 1.

## Acceptance evidence

```text
python3 -m pytest -q
16 passed, 1 GPU-marked test skipped

python.sh scripts/verify_environment.py --output /tmp/v3_phase0_environment.json
valid: true

python.sh -m pytest -m gpu tests/integration -q -p no:cacheprovider
1 passed

python.sh -m ruff check .
All checks passed!

python.sh -m ruff format --check .
11 files already formatted
```

Python compilation, `pyproject.toml` parsing, and shell syntax checks also
passed. No planner was constructed and no physical robot command was issued.

## Phase boundary

Phase 0 does not accept the staged MyCobot URDF, collision geometry, joint
limits, or TCP transform. Those are Phase 1 responsibilities. Isaac helpers
remain optional Phase 7 scaffolding and are not imported by the core package.
