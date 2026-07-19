# STATUS — MyCobot 280 M5 Constrained Approach Planner

Last updated: **2026-07-18**

## Current phase

**Phase 0 — repository bootstrap and environment verification: COMPLETE**

Roadmap: [`docs/implementation_phases.md`](docs/implementation_phases.md)  
Authoritative criteria: [`spec.md`](spec.md) §8 (Phases 0–10)

This status is initialized for v3. No v2 completion metrics, GUI results,
planning-success claims, or hardware-readiness claims carry forward.

## Phase roadmap (summary)

| Phase | Focus | Status |
|-------|-------|--------|
| 0 | Env / version guard | **Complete** |
| 1 | Robot model + spheres | Not started |
| 2 | Task frames / roll goals | Not started |
| 3 | `plan_grasp` nominal planning | Not started |
| 4 | Independent validation | Not started |
| 5 | Execution + zero residual seam | Not started |
| 6 | Randomized benchmark | Not started |
| 7 | Isaac Sim closed-loop viz/validation | Scaffolding staged; not started |
| 8 | Bounded residual RL (sim only) | Planned |
| 9 | Hardware interface + dry-run | Planned |
| 10 | Physical MyCobot 280 M5 validation | Planned |

## Implemented

- Python 3.10+ `src/` project layout.
- Exact cuRobo v0.8.0 Git-tag dependency declaration.
- Phase 0 environment/version guard and JSON report writer.
- Public cuRoboV2 import smoke test, marked `gpu`.
- Lightweight deterministic unit tests.
- pytest and ruff configuration.
- Fresh README, change inventory, references, and Apache-2.0 license.
- Phase 0–10 roadmap in `docs/implementation_phases.md` and `spec.md` §8.
- Phase 7 Isaac Sim scaffolding adapted from v2 (host scripts, URDF helpers,
  vendor obtain script, staging URDFs). Vendor package obtained locally via
  `third_party/mycobot_ros2` → sibling symlink (gitignored).

## Acceptance checklist (Phase 0)

- [x] `pytest tests/unit` passes (16 passed).
- [x] `ruff check .` passes.
- [x] `ruff format --check .` passes.
- [x] CUDA host environment report is valid.
- [x] `pytest -m gpu tests/integration` passes (1 passed on NVIDIA GB10).
- [x] Complete lightweight suite passes (16 passed, 1 GPU test skipped).
- [x] Python sources compile and `pyproject.toml` parses.
- [x] Core package has no ROS / Isaac Kit / hardware / RL runtime dependency.

## Known environment status

The lightweight container does not include cuRobo or direct GPU access; its
full suite passes with the GPU test skipped. On the DGX Spark host, cuRobo
v0.8.0, CUDA allocation, and required public imports all pass. See
[`docs/phase0_environment.md`](docs/phase0_environment.md).

PyTorch 2.10.0+cu130 warns that GB10 compute capability 12.1 is newer than the
wheel's advertised maximum 12.0. Allocation and the public API smoke pass; the
warning is recorded rather than suppressed and must be monitored during the
Phase 1 planner warmup/kernel gate.

Isaac Sim scaffolding is present for Phase 7+ but is **not** a Phase 0
acceptance dependency. Staging URDFs under `assets/mycobot_280_m5/urdf/` are
copied from v2 and still require Phase 1 provenance review before they are
authoritative.

## Next step

Land the tested Phase 0 commit on `wip_phase0` and initialize `main`, then
create `wip_phase1` from `main`. Phase 1 starts with asset provenance review.
Do not start Phase 7 Isaac player work until Phases 0–6 (or at least 0–4) meet
their gates per the roadmap.
