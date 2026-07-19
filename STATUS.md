# STATUS — MyCobot 280 M5 Constrained Approach Planner

Last updated: **2026-07-18**

## Current phase

**Phase 1 — MyCobot 280 M5 model and cuRobo configuration: COMPLETE**

Roadmap: [`docs/implementation_phases.md`](docs/implementation_phases.md)  
Authoritative criteria: [`spec.md`](spec.md) §8 (Phases 0–10)

This status is initialized for v3. No v2 completion metrics, GUI results,
planning-success claims, or hardware-readiness claims carry forward.

## Phase roadmap (summary)

| Phase | Focus | Status |
|-------|-------|--------|
| 0 | Env / version guard | **Complete** |
| 1 | Robot model + spheres | **Complete** |
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
- Phase 1 cuRobo format-2.0 robot YAML, pinned vendor provenance/license,
  corrected published velocity limits, explicit bare-flange TCP, independent
  CPU FK, named-state reordering, and five-case FK regression fixture.
- Static collision spheres cover every configured robot collision link;
  self-collision is enabled and exercised by the GPU planner warmup.

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
acceptance dependency. The Phase 1 URDF derivative and vendor license are now
tracked with provenance; vendor meshes remain obtained into gitignored
`third_party/`.

## Acceptance checklist (Phase 1)

- [x] cuRobo v0.8.0 robot mapping loads and planner constructs on GPU.
- [x] Planner warmup passes with self-collision enabled.
- [x] Joint names/order, position/velocity/acceleration/jerk consistency pass.
- [x] `tcp_link` is an explicit identity transform at the bare flange.
- [x] Every collision link has static spheres (32 total).
- [x] Five independent FK regression cases pass.
- [x] cuRobo and CPU default TCP FK agree within 1 µm numerical tolerance.
- [x] Unit suite passes (29 tests); GPU integration passes (2 tests).
- [x] No physical robot command is issued.

## Next step

Land the tested Phase 1 commit on `wip_phase1`, rebase/fast-forward `main`, then
create `wip_phase2` from updated `main`. Phase 2 implements robust task-frame
and roll-goal generation. Before hardware use, review the reduced collision
sphere coverage against all vendor meshes.
