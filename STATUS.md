# STATUS — MyCobot 280 M5 Constrained Approach Planner

Last updated: **2026-07-19**

## Current phase

**Phase 6 — randomized workspace benchmark: COMPLETE**

Roadmap: [`docs/implementation_phases.md`](docs/implementation_phases.md)  
Authoritative criteria: [`spec.md`](spec.md) §8 (Phases 0–10)

This status is initialized for v3. No v2 completion metrics, GUI results,
planning-success claims, or hardware-readiness claims carry forward.

## Phase roadmap (summary)

| Phase | Focus | Status |
|-------|-------|--------|
| 0 | Env / version guard | **Complete** |
| 1 | Robot model + spheres | **Complete** |
| 2 | Task frames / roll goals | **Complete** |
| 3 | `plan_grasp` nominal planning | **Complete** |
| 4 | Independent validation | **Complete** |
| 5 | Execution + zero residual seam | **Complete** |
| 6 | Randomized benchmark | **Complete** |
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
- Phase 7 Isaac Sim scaffolding (host scripts, URDF helpers, vendor obtain
  script, staging URDFs) plus retirement isolation so V3 no longer depends on
  the prior-project tree. Vendor package obtained locally via
  `third_party/mycobot_ros2` → sibling symlink (gitignored). See
  [`docs/v2_retirement.md`](docs/v2_retirement.md).
- Phase 1 cuRobo format-2.0 robot YAML, pinned vendor provenance/license,
  corrected published velocity limits, explicit bare-flange TCP, independent
  CPU FK, named-state reordering, and five-case FK regression fixture.
- Static collision spheres cover every configured robot collision link;
  self-collision is enabled and exercised by the GPU planner warmup.
- Phase 2 typed surface targets, configurable signed tool-axis conventions,
  robust tangent fallback, deterministic roll generation, rotation/quaternion
  validation, goal-index mapping, and public cuRobo `GoalToolPose` conversion.
- Phase 3 approach-only `plan_grasp` adapter, typed plans/failures, valid
  trajectory extraction, selected-roll mapping, planner profiles, empty scene,
  and a fresh planner backend for every v0.8.0 call and retry.
- Phase 3 lifecycle corrected to require fresh backend → reset seed →
  configured public warmup → reset seed → exactly one `plan_grasp`.
- Specification and roadmap now make cuRobo the exclusive global and local
  motion planner. CPU fallback is prohibited unless supplied by pinned cuRobo
  and validated; residuals and integrations may not generate replacement
  trajectories.
- Documentation-only exclusivity update: 76 unit tests pass; diff whitespace
  and IDE diagnostics are clean. The local unified CI wrapper remains
  incomplete because Ruff is not installed, and pytest reports existing
  `.pytest_cache` permission warnings.
- Container Ruff bootstrap: Cursor rule `40-container-dev-tools.mdc` plus
  `scripts/ensure_container_dev_tools.sh`; `./scripts/run_verification.sh ci`
  now installs Ruff when missing and passes (79 unit tests + Ruff).
- Phase 4 independent validator with typed reports, simulation/hardware
  profiles, synthetic fail-closed coverage, and GPU FK/self-collision checks
  on an explicitly empty world.
- Phase 5 typed residual observations, exact zero-output correction,
  deterministic safety projection, replay state provider, timestamp/watchdog
  handling, and in-memory-only trajectory execution.
- Root-squashed container verification uses explicit writable pytest and Ruff
  cache paths; the cumulative 90-test CI gate is warning-free.
- Phase 6 typed benchmark configuration/cases/results/summaries, deterministic
  root-seed sampling, 20/100 frozen parameter fixtures, seven-category failure
  taxonomy, exact failed-request replay, and JSON/Markdown reporting.

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

## Acceptance checklist (Phase 2)

- [x] Signed TCP approach axis aligns for x/y/z and signs ±1.
- [x] All generated rotations are finite, orthonormal, and right-handed.
- [x] All quaternions are normalized scalar-first `wxyz`.
- [x] Roll generation preserves target position and deterministic ordering.
- [x] Degenerate/near-parallel tangent hints use deterministic fallback.
- [x] Seeded property test passes for 512 randomized normals.
- [x] Three-roll public cuRoboV2 `GoalToolPose` conversion passes on GPU.
- [x] Phase 2 unit tests pass (22 tests); cumulative lightweight suite passes.

## Acceptance checklist (Phase 3)

- [x] Reachable seeded target produces free-space and terminal trajectories.
- [x] Selected goal index maps to the configured roll candidate.
- [x] Padded/non-finite samples are excluded or fail closed.
- [x] Terminal TCP samples remain within 5 mm of the target-normal line.
- [x] Identical seeded requests use distinct planner instances and reproduce
      approach trajectories and terminal FK pose within defined tolerances.
- [x] Terminal FK endpoint reaches the target within planner position tolerance
      after mandatory fresh-backend warmup.
- [x] Expected infeasibility and malformed backend output return structured
      failure categories.
- [x] All returned plans remain non-executable pending Phase 4 validation.
- [x] No physical robot command is issued.

## Acceptance checklist (Phase 4)

- [x] Synthetic trajectory deviations are detected by unit tests.
- [x] A deliberately curved terminal path fails lateral validation.
- [x] A reversed-progress path fails monotonicity validation.
- [x] A misoriented TCP fails orientation validation.
- [x] Unevaluated world collision fails closed.
- [x] Joint-limit and dynamics violations report waypoint indices.
- [x] A valid nominal path becomes executable only after independent validation.
- [x] GPU integration independently validates FK, terminal accuracy, and
      self-collision clearance on an empty world.
- [x] Non-empty world clearance remains unevaluated / fail-closed until a
      supported adapter lands.
- [x] Cumulative unit suite passes (76 tests).
- [x] All GPU integrations pass (5 tests on NVIDIA GB10) with the corrected
      lifecycle; the known PyTorch CUDA-capability warning remains recorded.
- [x] No physical robot command is issued.

## Acceptance checklist (Phase 5)

- [x] `ZeroResidualCorrector` reproduces the nominal joint command stream
      exactly.
- [x] Oversized synthetic translation and rotation corrections are explicitly
      clipped.
- [x] Corrections outside the terminal target-normal corridor are rejected.
- [x] Invalid plans, stale state, watchdog expiry, and joint-envelope
      violations stop before command emission.
- [x] Non-zero residuals cannot generate replacement joint trajectories in
      Phase 5.
- [x] The only output adapter is an in-memory dry-run log.
- [x] Core Phase 5 modules have no physical driver, ROS, Isaac, or RL
      dependency.
- [x] Cumulative unit suite passes (90 tests); Ruff lint and format pass.
- [x] Existing host GPU integration suite passes (5 tests); the recorded GB10
      PyTorch capability warning remains visible.
- [x] No physical robot command is issued.

## Acceptance checklist (Phase 6)

- [x] Cases reproduce exactly from a root seed and validated YAML contract.
- [x] Every failed case carries a complete serialized planning request.
- [x] Matching JSON and Markdown reports default to `artifacts/benchmarks/`.
- [x] Aggregation includes every case and cannot suppress failures.
- [x] Smoke/regression fixtures contain 20/100 compact parameter-only cases.
- [x] Planner seed sweeps copy the profile seed and construct fresh planners.
- [x] Optional execution rejection remains separate from planning taxonomy.
- [x] Cumulative CI gate passes (97 unit tests plus Ruff lint/format).
- [x] Host GPU dual-run smoke subset passes with zero disagreement
      (2 frozen smoke cases × configured repeats; full 20-case stage remains
      available via `scripts/benchmark_random_targets.py`).
- [x] Cumulative host GPU suite passes (6 tests); GB10 PyTorch capability
      warning remains visible.
- [x] No physical robot command is issued.

## Next step

Land the tested Phase 6 commit on `wip_phase6`, rebase/fast-forward `main`,
then create `wip_phase7` from updated `main`. Phase 7 implements the
Isaac Sim validated-plan player and host headless/GUI smoke gates.
