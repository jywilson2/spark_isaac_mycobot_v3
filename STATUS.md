# STATUS — MyCobot 280 M5 Constrained Approach Planner

Last updated: **2026-07-19**

## Current phase

**Phase 7.1 — Unknown-start normal-approach cube visualization: COMPLETE**

Roadmap: [`docs/implementation_phases.md`](docs/implementation_phases.md)  
Authoritative criteria: [`spec.md`](spec.md) §8 (Phases 0–11)

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
| 7 | Isaac Sim closed-loop viz/validation | **Complete** |
| 7.1 | Unknown-start cube approach visualization | **Complete** |
| 8 | Bounded residual RL (sim only) | Planned |
| 9 | Fabricated contact test tool | Requirements finalized |
| 9.1 | Contact test tool evaluation | Requirements finalized |
| 10 | Hardware interface + dry-run | Planned |
| 11 | Physical MyCobot 280 M5 validation | Planned |

## Implemented

- Python 3.10+ `src/` project layout.
- Exact cuRobo v0.8.0 Git-tag dependency declaration.
- Phase 0–7 complete (see prior STATUS history and phase reports).
- Phase 7.1 cube suite: validated config, FK-aligned Mode D goal bank,
  cube-world clearance, Mode C `plan_cspace` relocation, illuminated Isaac
  plan/playback process split, drive-target motion, PhysX prohibited-contact
  evidence, and null/`not_evaluated` tip metrics.

## Acceptance checklist (Phase 7.1)

- [x] Default run executes five episodes with Modes A and D.
- [x] Validation runs exercise all Modes A–D (A/B/C starts + D goals).
- [x] Unknown starts and 3D goals are seeded and exactly replayable.
- [x] Core configuration, cube scene geometry, A–D episode scheduler, report
      serialization, and fail-closed cube clearance evaluator are unit tested.
- [x] Every PASS satisfies independent geometry, limits/dynamics, and
      self/world-collision validation; unevaluated cube-scene clearance fails.
- [x] Isaac reports zero prohibited arm-to-cube/environment contact events on
      played episodes (headless 5/5; GUI all-modes 4/5 played with 0 contacts).
- [x] Per-episode results stream live; final console and JSON aggregates agree.
- [x] Lateral/axis p50 and p95 plus all failure counts are reported.
- [x] Isaac tip metrics remain null/`not_evaluated`.
- [x] Container CI (119 unit tests + Ruff), host GPU (8), Phase 7 GUI smoke,
      and Phase 7.1 headless/GUI smokes pass.
- [x] No physical command, alternate planner, or physical-accuracy claim is
      introduced.

### Measured host evidence (2026-07-19)

- Headless seed 123: **5/5** successes, success_rate `1.0`, 0 prohibited
  contacts, tip null; lateral p50/p95 ≈ `8.8e-5` / `6.7e-4` m.
- GUI `--all-modes`: A/B/C exercised; **4/5** successes + 1 structured
  trajopt failure; 0 contacts on played episodes; tip null; smoke exit 0.
- Simulation metrics only — not physical accuracy evidence.

## Next step

Begin Phase 8 on `wip_phase8` (bounded residual RL in sim only), or Phase 9
tool fabrication on `wip_phase9` when scheduled. Preserve Phase 7/7.1 GUI smoke
gates for any Isaac-path changes.
