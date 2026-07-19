# Phase 7.1 — Unknown-start normal-approach cube visualization

## Status

**Complete on `wip_phase7_1`.** Host CI/GPU/GUI acceptance evidence recorded below.

Authoritative acceptance criteria are in [`spec.md`](../spec.md) §8.

## Purpose

Run a configurable, replayable set of cuRobo-planned episodes in which the
circular bare-flange face approaches a small cube along the cube-face normal
and configured signed tool axis. Stream readable results while the suite runs,
then write matching machine-readable aggregate evidence.

Phase 7.1 extends, but does not replace:

- Phase 6 deterministic sampling, failure taxonomy, and replay;
- Phase 4 independent terminal/collision validation; and
- Phase 7 Isaac validated-plan playback.

## Implemented boundary

- `config/phase7_1_cube_suite.yml` — five episodes, A/D defaults, optional B/C,
  14 mm cube, **0.08 m** terminal standoff (host-feasible with collision
  spheres), goal-joint bank for Mode D FK-aligned cubes inside declared AABBs,
  lighting, and contact gates.
- `cube_scene.py` / `cube_suite.py` — typed cube geometry, scene revisions,
  frozen replay, A–D scheduling, console/JSON aggregation.
- Planner/validation — per-episode in-memory cuboid scenes, fresh-backend
  `plan_cspace` Mode C relocation, fail-closed cube-world clearance, Mode A
  start preflight.
- Isaac host path splits **planning** (`isaac_sim/plan_cube_suite.py`, cuRobo
  only) from **playback** (`isaac_sim/play_cube_suite.py`, Kit only) so Warp
  imported by cuRobo cannot break SimulationApp startup.
- `scene_setup.py` — dome + distant lights before reset; static
  contact-reporting cubes (`displayOpacity` as float array).
- `contact_monitor.py` — PhysX prohibited arm-to-cube counts only.
- Playback uses labeled joint-position **resets** only; planned motion uses
  drive targets. Tip metrics stay null / `not_evaluated`.

## Host commands

```bash
# Planning-only bundle (no Kit)
./scripts/host/spark_host_exec.sh \
  ./scripts/host/env.isaac_host.sh   # then isaac python:
# isaac_sim/plan_cube_suite.py --output-bundle artifacts/reports/....bundle.json

# Full smoke (plan process + Kit process)
./scripts/host/spark_host_exec.sh \
  ./scripts/host/smoke_phase7_1_cube_suite.sh --headless --auto-exit
./scripts/host/spark_host_exec.sh \
  ./scripts/host/smoke_phase7_1_cube_suite.sh --gui --auto-exit --all-modes
```

`./scripts/run_verification.sh spark` runs Phase 7 GUI smoke then the Phase 7.1
GUI `--all-modes` smoke.

## Measured acceptance evidence (2026-07-19)

| Gate | Result |
|------|--------|
| Container CI | 119 unit tests + Ruff lint/format **PASS** |
| Host GPU (`ci --with-gpu`) | 8 integration tests **PASS** |
| Phase 7 GUI smoke | `lighting_ready=true`, 6 waypoints, tip `not_evaluated` **PASS** |
| Phase 7.1 headless | 5/5 successes, 0 prohibited contacts, tip null **PASS** |
| Phase 7.1 GUI `--all-modes` | A/B/C exercised; 4/5 successes + 1 structured trajopt failure; 0 contacts on played episodes; tip null **PASS** |

Headless aggregate (seed 123): success_rate `1.0`, lateral p50/p95 ≈
`8.8e-5` / `6.7e-4` m, axis p50/p95 ≈ `3.6e-5` / `1.0e-2` rad.

These are **simulation** metrics only. No physical accuracy claim is made.

## Mode contract

### A — Independent unknown start (default)

Seeded bank starts; invalid starts fail closed via preflight.

### B — Chained start (optional)

Episode `k+1` begins at the prior successful endpoint (`use_last_success`).

### C — Relocate then approach (optional)

cuRobo `plan_cspace` unknown→nest, then `plan_grasp` nest→cube.

### D — 3D goal diversity (default)

FK-aligned cube centres from a seeded goal-joint bank, accepted only when the
cube centre lies in a declared conservative `g_base` AABB.

## Reporting

Live console rows and `artifacts/reports/phase7_1_cube_suite.json` include
pass/fail, contacts, clearances, p50/p95, seed, and frozen requests. Tip
position/orientation remain null and `not_evaluated`.
