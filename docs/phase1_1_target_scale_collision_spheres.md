# Phase 1.1 — Target-scale collision-sphere coverage

**Status:** Option B **armed** in default robot YAML (2026-08-01) on
`wip_phase1_1b`. Scaffolding (32) owns self-collision; the Option A cover
(1012 thickness-capped spheres) attaches to FIXED `*_world_cover` children
for world checks only.  
**Normative text:** [`spec.md`](../spec.md) §8 Phase 1.1.

## Option A regression diagnosis (2026-08-01)

Host script:
`scripts/host/diagnose_phase1_1_option_a_regression.sh` (forces
`collision_sphere_overlay_role: replace`).

| Suite | Finding |
|-------|---------|
| Phase 7.1 | Start clear (`clearance ≈ 0.23 m`). Tip-contact `plan_grasp` fails with cuRobo `Start or End state in collision` / `No grasp in goal set was reachable` — the **active cube stayed in the planning world**, so dense flange/body spheres collide with tip goals. |
| Phase 7.2 | Active cube already omitted; start clearances vs neighbors were non-negative under project sphere–AABB math for the sampled 2×5 seed. Residual armed failures are addressed by the dual-role split (self cost) plus the shared omit-active invariant applied uniformly (including Phase 7.1). |

**cuRobo API (v0.8.0):** `self_collision_ignore` is **per-link**
(`SelfCollisionKinematicsCfg.compute_sphere_pair_distance_with_link_pair_ignores`
in `/home/jywilson/curobo`). Option B therefore attaches dense spheres to
fixed virtual child links `*_world_cover` and ignores those links for
self-collision.

## Option B implementation (complete)

1. **Dual-role merge** (`apply_collision_sphere_overlay`, default role
   `dual`): keep 32 scaffolding spheres on real links; place 1012 dense
   spheres on `*_world_cover` FIXED children; extend ignore map so active
   self-collision link-pair count equals scaffolding-only. Role `replace`
   retained for Option A diagnosis only. Project-only key
   `collision_sphere_overlay_role` is stripped before cuRobo.
2. **Shared world builder** (`mycobot_curobo.planning_world`): omit the
   active contact cube for tip-contact planning/validation. Phase 7.2 runner
   and Phase 7.1 tip-contact plan path use it; start preflight and Mode C
   relocation still keep the cube for body-clip detection.
3. **Config-time field check:** unit test
   `tests/unit/test_option_b_field_geometry.py` over named 2×5 / 2×10 / 2×20
   suites.
4. **Armed integration 2×5 (2026-08-01):** headless and GUI both exit 0 with
   dual overlay via
   `scripts/host/smoke_phase7_2_integration_2x5_option_b.sh --root-seed 4242`
   — 2/2 episodes, 10/10 tip contacts, 0 body contacts, 0 planning failures.
5. **Planning-time evidence (host DGX Spark):** integration 2×5 seed 4242 —
   scaffolding p50/p95 = 4.282 / 6.244 s; Option B dual = 5.000 / 6.993 s;
   ratios **1.168× / 1.120×**. Declared host budget p50 ≤ 1.50×, p95 ≤ 2.00×
   → **PASS**. See `artifacts/reports/phase1_1_option_b_timing/` (local).
   Orin AGX calibration not in scope for this landing.
6. **Armed unseeded 2×20 (2026-08-01):** 10/10 suite passes; 40 tip contacts /
   run; 0 body contacts; 0 target failures / failed episodes; 33 planning
   retries absorbed by budgets. See
   `artifacts/reports/phase1_1_option_b_unseeded_2x20/` (local).
7. **Default YAML armed** with
   `collision_sphere_overlay_path` + `collision_sphere_overlay_role: dual`.
   Post-re-arm headless integration 2×5 (`--root-seed 4242`, no trial
   wrapper) also exit 0 — 10 tip / 0 body / 0 plan fails.

## Intent

Regenerate the MyCobot static cuRobo collision spheres so planning and
independent world clearance can detect cuboid obstacles as small as the Phase
7.2 target cubes (`target_edge_m`, default 14 mm), using a **sparse** cover
dictated by that edge length and each link’s mesh geometry **without**
destroying self-collision feasibility.

PhysX body contact remains Isaac playback evidence only. It does not feed
`plan_grasp`.

## Chosen covering algorithm (Option A cover artifact — thickness-capped)

Host script:
`scripts/host/regenerate_target_scale_collision_spheres.py`

1. Load each collision-link COLLADA mesh (`float_array` ids containing
   `position`, scaled by `<unit meter>`).
2. Apply URDF collision origin `xyz` / `rpy` into the link frame.
3. Voxel-downsample vertices at pitch `0.4 * E`.
4. Greedy ball cover: seed at the first remaining sample; estimate **local
   medial / thickness** from nearby vertices; set
   `max_radius = min(0.85 * medial, E)`; floor at `0.25 * E` (capped by
   thickness).
5. Densify for raw vertices still outside all spheres (same radius caps).
6. Write overlay
   [`config/robots/mycobot_280_m5_phase1_1_spheres.yml`](../config/robots/mycobot_280_m5_phase1_1_spheres.yml)
   with `generator.option: A_thickness_capped`.

`load_robot_model_spec` / `load_curobo_robot_config` merge the overlay **when
`collision_sphere_overlay_path` is set**. Default role is Option B `dual`
(scaffolding + virtual world-cover links). Role `replace` is the historical
Option A full-replace (diagnosis only). Suite load still fails closed if
`target_edge_m < min_detectable_obstacle_edge_m`.
`load_curobo_robot_config` strips project-only keys
(`min_detectable_obstacle_edge_m`, `collision_sphere_overlay_path`,
`collision_sphere_overlay_role`) before cuRobo `KinematicsLoaderCfg`. The
regenerator refuses overlays with `|center| > 0.5 m` or more than 2048
spheres (unit-scale guards).

## Sphere counts (E = 0.014 m)

| Link | Phase 1 scaffolding | Option A cover (world-only under B) |
|------|--------------------:|------------------------------------:|
| g_base | 4 | 256 |
| joint1 | 4 | 158 |
| joint2 | 4 | 191 |
| joint3 | 4 | 125 |
| joint4 | 4 | 93 |
| joint5 | 4 | 43 |
| joint6 | 4 | 81 |
| joint6_flange | 4 | 65 |
| **total** | **32** | **1012** (+ 32 scaffolding when dual-armed = **1044**) |

The rejected first cover used 128 spheres with radii up to `2E` and failed
self-collision at the zero pose.

## Regenerate

```bash
./scripts/download_mycobot_ros2.sh   # if meshes missing
python3 scripts/host/regenerate_target_scale_collision_spheres.py
```

## Verification gates (re-arming complete)

1. GPU: zero + mid-reach self-clear with dual overlay — **passed**.
2. GPU: body-clip edge-`E` cube yields non-positive clearance — **passed**.
3. GPU: Phase 7.1 / 7.2 planning with dual trial-armed — **passed**.
4. Host headless + GUI integration 2×5 with dual — **passed**.
5. Planning-time evidence vs scaffolding — **passed** (host budget 1.50× /
   2.00×; measured 1.168× / 1.120×).
6. Armed unseeded 2×20 robustness — **passed** (10/10).
