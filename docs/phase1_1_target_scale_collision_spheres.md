# Phase 1.1 — Target-scale collision-sphere coverage

**Status:** Option B **accepted and implementing** on `wip_phase1_1b`
(2026-08-01). Option A cover artifact (1012 spheres) is reused as the
world-only set; default robot YAML stays on scaffolding until the Option B
acceptance gate passes.  
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

## Option B implementation (in progress)

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
4. **Still open before re-arming default YAML:** GPU Option B self-clear +
   body-clip; Phase 7.1 / 7.2 GPU suites with dual overlay trial-armed;
   integration 2×5 headless + GUI; planning-time p50/p95; armed unseeded
   2×20 batch.

## Intent

Regenerate the MyCobot static cuRobo collision spheres so planning and
independent world clearance can detect cuboid obstacles as small as the Phase
7.2 target cubes (`target_edge_m`, default 14 mm), using a **sparse** cover
dictated by that edge length and each link’s mesh geometry **without**
destroying self-collision feasibility.

PhysX body contact remains Isaac playback evidence only. It does not feed
`plan_grasp`.

## Chosen covering algorithm (Option A — thickness-capped)

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
Option A full-replace (diagnosis only). Default robot YAML leaves the overlay
path commented out until the Option B gate passes. Suite load still fails
closed if `target_edge_m < min_detectable_obstacle_edge_m`.
`load_curobo_robot_config` strips project-only keys
(`min_detectable_obstacle_edge_m`, `collision_sphere_overlay_path`,
`collision_sphere_overlay_role`) before cuRobo `KinematicsLoaderCfg`. The
regenerator refuses overlays with `|center| > 0.5 m` or more than 2048
spheres (unit-scale guards).

## Sphere counts (E = 0.014 m)

| Link | Phase 1 scaffolding | Option A (thickness-capped) |
|------|--------------------:|----------------------------:|
| g_base | 4 | 256 |
| joint1 | 4 | 158 |
| joint2 | 4 | 191 |
| joint3 | 4 | 125 |
| joint4 | 4 | 93 |
| joint5 | 4 | 43 |
| joint6 | 4 | 81 |
| joint6_flange | 4 | 65 |
| **total** | **32** | **1012** |

The rejected first cover used 128 spheres with radii up to `2E` and failed
self-collision at the zero pose.

## Regenerate

```bash
./scripts/download_mycobot_ros2.sh   # if meshes missing
python3 scripts/host/regenerate_target_scale_collision_spheres.py
```

## Verification gates before re-arming

1. GPU: zero + mid-reach `validate_start_state` self-clear with trial overlay
   (`tests/integration/test_phase1_1_collision_spheres_gpu.py`) — **passed**.
2. GPU: body-clip edge-`E` cube yields non-positive sphere–AABB clearance —
   **passed**.
3. GPU: Phase 7.1 / 7.2 planning suites with overlay armed — **currently fail**
   (`Start or End state in collision`); do not re-arm until green.
4. Host headless + GUI integration smoke
   `smoke_phase7_2_integration_2x5.sh` (2 episodes × 5 targets; enable via
   `--with-integration-smoke`) per Phase 1.1 acceptance.
5. Planning-time evidence (spec §8 Phase 1.1, added 2026-08-01) — **not yet
   measured**: overlay-vs-scaffolding per-leg plan p50/p95 ratio on the host;
   plus a device calibration run if an embedded planner target (e.g. Jetson
   Orin AGX) is in scope for Phase 10+. Re-arming is reviewed against a
   budget declared for the intended deployment target; no numeric budget is
   invented ahead of the measurements.
