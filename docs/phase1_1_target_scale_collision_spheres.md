# Phase 1.1 — Target-scale collision-sphere coverage

**Status:** Option A implemented; overlay disarmed (planning regressions)  
**Normative text:** [`spec.md`](../spec.md) §8 Phase 1.1.

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
`collision_sphere_overlay_path` is set**. Default robot YAML leaves Option A
commented out: trial-enabled GPU self-clear passes, but arming regresses Phase
7.1 / 7.2 planning against target cubes. Suite load still fails closed if
`target_edge_m < min_detectable_obstacle_edge_m`.
`load_curobo_robot_config` strips project-only keys before cuRobo
`KinematicsLoaderCfg`. The regenerator refuses overlays with `|center| > 0.5 m`
or more than 2048 spheres (unit-scale guards).

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
