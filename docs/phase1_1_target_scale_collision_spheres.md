# Phase 1.1 — Target-scale collision-sphere coverage

**Status:** Implemented  
**Normative text:** [`spec.md`](../spec.md) §8 Phase 1.1.

## Intent

Regenerate the MyCobot static cuRobo collision spheres so planning and
independent world clearance can detect cuboid obstacles as small as the Phase
7.2 target cubes (`target_edge_m`, default 14 mm), using a **sparse** cover
dictated by that edge length and each link’s mesh geometry.

PhysX body contact remains Isaac playback evidence only. It does not feed
`plan_grasp`.

## Implemented covering algorithm

Host script:
`scripts/host/regenerate_target_scale_collision_spheres.py`

1. Load each collision-link COLLADA mesh (`float_array` ids containing
   `position`, scaled by `<unit meter>`).
2. Apply URDF collision origin `xyz` / `rpy` into the link frame.
3. Voxel-downsample vertices at pitch `0.5 * E`.
4. Greedy ball cover: seed at the first remaining sample; cover neighbors within
   `max_radius = 2 * E`; radius at least `0.5 * E`.
5. Grow existing radii (still capped at `2 * E`) so remaining raw vertices near a
   sphere are covered; add a new min-radius sphere only when a vertex lies beyond
   the cap.
6. Write overlay
   [`config/robots/mycobot_280_m5_phase1_1_spheres.yml`](../config/robots/mycobot_280_m5_phase1_1_spheres.yml).

`load_robot_model_spec` / `load_curobo_robot_config` merge the overlay. Suite
load fails closed if `target_edge_m < min_detectable_obstacle_edge_m`.

## Sphere counts (E = 0.014 m)

| Link | Phase 1 scaffolding | Phase 1.1 cover |
|------|--------------------:|----------------:|
| g_base | 4 | 23 |
| joint1 | 4 | 24 |
| joint2 | 4 | 14 |
| joint3 | 4 | 23 |
| joint4 | 4 | 20 |
| joint5 | 4 | 9 |
| joint6 | 4 | 12 |
| joint6_flange | 4 | 3 |
| **total** | **32** | **128** |

## Regenerate

```bash
./scripts/download_mycobot_ros2.sh   # if meshes missing
python3 scripts/host/regenerate_target_scale_collision_spheres.py
```

## Review / follow-up

- Host GUI smoke: body-contact rate vs prior sparse spheres on dense multi-target
  fields.
- Optional later: positive `minimum_world_collision_clearance_m` margin.
