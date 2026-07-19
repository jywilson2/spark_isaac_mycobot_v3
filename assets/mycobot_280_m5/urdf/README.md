# MyCobot 280 M5 URDF assets

## Staging copies (in git)

| File | Role |
|------|------|
| [`mycobot_280_m5_kinematics.urdf`](mycobot_280_m5_kinematics.urdf) | Joint chain only (`g_base` → `joint6_flange`). Useful for CI/FK without meshes. |
| [`mycobot_280_m5_curobo.urdf`](mycobot_280_m5_curobo.urdf) | v2-derived cuRobo-oriented URDF staging copy. **Phase 1 must re-validate** joint limits, TCP, collision spheres, and license before treating it as authoritative. |

These files were copied from `spark_isaac_mycobot_v2` as simulation scaffolding.
They are **not** Phase 1 acceptance evidence until provenance is recorded in
`STATUS.md` / Phase 1 docs.

## Full vendor package (local, gitignored)

```bash
./scripts/download_mycobot_ros2.sh
```

Clones or symlinks [elephantrobotics/mycobot_ros2](https://github.com/elephantrobotics/mycobot_ros2)
into `third_party/mycobot_ros2/`. Preferred URDF:

`mycobot_description/urdf/mycobot_280_m5/mycobot_280_m5.urdf`

On this workspace, a sibling checkout at
`/workspaces/isaac_ros-dev/src/mycobot_ros2` is preferred via a relative symlink.
