# MyCobot 280 M5 URDF assets

## Phase 1 copies (in git)

| File | Role |
|------|------|
| [`mycobot_280_m5_kinematics.urdf`](mycobot_280_m5_kinematics.urdf) | Joint chain only (`g_base` → `joint6_flange`). Useful for CI/FK without meshes. |
| [`mycobot_280_m5_curobo.urdf`](mycobot_280_m5_curobo.urdf) | Phase 1 cuRobo URDF derived from the pinned vendor file. Vendor zero-velocity placeholders are replaced by the published 160 deg/s maximum; transforms and position limits are unchanged. |

Source: `elephantrobotics/mycobot_ros2`, branch `humble`, commit
`3999e2cda7460d61f4fd2ffaa31049f000eae7a8`, BSD-2-Clause. The retained license
is [`../VENDOR_LICENSE`](../VENDOR_LICENSE). Validation and assumptions are in
[`docs/phase1_robot_model.md`](../../../docs/phase1_robot_model.md).

## Full vendor package (local, gitignored)

```bash
./scripts/download_mycobot_ros2.sh
```

Clones or symlinks [elephantrobotics/mycobot_ros2](https://github.com/elephantrobotics/mycobot_ros2)
into `third_party/mycobot_ros2/`. Preferred URDF:

`mycobot_description/urdf/mycobot_280_m5/mycobot_280_m5.urdf`

On this workspace, a sibling checkout at
`/workspaces/isaac_ros-dev/src/mycobot_ros2` is preferred via a relative symlink.
