# Phase 7 — Isaac Sim validated-plan playback

## Scope

Phase 7 exports an independently validated cuRobo plan through a versioned,
Isaac-neutral JSON contract and replays its named joint trajectory in Isaac Sim
6.x. Isaac remains an execution visualization/measurement backend; it does not
plan, alter, or authorize trajectories.

## Playback contract

`mycobot_curobo.plan_io.PlaybackPlan` carries schema version 1, request and
validation status, exact six-joint order, SI timestep/positions/optional
velocities, target pose (`wxyz`), approach direction, selected roll, and a
compact copy of cuRobo validation metrics. `require_executable_plan` rejects
plans not explicitly marked executable before the player imports
`SimulationApp`.

The committed `tests/data/phase7_validated_plan.json` is a compact synthetic
playback fixture. It exercises visualization wiring only and is not planning,
workspace, or physical-accuracy evidence.

## Host workflow

Isaac Kit runs on the DGX Spark host. From the Isaac ROS/Cursor container,
delegate each command:

```bash
./scripts/host/spark_host_exec.sh ./scripts/host/check_prereqs.sh
./scripts/host/spark_host_exec.sh ./scripts/host/smoke_isaac_viz.sh --headless
./scripts/host/spark_host_exec.sh ./scripts/host/smoke_isaac_viz.sh --gui --auto-exit
# Keep the window open to verify lighting (close Kit to finish):
./scripts/host/spark_host_exec.sh ./scripts/host/smoke_isaac_viz.sh --gui --no-auto-exit
```

The smoke checks host prerequisites, obtains the pinned vendor URDF when
missing, creates the prepared USD when missing, and runs
`isaac_sim/play_nominal_plan.py`. Players disable Kit auto light-rig before
`open_stage` (avoids the "No lights found… applying Default" warning), create
UsdLux lights, and switch the viewport to stage-lighting mode
(`SetLightingMenuModeCommand`); otherwise Kit can leave stage lights hidden.
`./scripts/run_verification.sh spark` makes the GUI smoke mandatory and provides
no environment bypass.

The player discovers the articulation root, maps exact revolute names to Isaac
DOF indices, applies each waypoint at `dt_s`, and writes a separate simulation
metrics JSON. If the imported USD has no `tcp_link` prim, successful joint
playback is still reported while tip metrics remain `null` and
`not_evaluated`; values are never inferred or fabricated.

## Verification

- Container CI: exit 0 — 108 unit tests, Ruff lint, and Ruff format.
- Host prerequisite check: exit 0 — Isaac Sim `python.sh`, vendor URDF, and
  host paths resolved.
- URDF conversion: exit 0 — Isaac 6 generated the nested prepared
  `mycobot_280_m5.usda`; smoke discovery accepts the importer output layout.
- Host headless smoke: exit 0 — all 6 waypoints played.
- Host GUI smoke: exit 0 — all 6 waypoints played and auto-exited.

The prepared USD does not currently contain an exact `tcp_link` prim, so both
smokes correctly report `tip_metrics_status: not_evaluated` with null position
and orientation errors. Joint playback acceptance passed; tip metrics were not
invented. Isaac emitted existing audio-device and protobuf registration
warnings; they did not prevent stage/articulation playback.

Simulation metrics are simulation-only and do not support a real-world
accuracy claim. No physical hardware command is present.
