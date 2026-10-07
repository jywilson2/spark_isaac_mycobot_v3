# Touch-failure characterization (opt-in)

This is a measurement mode, not a roadmap phase. It does not change default
smoke, integration 2×5 placement, planner profiles, the armed Option B
collision-sphere robot, or pass/fail thresholds. It does not add guarded
contact moves, an ESDF/TSDF world, MPC, or a second planner.

Judgment: keep it as a dedicated, documented, opt-in mode under the current
Phase 7.2–7.5 contracts (`spec.md` §8, “Touch-failure characterization”).
Phase 7.4 remains the Z-variability phase; this report is not that phase.

```bash
./scripts/host/run_touch_characterization.sh
python3 scripts/run_touch_characterization.py --no-planner
```

Config: `config/touch_characterization.yml`. Figures and the summary JSON:
`docs/figures/`.

## What it measures

| Measurement | What it answers | What it does not change |
|-------------|-----------------|-------------------------|
| Failure bins | Why a seeded multi-target leg failed | Smoke pass/fail |
| Flange sweep | Neighbor blocks the Ø31 mm flange regardless of planner tuning | Placement floors |
| World-model proxy | Scaffolding spheres clear a cuboid the dense overlay hits | Armed Option B robot |
| Sphere-density curve | False-infeasible rate versus sphere count | Default sphere YAML |
| Workspace map vs AABB | Rim and start-collision counts of two placement policies | Integration sampler |
| Open-loop tip error | Executed tip error with the terminal joint snap off | Snap stays on for smoke |

Planning uses `MultiTargetEpisodeRunner` and `benchmark_reproducible`.
Phase 6 `planning_failure_category` maps `no_reachable_ik` to the IK bin.
Every other planning failure, including collision infeasibility, is
`trajopt_plan_grasp_failure`. The Phase 6 name is kept on each row as
`phase6_category`.

Geometric infeasibility is a sphere of radius `flange_diameter_m / 2` swept
from the pre-approach point to the contact point. Negative clearance to a
neighbor AABB wins over the planner result. The active contact cube is not
a neighbor.

World-model blindness (`dense_overlay_vs_scaffolding_32`) fires only on a
validated plan when scaffolding clearance is ≥ 0 and overlay clearance is
< 0. It is a proxy, not a PhysX replay of hundreds of fields.

The sphere curve’s known-valid queries are poses where every reference
sphere *centre* clears a small cuboid and at least one sphere *radius*
intersects it. Marking that pose in collision is a false infeasible relative
to the centre point cloud. Distal-only uses `joint5`, `joint6`, and
`joint6_flange`. Self versus world uses the scaffolding set and the overlay
set in memory. The 1012-sphere overlay is not written back as the default
robot.

Workspace-map draws are uniform among successful centres in
`artifacts/workspace/tip_contact_workspace_v1.json` that lie inside the
characterization field AABB. Forward AABB draws are uniform in that same
box. Rim and start-collision counts are independent. Accepted means neither.

Open-loop playback is `--disable-terminal-joint-snap` on
`isaac_sim/play_multi_target_suite.py`. The flag defaults off. Approach
direction for the error split is the opposite of the face outward normal.
`along_approach_m` is signed. These are simulation metres, not a hardware
accuracy claim.

## Results

Host command: `./scripts/host/run_touch_characterization.sh` (20 fields × 2
targets, seed 4242, one planning attempt per target, 200 placement draws, 8
joint samples, planner profile `benchmark_reproducible`). Exit 0 in about
7.5 minutes on the DGX Spark GPU. Numbers are simulation metrics only.
They are not a real-world accuracy claim. Full tables live in
`docs/figures/touch_characterization_summary.json`.

Figures:

- `docs/figures/touch_characterization_failure_bins.png` (and `.svg`)
- `docs/figures/touch_characterization_sphere_curve.png` (and `.svg`)
- `docs/figures/touch_characterization_open_loop_tip_error.png` (and `.svg`)

### Failure attribution (40 attempts)

| Bin | Count |
| --- | ---: |
| geometric infeasibility | 0 |
| world-model blindness (proxy) | 0 |
| IK failure | 0 |
| trajopt / plan_grasp failure | 40 |
| validation rejection | 0 |
| success | 0 |

Phase 6 category for all 40 is `trajectory_optimization_failure`. Planner
status strings:

| Status | Count |
| --- | ---: |
| Planning to grasp pose failed. | 29 |
| Goalset planning returned None. | 6 |
| Planning to approach pose failed. | 5 |

Those strings do not contain the Phase 6 IK or collision tokens, so they
stay in the trajopt bin. The analytic flange sweep blocked 0 of 40 targets
(`geometric_precheck`). The random sampler’s end-effector floor already
keeps neighbor cubes outside a flange-radius approach disk, so this config
does not produce geometric infeasibility. The world-model proxy did not
fire: it only compares dense-overlay clearance against the 32 scaffolding
spheres on a validated plan, and this run validated none.

### Sphere-cover curve (8 synthetic queries)

Known-valid queries place a grazing cuboid on a reference overlay sphere.
False-infeasible means the subset reports negative clearance.

| Label | Spheres | False-infeasible rate |
| --- | ---: | ---: |
| overlay 5% | 51 | 0.00 |
| overlay 10% | 100 | 0.00 |
| overlay 25% | 253 | 1.00 |
| overlay 50% | 505 | 0.875 |
| overlay 100% | 1012 | 1.00 |
| distal overlay (joint5, joint6, flange) | 189 | 1.00 |
| scaffolding self (32) | 32 | 0.00 |

The 50% point is below the 25% point because even subsampling can drop the
particular grazing sphere on one of the eight queries. The scaffolding-32
rate is 0 because these queries were built against overlay world spheres,
not the scaffolding set. The overlay stays unarmed as a YAML change; the
default robot file is untouched.

### Placement comparison (200 draws)

The characterization field AABB sits inside the 0.36 m radial rim, so rim
failures are 0 for both policies.

| Policy | Rim | Start collision | Accepted |
| --- | ---: | ---: | ---: |
| forward AABB | 0 | 1 | 199 |
| workspace map | 0 | 0 | 200 |

That is a one-draw difference on this box. The integration 2×5 suite stays
on its layout arc.

### Open-loop terminal error

The characterization planner produced no validated trajectory, so playback
used episode 0 of the existing
`artifacts/reports/phase7_2_multi_target_standard_2x10.bundle.json` with
`--disable-terminal-joint-snap --headless --auto-exit`. Kit exit 0. All 10
legs reported allowed tip contact. With the terminal joint snap off, tip
error versus the planned goal (simulation metres):

| Component | Median | Min | Max |
| --- | ---: | ---: | ---: |
| lateral | 0.73 mm | 0.44 mm | 1.94 mm |
| along approach (signed; negative is short of the goal) | −3.71 mm | −3.89 mm | −3.51 mm |
| total | 3.90 mm | 3.54 mm | 4.09 mm |

The shortfall is almost entirely along the approach normal. Default smoke
scripts still snap the terminal joints.
