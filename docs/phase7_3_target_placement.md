# Phase 7.3 — Controllable target-block placement

**Status:** Implemented  
**Branch:** `wip_phase7_3`  
**Normative text:** [`spec.md`](../spec.md) §8 Phase 7.3.

## Intent

Finer control over Phase 7.2 numbered target-block placement than `grid` /
`manual` alone, with fail-closed separation and keep-out checks. Also stabilizes
GitHub Actions CI execution for this repository.

## Placement policies

| `placement` | Description |
|-------------|-------------|
| `manual` | Explicit `targets[]` (Phase 7.2) |
| `grid` | XY lattice + mid-Z band; per-episode phase shift |
| `random` | Seeded constrained-random in AABB / Z band |
| `layout` | Named `rows` or `arc` layouts |

Shared controls:

- `min_center_separation_m` (default / floor, **approach-plane** metric:
  `target_edge_m + flange_diameter_assumption_m + ee_approach_clearance_m`,
  with `ee_approach_clearance_m` defaulting to `flange_diameter_assumption_m`)
- `keep_outs` — optional AABB list; target cubes may not intersect
  (grid placement retries seed offsets when a phase would violate)
- EE-clearance floor is a **lower bound** on centre spacing (not a packing
  density target); fields may use the radial envelope with keep-outs
- `max_placement_attempts` — random sampling budget (default 1000)

Core module: `mycobot_curobo.target_placement`.

## Demo video

[`videos/mycobot_280_m5_2x20.mp4`](videos/mycobot_280_m5_2x20.mp4)
(1 min 58 s, 2880×1800 @ 30 fps, H.264, 6.4 MB; captured 2026-07-23 on the
DGX Spark host with the smoke wrapper's `--record` option).

The clip shows `smoke_phase7_2_standard_2x20.sh --gui` — the densest named
suite (2 episodes × 20 targets on the two-ring manual field, 14 mm cubes) —
and demonstrates the completed Phase 7.3 machinery end to end:

- fail-closed placement: 20 centres validated against the approach-plane
  EE-clearance floor and the ±0.10 m base keep-out before planning starts;
- per-episode shuffle contact order and planner seeds over an identical
  manual field;
- viewport 7-segment target ID labels and contact-state cube highlights
  (the active target renders yellow, contacted cubes are removed);
- deferral / reconsider replanning when a leg fails validation mid-episode
  (console warnings show per-leg `TIP CONTACT a->b plan_s/motion_s/ttc_s`).

The file is named for the robot and suite size (`mycobot_280_m5` + `2x20`).
Companion assets: `videos/mycobot_280_m5_2x20_preview.gif` (18 s autoplaying
excerpt embedded in the README) and `videos/mycobot_280_m5_2x20_poster.jpg`
(still frame). Note GitHub does not stream repository mp4s in the browser —
raw links are served as `application/octet-stream` with `nosniff`, so
clicking the mp4 downloads it; play it locally with any H.264-capable
player. The README streams the same recording in an inline player via the
manually uploaded GitHub `user-attachments` asset
`https://github.com/user-attachments/assets/e1632486-8215-4b7e-8963-d726cd621b28`
(uploaded 2026-07-23 through the GitHub web editor; the repository copies
here remain the canonical offline artifacts).

## Example configs

- `config/phase7_3_multi_target_random.yml`
- `config/phase7_3_multi_target_layout_rows.yml`
- `config/phase7_3_multi_target_layout_arc.yml`

```bash
./scripts/host/smoke_phase7_2_multi_target.sh --config config/phase7_3_multi_target_random.yml --gui --auto-exit
```

## Also landed on this branch

- GitHub Actions CI: CPU deps + `--no-deps` editable install +
  `SPARK_PYTEST_PYTHON` in `.github/workflows/pytest.yml`
- Viewport 7-segment target ID labels (parent-local Z offset)
- Grid mid-Z variability (`0.5 * arm_z_motion_range_m`)
- Contact-state cube highlights; tip collision vs non-contact targets

## Measured tip-contact workspace map (candidate region)

Before expanding integration `field_aabb` further, measure +Z tip-face
`plan_grasp` reachability under the radial rim / keep-outs:

```bash
./scripts/host/spark_host_exec.sh python \
  scripts/host/measure_tip_contact_workspace.py \
  --grid-step-m 0.06 --z-layers 2 \
  --output artifacts/workspace/tip_contact_workspace_v1.json
```

Artifact declaration: `measured_tip_contact_candidate_region_v1`. Host GPU
evidence (2026-07-22): **86/114** successes; success AABB roughly
`[-0.22,-0.22,0.10]…[0.26,0.26,0.22]` m in `g_base`. This is **not** a claim
that the full geometric disk is dexterous. Integration 2×5 uses a
multi-quadrant open arc (`layout: arc`, R=0.20 m, span≈240°) of flange-sized
cubes under flange-face containment (not a forward-only AABB grid).

CPU unit coverage: `tests/unit/test_tip_contact_workspace.py`.

## Related

- Phase 7.2: [`phase7_2_multi_target_contact.md`](phase7_2_multi_target_contact.md)
- Integration smoke (opt-in): `smoke_phase7_2_integration_2x5.sh`
- Roadmap: [`implementation_phases.md`](implementation_phases.md)
