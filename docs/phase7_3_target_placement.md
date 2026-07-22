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

- `min_center_separation_m` (default / floor:
  `target_edge_m + flange_diameter_assumption_m` for EE tip/flange clearance;
  must not fall back to `2 * target_edge_m` alone when the flange is larger
  than the cube edge)
- `keep_outs` — optional AABB list; target cubes may not intersect
- `max_placement_attempts` — random sampling budget (default 1000)

Core module: `mycobot_curobo.target_placement`.

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

## Related

- Phase 7.2: [`phase7_2_multi_target_contact.md`](phase7_2_multi_target_contact.md)
- Integration smoke (opt-in): `smoke_phase7_2_integration_2x5.sh`
- Roadmap: [`implementation_phases.md`](implementation_phases.md)
