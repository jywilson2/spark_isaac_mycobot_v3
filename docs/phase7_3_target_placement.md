# Phase 7.3 — Controllable target-block placement (under consideration)

**Status:** Under consideration / brainstorm with Cursor. Not started.
**Branch (when work begins):** `wip_phase7_3`

## Intent (draft)

Give operators and suite authors finer control over where Phase 7.2 numbered
target blocks are placed in `g_base`, beyond today’s `grid` AABB lattice and
fully enumerated `manual` lists. Exact APIs, sampling policies, and acceptance
gates are **to be defined later**.

## Brainstorm themes (non-normative)

Possible directions under discussion (none are committed requirements yet):

- Explicit per-target pose editors / YAML schemas with validation against
  reachability and mutual clearance.
- Seeded “constrained random” placement inside a declared volume with
  minimum-separation and keep-out regions.
- Named layouts (rows, arcs, shelves) parameterized without listing every
  centre by hand.
- Stronger CLI/GUI overrides for interactive re-placement during Isaac review.
- Clear fail-closed errors when a requested layout is infeasible (same spirit
  as Phase 7.2 structured planning failures).

## Also in this revision

Stabilize **GitHub Actions CI** execution for this repository. Landed on this
branch (workflow-only fix, no container/system Python change):

- [`.github/workflows/pytest.yml`](../.github/workflows/pytest.yml) installs
  CPU-safe `numpy` / `pyyaml` / `pytest` / `ruff`, then
  `pip install --no-deps -e .` (avoids pulling `nvidia-curobo` on runners).
- Sets `SPARK_PYTEST_PYTHON` to the `actions/setup-python` interpreter so
  `./scripts/run_verification.sh ci` does not run against bare
  `/usr/bin/python3` without pytest.

Also landed (Phase 7.2 gap / Phase 7.3 early placement tweak):

- Viewport-visible 7-segment target ID geometry in
  `isaac_sim/scene_setup.py` `add_target_label` (parent-local Z offset under
  the cube prim; an earlier world-center child translate double-counted pose).
- Grid mid-Z variability: centres spaced across
  `0.5 * arm_z_motion_range_m` about the field AABB mid-Z
  (`arm_z_motion_range_m` declared; typically vendor working radius 0.28 m).

Further volumetric / constrained-random placement APIs remain **to be defined**.

## Normative status

Until Phase 7.3 requirements are written into `spec.md` §8 and accepted:

- Prefer Phase 7.2 `grid` / `manual` placement contracts.
- Do not treat this brainstorm note as acceptance criteria.
- Do not implement placement features on `wip_phase7_2` after landing.

## Related

- Phase 7.2 report: [`phase7_2_multi_target_contact.md`](phase7_2_multi_target_contact.md)
- Roadmap: [`implementation_phases.md`](implementation_phases.md)
- Spec: `spec.md` §8 Phase 7.3 (placeholder)
