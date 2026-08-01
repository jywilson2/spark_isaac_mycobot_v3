"""Config-time dense-sphere vs neighbor packing checks for named suites.

Approximates the Option B acceptance geometry check without GPU IK: every
pair of same-z targets must keep an approach-plane centre separation at least
``edge + 2 * max_world_cover_radius`` (dense cover radii are capped at ``E``).
Suites already enforce a stronger EE-clearance floor; this test fails closed
if a future YAML packs denser than the cover can survive.
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from mycobot_curobo.multi_target import (
    load_multi_target_suite_config,
    sample_multi_target_episodes,
)
from mycobot_curobo.target_placement import (
    approach_plane_separation_m,
    ee_clearance_min_center_separation_m,
)

ROOT = Path(__file__).resolve().parents[2]
OVERLAY = ROOT / "config" / "robots" / "mycobot_280_m5_phase1_1_spheres.yml"
NAMED_SUITES = (
    ROOT / "config" / "phase7_2_multi_target_integration_2x5.yml",
    ROOT / "config" / "phase7_2_multi_target_standard_2x10.yml",
    ROOT / "config" / "phase7_2_multi_target_standard_2x20.yml",
)


def _max_overlay_radius_m() -> float:
    payload = yaml.safe_load(OVERLAY.read_text(encoding="utf-8"))
    radii = [
        float(sphere["radius"])
        for spheres in payload["collision_spheres"].values()
        for sphere in spheres
    ]
    return max(radii)


@pytest.mark.parametrize("suite_path", NAMED_SUITES, ids=lambda p: p.stem)
def test_named_suite_center_separation_survives_dense_cover_radii(suite_path: Path) -> None:
    if not OVERLAY.is_file():
        pytest.skip("Phase 1.1 overlay missing")
    if not suite_path.is_file():
        pytest.skip(f"suite missing: {suite_path}")

    suite = load_multi_target_suite_config(suite_path)
    episode = sample_multi_target_episodes(suite, root_seed=4242, episode_count=1)[0]
    targets = episode.field.targets
    edge = float(suite.target_edge_m)
    max_r = _max_overlay_radius_m()
    # Conservative floor: two dense radii plus cube edges in the approach plane.
    dense_floor = edge + 2.0 * max_r
    ee_floor = ee_clearance_min_center_separation_m(
        edge_m=edge,
        flange_diameter_assumption_m=suite.flange_diameter_assumption_m,
        ee_approach_clearance_m=suite.ee_approach_clearance_m,
    )
    required = max(dense_floor, ee_floor)

    for i, left in enumerate(targets):
        for right in targets[i + 1 :]:
            sep = approach_plane_separation_m(
                left.center_m, right.center_m, suite.outward_normal_base
            )
            assert sep + 1.0e-9 >= required, (
                f"{suite_path.name}: {left.target_id}/{right.target_id} "
                f"sep={sep:.6f} m < required {required:.6f} m "
                f"(dense_floor={dense_floor:.6f}, ee_floor={ee_floor:.6f})"
            )
