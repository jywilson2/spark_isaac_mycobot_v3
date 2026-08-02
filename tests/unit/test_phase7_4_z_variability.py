"""Unit tests for Phase 7.4 Z band, delta_z_m, Z-aware floor, dexterous reach."""

from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path

import pytest
import yaml

from mycobot_curobo.errors import ConfigurationError
from mycobot_curobo.multi_target import (
    build_grid_centers,
    build_target_field,
    load_multi_target_suite_config,
    sample_multi_target_episodes,
)
from mycobot_curobo.target_placement import (
    DEFAULT_DEXTEROUS_REACH,
    DEFAULT_Z_BAND_FRACTION,
    DexterousReachModel,
    ReachRejectionBudget,
    approach_plane_separation_m,
    arm_reach_z_bounds,
    build_random_centers,
    center_outside_arm_reach,
    center_outside_dexterous_reach,
    resolve_z_band_half_m,
    top_face_delta_m,
    validate_centers_separation,
    z_aware_required_separation_m,
    z_band_bounds,
)

ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_ARTIFACT = ROOT / "artifacts/workspace/tip_contact_workspace_v1.json"


def test_default_z_band_is_half_arm_range() -> None:
    assert DEFAULT_Z_BAND_FRACTION == pytest.approx(0.5)
    half = resolve_z_band_half_m(arm_z_motion_range_m=0.28)
    assert half == pytest.approx(0.5 * 0.5 * 0.28)
    mid, z_lo, z_hi = z_band_bounds((0.0, 0.0, 0.12), (0.2, 0.2, 0.22), arm_z_motion_range_m=0.28)
    assert mid == pytest.approx(0.17)
    assert z_hi - z_lo == pytest.approx(0.14)


def test_delta_z_m_overrides_fraction_and_is_unclamped() -> None:
    half = resolve_z_band_half_m(arm_z_motion_range_m=0.28, z_band_fraction=0.5, delta_z_m=0.40)
    assert half == pytest.approx(0.20)
    half_frac = resolve_z_band_half_m(arm_z_motion_range_m=0.28, z_band_fraction=1.5)
    assert half_frac == pytest.approx(0.5 * 1.5 * 0.28)


def test_z_aware_floor_adds_clamped_top_face_delta() -> None:
    required = z_aware_required_separation_m(
        (0.0, 0.0, 0.10),
        (0.10, 0.0, 0.20),
        min_center_separation_m=0.076,
        edge_m=0.014,
        outward_normal_base=(0.0, 0.0, 1.0),
        z_separation_gain=1.0,
        pre_approach_distance_m=0.05,
    )
    assert required == pytest.approx(0.076 + 0.05)
    flat = z_aware_required_separation_m(
        (0.0, 0.0, 0.16),
        (0.10, 0.0, 0.16),
        min_center_separation_m=0.076,
        edge_m=0.014,
        outward_normal_base=(0.0, 0.0, 1.0),
        z_separation_gain=1.0,
        pre_approach_distance_m=0.05,
    )
    assert flat == pytest.approx(0.076)
    assert top_face_delta_m(
        (0.0, 0.0, 0.10),
        (0.0, 0.0, 0.20),
        edge_m=0.014,
        outward_normal_base=(0.0, 0.0, 1.0),
    ) == pytest.approx(0.10)


def test_z_separation_gain_below_one_fail_closed(tmp_path: Path) -> None:
    source = ROOT / "config/phase7_2_multi_target_grid.yml"
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    payload["z_separation_gain"] = 0.5
    path = tmp_path / "gain.yml"
    path.write_text(yaml.safe_dump(payload), encoding="utf-8")
    with pytest.raises(ConfigurationError, match="z_separation_gain"):
        load_multi_target_suite_config(path)


def test_validate_z_aware_pair_fail_closed() -> None:
    with pytest.raises(ConfigurationError, match="min_center_separation"):
        validate_centers_separation(
            [(0.0, 0.0, 0.10), (0.08, 0.0, 0.20)],
            min_center_separation_m=0.076,
            edge_m=0.014,
            z_separation_gain=1.0,
            pre_approach_distance_m=0.05,
        )


def test_arm_reach_rejects_high_z() -> None:
    lo = (0.0, 0.0, 0.12)
    hi = (0.2, 0.2, 0.22)
    mid, z_lo, z_hi = arm_reach_z_bounds(lo, hi, arm_z_motion_range_m=0.28)
    assert mid == pytest.approx(0.17)
    assert z_hi - z_lo == pytest.approx(0.28)
    assert center_outside_arm_reach(
        (0.1, 0.0, z_hi + 0.02),
        edge_m=0.014,
        field_minimum_m=lo,
        field_maximum_m=hi,
        arm_z_motion_range_m=0.28,
    )
    assert not center_outside_arm_reach(
        (0.1, 0.0, mid),
        edge_m=0.014,
        field_minimum_m=lo,
        field_maximum_m=hi,
        arm_z_motion_range_m=0.28,
    )


def test_dexterous_reach_rejects_far_high_centre() -> None:
    assert center_outside_dexterous_reach(
        (0.30, 0.30, 0.45),
        edge_m=0.014,
        outward_normal_base=(0.0, 0.0, 1.0),
        reach=DEFAULT_DEXTEROUS_REACH,
    )
    assert not center_outside_dexterous_reach(
        (0.12, 0.0, 0.12),
        edge_m=0.014,
        outward_normal_base=(0.0, 0.0, 1.0),
        reach=DEFAULT_DEXTEROUS_REACH,
    )


def test_workspace_artifact_successes_pass_dexterous_screen() -> None:
    artifact = json.loads(WORKSPACE_ARTIFACT.read_text(encoding="utf-8"))
    edge = float(artifact["config"]["target_edge_m"])
    normal = tuple(float(v) for v in artifact["config"]["outward_normal_base"])
    for sample in artifact["results"]:
        if not sample["succeeded"]:
            continue
        assert not center_outside_dexterous_reach(
            sample["center_m"],
            edge_m=edge,
            outward_normal_base=normal,
            reach=DEFAULT_DEXTEROUS_REACH,
        ), sample["sample_id"]


def test_max_reach_rejections_fail_closed_names_episode_and_centre() -> None:
    budget = ReachRejectionBudget(max_rejections=0, episode_index=1)
    with pytest.raises(ConfigurationError, match="max_reach_rejections=0"):
        budget.reject((0.2, 0.0, 0.40))
    assert budget.count == 1
    assert budget.rejections[0].episode_index == 1


def test_random_regenerates_full_count_with_budget() -> None:
    budget = ReachRejectionBudget(max_rejections=200, episode_index=0)
    centers = build_random_centers(
        8,
        (-0.22, -0.22, 0.08),
        (0.22, 0.22, 0.38),
        arm_z_motion_range_m=0.28,
        edge_m=0.014,
        min_center_separation_m=0.076,
        placement_seed=11,
        max_placement_attempts=20000,
        max_target_radial_m=0.36,
        delta_z_m=0.30,
        pre_approach_distance_m=0.01,
        reach_budget=budget,
    )
    assert len(centers) == 8
    for center in centers:
        assert not center_outside_dexterous_reach(
            center,
            edge_m=0.014,
            outward_normal_base=(0.0, 0.0, 1.0),
        )


def test_suite_wide_rejection_budget_across_episodes(tmp_path: Path) -> None:
    source = ROOT / "config/phase7_4_multi_target_standard_2x20_delta_z_0_30.yml"
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    payload["max_reach_rejections"] = 0
    payload["target_count"] = 4
    payload["episode_count"] = 2
    # CPU unit path has no tip-IK screen.
    payload["require_tip_ik"] = False
    # Force a tiny near-base keep-out so many high-Z draws are out of reach.
    payload["delta_z_m"] = 0.50
    payload["field_aabb"] = {
        "minimum_m": [0.18, -0.05, 0.08],
        "maximum_m": [0.28, 0.05, 0.58],
    }
    path = tmp_path / "budget.yml"
    path.write_text(yaml.safe_dump(payload), encoding="utf-8")
    config = load_multi_target_suite_config(path)
    with pytest.raises(ConfigurationError, match="max_reach_rejections"):
        sample_multi_target_episodes(config, root_seed=7)


def test_seed_reproducible_rejection_sequence() -> None:
    config = load_multi_target_suite_config(
        ROOT / "config/phase7_4_multi_target_standard_2x20_delta_z_0_30.yml"
    )
    # Placement reproducibility without host tip-IK oracle.
    config = replace(config, require_tip_ik=False)
    first = sample_multi_target_episodes(config, root_seed=4242)
    second = sample_multi_target_episodes(config, root_seed=4242)
    assert [t.center_m for t in first[0].field.targets] == [
        t.center_m for t in second[0].field.targets
    ]
    assert first[-1].reach_rejections == second[-1].reach_rejections


def test_manual_list_dexterous_fail_closed(tmp_path: Path) -> None:
    source = ROOT / "config/phase7_2_multi_target_manual.yml"
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    payload["targets"] = [
        {
            "target_id": "1",
            "center_m": [0.30, 0.30, 0.45],
            "outward_normal_base": [0.0, 0.0, 1.0],
        }
    ]
    payload["target_count"] = 1
    path = tmp_path / "manual_oor.yml"
    path.write_text(yaml.safe_dump(payload), encoding="utf-8")
    config = load_multi_target_suite_config(path)
    with pytest.raises(ConfigurationError, match="dexterous reach"):
        build_target_field(config, order_seed=0)


def test_widened_phase7_4_config_loads_and_samples() -> None:
    config = load_multi_target_suite_config(ROOT / "config/phase7_4_multi_target_widened_z.yml")
    assert config.delta_z_m == pytest.approx(0.14)
    assert config.z_band_fraction == pytest.approx(0.5)
    assert config.z_separation_gain == pytest.approx(1.0)
    assert config.dexterous_reach.R_wrist_max_m == pytest.approx(
        DEFAULT_DEXTEROUS_REACH.R_wrist_max_m
    )
    episodes = sample_multi_target_episodes(config, root_seed=7401)
    assert len(episodes) == 2
    zs = [t.center_m[2] for ep in episodes for t in ep.field.targets]
    assert max(zs) - min(zs) > 0.02


def test_default_grid_regression_matches_half_arm_band() -> None:
    config = load_multi_target_suite_config(ROOT / "config/phase7_2_multi_target_grid.yml")
    assert config.z_band_fraction == pytest.approx(0.5)
    assert config.delta_z_m is None
    centers = build_grid_centers(
        config.target_count,
        config.field_minimum_m,
        config.field_maximum_m,
        arm_z_motion_range_m=config.arm_z_motion_range_m,
        z_band_fraction=config.z_band_fraction,
        delta_z_m=config.delta_z_m,
    )
    field = build_target_field(config, order_seed=0)
    assert len(field.targets) == config.target_count
    zs = [c[2] for c in centers]
    mid = 0.5 * (config.field_minimum_m[2] + config.field_maximum_m[2])
    half = resolve_z_band_half_m(arm_z_motion_range_m=config.arm_z_motion_range_m)
    assert min(zs) >= mid - half - 1.0e-9
    assert max(zs) <= mid + half + 1.0e-9


def test_standard_2x20_delta_z_030_regenerates_with_z_span() -> None:
    config = load_multi_target_suite_config(
        ROOT / "config/phase7_4_multi_target_standard_2x20_delta_z_0_30.yml"
    )
    assert config.delta_z_m == pytest.approx(0.30)
    assert config.arm_z_motion_range_m == pytest.approx(0.28)
    assert config.max_reach_rejections == 20000
    assert config.target_count == 20
    assert config.require_tip_ik is True
    assert config.max_ik_rejections == 20000
    assert config.max_field_regenerations == 5
    assert config.max_consecutive_unplanned_targets == 10
    assert config.order.value == "z_desc"
    assert config.fixed_roll_rad is None
    assert len(config.roll_candidates_rad) == 8
    # CPU unit sample skips tip IK; host smoke enforces require_tip_ik.
    config = replace(config, require_tip_ik=False)
    stats: list = []
    episodes = sample_multi_target_episodes(config, root_seed=4242, placement_stats_out=stats)
    assert len(episodes) == 2
    assert stats[0].generation_duration_s >= 0.0
    for episode in episodes:
        zs = [t.center_m[2] for t in episode.field.targets]
        assert len(episode.field.targets) == 20
        assert max(zs) - min(zs) > 0.10
        for target in episode.field.targets:
            assert not center_outside_dexterous_reach(
                target.center_m,
                edge_m=config.target_edge_m,
                outward_normal_base=config.outward_normal_base,
                reach=config.dexterous_reach,
            )


def test_standard_2x20_still_packs_under_z_aware_floor() -> None:
    config = load_multi_target_suite_config(
        ROOT / "config/phase7_2_multi_target_standard_2x20.yml"
    )
    field = build_target_field(config, order_seed=7)
    assert len(field.targets) == 20
    for index, first in enumerate(field.targets):
        for second in field.targets[index + 1 :]:
            required = z_aware_required_separation_m(
                first.center_m,
                second.center_m,
                min_center_separation_m=config.min_center_separation_m,
                edge_m=config.target_edge_m,
                outward_normal_base=config.outward_normal_base,
                z_separation_gain=config.z_separation_gain,
                pre_approach_distance_m=config.pre_approach_distance_m,
            )
            observed = approach_plane_separation_m(
                first.center_m, second.center_m, config.outward_normal_base
            )
            assert observed + 1.0e-12 >= required


def test_dexterous_reach_yaml_override(tmp_path: Path) -> None:
    source = ROOT / "config/phase7_2_multi_target_grid.yml"
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    payload["dexterous_reach"] = {
        "shoulder_height_m": 0.13156,
        "L_wrist_to_tcp_m": 0.11878,
        "R_wrist_max_m": 0.20,
        "reach_margin_m": 0.0,
    }
    path = tmp_path / "reach.yml"
    path.write_text(yaml.safe_dump(payload), encoding="utf-8")
    config = load_multi_target_suite_config(path)
    assert config.dexterous_reach == DexterousReachModel(
        shoulder_height_m=0.13156,
        L_wrist_to_tcp_m=0.11878,
        R_wrist_max_m=0.20,
        reach_margin_m=0.0,
    )
