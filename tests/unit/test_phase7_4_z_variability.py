"""Unit tests for Phase 7.4 Z band, delta_z_m, Z-aware floor, and ROM retries."""

from __future__ import annotations

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
    DEFAULT_Z_BAND_FRACTION,
    MAX_ROM_SUBSTITUTE_RETRIES,
    approach_plane_separation_m,
    arm_reach_z_bounds,
    build_random_centers,
    center_outside_arm_reach,
    resolve_z_band_half_m,
    top_face_delta_m,
    validate_centers_separation,
    z_aware_required_separation_m,
    z_band_bounds,
)

ROOT = Path(__file__).resolve().parents[2]


def test_default_z_band_is_half_arm_range() -> None:
    assert DEFAULT_Z_BAND_FRACTION == pytest.approx(0.5)
    half = resolve_z_band_half_m(arm_z_motion_range_m=0.28)
    assert half == pytest.approx(0.5 * 0.5 * 0.28)
    mid, z_lo, z_hi = z_band_bounds((0.0, 0.0, 0.12), (0.2, 0.2, 0.22), arm_z_motion_range_m=0.28)
    assert mid == pytest.approx(0.17)
    assert z_hi - z_lo == pytest.approx(0.14)


def test_delta_z_m_overrides_fraction_and_is_unclamped() -> None:
    # Absolute full-band width; may exceed arm_z_motion_range_m (no clamp).
    half = resolve_z_band_half_m(arm_z_motion_range_m=0.28, z_band_fraction=0.5, delta_z_m=0.40)
    assert half == pytest.approx(0.20)
    # Fraction may also exceed 1.0 without clamp.
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
    # |Δz_top| = 0.10 > pre_approach 0.05 → additive 0.05
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


def test_rom_substitute_retries_exhausted() -> None:
    # Band far above the reach envelope → every sample fails ROM.
    with pytest.raises(ConfigurationError, match="arm-reach substitute retries"):
        build_random_centers(
            1,
            (0.10, -0.05, 0.12),
            (0.20, 0.05, 0.22),
            arm_z_motion_range_m=0.05,
            edge_m=0.014,
            min_center_separation_m=0.05,
            placement_seed=1,
            max_placement_attempts=50,
            delta_z_m=2.0,
            pre_approach_distance_m=0.05,
        )
    assert MAX_ROM_SUBSTITUTE_RETRIES == 3


def test_widened_phase7_4_config_loads_and_samples() -> None:
    config = load_multi_target_suite_config(ROOT / "config/phase7_4_multi_target_widened_z.yml")
    assert config.delta_z_m == pytest.approx(0.14)
    assert config.z_band_fraction == pytest.approx(0.5)
    assert config.z_separation_gain == pytest.approx(1.0)
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
