"""Unit tests for Phase 7.3 random/layout placement and keep-outs."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from mycobot_curobo.errors import ConfigurationError
from mycobot_curobo.multi_target import (
    PlacementPolicy,
    build_target_field,
    load_multi_target_suite_config,
    sample_multi_target_episodes,
)
from mycobot_curobo.target_placement import (
    KeepOutAabb,
    LayoutName,
    LayoutSpec,
    build_layout_centers,
    build_random_centers,
    ee_clearance_min_center_separation_m,
    validate_centers_separation,
)

ROOT = Path(__file__).resolve().parents[2]


def test_random_placement_deterministic_and_diverse() -> None:
    config = load_multi_target_suite_config(ROOT / "config/phase7_3_multi_target_random.yml")
    assert config.placement is PlacementPolicy.RANDOM
    assert config.keep_outs
    episodes = sample_multi_target_episodes(config)
    assert len(episodes) == 2
    a = tuple(t.center_m for t in episodes[0].field.targets)
    b = tuple(t.center_m for t in episodes[1].field.targets)
    assert a != b
    assert sample_multi_target_episodes(config)[0].field.targets[0].center_m == a[0]


def test_rows_and_arc_layouts_load() -> None:
    rows = load_multi_target_suite_config(ROOT / "config/phase7_3_multi_target_layout_rows.yml")
    arc = load_multi_target_suite_config(ROOT / "config/phase7_3_multi_target_layout_arc.yml")
    assert rows.placement is PlacementPolicy.LAYOUT
    assert rows.layout is not None and rows.layout.name is LayoutName.ROWS
    assert arc.layout is not None and arc.layout.name is LayoutName.ARC
    field_rows = build_target_field(rows, order_seed=1, placement_seed=11)
    field_arc = build_target_field(arc, order_seed=1, placement_seed=11)
    assert len(field_rows.targets) == 5
    assert len(field_arc.targets) == 4


def test_keep_out_and_separation_fail_closed() -> None:
    keep = KeepOutAabb(minimum_m=(0.0, 0.0, 0.0), maximum_m=(0.1, 0.1, 0.1))
    with pytest.raises(ConfigurationError, match="keep_out"):
        validate_centers_separation(
            [(0.05, 0.05, 0.05)],
            min_center_separation_m=0.02,
            edge_m=0.014,
            keep_outs=(keep,),
        )
    with pytest.raises(ConfigurationError, match="min_center_separation"):
        validate_centers_separation(
            [(0.0, 0.0, 0.0), (0.01, 0.0, 0.0)],
            min_center_separation_m=0.05,
            edge_m=0.014,
        )


def test_random_placement_exhausted_attempts() -> None:
    with pytest.raises(ConfigurationError, match="random placement failed"):
        build_random_centers(
            8,
            (0.0, 0.0, 0.0),
            (0.05, 0.05, 0.05),
            arm_z_motion_range_m=0.02,
            edge_m=0.014,
            min_center_separation_m=0.04,
            placement_seed=1,
            max_placement_attempts=20,
        )


def test_arc_outside_aabb_fails() -> None:
    layout = LayoutSpec(
        name=LayoutName.ARC,
        radius_m=0.5,
        span_rad=1.0,
        center_xy_m=(0.2, -0.1),
        z_m=0.16,
    )
    with pytest.raises(ConfigurationError, match="outside field_aabb"):
        build_layout_centers(
            3,
            (0.12, -0.2, 0.13),
            (0.28, 0.02, 0.19),
            layout=layout,
            arm_z_motion_range_m=0.28,
            edge_m=0.014,
            min_center_separation_m=0.02,
        )


def test_manual_too_close_fails_closed(tmp_path: Path) -> None:
    source = ROOT / "config/phase7_2_multi_target_manual.yml"
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    payload["min_center_separation_m"] = 0.5
    path = tmp_path / "close.yml"
    # Suite YAML path is arbitrary; robot E comes from default robot config.
    path.write_text(yaml.safe_dump(payload), encoding="utf-8")
    config = load_multi_target_suite_config(path)
    with pytest.raises(ConfigurationError, match="min_center_separation"):
        build_target_field(config, order_seed=0)


def test_ee_clearance_separation_default_and_floor() -> None:
    """Default separation is edge+flange; values below that floor fail closed."""

    assert ee_clearance_min_center_separation_m(0.014, 0.031) == pytest.approx(0.045)
    grid = load_multi_target_suite_config(ROOT / "config/phase7_2_multi_target_grid.yml")
    floor = ee_clearance_min_center_separation_m(
        grid.target_edge_m, grid.flange_diameter_assumption_m
    )
    assert grid.min_center_separation_m == pytest.approx(floor)
    field = build_target_field(grid, order_seed=0)
    centers = [target.center_m for target in field.targets]
    validate_centers_separation(
        centers,
        min_center_separation_m=floor,
        edge_m=grid.target_edge_m,
    )


def test_min_center_separation_below_ee_floor_rejected(tmp_path: Path) -> None:
    source = ROOT / "config/phase7_2_multi_target_grid.yml"
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    payload["min_center_separation_m"] = 0.03  # below edge+flange (=0.045)
    path = tmp_path / "too_close_sep.yml"
    path.write_text(yaml.safe_dump(payload), encoding="utf-8")
    with pytest.raises(ConfigurationError, match="EE clearance floor"):
        load_multi_target_suite_config(path)
