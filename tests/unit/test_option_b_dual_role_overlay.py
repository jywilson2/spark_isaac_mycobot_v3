"""Unit tests for Phase 1.1 Option B dual-role sphere overlay merge."""

from __future__ import annotations

import copy
from pathlib import Path

import pytest
import yaml

from mycobot_curobo.robot_model import (
    WORLD_COVER_LINK_SUFFIX,
    apply_collision_sphere_overlay,
    count_active_self_collision_link_pairs,
    load_curobo_robot_config,
    load_robot_model_spec,
    world_cover_link_name,
)

ROOT = Path(__file__).resolve().parents[2]
ROBOT = ROOT / "config" / "robots" / "mycobot_280_m5.yml"
OVERLAY = ROOT / "config" / "robots" / "mycobot_280_m5_phase1_1_spheres.yml"


def _trial_dual(tmp_path: Path) -> Path:
    payload = yaml.safe_load(ROBOT.read_text(encoding="utf-8"))
    kin = payload["robot_cfg"]["kinematics"]
    kin["collision_sphere_overlay_path"] = "config/robots/mycobot_280_m5_phase1_1_spheres.yml"
    kin["collision_sphere_overlay_role"] = "dual"
    path = tmp_path / "trial_option_b.yml"
    # Write beside robot YAML so relative overlay resolve works via repo root.
    path = ROOT / "config" / "robots" / "_tmp_option_b_unit.yml"
    path.write_text(yaml.safe_dump(payload), encoding="utf-8")
    return path


def test_world_cover_link_name() -> None:
    assert world_cover_link_name("joint6_flange") == f"joint6_flange{WORLD_COVER_LINK_SUFFIX}"


def test_count_active_self_collision_link_pairs_respects_ignore() -> None:
    links = ("a", "b", "c")
    ignore = {"a": ["b"], "b": ["a", "c"], "c": ["b"]}
    # Only a-c remains.
    assert count_active_self_collision_link_pairs(links, ignore) == 1


def test_option_b_dual_preserves_scaffolding_and_adds_world_cover_links(
    tmp_path: Path,
) -> None:
    if not OVERLAY.is_file():
        pytest.skip("Option A overlay missing")
    trial = _trial_dual(tmp_path)
    try:
        before = yaml.safe_load(ROBOT.read_text(encoding="utf-8"))["robot_cfg"]["kinematics"]
        scaffold_links = list(before["collision_link_names"])
        scaffold_active = count_active_self_collision_link_pairs(
            scaffold_links, before["self_collision_ignore"]
        )
        scaffold_spheres = sum(len(before["collision_spheres"][link]) for link in scaffold_links)

        spec = load_robot_model_spec(trial)
        world_links = [world_cover_link_name(link) for link in scaffold_links]
        assert all(link in spec.collision_sphere_count_by_link for link in scaffold_links)
        assert all(link in spec.collision_sphere_count_by_link for link in world_links)
        total = sum(spec.collision_sphere_count_by_link.values())
        assert total == scaffold_spheres + 1012

        cfg = load_curobo_robot_config(trial)
        kin = cfg["robot_cfg"]["kinematics"]
        assert "collision_sphere_overlay_path" not in kin
        assert "collision_sphere_overlay_role" not in kin
        assert "min_detectable_obstacle_edge_m" not in kin
        for link in world_links:
            assert link in kin["extra_links"]
            assert kin["extra_links"][link]["joint_type"] == "FIXED"
            assert kin["extra_links"][link]["parent_link_name"] == link.removesuffix(
                WORLD_COVER_LINK_SUFFIX
            )
            assert link not in kin.get("mesh_link_names", [])
        merged_active = count_active_self_collision_link_pairs(
            kin["collision_link_names"], kin["self_collision_ignore"]
        )
        assert merged_active == scaffold_active
    finally:
        trial.unlink(missing_ok=True)


def test_option_a_replace_role_still_available_for_diagnosis(tmp_path: Path) -> None:
    if not OVERLAY.is_file():
        pytest.skip("Option A overlay missing")
    payload = yaml.safe_load(ROBOT.read_text(encoding="utf-8"))
    kin = payload["robot_cfg"]["kinematics"]
    kin["collision_sphere_overlay_path"] = "config/robots/mycobot_280_m5_phase1_1_spheres.yml"
    kin["collision_sphere_overlay_role"] = "replace"
    trial = ROOT / "config" / "robots" / "_tmp_option_a_replace_unit.yml"
    trial.write_text(yaml.safe_dump(payload), encoding="utf-8")
    try:
        # Mutate a deep copy through the public API helper.
        kinematics = copy.deepcopy(payload["robot_cfg"]["kinematics"])
        apply_collision_sphere_overlay(kinematics, trial)
        assert sum(len(v) for v in kinematics["collision_spheres"].values()) == 1012
        assert all(
            not str(link).endswith(WORLD_COVER_LINK_SUFFIX)
            for link in kinematics["collision_spheres"]
        )
    finally:
        trial.unlink(missing_ok=True)
