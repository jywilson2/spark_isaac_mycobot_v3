"""Unit tests for the shared tip-contact planning-world invariant."""

from __future__ import annotations

import pytest

from mycobot_curobo.cube_scene import CubeGeometry
from mycobot_curobo.errors import ConfigurationError
from mycobot_curobo.planning_world import leg_world_geometries, leg_world_scene_dict


def _cube(name: str, x: float = 0.0) -> CubeGeometry:
    return CubeGeometry(center_m=(x, 0.0, 0.16), edge_m=0.014, name=name)


def test_leg_world_omits_active_contact_and_keeps_neighbors() -> None:
    remaining = (_cube("a", 0.0), _cube("b", 0.1), _cube("c", 0.2))
    world = leg_world_geometries(remaining, active_contact_name="b")
    assert [cube.name for cube in world] == ["a", "c"]


def test_leg_world_empty_when_sole_active_cube_omitted() -> None:
    sole = (_cube("only"),)
    world = leg_world_geometries(sole, active_contact_name="only")
    assert world == ()
    assert leg_world_scene_dict(sole, active_contact_name="only") == {"cuboid": {}}


def test_leg_world_extra_exclusions() -> None:
    remaining = (_cube("a"), _cube("b"), _cube("c"))
    world = leg_world_geometries(remaining, active_contact_name="a", exclude_names=("c",))
    assert [cube.name for cube in world] == ["b"]


def test_leg_world_fails_closed_when_active_missing() -> None:
    with pytest.raises(ConfigurationError, match="not among remaining"):
        leg_world_geometries((_cube("a"),), active_contact_name="missing")
