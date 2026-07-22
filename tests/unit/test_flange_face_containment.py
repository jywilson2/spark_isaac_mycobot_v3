"""CPU tests for flange-disk vs contact-face overhang detection."""

from __future__ import annotations

from pathlib import Path

import pytest

from mycobot_curobo.cube_scene import (
    CubeGeometry,
    flange_disk_collides_contact_face,
    flange_disk_cube_clearance_m,
    flange_disk_face_overhang_m,
)
from mycobot_curobo.multi_target import load_multi_target_suite_config

ROOT = Path(__file__).resolve().parents[2]


def test_flange_disk_face_overhang_detects_undersized_face() -> None:
    """14 mm face vs 31 mm flange at face centre → ~8.5 mm overhang."""

    cube = CubeGeometry((0.2, 0.0, 0.15), 0.014, name="small")
    face_z = 0.15 + 0.5 * 0.014
    tcp = (0.2, 0.0, face_z)
    overhang = flange_disk_face_overhang_m(tcp, (0.0, 0.0, 1.0), 0.031, cube)
    assert overhang == pytest.approx(0.5 * 0.031 - 0.5 * 0.014)
    assert flange_disk_collides_contact_face(tcp, (0.0, 0.0, 1.0), 0.031, cube)


def test_flange_disk_face_contained_when_edge_matches_flange() -> None:
    cube = CubeGeometry((0.2, 0.0, 0.15), 0.031, name="flange_sized")
    face_z = 0.15 + 0.5 * 0.031
    tcp = (0.2, 0.0, face_z)
    assert flange_disk_face_overhang_m(tcp, (0.0, 0.0, 1.0), 0.031, cube) == pytest.approx(
        0.0, abs=1.0e-12
    )
    assert not flange_disk_collides_contact_face(tcp, (0.0, 0.0, 1.0), 0.031, cube)


def test_flange_disk_overhang_grows_with_lateral_tcp_offset() -> None:
    cube = CubeGeometry((0.0, 0.0, 0.0), 0.031, name="cube")
    tcp = (0.005, 0.0, 0.0155)
    overhang = flange_disk_face_overhang_m(tcp, (0.0, 0.0, 1.0), 0.031, cube)
    assert overhang == pytest.approx(0.005)


def test_integration_suite_requires_flange_face_containment() -> None:
    config = load_multi_target_suite_config(
        ROOT / "config/phase7_2_multi_target_integration_2x5.yml"
    )
    assert config.require_flange_face_containment is True
    assert config.target_edge_m + 1.0e-12 >= config.flange_diameter_assumption_m


def test_flange_disk_cube_clearance_detects_neighbor_graze() -> None:
    """TCP above a neighbor cube with flange radius past the face → negative clear."""

    cube = CubeGeometry((0.2, 0.0, 0.15), 0.031, name="neighbor")
    # TCP laterally overlapping the cube top, Z just above the top face.
    top_z = 0.15 + 0.5 * 0.031
    tcp = (0.2, 0.0, top_z + 0.001)
    clear = flange_disk_cube_clearance_m(tcp, 0.031, cube)
    assert clear < 0.0
    # Far lateral TCP is clear.
    clear_far = flange_disk_cube_clearance_m((0.4, 0.0, top_z + 0.001), 0.031, cube)
    assert clear_far > 0.0
