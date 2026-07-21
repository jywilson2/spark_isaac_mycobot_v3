"""Phase 1 unit tests for model provenance, ordering, limits, and CPU FK."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest
import yaml

from mycobot_curobo.errors import ConfigurationError
from mycobot_curobo.robot_model import (
    BASE_LINK,
    FLANGE_LINK,
    JOINT_NAMES,
    TCP_LINK,
    forward_kinematics,
    load_curobo_robot_config,
    load_robot_model_spec,
    reorder_joint_state,
)

ROOT = Path(__file__).resolve().parents[2]
ROBOT_CONFIG = ROOT / "config" / "robots" / "mycobot_280_m5.yml"
FK_FIXTURE = ROOT / "tests" / "data" / "phase1_fk_regression.json"


def test_robot_config_has_explicit_frames_order_and_empty_contact_links() -> None:
    spec = load_robot_model_spec(ROBOT_CONFIG)

    assert spec.base_link == BASE_LINK
    assert spec.flange_link == FLANGE_LINK
    assert spec.tcp_link == TCP_LINK
    assert spec.tool_frames == (TCP_LINK,)
    assert spec.joint_names == JOINT_NAMES
    assert np.array_equal(spec.default_joint_position_rad, np.zeros(6))


def test_every_collision_link_has_static_spheres() -> None:
    spec = load_robot_model_spec(ROBOT_CONFIG)

    assert set(spec.collision_sphere_count_by_link) == {
        "g_base",
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
        "joint6_flange",
    }
    assert all(count >= 1 for count in spec.collision_sphere_count_by_link.values())
    assert sum(spec.collision_sphere_count_by_link.values()) == 128
    assert spec.min_detectable_obstacle_edge_m == pytest.approx(0.014)


def test_limits_are_finite_consistent_and_in_si_units() -> None:
    limits = load_robot_model_spec(ROBOT_CONFIG).limits

    assert limits.names == JOINT_NAMES
    assert np.all(np.isfinite(limits.lower_rad))
    assert np.all(limits.lower_rad < limits.upper_rad)
    assert np.all(limits.velocity_rad_s > 0.0)
    assert np.all(limits.acceleration_rad_s2 == 3.0)
    assert np.all(limits.jerk_rad_s3 == 30.0)
    assert np.allclose(limits.velocity_rad_s, np.deg2rad(160.0), atol=2.0e-6)


def test_curobo_adapter_resolves_external_paths_absolutely() -> None:
    payload = load_curobo_robot_config(ROBOT_CONFIG)
    kinematics = payload["robot_cfg"]["kinematics"]

    assert Path(kinematics["urdf_path"]).is_absolute()
    assert Path(kinematics["urdf_path"]).is_file()
    assert Path(kinematics["asset_root_path"]).is_absolute()
    assert kinematics["grasp_contact_link_names"] == []


def test_explicit_reorder_joint_state() -> None:
    reversed_names = tuple(reversed(JOINT_NAMES))
    reversed_values = tuple(float(index) for index in reversed(range(6)))

    reordered = reorder_joint_state(reversed_values, reversed_names)

    assert np.array_equal(reordered, np.arange(6, dtype=float))


@pytest.mark.parametrize(
    ("names", "message"),
    [
        (JOINT_NAMES[:-1], "missing"),
        (JOINT_NAMES[:-1] + (JOINT_NAMES[0],), "duplicates"),
        (JOINT_NAMES[:-1] + ("unknown_joint",), "unknown"),
    ],
)
def test_reorder_rejects_ambiguous_names(names: tuple[str, ...], message: str) -> None:
    with pytest.raises(ConfigurationError, match=message):
        reorder_joint_state(np.zeros(len(names)), names)


def test_config_rejects_silent_joint_reordering(tmp_path: Path) -> None:
    payload = yaml.safe_load(ROBOT_CONFIG.read_text(encoding="utf-8"))
    names = payload["robot_cfg"]["kinematics"]["cspace"]["joint_names"]
    names[0], names[1] = names[1], names[0]
    candidate = tmp_path / "config" / "robots" / "robot.yml"
    candidate.parent.mkdir(parents=True)
    candidate.write_text(yaml.safe_dump(payload), encoding="utf-8")

    with pytest.raises(ConfigurationError, match="joint_names must exactly equal"):
        load_robot_model_spec(candidate)


def test_fk_regression_five_known_joint_states() -> None:
    spec = load_robot_model_spec(ROBOT_CONFIG)
    fixture = json.loads(FK_FIXTURE.read_text(encoding="utf-8"))

    assert fixture["frame"] == BASE_LINK
    assert fixture["tcp_link"] == TCP_LINK
    assert len(fixture["cases"]) >= 5
    for case in fixture["cases"]:
        pose = forward_kinematics(case["joint_position_rad"], spec=spec)
        assert np.allclose(pose.position_m, case["position_m"], atol=1.0e-10)
        # q and -q encode the same orientation; compare the absolute dot.
        expected_q = np.asarray(case["quaternion_wxyz"], dtype=float)
        assert abs(float(np.dot(pose.quaternion_wxyz, expected_q))) > 1.0 - 1.0e-10


def test_fk_rejects_nonfinite_and_out_of_limit_joint_states() -> None:
    spec = load_robot_model_spec(ROBOT_CONFIG)
    nonfinite = np.zeros(6)
    nonfinite[2] = np.nan
    outside = np.zeros(6)
    outside[0] = spec.limits.upper_rad[0] + 0.1

    with pytest.raises(ConfigurationError, match="finite"):
        forward_kinematics(nonfinite, spec=spec)
    with pytest.raises(ConfigurationError, match="limits"):
        forward_kinematics(outside, spec=spec)
