"""Phase 2 tests for target validation and deterministic task frames."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from mycobot_curobo.config import load_app_config
from mycobot_curobo.errors import ConfigurationError
from mycobot_curobo.frames import (
    TaskFrameConfig,
    build_task_frame_candidates,
    deterministic_tangent,
    validate_rotation_matrix,
)
from mycobot_curobo.goal_set import build_surface_goal_set
from mycobot_curobo.targets import (
    DEFAULT_ROLL_CANDIDATES_RAD,
    SurfaceTarget,
    normalize_angle_rad,
    normalize_vector,
)

ROOT = Path(__file__).resolve().parents[2]


def _target(**overrides) -> SurfaceTarget:
    values = {
        "position_base_m": [0.15, -0.05, 0.2],
        "surface_normal_base": [0.2, -0.3, 0.9],
        "tangent_hint_base": [1.0, 0.0, 0.0],
        "pre_approach_distance_m": 0.05,
        "target_id": "unit-target",
    }
    values.update(overrides)
    return SurfaceTarget.create(**values)


def test_surface_target_normalizes_normal_and_default_rolls() -> None:
    target = _target()

    assert np.isclose(np.linalg.norm(target.surface_normal_base), 1.0)
    assert target.roll_candidates_rad == DEFAULT_ROLL_CANDIDATES_RAD
    assert target.fixed_roll_rad is None
    assert target.position_base_m.flags.writeable is False


@pytest.mark.parametrize(
    ("overrides", "message"),
    [
        ({"surface_normal_base": [0.0, 0.0, 0.0]}, "magnitude"),
        ({"surface_normal_base": [np.nan, 0.0, 1.0]}, "finite"),
        ({"position_base_m": [0.0, 0.0]}, "shape"),
        ({"fixed_roll_rad": 0.0, "roll_candidates_rad": [0.0]}, "mutually exclusive"),
        ({"roll_candidates_rad": [0.0, 2.0 * math.pi]}, "duplicates"),
        ({"pre_approach_distance_m": 0.001}, "within"),
        ({"pre_approach_distance_m": 0.5}, "within"),
        ({"target_id": " "}, "non-empty"),
        ({"tool_frame": "visual_mesh"}, "tcp_link"),
    ],
)
def test_surface_target_rejects_invalid_input(overrides, message: str) -> None:
    with pytest.raises(ConfigurationError, match=message):
        _target(**overrides)


def test_angle_normalization_has_stable_zero() -> None:
    assert normalize_angle_rad(0.0) == 0.0
    assert normalize_angle_rad(2.0 * math.pi) == 0.0
    assert math.isclose(normalize_angle_rad(-math.pi / 2.0), 3.0 * math.pi / 2.0)


@pytest.mark.parametrize("axis", ["x", "y", "z"])
@pytest.mark.parametrize("sign", [-1, 1])
def test_signed_tcp_axis_aligns_with_approach_for_all_conventions(axis: str, sign: int) -> None:
    target = _target()
    config = TaskFrameConfig(tool_approach_axis=axis, tool_approach_sign=sign)
    axis_index = {"x": 0, "y": 1, "z": 2}[axis]
    expected_approach = -target.surface_normal_base

    candidates = build_task_frame_candidates(target, config)

    assert len(candidates) == 8
    for candidate in candidates:
        physical_axis = sign * candidate.rotation_base_from_tool[:, axis_index]
        assert np.allclose(physical_axis, expected_approach, atol=1.0e-10)
        validate_rotation_matrix(candidate.rotation_base_from_tool)
        assert math.isclose(np.linalg.det(candidate.rotation_base_from_tool), 1.0)
        assert math.isclose(np.linalg.norm(candidate.quaternion_wxyz), 1.0)
        assert np.array_equal(candidate.position_base_m, target.position_base_m)


def test_nearly_parallel_tangent_uses_deterministic_fallback() -> None:
    approach = normalize_vector([0.0, 0.0, -1.0], label="approach")
    almost_parallel = np.array([1.0e-14, 0.0, 1.0])

    first = deterministic_tangent(approach, almost_parallel, epsilon=1.0e-9)
    second = deterministic_tangent(approach, almost_parallel, epsilon=1.0e-9)

    assert np.array_equal(first, second)
    assert np.allclose(first, [1.0, 0.0, 0.0])
    assert abs(float(np.dot(first, approach))) < 1.0e-12


def test_seeded_random_normals_produce_valid_deterministic_frames() -> None:
    generator = np.random.default_rng(20260718)
    normals = generator.normal(size=(512, 3))
    normals[:6] = np.array(
        [
            [1.0, 1.0e-12, 0.0],
            [-1.0, 0.0, 1.0e-12],
            [0.0, 1.0, -1.0e-12],
            [1.0e-12, -1.0, 0.0],
            [0.0, 1.0e-12, 1.0],
            [-1.0e-12, 0.0, -1.0],
        ]
    )

    for index, normal in enumerate(normals):
        target = _target(
            surface_normal_base=normal,
            tangent_hint_base=normal + np.array([1.0e-13, 0.0, 0.0]),
            roll_candidates_rad=[0.0, math.pi / 3.0],
            target_id=f"random-{index}",
        )
        first = build_task_frame_candidates(target)
        second = build_task_frame_candidates(target)
        assert len(first) == 2
        for candidate_a, candidate_b in zip(first, second, strict=True):
            assert np.array_equal(
                candidate_a.rotation_base_from_tool,
                candidate_b.rotation_base_from_tool,
            )
            assert np.allclose(
                -candidate_a.rotation_base_from_tool[:, 2],
                -target.surface_normal_base,
                atol=1.0e-10,
            )


def test_fixed_roll_builds_single_candidate_and_goal_mapping() -> None:
    target = _target(fixed_roll_rad=-math.pi / 2.0)

    goal_set = build_surface_goal_set(target)

    assert len(goal_set.candidates) == 1
    assert math.isclose(goal_set.roll_for_goal_index(0), 3.0 * math.pi / 2.0)
    with pytest.raises(ConfigurationError, match="outside"):
        goal_set.roll_for_goal_index(1)


def test_app_config_drives_task_frame_and_roll_defaults() -> None:
    config = load_app_config(ROOT / "config" / "app.yml")

    assert config.tool_frame == "tcp_link"
    assert config.task_frame.tool_approach_axis == "z"
    assert config.task_frame.tool_approach_sign == -1
    assert config.default_roll_candidates_rad == DEFAULT_ROLL_CANDIDATES_RAD
    assert config.robot_config_path.name == "mycobot_280_m5.yml"


def test_app_config_rejects_duplicate_rolls(tmp_path: Path) -> None:
    payload = yaml.safe_load((ROOT / "config" / "app.yml").read_text(encoding="utf-8"))
    payload["default_roll_candidates_deg"] = [0, 360]
    path = tmp_path / "app.yml"
    path.write_text(yaml.safe_dump(payload), encoding="utf-8")

    with pytest.raises(ConfigurationError, match="unique"):
        load_app_config(path)
