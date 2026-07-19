"""Phase 5 residual contract tests."""

from __future__ import annotations

import pytest

from mycobot_curobo.errors import ConfigurationError
from mycobot_curobo.residual import (
    CartesianResidual,
    ResidualObservation,
    ZeroResidualCorrector,
)


def _observation() -> ResidualObservation:
    return ResidualObservation.create(
        request_id="phase5-unit",
        waypoint_index=0,
        command_time_s=1.0,
        measured_timestamp_s=1.0,
        nominal_joint_position_rad=[0.0] * 6,
        measured_joint_position_rad=[0.0] * 6,
        nominal_tcp_position_base_m=[0.1, 0.0, 0.2],
        goal_position_base_m=[0.1, 0.0, 0.2],
        approach_direction_base=[0.0, 0.0, 1.0],
    )


def test_zero_corrector_returns_exact_zero() -> None:
    residual = ZeroResidualCorrector().correction(_observation())

    assert residual == CartesianResidual.zero()
    assert residual.is_zero


def test_cartesian_residual_rejects_non_finite_or_wrong_shape() -> None:
    with pytest.raises(ConfigurationError, match="exactly 3"):
        CartesianResidual.create([0.0, 0.0], [0.0, 0.0, 0.0])
    with pytest.raises(ConfigurationError, match="finite"):
        CartesianResidual.create([0.0, float("nan"), 0.0], [0.0, 0.0, 0.0])


def test_observation_requires_unit_approach_and_matching_joint_vectors() -> None:
    values = vars(_observation())
    with pytest.raises(ConfigurationError, match="unit length"):
        ResidualObservation.create(**{**values, "approach_direction_base": [0.0, 0.0, 2.0]})
    with pytest.raises(ConfigurationError, match="equal non-zero"):
        ResidualObservation.create(**{**values, "measured_joint_position_rad": [0.0] * 5})
