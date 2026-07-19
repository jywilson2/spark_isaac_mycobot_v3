"""Phase 5 deterministic residual safety projection tests."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from mycobot_curobo.residual import CartesianResidual, ResidualObservation
from mycobot_curobo.robot_model import JOINT_NAMES, JointLimits
from mycobot_curobo.safety import (
    SafetyProjector,
    SafetyStatus,
    load_residual_safety_profile,
)

ROOT = Path(__file__).resolve().parents[2]


def _limits() -> JointLimits:
    return JointLimits(
        names=JOINT_NAMES,
        lower_rad=np.full(6, -2.0),
        upper_rad=np.full(6, 2.0),
        velocity_rad_s=np.ones(6),
        acceleration_rad_s2=np.ones(6),
        jerk_rad_s3=np.ones(6),
    )


def _observation() -> ResidualObservation:
    return ResidualObservation.create(
        request_id="phase5-safety",
        waypoint_index=1,
        command_time_s=10.0,
        measured_timestamp_s=10.0,
        nominal_joint_position_rad=[0.0] * 6,
        measured_joint_position_rad=[0.0] * 6,
        nominal_tcp_position_base_m=[0.1, 0.0, 0.18],
        goal_position_base_m=[0.1, 0.0, 0.2],
        approach_direction_base=[0.0, 0.0, 1.0],
    )


def _projector() -> SafetyProjector:
    profile = load_residual_safety_profile(
        "simulation_zero_residual",
        ROOT / "config" / "residual_safety.yml",
    )
    return SafetyProjector(profile, _limits())


def test_oversized_residual_is_explicitly_clipped() -> None:
    decision = _projector().project(
        _observation(),
        CartesianResidual.create([0.0, 0.0, 0.01], [0.0, 0.1, 0.0]),
        evaluated_at_s=10.0,
        plan_executable=True,
    )

    assert decision.status is SafetyStatus.CLIPPED
    assert decision.projected_residual is not None
    assert np.isclose(
        np.linalg.norm(decision.projected_residual.translation_base_m),
        _projector().profile.max_translation_m,
    )
    assert np.isclose(
        np.linalg.norm(decision.projected_residual.rotation_vector_base_rad),
        _projector().profile.max_rotation_rad,
    )
    assert set(decision.reasons) == {"translation_clipped", "rotation_clipped"}


def test_correction_cannot_leave_terminal_corridor() -> None:
    decision = _projector().project(
        _observation(),
        CartesianResidual.create([0.01, 0.0, 0.0], [0.0, 0.0, 0.0]),
        evaluated_at_s=10.0,
        plan_executable=True,
    )

    # Magnitude projection occurs first, but a deliberately tighter synthetic
    # corridor proves the fail-closed path independently of clipping.
    assert decision.accepted
    tight_profile = _projector().profile
    tight_projector = SafetyProjector(
        type(tight_profile)(**{**vars(tight_profile), "max_lateral_error_m": 0.001}),
        _limits(),
    )
    rejected = tight_projector.project(
        _observation(),
        CartesianResidual.create([0.01, 0.0, 0.0], [0.0, 0.0, 0.0]),
        evaluated_at_s=10.0,
        plan_executable=True,
    )
    assert rejected.status is SafetyStatus.REJECTED
    assert rejected.projected_residual is None
    assert "terminal_corridor_violation" in rejected.reasons


def test_stale_state_non_executable_plan_and_joint_envelope_reject() -> None:
    observation = _observation()
    stale = _projector().project(
        observation,
        CartesianResidual.zero(),
        evaluated_at_s=11.0,
        plan_executable=False,
    )
    assert stale.status is SafetyStatus.REJECTED
    assert {"plan_not_executable", "stale_robot_state", "watchdog_timeout"} <= set(stale.reasons)

    outside = ResidualObservation.create(
        **{
            **vars(observation),
            "nominal_joint_position_rad": [1.99] * 6,
        }
    )
    decision = _projector().project(
        outside,
        CartesianResidual.zero(),
        evaluated_at_s=10.0,
        plan_executable=True,
    )
    assert "nominal_joint_outside_feasibility_envelope" in decision.reasons
