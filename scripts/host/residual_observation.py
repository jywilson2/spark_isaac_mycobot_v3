"""Host-side residual observation bridge for Isaac Lab / Isaac Sim wrappers.

Core ``ResidualObservation`` construction stays in mycobot_curobo; this module
is a thin import convenience for host scripts and never enables hardware.
"""

from __future__ import annotations

from typing import Sequence

from mycobot_curobo.residual import ResidualObservation


def build_residual_observation_from_sim(
    *,
    request_id: str,
    waypoint_index: int,
    command_time_s: float,
    measured_timestamp_s: float,
    nominal_joint_position_rad: Sequence[float],
    measured_joint_position_rad: Sequence[float],
    nominal_tcp_position_base_m: Sequence[float],
    goal_position_base_m: Sequence[float],
    approach_direction_base: Sequence[float],
) -> ResidualObservation:
    """Construct a typed residual observation from sim-measured fields."""

    return ResidualObservation.create(
        request_id=request_id,
        waypoint_index=waypoint_index,
        command_time_s=command_time_s,
        measured_timestamp_s=measured_timestamp_s,
        nominal_joint_position_rad=nominal_joint_position_rad,
        measured_joint_position_rad=measured_joint_position_rad,
        nominal_tcp_position_base_m=nominal_tcp_position_base_m,
        goal_position_base_m=goal_position_base_m,
        approach_direction_base=approach_direction_base,
    )
