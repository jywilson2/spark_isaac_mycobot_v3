"""Typed bounded-residual contract with a zero-output Phase 5 implementation."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol, Sequence

from mycobot_curobo.errors import ConfigurationError


def _finite_tuple(values: Sequence[float], length: int, label: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != length:
        raise ConfigurationError(f"{label} must contain exactly {length} values")
    if not all(math.isfinite(value) for value in result):
        raise ConfigurationError(f"{label} must contain only finite values")
    return result


@dataclass(frozen=True)
class CartesianResidual:
    """Small base-frame translation and rotation-vector correction in SI units."""

    translation_base_m: tuple[float, float, float]
    rotation_vector_base_rad: tuple[float, float, float]

    @classmethod
    def create(
        cls,
        translation_base_m: Sequence[float],
        rotation_vector_base_rad: Sequence[float],
    ) -> CartesianResidual:
        """Validate explicit three-vectors without silently clamping them."""

        return cls(
            translation_base_m=_finite_tuple(translation_base_m, 3, "residual translation_base_m"),
            rotation_vector_base_rad=_finite_tuple(
                rotation_vector_base_rad, 3, "residual rotation_vector_base_rad"
            ),
        )

    @classmethod
    def zero(cls) -> CartesianResidual:
        """Return the exact zero correction."""

        return cls(
            translation_base_m=(0.0, 0.0, 0.0),
            rotation_vector_base_rad=(0.0, 0.0, 0.0),
        )

    @property
    def is_zero(self) -> bool:
        """Whether all correction components are exactly zero."""

        return self == self.zero()


@dataclass(frozen=True)
class ResidualObservation:
    """One timestamped nominal/measured execution sample in the base frame."""

    request_id: str
    waypoint_index: int
    command_time_s: float
    measured_timestamp_s: float
    nominal_joint_position_rad: tuple[float, ...]
    measured_joint_position_rad: tuple[float, ...]
    nominal_tcp_position_base_m: tuple[float, float, float]
    goal_position_base_m: tuple[float, float, float]
    approach_direction_base: tuple[float, float, float]

    @classmethod
    def create(
        cls,
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
        """Construct a finite observation with an explicitly normalized axis."""

        import numpy as np

        if not request_id:
            raise ConfigurationError("residual observation request_id must be non-empty")
        if waypoint_index < 0:
            raise ConfigurationError("residual observation waypoint_index must be non-negative")
        times = (float(command_time_s), float(measured_timestamp_s))
        if not all(math.isfinite(value) and value >= 0.0 for value in times):
            raise ConfigurationError(
                "residual observation timestamps must be finite and non-negative"
            )
        nominal = _finite_tuple(
            nominal_joint_position_rad,
            len(nominal_joint_position_rad),
            "nominal joint position",
        )
        measured = _finite_tuple(
            measured_joint_position_rad,
            len(measured_joint_position_rad),
            "measured joint position",
        )
        if not nominal or len(nominal) != len(measured):
            raise ConfigurationError(
                "nominal and measured joint vectors must have equal non-zero size"
            )
        approach = _finite_tuple(approach_direction_base, 3, "approach direction")
        norm = float(np.linalg.norm(approach))
        if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1.0e-8):
            raise ConfigurationError("approach direction must be unit length")
        return cls(
            request_id=request_id,
            waypoint_index=waypoint_index,
            command_time_s=times[0],
            measured_timestamp_s=times[1],
            nominal_joint_position_rad=nominal,
            measured_joint_position_rad=measured,
            nominal_tcp_position_base_m=_finite_tuple(
                nominal_tcp_position_base_m, 3, "nominal TCP position"
            ),
            goal_position_base_m=_finite_tuple(goal_position_base_m, 3, "goal position"),
            approach_direction_base=approach,
        )


class ResidualCorrector(Protocol):
    """Return a bounded local correction, never a replacement trajectory."""

    def correction(self, observation: ResidualObservation) -> CartesianResidual:
        """Compute one correction from a typed execution observation."""


class ZeroResidualCorrector:
    """Phase 5 corrector that leaves every cuRobo command unchanged."""

    def correction(self, observation: ResidualObservation) -> CartesianResidual:
        del observation
        return CartesianResidual.zero()
