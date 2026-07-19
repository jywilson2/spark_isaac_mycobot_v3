"""Typed surface-target contract for constrained approach planning.

Targets carry only base-frame task geometry and explicit tool/roll policy.
They do not contain cuRobo tensors, planner state, or hidden frame transforms.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np

from mycobot_curobo.errors import ConfigurationError
from mycobot_curobo.robot_model import TCP_LINK

DEFAULT_ROLL_CANDIDATES_RAD: tuple[float, ...] = tuple(
    math.radians(degrees) for degrees in range(0, 360, 45)
)


def _finite_vector(value: Sequence[float], label: str) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (3,):
        raise ConfigurationError(f"{label} must have shape (3,), got {result.shape}")
    if not np.all(np.isfinite(result)):
        raise ConfigurationError(f"{label} must contain only finite values")
    result = result.copy()
    result.setflags(write=False)
    return result


def normalize_vector(
    value: Sequence[float],
    *,
    label: str,
    epsilon: float = 1.0e-9,
) -> np.ndarray:
    """Normalize a finite 3-vector, rejecting degenerate input."""

    vector = _finite_vector(value, label)
    norm = float(np.linalg.norm(vector))
    if not math.isfinite(norm) or norm <= epsilon:
        raise ConfigurationError(f"{label} magnitude must be greater than {epsilon}")
    normalized = np.asarray(vector / norm, dtype=float)
    normalized.setflags(write=False)
    return normalized


def normalize_angle_rad(angle_rad: float) -> float:
    """Normalize a finite angle to ``[0, 2π)`` deterministically."""

    angle = float(angle_rad)
    if not math.isfinite(angle):
        raise ConfigurationError("roll angle must be finite")
    normalized = angle % (2.0 * math.pi)
    return 0.0 if math.isclose(normalized, 2.0 * math.pi, abs_tol=1.0e-12) else normalized


@dataclass(frozen=True)
class SurfaceTarget:
    """Validated target point and surface geometry in ``base_link``."""

    position_base_m: np.ndarray
    surface_normal_base: np.ndarray
    tangent_hint_base: np.ndarray | None
    fixed_roll_rad: float | None
    roll_candidates_rad: tuple[float, ...]
    pre_approach_distance_m: float
    tool_frame: str
    target_id: str

    @classmethod
    def create(
        cls,
        *,
        position_base_m: Sequence[float],
        surface_normal_base: Sequence[float],
        tangent_hint_base: Sequence[float] | None = None,
        fixed_roll_rad: float | None = None,
        roll_candidates_rad: Sequence[float] | None = None,
        pre_approach_distance_m: float = 0.05,
        tool_frame: str = TCP_LINK,
        target_id: str,
        normal_epsilon: float = 1.0e-9,
        min_pre_approach_m: float = 0.01,
        max_pre_approach_m: float = 0.15,
    ) -> SurfaceTarget:
        """Build a target while normalizing only explicitly allowed inputs."""

        position = _finite_vector(position_base_m, "position_base_m")
        normal = normalize_vector(
            surface_normal_base,
            label="surface_normal_base",
            epsilon=normal_epsilon,
        )
        tangent = (
            None
            if tangent_hint_base is None
            else _finite_vector(tangent_hint_base, "tangent_hint_base")
        )
        if fixed_roll_rad is not None and roll_candidates_rad is not None:
            raise ConfigurationError("fixed roll and roll candidates are mutually exclusive")
        if fixed_roll_rad is not None:
            fixed = normalize_angle_rad(fixed_roll_rad)
            candidates: tuple[float, ...] = ()
        else:
            fixed = None
            source = (
                DEFAULT_ROLL_CANDIDATES_RAD
                if roll_candidates_rad is None
                else tuple(roll_candidates_rad)
            )
            candidates = tuple(normalize_angle_rad(angle) for angle in source)
            if not candidates:
                raise ConfigurationError("at least one roll candidate is required")
            rounded = {round(angle, 12) for angle in candidates}
            if len(rounded) != len(candidates):
                raise ConfigurationError("roll candidates contain duplicates after normalization")
        distance = float(pre_approach_distance_m)
        if not math.isfinite(distance):
            raise ConfigurationError("pre_approach_distance_m must be finite")
        if not min_pre_approach_m <= distance <= max_pre_approach_m:
            raise ConfigurationError(
                "pre_approach_distance_m must be within "
                f"[{min_pre_approach_m}, {max_pre_approach_m}]"
            )
        if tool_frame != TCP_LINK:
            raise ConfigurationError(f"tool_frame must be explicit Phase 1 frame {TCP_LINK!r}")
        if not target_id or not target_id.strip():
            raise ConfigurationError("target_id must be a non-empty string")
        return cls(
            position_base_m=position,
            surface_normal_base=normal,
            tangent_hint_base=tangent,
            fixed_roll_rad=fixed,
            roll_candidates_rad=candidates,
            pre_approach_distance_m=distance,
            tool_frame=tool_frame,
            target_id=target_id,
        )

    @property
    def effective_roll_candidates_rad(self) -> tuple[float, ...]:
        """Return the exact ordered roll candidates used to build a goal set."""

        if self.fixed_roll_rad is not None:
            return (self.fixed_roll_rad,)
        return self.roll_candidates_rad
