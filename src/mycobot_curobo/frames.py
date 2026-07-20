"""Robust task-frame construction for surface-normal approaches.

The configured signed TCP axis is aligned with the desired approach direction
(``-surface_normal`` by default). Remaining axes are built from a projected
tangent or a deterministic least-aligned basis fallback.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from mycobot_curobo.errors import ConfigurationError
from mycobot_curobo.robot_model import rotation_matrix_to_quaternion_wxyz
from mycobot_curobo.targets import SurfaceTarget, normalize_vector

_AXIS_INDEX = {"x": 0, "y": 1, "z": 2}


@dataclass(frozen=True)
class TaskFrameConfig:
    """Explicit TCP approach-axis convention."""

    tool_approach_axis: str = "z"
    tool_approach_sign: int = 1
    approach_against_outward_normal: bool = True
    tangent_epsilon: float = 1.0e-9
    orthonormal_tolerance: float = 1.0e-8

    def __post_init__(self) -> None:
        if self.tool_approach_axis not in _AXIS_INDEX:
            raise ConfigurationError("tool_approach_axis must be one of x, y, z")
        if self.tool_approach_sign not in (-1, 1):
            raise ConfigurationError("tool_approach_sign must be -1 or +1")
        if self.tangent_epsilon <= 0.0:
            raise ConfigurationError("tangent_epsilon must be positive")


@dataclass(frozen=True)
class TaskFrameCandidate:
    """One deterministic roll candidate for a surface target."""

    target_id: str
    tool_frame: str
    roll_rad: float
    position_base_m: np.ndarray
    rotation_base_from_tool: np.ndarray
    quaternion_wxyz: np.ndarray
    approach_direction_base: np.ndarray


def deterministic_tangent(
    approach_direction: np.ndarray,
    tangent_hint: np.ndarray | None,
    *,
    epsilon: float,
) -> np.ndarray:
    """Return a unit tangent perpendicular to the approach direction."""

    approach = normalize_vector(approach_direction, label="approach_direction")
    if tangent_hint is not None:
        hint = np.asarray(tangent_hint, dtype=float)
        projected = hint - float(np.dot(hint, approach)) * approach
        norm = float(np.linalg.norm(projected))
        if math.isfinite(norm) and norm > epsilon:
            return normalize_vector(projected, label="projected tangent", epsilon=epsilon)

    # The least-aligned world basis maximizes the projected norm. np.argmin is
    # deterministic on ties and therefore makes randomized tests replayable.
    basis = np.eye(3, dtype=float)[int(np.argmin(np.abs(approach)))]
    projected = basis - float(np.dot(basis, approach)) * approach
    return normalize_vector(projected, label="fallback tangent", epsilon=epsilon)


def _rotation_about_axis(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    x, y, z = normalize_vector(axis, label="roll axis")
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    one_minus_c = 1.0 - c
    return np.array(
        [
            [
                c + x * x * one_minus_c,
                x * y * one_minus_c - z * s,
                x * z * one_minus_c + y * s,
            ],
            [
                y * x * one_minus_c + z * s,
                c + y * y * one_minus_c,
                y * z * one_minus_c - x * s,
            ],
            [
                z * x * one_minus_c - y * s,
                z * y * one_minus_c + x * s,
                c + z * z * one_minus_c,
            ],
        ],
        dtype=float,
    )


def validate_rotation_matrix(rotation: np.ndarray, *, tolerance: float = 1.0e-8) -> None:
    """Raise when a matrix is non-finite, non-orthonormal, or left-handed."""

    matrix = np.asarray(rotation, dtype=float)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ConfigurationError("rotation must be a finite 3x3 matrix")
    if not np.allclose(matrix.T @ matrix, np.eye(3), atol=tolerance):
        raise ConfigurationError("rotation axes are not orthonormal")
    determinant = float(np.linalg.det(matrix))
    if not math.isclose(determinant, 1.0, abs_tol=tolerance):
        raise ConfigurationError(f"rotation determinant must be +1, got {determinant}")


def build_task_frame_candidates(
    target: SurfaceTarget,
    config: TaskFrameConfig = TaskFrameConfig(),
) -> tuple[TaskFrameCandidate, ...]:
    """Build ordered target frames for all permitted roll candidates."""

    approach = (
        -target.surface_normal_base
        if config.approach_against_outward_normal
        else target.surface_normal_base
    )
    approach = normalize_vector(approach, label="approach direction")
    axis_index = _AXIS_INDEX[config.tool_approach_axis]
    signed_tool_column = approach / float(config.tool_approach_sign)
    tangent = deterministic_tangent(
        approach,
        target.tangent_hint_base,
        epsilon=config.tangent_epsilon,
    )

    # Assign cyclic columns (i, i+1, i+2). cross(col_i, col_i+1) is col_i+2,
    # so the constructed matrix is right-handed for x, y, and z approach axes.
    tangent_index = (axis_index + 1) % 3
    final_index = (axis_index + 2) % 3
    base_rotation = np.empty((3, 3), dtype=float)
    base_rotation[:, axis_index] = signed_tool_column
    base_rotation[:, tangent_index] = tangent
    base_rotation[:, final_index] = np.cross(signed_tool_column, tangent)
    validate_rotation_matrix(
        base_rotation,
        tolerance=config.orthonormal_tolerance,
    )

    candidates: list[TaskFrameCandidate] = []
    for roll_rad in target.effective_roll_candidates_rad:
        rotation = _rotation_about_axis(approach, roll_rad) @ base_rotation
        validate_rotation_matrix(rotation, tolerance=config.orthonormal_tolerance)
        quaternion = rotation_matrix_to_quaternion_wxyz(rotation)
        candidates.append(
            TaskFrameCandidate(
                target_id=target.target_id,
                tool_frame=target.tool_frame,
                roll_rad=roll_rad,
                position_base_m=target.position_base_m.copy(),
                rotation_base_from_tool=rotation,
                quaternion_wxyz=quaternion,
                approach_direction_base=approach.copy(),
            )
        )
    return tuple(candidates)
