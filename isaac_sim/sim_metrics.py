"""NumPy-only pose metrics for Isaac Sim playback results."""

from __future__ import annotations

import math
from typing import Sequence

import numpy as np


def tip_position_error_m(actual_m: Sequence[float], goal_m: Sequence[float]) -> float:
    """Return Euclidean TCP position error in meters."""

    actual = np.asarray(actual_m, dtype=float)
    goal = np.asarray(goal_m, dtype=float)
    if actual.shape != (3,) or goal.shape != (3,):
        raise ValueError("tip positions must be 3-vectors")
    if not np.all(np.isfinite(actual)) or not np.all(np.isfinite(goal)):
        raise ValueError("tip positions must be finite")
    return float(np.linalg.norm(actual - goal))


def orientation_error_rad(
    actual_wxyz: Sequence[float],
    goal_wxyz: Sequence[float],
) -> float:
    """Return shortest angular distance between scalar-first quaternions."""

    actual = np.asarray(actual_wxyz, dtype=float)
    goal = np.asarray(goal_wxyz, dtype=float)
    if actual.shape != (4,) or goal.shape != (4,):
        raise ValueError("orientations must be wxyz 4-vectors")
    if not np.all(np.isfinite(actual)) or not np.all(np.isfinite(goal)):
        raise ValueError("orientations must be finite")
    actual_norm = float(np.linalg.norm(actual))
    goal_norm = float(np.linalg.norm(goal))
    if actual_norm <= 0.0 or goal_norm <= 0.0:
        raise ValueError("orientation quaternions must be non-zero")
    dot = float(np.dot(actual / actual_norm, goal / goal_norm))
    return 2.0 * math.acos(float(np.clip(abs(dot), -1.0, 1.0)))
