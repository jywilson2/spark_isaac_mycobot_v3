"""Kit-independent joint-name mapping for Isaac articulation playback."""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np

from mycobot_curobo.robot_model import JOINT_NAMES

REVOLUTE_JOINT_NAMES = JOINT_NAMES


def revolute_dof_indices(
    articulation_dof_names: Sequence[str],
    required_names: Sequence[str] = REVOLUTE_JOINT_NAMES,
) -> tuple[int, ...]:
    """Map required revolute joints into articulation DOF order."""

    available = tuple(articulation_dof_names)
    if len(set(available)) != len(available):
        raise ValueError("articulation DOF names contain duplicates")
    required = tuple(required_names)
    if len(set(required)) != len(required):
        raise ValueError("required revolute joint names contain duplicates")
    missing = [name for name in required if name not in available]
    if missing:
        raise ValueError(f"articulation is missing required DOFs: {missing}")
    return tuple(available.index(name) for name in required)


def articulation_position_targets(
    revolute_position_rad: Sequence[float],
    articulation_dof_names: Sequence[str],
    current_position_rad: Sequence[float],
) -> np.ndarray:
    """Insert six ordered plan values into a full articulation DOF vector."""

    revolute = np.asarray(revolute_position_rad, dtype=float)
    current = np.asarray(current_position_rad, dtype=float)
    if revolute.shape != (len(REVOLUTE_JOINT_NAMES),):
        raise ValueError("revolute positions must have shape (6,)")
    if current.shape != (len(articulation_dof_names),):
        raise ValueError("current positions must match articulation DOF names")
    if not np.all(np.isfinite(revolute)) or not np.all(np.isfinite(current)):
        raise ValueError("joint positions must be finite")
    targets = current.copy()
    targets[list(revolute_dof_indices(articulation_dof_names))] = revolute
    return targets
