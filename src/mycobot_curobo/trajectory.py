"""Backend-neutral joint trajectory contracts and cuRobo extraction."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np

from mycobot_curobo.errors import ConfigurationError


def _numpy(value: Any) -> np.ndarray:
    """Convert NumPy/torch-like values without importing torch."""

    current = value
    if hasattr(current, "detach"):
        current = current.detach()
    if hasattr(current, "cpu"):
        current = current.cpu()
    if hasattr(current, "numpy"):
        current = current.numpy()
    return np.asarray(current)


def _scalar(value: Any, label: str) -> float:
    array = _numpy(value).reshape(-1)
    if array.size != 1:
        raise ConfigurationError(f"{label} must contain exactly one value")
    result = float(array[0])
    if not math.isfinite(result):
        raise ConfigurationError(f"{label} must be finite")
    return result


def _trajectory_matrix(value: Any, label: str) -> np.ndarray:
    array = _numpy(value)
    while array.ndim > 2 and array.shape[0] == 1:
        array = array[0]
    if array.ndim != 2:
        raise ConfigurationError(f"{label} must resolve to [waypoint, joint], got {array.shape}")
    return np.asarray(array, dtype=float)


@dataclass(frozen=True)
class JointTrajectory:
    """Finite joint trajectory in SI units and explicit joint order."""

    joint_names: tuple[str, ...]
    position_rad: np.ndarray
    velocity_rad_s: np.ndarray | None
    acceleration_rad_s2: np.ndarray | None
    jerk_rad_s3: np.ndarray | None
    dt_s: float

    @property
    def sample_count(self) -> int:
        return int(self.position_rad.shape[0])


def extract_curobo_trajectory(
    state: Any,
    valid_last_tstep: Any,
    *,
    expected_joint_names: tuple[str, ...],
    label: str,
) -> JointTrajectory:
    """Crop a cuRobo interpolated trajectory before checking finiteness.

    cuRobo's ``interpolated_last_tstep`` is the exclusive trim endpoint used by
    its own ``trim_joint_state_trajectory`` helper. A value of zero means the
    full trajectory.
    """

    if state is None:
        raise ConfigurationError(f"{label} trajectory is missing")
    names = tuple(getattr(state, "joint_names", ()) or ())
    if names != expected_joint_names:
        raise ConfigurationError(
            f"{label} joint names {names!r} do not match {expected_joint_names!r}"
        )
    position = _trajectory_matrix(getattr(state, "position", None), f"{label} position")
    endpoint = (
        position.shape[0]
        if valid_last_tstep is None
        else int(_scalar(valid_last_tstep, f"{label} valid_last_tstep"))
    )
    if endpoint == 0:
        endpoint = position.shape[0]
    if endpoint < 1 or endpoint > position.shape[0]:
        raise ConfigurationError(
            f"{label} valid endpoint {endpoint} outside [1, {position.shape[0]}]"
        )

    def optional_matrix(attribute: str, units_label: str) -> np.ndarray | None:
        value = getattr(state, attribute, None)
        if value is None:
            return None
        matrix = _trajectory_matrix(value, f"{label} {units_label}")
        if matrix.shape != position.shape:
            raise ConfigurationError(
                f"{label} {units_label} shape {matrix.shape} != {position.shape}"
            )
        return matrix[:endpoint].copy()

    cropped_position = position[:endpoint].copy()
    velocity = optional_matrix("velocity", "velocity")
    acceleration = optional_matrix("acceleration", "acceleration")
    jerk = optional_matrix("jerk", "jerk")
    arrays = [cropped_position, velocity, acceleration, jerk]
    if any(array is not None and not np.all(np.isfinite(array)) for array in arrays):
        raise ConfigurationError(f"{label} trajectory contains non-finite valid samples")
    if cropped_position.shape[1] != len(expected_joint_names):
        raise ConfigurationError(
            f"{label} trajectory has {cropped_position.shape[1]} joints; "
            f"expected {len(expected_joint_names)}"
        )
    dt_value = getattr(state, "dt", None)
    if dt_value is None:
        raise ConfigurationError(f"{label} trajectory dt is missing")
    dt_s = _scalar(dt_value, f"{label} dt")
    if dt_s <= 0.0:
        raise ConfigurationError(f"{label} trajectory dt must be positive")
    return JointTrajectory(
        joint_names=expected_joint_names,
        position_rad=cropped_position,
        velocity_rad_s=velocity,
        acceleration_rad_s2=acceleration,
        jerk_rad_s3=jerk,
        dt_s=dt_s,
    )


def concatenate_trajectories(
    approach: JointTrajectory,
    terminal: JointTrajectory,
    *,
    boundary_tolerance_rad: float = 1.0e-6,
) -> JointTrajectory:
    """Concatenate two segments, removing one duplicate boundary sample."""

    if approach.joint_names != terminal.joint_names:
        raise ConfigurationError("trajectory joint-name order differs at segment boundary")
    if not math.isclose(approach.dt_s, terminal.dt_s, rel_tol=0.0, abs_tol=1.0e-9):
        raise ConfigurationError("trajectory dt differs at segment boundary")
    if not np.allclose(
        approach.position_rad[-1],
        terminal.position_rad[0],
        atol=boundary_tolerance_rad,
        rtol=0.0,
    ):
        raise ConfigurationError("approach/terminal position boundary is discontinuous")
    terminal_start = 1

    def combine(first: np.ndarray | None, second: np.ndarray | None) -> np.ndarray | None:
        if first is None and second is None:
            return None
        if first is None or second is None:
            raise ConfigurationError("trajectory derivative availability differs by segment")
        return np.concatenate([first, second[terminal_start:]], axis=0)

    return JointTrajectory(
        joint_names=approach.joint_names,
        position_rad=np.concatenate(
            [approach.position_rad, terminal.position_rad[terminal_start:]],
            axis=0,
        ),
        velocity_rad_s=combine(approach.velocity_rad_s, terminal.velocity_rad_s),
        acceleration_rad_s2=combine(
            approach.acceleration_rad_s2,
            terminal.acceleration_rad_s2,
        ),
        jerk_rad_s3=combine(approach.jerk_rad_s3, terminal.jerk_rad_s3),
        dt_s=approach.dt_s,
    )
