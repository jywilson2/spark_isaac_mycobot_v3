"""Bounded local Cartesian residual → joint-delta mapping (Phase 8).

This is a finite-difference Jacobian map around the nominal waypoint. It is not
an IK solver, planner, or replacement-trajectory generator: Δq is clamped by
configuration and applied only as a local correction to an already validated
nominal joint sample.
"""

from __future__ import annotations

import math
from typing import Protocol, Sequence

import numpy as np

from mycobot_curobo.errors import ConfigurationError
from mycobot_curobo.residual import CartesianResidual
from mycobot_curobo.robot_model import Pose, RobotModelSpec, forward_kinematics


class ResidualJointMapper(Protocol):
    """Map a projected Cartesian residual to a bounded joint delta."""

    def map_to_joint_delta(
        self,
        residual: CartesianResidual,
        nominal_joint_position_rad: Sequence[float],
    ) -> tuple[float, ...]:
        """Return Δq in radians with the same length as the nominal joint vector."""


def _pose_error_twist(residual: CartesianResidual) -> np.ndarray:
    """Stack translational residual with a small orientation residual (6,)."""

    translation = np.asarray(residual.translation_base_m, dtype=float)
    rotation = np.asarray(residual.rotation_vector_base_rad, dtype=float)
    return np.concatenate([translation, rotation])


class FiniteDifferenceResidualMapper:
    """Local FK Jacobian map with an explicit per-joint Δq clamp."""

    def __init__(
        self,
        *,
        robot_spec: RobotModelSpec,
        max_joint_delta_rad: float,
        joint_step_rad: float = 1.0e-4,
        pose_evaluator=None,
    ) -> None:
        if not math.isfinite(max_joint_delta_rad) or max_joint_delta_rad <= 0.0:
            raise ConfigurationError("max_joint_delta_rad must be finite and positive")
        if not math.isfinite(joint_step_rad) or joint_step_rad <= 0.0:
            raise ConfigurationError("joint_step_rad must be finite and positive")
        self._robot_spec = robot_spec
        self._max_joint_delta_rad = float(max_joint_delta_rad)
        self._joint_step_rad = float(joint_step_rad)
        self._pose_evaluator = pose_evaluator

    def _evaluate(self, joint_position_rad: Sequence[float]) -> Pose:
        if self._pose_evaluator is not None:
            return self._pose_evaluator.evaluate(tuple(float(v) for v in joint_position_rad))
        return forward_kinematics(joint_position_rad, spec=self._robot_spec)

    def map_to_joint_delta(
        self,
        residual: CartesianResidual,
        nominal_joint_position_rad: Sequence[float],
    ) -> tuple[float, ...]:
        if residual.is_zero:
            return tuple(0.0 for _ in nominal_joint_position_rad)
        q0 = np.asarray(nominal_joint_position_rad, dtype=float)
        if q0.ndim != 1 or q0.size == 0:
            raise ConfigurationError("nominal joint position must be a non-empty vector")
        if not np.all(np.isfinite(q0)):
            raise ConfigurationError("nominal joint position must be finite")

        pose0 = self._evaluate(q0)
        jacobian = np.zeros((6, q0.size), dtype=float)
        step = self._joint_step_rad
        for joint_index in range(q0.size):
            qp = q0.copy()
            qp[joint_index] += step
            posep = self._evaluate(qp)
            d_pos = posep.position_m - pose0.position_m
            # Approximate angular velocity from relative quaternion (small angle).
            q0w = pose0.quaternion_wxyz
            qpw = posep.quaternion_wxyz
            # q_rel = qp * conj(q0); for close poses use 2*xyz of relative.
            w0, x0, y0, z0 = q0w
            wp, xp, yp, zp = qpw
            # conj(q0) = (w0, -x0, -y0, -z0)
            wr = wp * w0 + xp * x0 + yp * y0 + zp * z0
            xr = wp * (-x0) + xp * w0 + yp * (-z0) - zp * (-y0)
            yr = wp * (-y0) - xp * (-z0) + yp * w0 + zp * (-x0)
            zr = wp * (-z0) + xp * (-y0) - yp * (-x0) + zp * w0
            if wr < 0.0:
                xr, yr, zr = -xr, -yr, -zr
            d_rot = 2.0 * np.array([xr, yr, zr], dtype=float)
            jacobian[:3, joint_index] = d_pos / step
            jacobian[3:, joint_index] = d_rot / step

        twist = _pose_error_twist(residual)
        # Damped least squares avoids singular local maps without becoming IK search.
        damping = 1.0e-6
        jtj = jacobian.T @ jacobian + damping * np.eye(q0.size)
        delta = np.linalg.solve(jtj, jacobian.T @ twist)
        max_abs = float(np.max(np.abs(delta))) if delta.size else 0.0
        if max_abs > self._max_joint_delta_rad:
            delta = delta * (self._max_joint_delta_rad / max_abs)
        if not np.all(np.isfinite(delta)):
            raise ConfigurationError("residual joint delta mapping produced non-finite Δq")
        return tuple(float(value) for value in delta)


class FixedJointDeltaMapper:
    """Test/helper mapper that ignores Cartesian content and returns a fixed Δq."""

    def __init__(self, joint_delta_rad: Sequence[float]) -> None:
        self._delta = tuple(float(value) for value in joint_delta_rad)
        if not self._delta or not all(math.isfinite(value) for value in self._delta):
            raise ConfigurationError("fixed joint delta must be finite and non-empty")

    def map_to_joint_delta(
        self,
        residual: CartesianResidual,
        nominal_joint_position_rad: Sequence[float],
    ) -> tuple[float, ...]:
        if residual.is_zero:
            return tuple(0.0 for _ in nominal_joint_position_rad)
        if len(self._delta) != len(nominal_joint_position_rad):
            raise ConfigurationError("fixed joint delta length does not match nominal joints")
        return self._delta
