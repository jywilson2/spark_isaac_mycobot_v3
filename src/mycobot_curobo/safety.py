"""Deterministic fail-closed projection for bounded Cartesian residuals."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

import numpy as np
import yaml

from mycobot_curobo.errors import ConfigurationError
from mycobot_curobo.residual import CartesianResidual, ResidualObservation
from mycobot_curobo.robot_model import JointLimits


@dataclass(frozen=True)
class ResidualSafetyProfile:
    """Configured residual and execution bounds in SI units."""

    name: str
    max_translation_m: float
    max_rotation_rad: float
    max_lateral_error_m: float
    minimum_joint_limit_margin_rad: float
    max_state_age_s: float
    watchdog_timeout_s: float


class SafetyStatus(str, Enum):
    """Outcome of deterministic residual projection."""

    ACCEPTED = "accepted"
    CLIPPED = "clipped"
    REJECTED = "rejected"


@dataclass(frozen=True)
class SafetyDecision:
    """Projected residual plus explicit status and machine-readable reasons."""

    status: SafetyStatus
    projected_residual: CartesianResidual | None
    reasons: tuple[str, ...]
    lateral_error_m: float | None

    @property
    def accepted(self) -> bool:
        return self.status is not SafetyStatus.REJECTED


def load_residual_safety_profile(
    name: str,
    path: Path | str = Path("config/residual_safety.yml"),
) -> ResidualSafetyProfile:
    """Load one finite, strictly positive safety profile."""

    source = Path(path)
    if not source.is_file():
        raise ConfigurationError(f"residual safety config not found: {source}")
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    try:
        values = payload["profiles"][name]
    except (KeyError, TypeError) as exc:
        raise ConfigurationError(f"unknown residual safety profile: {name}") from exc
    profile = ResidualSafetyProfile(name=name, **values)
    for field_name, value in vars(profile).items():
        if field_name == "name":
            continue
        numeric = float(value)
        if not math.isfinite(numeric) or numeric <= 0.0:
            raise ConfigurationError(
                f"residual safety profile {name!r} field {field_name!r} "
                "must be finite and positive"
            )
    return profile


def lateral_distance_m(
    position_base_m: np.ndarray,
    goal_position_base_m: np.ndarray,
    approach_direction_base: np.ndarray,
) -> float:
    """Distance from a base-frame position to the target-normal line."""

    displacement = position_base_m - goal_position_base_m
    axial = float(displacement @ approach_direction_base)
    return float(np.linalg.norm(displacement - axial * approach_direction_base))


def _clip_norm(vector: np.ndarray, maximum: float) -> tuple[np.ndarray, bool]:
    norm = float(np.linalg.norm(vector))
    if norm <= maximum:
        return vector, False
    return vector * (maximum / norm), True


class SafetyProjector:
    """Clip residual magnitude and reject stale, infeasible, or off-corridor samples."""

    def __init__(self, profile: ResidualSafetyProfile, joint_limits: JointLimits) -> None:
        if joint_limits.lower_rad.shape != joint_limits.upper_rad.shape:
            raise ConfigurationError("joint lower/upper limit shapes differ")
        self.profile = profile
        self.joint_limits = joint_limits

    def project(
        self,
        observation: ResidualObservation,
        residual: CartesianResidual,
        *,
        evaluated_at_s: float,
        plan_executable: bool,
    ) -> SafetyDecision:
        """Return an explicit clipped/accepted decision or reject without a command."""

        now = float(evaluated_at_s)
        if not math.isfinite(now) or now < 0.0:
            raise ConfigurationError("safety evaluation time must be finite and non-negative")
        reasons: list[str] = []
        if not plan_executable:
            reasons.append("plan_not_executable")
        state_age = now - observation.measured_timestamp_s
        if state_age < 0.0:
            reasons.append("state_timestamp_in_future")
        elif state_age > self.profile.max_state_age_s:
            reasons.append("stale_robot_state")
        if now - observation.command_time_s > self.profile.watchdog_timeout_s:
            reasons.append("watchdog_timeout")

        nominal_joint = np.asarray(observation.nominal_joint_position_rad, dtype=float)
        measured_joint = np.asarray(observation.measured_joint_position_rad, dtype=float)
        expected_shape = self.joint_limits.lower_rad.shape
        if nominal_joint.shape != expected_shape or measured_joint.shape != expected_shape:
            reasons.append("joint_vector_shape")
        else:
            lower = self.joint_limits.lower_rad + self.profile.minimum_joint_limit_margin_rad
            upper = self.joint_limits.upper_rad - self.profile.minimum_joint_limit_margin_rad
            if np.any(lower > upper):
                raise ConfigurationError("joint-limit margin leaves no feasible envelope")
            if np.any(nominal_joint < lower) or np.any(nominal_joint > upper):
                reasons.append("nominal_joint_outside_feasibility_envelope")
            if np.any(measured_joint < lower) or np.any(measured_joint > upper):
                reasons.append("measured_joint_outside_feasibility_envelope")

        translation, translation_clipped = _clip_norm(
            np.asarray(residual.translation_base_m, dtype=float),
            self.profile.max_translation_m,
        )
        rotation, rotation_clipped = _clip_norm(
            np.asarray(residual.rotation_vector_base_rad, dtype=float),
            self.profile.max_rotation_rad,
        )
        corrected_position = (
            np.asarray(observation.nominal_tcp_position_base_m, dtype=float) + translation
        )
        lateral_error = lateral_distance_m(
            corrected_position,
            np.asarray(observation.goal_position_base_m, dtype=float),
            np.asarray(observation.approach_direction_base, dtype=float),
        )
        if lateral_error > self.profile.max_lateral_error_m:
            reasons.append("terminal_corridor_violation")

        if reasons:
            return SafetyDecision(
                status=SafetyStatus.REJECTED,
                projected_residual=None,
                reasons=tuple(reasons),
                lateral_error_m=lateral_error,
            )
        projected = CartesianResidual.create(translation, rotation)
        clipped_reasons = tuple(
            reason
            for clipped, reason in (
                (translation_clipped, "translation_clipped"),
                (rotation_clipped, "rotation_clipped"),
            )
            if clipped
        )
        return SafetyDecision(
            status=SafetyStatus.CLIPPED if clipped_reasons else SafetyStatus.ACCEPTED,
            projected_residual=projected,
            reasons=clipped_reasons,
            lateral_error_m=lateral_error,
        )
