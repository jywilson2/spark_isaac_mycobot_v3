"""Typed bounded-residual contract with zero and Phase 8 policy correctors."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol, Sequence

import numpy as np

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


def observation_feature_vector(observation: ResidualObservation) -> np.ndarray:
    """Build the Phase 8 policy feature vector (tip error + joint tracking error)."""

    tip_error = np.asarray(observation.goal_position_base_m, dtype=float) - np.asarray(
        observation.nominal_tcp_position_base_m, dtype=float
    )
    joint_error = np.asarray(observation.measured_joint_position_rad, dtype=float) - np.asarray(
        observation.nominal_joint_position_rad, dtype=float
    )
    return np.concatenate([tip_error, joint_error])


class ResidualCorrector(Protocol):
    """Return a bounded local correction, never a replacement trajectory."""

    def correction(self, observation: ResidualObservation) -> CartesianResidual:
        """Compute one correction from a typed execution observation."""


class ZeroResidualCorrector:
    """Phase 5 corrector that leaves every cuRobo command unchanged."""

    def correction(self, observation: ResidualObservation) -> CartesianResidual:
        del observation
        return CartesianResidual.zero()


@dataclass(frozen=True)
class ResidualPolicyCheckpoint:
    """Advisory linear residual policy weights (validation remains authoritative)."""

    schema_version: int
    translation_gain: tuple[tuple[float, ...], ...]
    rotation_gain: tuple[tuple[float, ...], ...]
    translation_bias_m: tuple[float, float, float]
    rotation_bias_rad: tuple[float, float, float]
    feature_dim: int
    sim_only: bool = True

    def to_dict(self) -> dict[str, object]:
        return {
            "schema_version": self.schema_version,
            "translation_gain": [list(row) for row in self.translation_gain],
            "rotation_gain": [list(row) for row in self.rotation_gain],
            "translation_bias_m": list(self.translation_bias_m),
            "rotation_bias_rad": list(self.rotation_bias_rad),
            "feature_dim": self.feature_dim,
            "sim_only": bool(self.sim_only),
            "advisory": True,
        }

    @classmethod
    def from_dict(cls, payload: dict[str, object]) -> ResidualPolicyCheckpoint:
        try:
            schema_version = int(payload["schema_version"])
            feature_dim = int(payload["feature_dim"])
            translation_gain = tuple(
                tuple(float(value) for value in row)
                for row in payload["translation_gain"]  # type: ignore[arg-type]
            )
            rotation_gain = tuple(
                tuple(float(value) for value in row)
                for row in payload["rotation_gain"]  # type: ignore[arg-type]
            )
            translation_bias = _finite_tuple(
                payload["translation_bias_m"], 3, "translation_bias_m"
            )  # type: ignore[arg-type]
            rotation_bias = _finite_tuple(payload["rotation_bias_rad"], 3, "rotation_bias_rad")  # type: ignore[arg-type]
            sim_only = bool(payload.get("sim_only", True))
        except (KeyError, TypeError, ValueError) as exc:
            raise ConfigurationError("invalid residual policy checkpoint payload") from exc
        if schema_version != 1:
            raise ConfigurationError(f"unsupported residual checkpoint schema: {schema_version}")
        if not sim_only:
            raise ConfigurationError("residual checkpoints must be labeled sim_only=true")
        if len(translation_gain) != 3 or any(len(row) != feature_dim for row in translation_gain):
            raise ConfigurationError("translation_gain must be 3 x feature_dim")
        if len(rotation_gain) != 3 or any(len(row) != feature_dim for row in rotation_gain):
            raise ConfigurationError("rotation_gain must be 3 x feature_dim")
        return cls(
            schema_version=schema_version,
            translation_gain=translation_gain,
            rotation_gain=rotation_gain,
            translation_bias_m=translation_bias,
            rotation_bias_rad=rotation_bias,
            feature_dim=feature_dim,
            sim_only=True,
        )


def save_residual_policy_checkpoint(
    checkpoint: ResidualPolicyCheckpoint, path: Path | str
) -> Path:
    """Write an advisory sim-only residual checkpoint."""

    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps(checkpoint.to_dict(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return destination


def load_residual_policy_checkpoint(path: Path | str) -> ResidualPolicyCheckpoint:
    """Load an advisory sim-only residual checkpoint."""

    source = Path(path)
    if not source.is_file():
        raise ConfigurationError(f"residual policy checkpoint not found: {source}")
    payload = json.loads(source.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ConfigurationError("residual policy checkpoint must be a JSON object")
    return ResidualPolicyCheckpoint.from_dict(payload)


class PolicyResidualCorrector:
    """Non-zero ResidualCorrector driven by an advisory linear checkpoint."""

    def __init__(self, checkpoint: ResidualPolicyCheckpoint) -> None:
        self._checkpoint = checkpoint
        self._translation_gain = np.asarray(checkpoint.translation_gain, dtype=float)
        self._rotation_gain = np.asarray(checkpoint.rotation_gain, dtype=float)
        self._translation_bias = np.asarray(checkpoint.translation_bias_m, dtype=float)
        self._rotation_bias = np.asarray(checkpoint.rotation_bias_rad, dtype=float)

    @property
    def checkpoint(self) -> ResidualPolicyCheckpoint:
        return self._checkpoint

    def correction(self, observation: ResidualObservation) -> CartesianResidual:
        features = observation_feature_vector(observation)
        if features.size != self._checkpoint.feature_dim:
            raise ConfigurationError(
                "residual observation feature dimension does not match checkpoint"
            )
        translation = self._translation_gain @ features + self._translation_bias
        rotation = self._rotation_gain @ features + self._rotation_bias
        return CartesianResidual.create(translation, rotation)


class FixedResidualCorrector:
    """Deterministic non-zero corrector for contract and unit tests."""

    def __init__(self, residual: CartesianResidual) -> None:
        self._residual = residual

    def correction(self, observation: ResidualObservation) -> CartesianResidual:
        del observation
        return self._residual
