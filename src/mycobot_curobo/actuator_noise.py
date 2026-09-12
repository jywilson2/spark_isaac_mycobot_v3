"""Configuration-driven joint actuator / servo disturbance (sim-only).

Distinct from:
- tip bias / TCP model mismatch (systematic Cartesian offset), and
- measurement noise on sensed joint positions.

Actuator noise models motor/servo tracking error on commanded joints.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np
import yaml

from mycobot_curobo.errors import ConfigurationError
from mycobot_curobo.robot_model import JOINT_NAMES


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


@dataclass(frozen=True)
class ActuatorNoiseProfile:
    """Joint-space actuator disturbance profile."""

    name: str
    enabled: bool
    joint_std_rad: float
    seed: int
    clamp_to_limits: bool
    description: str = ""

    def to_dict(self) -> dict[str, object]:
        return {
            "name": self.name,
            "enabled": self.enabled,
            "joint_std_rad": self.joint_std_rad,
            "seed": self.seed,
            "clamp_to_limits": self.clamp_to_limits,
            "description": self.description,
            "sim_only": True,
        }


def load_actuator_noise_profile(
    name: str,
    path: Path | str | None = None,
) -> ActuatorNoiseProfile:
    """Load a named actuator-noise profile from YAML."""

    source = Path(path) if path is not None else _repo_root() / "config" / "actuator_noise.yml"
    if not source.is_file():
        raise ConfigurationError(f"actuator noise config not found: {source}")
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(payload, dict) or "profiles" not in payload:
        raise ConfigurationError("actuator noise config must define profiles")
    profiles = payload["profiles"]
    if not isinstance(profiles, dict) or name not in profiles:
        raise ConfigurationError(f"unknown actuator noise profile: {name}")
    raw = profiles[name]
    if not isinstance(raw, dict):
        raise ConfigurationError(f"actuator noise profile {name!r} must be a mapping")
    enabled = bool(raw.get("enabled", True))
    joint_std = float(raw.get("joint_std_rad", 0.0))
    if not math.isfinite(joint_std) or joint_std < 0.0:
        raise ConfigurationError("joint_std_rad must be finite and non-negative")
    if enabled and joint_std <= 0.0:
        raise ConfigurationError(
            f"actuator noise profile {name!r} is enabled but joint_std_rad is not positive"
        )
    seed = int(raw.get("seed", 9001))
    clamp = bool(raw.get("clamp_to_limits", True))
    description = str(raw.get("description", ""))
    return ActuatorNoiseProfile(
        name=name,
        enabled=enabled,
        joint_std_rad=joint_std,
        seed=seed,
        clamp_to_limits=clamp,
        description=description,
    )


class JointActuatorNoiseModel:
    """Sample additive joint disturbances for commanded / executed states."""

    def __init__(self, profile: ActuatorNoiseProfile) -> None:
        self._profile = profile

    @property
    def profile(self) -> ActuatorNoiseProfile:
        return self._profile

    def sample_delta_rad(
        self,
        *,
        count: int,
        rng: np.random.Generator,
        n_joints: int = len(JOINT_NAMES),
    ) -> np.ndarray:
        """Return ``(count, n_joints)`` additive Δq samples (zeros when disabled)."""

        if count < 0:
            raise ConfigurationError("count must be non-negative")
        if n_joints <= 0:
            raise ConfigurationError("n_joints must be positive")
        if not self._profile.enabled or self._profile.joint_std_rad == 0.0:
            return np.zeros((count, n_joints), dtype=float)
        return rng.normal(0.0, self._profile.joint_std_rad, size=(count, n_joints))

    def disturb_joint_positions(
        self,
        position_rad: Sequence[Sequence[float]] | np.ndarray,
        *,
        seed: int | None = None,
        lower_rad: Sequence[float] | None = None,
        upper_rad: Sequence[float] | None = None,
    ) -> np.ndarray:
        """Return commanded joints plus actuator disturbance."""

        positions = np.asarray(position_rad, dtype=float)
        if positions.ndim != 2:
            raise ConfigurationError("position_rad must be a 2-D array of joint waypoints")
        if positions.shape[1] != len(JOINT_NAMES):
            raise ConfigurationError(
                f"position_rad must have shape (N, {len(JOINT_NAMES)}), got {positions.shape}"
            )
        rng = np.random.default_rng(self._profile.seed if seed is None else int(seed))
        delta = self.sample_delta_rad(count=int(positions.shape[0]), rng=rng)
        disturbed = positions + delta
        if self._profile.clamp_to_limits and lower_rad is not None and upper_rad is not None:
            lower = np.asarray(lower_rad, dtype=float)
            upper = np.asarray(upper_rad, dtype=float)
            if lower.shape != (len(JOINT_NAMES),) or upper.shape != (len(JOINT_NAMES),):
                raise ConfigurationError("joint limit vectors must match JOINT_NAMES length")
            disturbed = np.clip(disturbed, lower, upper)
        return disturbed
