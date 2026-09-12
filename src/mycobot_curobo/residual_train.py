"""Sim-only offline residual-policy training (no physical hardware path)."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from mycobot_curobo.actuator_noise import (
    ActuatorNoiseProfile,
    JointActuatorNoiseModel,
    load_actuator_noise_profile,
)
from mycobot_curobo.errors import ConfigurationError
from mycobot_curobo.residual import (
    ResidualObservation,
    ResidualPolicyCheckpoint,
    observation_feature_vector,
    save_residual_policy_checkpoint,
)


def assert_sim_only_training_environment() -> None:
    """Refuse training when the hardware-enable flag is set."""

    if os.environ.get("ENABLE_MYCOBOT_HARDWARE_TESTS", "0") == "1":
        raise ConfigurationError(
            "Phase 8 residual training refuses ENABLE_MYCOBOT_HARDWARE_TESTS=1; "
            "training is sim-only"
        )


@dataclass(frozen=True)
class ResidualTrainSummary:
    """Offline training metrics with explicit sim-only labeling."""

    sample_count: int
    mean_tip_error_before_m: float
    mean_tip_error_after_m: float
    checkpoint_path: str
    sim_only: bool
    backend: str
    actuator_noise_profile: str
    actuator_noise_joint_std_rad: float


def _synthetic_observations(
    *,
    count: int,
    seed: int,
    tip_bias_m: np.ndarray,
    actuator_model: JointActuatorNoiseModel,
) -> tuple[list[ResidualObservation], np.ndarray, np.ndarray]:
    rng = np.random.default_rng(seed)
    observations: list[ResidualObservation] = []
    features: list[np.ndarray] = []
    targets_t: list[np.ndarray] = []
    for index in range(count):
        goal = np.array([0.15, 0.0, 0.20], dtype=float) + rng.normal(0.0, 0.01, size=3)
        nominal_tcp = goal - tip_bias_m + rng.normal(0.0, 0.0005, size=3)
        joint_nominal = rng.normal(0.0, 0.05, size=6)
        # Actuator disturbance on executed joints (motor/servo), then sensor noise.
        actuator_delta = actuator_model.sample_delta_rad(count=1, rng=rng)[0]
        joint_executed = joint_nominal + actuator_delta
        measurement_noise = rng.normal(0.0, 0.01, size=6)
        joint_measured = joint_executed + measurement_noise
        observation = ResidualObservation.create(
            request_id=f"phase8-train-{index}",
            waypoint_index=0,
            command_time_s=1.0,
            measured_timestamp_s=1.0,
            nominal_joint_position_rad=joint_nominal,
            measured_joint_position_rad=joint_measured,
            nominal_tcp_position_base_m=nominal_tcp,
            goal_position_base_m=goal,
            approach_direction_base=[0.0, 0.0, 1.0],
        )
        observations.append(observation)
        features.append(observation_feature_vector(observation))
        # Ideal local correction cancels the systematic tip bias (sim mismatch).
        targets_t.append(-tip_bias_m)
    return observations, np.asarray(features, dtype=float), np.asarray(targets_t, dtype=float)


def train_offline_residual_policy(
    *,
    output_path: str,
    sample_count: int = 128,
    seed: int = 8008,
    tip_bias_m: tuple[float, float, float] = (0.001, 0.0, 0.0),
    actuator_noise_profile: str = "simulation_default",
    actuator_noise_config: Path | str | None = None,
) -> ResidualTrainSummary:
    """Fit a linear residual policy on synthetic sim mismatch + actuator noise."""

    assert_sim_only_training_environment()
    if sample_count < 8:
        raise ConfigurationError("sample_count must be at least 8")
    bias = np.asarray(tip_bias_m, dtype=float)
    if bias.shape != (3,) or not np.all(np.isfinite(bias)):
        raise ConfigurationError("tip_bias_m must be a finite length-3 vector")

    profile: ActuatorNoiseProfile = load_actuator_noise_profile(
        actuator_noise_profile,
        path=actuator_noise_config,
    )
    actuator_model = JointActuatorNoiseModel(profile)

    observations, features, targets = _synthetic_observations(
        count=sample_count,
        seed=seed,
        tip_bias_m=bias,
        actuator_model=actuator_model,
    )
    feature_dim = int(features.shape[1])
    # Augment with bias column for affine fit.
    design = np.concatenate([features, np.ones((features.shape[0], 1))], dtype=float, axis=1)
    solution, *_ = np.linalg.lstsq(design, targets, rcond=None)
    gain = solution[:feature_dim, :].T  # 3 x feature_dim
    translation_bias = tuple(float(value) for value in solution[feature_dim, :])
    rotation_gain = tuple(tuple(0.0 for _ in range(feature_dim)) for _ in range(3))
    checkpoint = ResidualPolicyCheckpoint(
        schema_version=1,
        translation_gain=tuple(tuple(float(v) for v in row) for row in gain),
        rotation_gain=rotation_gain,
        translation_bias_m=translation_bias,
        rotation_bias_rad=(0.0, 0.0, 0.0),
        feature_dim=feature_dim,
        sim_only=True,
    )
    path = save_residual_policy_checkpoint(checkpoint, output_path)

    before = []
    after = []
    for observation, target in zip(observations, targets, strict=True):
        tip_error = np.asarray(observation.goal_position_base_m) - np.asarray(
            observation.nominal_tcp_position_base_m
        )
        before.append(float(np.linalg.norm(tip_error)))
        feature = observation_feature_vector(observation)
        predicted = gain @ feature + np.asarray(translation_bias)
        # Residual is applied to TCP as +translation; ideal cancels bias so tip_error+pred ≈ 0
        after.append(float(np.linalg.norm(tip_error + predicted)))
        del target

    return ResidualTrainSummary(
        sample_count=sample_count,
        mean_tip_error_before_m=float(np.mean(before)),
        mean_tip_error_after_m=float(np.mean(after)),
        checkpoint_path=str(path),
        sim_only=True,
        backend="offline_synthetic_sim",
        actuator_noise_profile=profile.name,
        actuator_noise_joint_std_rad=float(profile.joint_std_rad),
    )
