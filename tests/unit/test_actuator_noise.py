"""Unit tests for Phase 8 actuator noise model."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from mycobot_curobo.actuator_noise import (
    JointActuatorNoiseModel,
    load_actuator_noise_profile,
)
from mycobot_curobo.errors import ConfigurationError
from mycobot_curobo.residual_train import train_offline_residual_policy
from mycobot_curobo.robot_model import JOINT_NAMES

ROOT = Path(__file__).resolve().parents[2]


def test_load_default_actuator_noise_profile() -> None:
    profile = load_actuator_noise_profile(
        "simulation_default",
        ROOT / "config" / "actuator_noise.yml",
    )
    assert profile.enabled is True
    assert profile.joint_std_rad == pytest.approx(0.005)
    assert profile.to_dict()["sim_only"] is True


def test_disabled_actuator_noise_is_identity() -> None:
    profile = load_actuator_noise_profile(
        "simulation_none",
        ROOT / "config" / "actuator_noise.yml",
    )
    model = JointActuatorNoiseModel(profile)
    positions = np.zeros((4, len(JOINT_NAMES)), dtype=float)
    disturbed = model.disturb_joint_positions(positions, seed=1)
    assert np.allclose(disturbed, positions)


def test_enabled_actuator_noise_changes_joints() -> None:
    profile = load_actuator_noise_profile(
        "simulation_default",
        ROOT / "config" / "actuator_noise.yml",
    )
    model = JointActuatorNoiseModel(profile)
    positions = np.zeros((8, len(JOINT_NAMES)), dtype=float)
    disturbed = model.disturb_joint_positions(positions, seed=42)
    assert not np.allclose(disturbed, positions)
    assert np.all(np.isfinite(disturbed))


def test_unknown_actuator_profile_raises() -> None:
    with pytest.raises(ConfigurationError, match="unknown actuator noise profile"):
        load_actuator_noise_profile("not_a_real_profile", ROOT / "config" / "actuator_noise.yml")


def test_train_offline_records_actuator_noise_profile(tmp_path: Path) -> None:
    out = tmp_path / "policy.json"
    summary = train_offline_residual_policy(
        output_path=str(out),
        sample_count=16,
        seed=11,
        tip_bias_m=(0.002, 0.0, 0.0),
        actuator_noise_profile="simulation_default",
    )
    assert summary.sim_only is True
    assert summary.actuator_noise_profile == "simulation_default"
    assert summary.actuator_noise_joint_std_rad == pytest.approx(0.005)
    assert out.is_file()
    assert summary.mean_tip_error_after_m < summary.mean_tip_error_before_m
