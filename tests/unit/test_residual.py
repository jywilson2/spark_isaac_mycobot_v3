"""Phase 5/8 residual contract tests."""

from __future__ import annotations

from pathlib import Path

import pytest

from mycobot_curobo.errors import ConfigurationError
from mycobot_curobo.residual import (
    CartesianResidual,
    PolicyResidualCorrector,
    ResidualObservation,
    ResidualPolicyCheckpoint,
    ZeroResidualCorrector,
    load_residual_policy_checkpoint,
    save_residual_policy_checkpoint,
)
from mycobot_curobo.residual_train import (
    assert_sim_only_training_environment,
    train_offline_residual_policy,
)


def _observation() -> ResidualObservation:
    return ResidualObservation.create(
        request_id="phase5-unit",
        waypoint_index=0,
        command_time_s=1.0,
        measured_timestamp_s=1.0,
        nominal_joint_position_rad=[0.0] * 6,
        measured_joint_position_rad=[0.0] * 6,
        nominal_tcp_position_base_m=[0.1, 0.0, 0.2],
        goal_position_base_m=[0.1, 0.0, 0.2],
        approach_direction_base=[0.0, 0.0, 1.0],
    )


def test_zero_corrector_returns_exact_zero() -> None:
    residual = ZeroResidualCorrector().correction(_observation())

    assert residual == CartesianResidual.zero()
    assert residual.is_zero


def test_cartesian_residual_rejects_non_finite_or_wrong_shape() -> None:
    with pytest.raises(ConfigurationError, match="exactly 3"):
        CartesianResidual.create([0.0, 0.0], [0.0, 0.0, 0.0])
    with pytest.raises(ConfigurationError, match="finite"):
        CartesianResidual.create([0.0, float("nan"), 0.0], [0.0, 0.0, 0.0])


def test_observation_requires_unit_approach_and_matching_joint_vectors() -> None:
    values = vars(_observation())
    with pytest.raises(ConfigurationError, match="unit length"):
        ResidualObservation.create(**{**values, "approach_direction_base": [0.0, 0.0, 2.0]})
    with pytest.raises(ConfigurationError, match="equal non-zero"):
        ResidualObservation.create(**{**values, "measured_joint_position_rad": [0.0] * 5})


def test_policy_residual_corrector_is_nonzero_and_round_trips_checkpoint(
    tmp_path: Path,
) -> None:
    feature_dim = 9
    checkpoint = ResidualPolicyCheckpoint(
        schema_version=1,
        translation_gain=tuple(tuple(0.0 for _ in range(feature_dim)) for _ in range(3)),
        rotation_gain=tuple(tuple(0.0 for _ in range(feature_dim)) for _ in range(3)),
        translation_bias_m=(0.001, 0.0, 0.0),
        rotation_bias_rad=(0.0, 0.0, 0.0),
        feature_dim=feature_dim,
        sim_only=True,
    )
    path = save_residual_policy_checkpoint(checkpoint, tmp_path / "policy.json")
    loaded = load_residual_policy_checkpoint(path)
    residual = PolicyResidualCorrector(loaded).correction(_observation())
    assert residual.translation_base_m == pytest.approx((0.001, 0.0, 0.0))
    assert not residual.is_zero


def test_offline_training_is_sim_only_and_writes_checkpoint(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.delenv("ENABLE_MYCOBOT_HARDWARE_TESTS", raising=False)
    assert_sim_only_training_environment()
    summary = train_offline_residual_policy(
        output_path=str(tmp_path / "phase8_policy.json"),
        sample_count=32,
        seed=8008,
        tip_bias_m=(0.001, 0.0, 0.0),
    )
    assert summary.sim_only is True
    assert summary.backend == "offline_synthetic_sim"
    assert summary.mean_tip_error_after_m <= summary.mean_tip_error_before_m
    assert Path(summary.checkpoint_path).is_file()

    monkeypatch.setenv("ENABLE_MYCOBOT_HARDWARE_TESTS", "1")
    with pytest.raises(ConfigurationError, match="sim-only"):
        assert_sim_only_training_environment()
