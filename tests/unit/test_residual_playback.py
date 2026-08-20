"""Unit tests for residual+noise playback apply path."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from mycobot_curobo.execution import CpuTcpPoseEvaluator
from mycobot_curobo.residual import (
    ResidualPolicyCheckpoint,
    save_residual_policy_checkpoint,
)
from mycobot_curobo.residual_playback import (
    apply_cartesian_tip_bias_to_bundle,
    apply_residual_noise_to_bundle,
    apply_residual_to_joint_positions,
    build_default_residual_stack,
)
from mycobot_curobo.robot_model import JOINT_NAMES, load_robot_model_spec


def _tiny_checkpoint(path: Path, *, translation_bias_m=(0.0005, 0.0, 0.0)) -> Path:
    feature_dim = 9
    checkpoint = ResidualPolicyCheckpoint(
        schema_version=1,
        translation_gain=tuple(tuple(0.0 for _ in range(feature_dim)) for _ in range(3)),
        rotation_gain=tuple(tuple(0.0 for _ in range(feature_dim)) for _ in range(3)),
        translation_bias_m=tuple(float(v) for v in translation_bias_m),
        rotation_bias_rad=(0.0, 0.0, 0.0),
        feature_dim=feature_dim,
        sim_only=True,
    )
    return save_residual_policy_checkpoint(checkpoint, path)


def _one_traj_bundle() -> tuple[dict, str, Path]:
    root = Path(__file__).resolve().parents[2]
    source = root / "artifacts/reports/phase7_2_multi_target_integration_2x5.bundle.json"
    assert source.is_file(), f"missing frozen bundle fixture: {source}"
    bundle = json.loads(source.read_text(encoding="utf-8"))
    first_id = next(iter(bundle["trajectories"]))
    bundle["trajectories"] = {first_id: bundle["trajectories"][first_id]}
    return bundle, first_id, root


def test_apply_residual_with_noise_exercises_nonzero_path(tmp_path: Path) -> None:
    checkpoint = _tiny_checkpoint(tmp_path / "policy.json")
    corrector, projector, mapper, pose_evaluator = build_default_residual_stack(
        checkpoint_path=str(checkpoint),
        residual_safety_profile="simulation_bounded_residual",
    )
    positions = np.zeros((4, len(JOINT_NAMES)), dtype=float)
    tip = pose_evaluator.evaluate(tuple(float(v) for v in positions[0])).position_m
    goal = (float(tip[0]), float(tip[1]), float(tip[2]) + 0.01)
    corrected, stats = apply_residual_to_joint_positions(
        positions,
        goal_position_base_m=goal,
        approach_direction_base=(0.0, 0.0, 1.0),
        corrector=corrector,
        projector=projector,
        joint_mapper=mapper,
        pose_evaluator=pose_evaluator,
        measurement_noise_std_rad=0.01,
        noise_seed=11,
    )
    assert stats.sim_only is True
    assert stats.waypoint_count == 4
    assert stats.measurement_noise_std_rad == 0.01
    assert stats.residual_nonzero_count >= 1
    assert stats.safety_reject_count == 0
    assert stats.applied_count >= 1
    assert np.all(np.isfinite(corrected))
    assert not np.allclose(corrected, positions)


def test_build_default_residual_stack_is_cwd_independent(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.chdir(tmp_path)
    corrector, projector, mapper, pose_evaluator = build_default_residual_stack(
        checkpoint_path=None,
        residual_safety_profile="simulation_bounded_residual",
    )
    assert projector.profile.name == "simulation_bounded_residual"
    tip = pose_evaluator.evaluate(tuple(0.0 for _ in JOINT_NAMES))
    assert tip.position_m.shape == (3,)
    del corrector, mapper


def test_apply_residual_noise_to_bundle_labels_sim_only(tmp_path: Path) -> None:
    checkpoint = _tiny_checkpoint(tmp_path / "policy.json")
    bundle, first_id, _root = _one_traj_bundle()
    payload, stats = apply_residual_noise_to_bundle(
        bundle,
        checkpoint_path=str(checkpoint),
        measurement_noise_std_rad=0.01,
        noise_seed=8008,
    )
    assert payload["residual_playback"]["sim_only"] is True
    assert payload["residual_playback"]["mode"] == "residual_correct"
    assert first_id in stats
    assert stats[first_id].waypoint_count > 0


def test_inject_tip_bias_shifts_tcp_toward_bias(tmp_path: Path) -> None:
    del tmp_path
    tip_bias = np.array([0.008, 0.0, 0.0], dtype=float)
    bundle, first_id, root = _one_traj_bundle()
    nominal = np.asarray(bundle["trajectories"][first_id]["position_rad"], dtype=float)
    biased_payload, stats = apply_cartesian_tip_bias_to_bundle(
        bundle,
        tip_bias_m=tuple(float(v) for v in tip_bias),
        residual_safety_profile="simulation_demo_visible",
        measurement_noise_std_rad=0.0,
    )
    assert biased_payload["residual_playback"]["mode"] == "inject_tip_bias"
    assert stats[first_id].applied_count >= 1
    biased = np.asarray(
        biased_payload["trajectories"][first_id]["position_rad"], dtype=float
    )
    assert not np.allclose(biased, nominal)
    pose_evaluator = CpuTcpPoseEvaluator(
        load_robot_model_spec(root / "config" / "robots" / "mycobot_280_m5.yml")
    )
    # Compare mid-trajectory TCP shift (avoid endpoints that may fallback).
    mid = nominal.shape[0] // 2
    tcp_nom = pose_evaluator.evaluate(tuple(float(v) for v in nominal[mid])).position_m
    tcp_bias = pose_evaluator.evaluate(tuple(float(v) for v in biased[mid])).position_m
    delta = np.asarray(tcp_bias, dtype=float) - np.asarray(tcp_nom, dtype=float)
    # Mapper is local / approximate; require most of the X bias and small Y/Z.
    assert float(delta[0]) > 0.004
    assert abs(float(delta[1])) < 0.004
    assert abs(float(delta[2])) < 0.004


def test_residual_on_biased_reduces_tip_error_vs_biased_only(tmp_path: Path) -> None:
    tip_bias = (0.008, 0.0, 0.0)
    bundle, first_id, root = _one_traj_bundle()
    biased_payload, _ = apply_cartesian_tip_bias_to_bundle(
        bundle,
        tip_bias_m=tip_bias,
        residual_safety_profile="simulation_demo_visible",
        measurement_noise_std_rad=0.0,
    )
    checkpoint = _tiny_checkpoint(
        tmp_path / "cancel.json",
        translation_bias_m=(-tip_bias[0], -tip_bias[1], -tip_bias[2]),
    )
    corrected_payload, corr_stats = apply_residual_noise_to_bundle(
        biased_payload,
        checkpoint_path=str(checkpoint),
        measurement_noise_std_rad=0.0,
        noise_seed=8008,
        residual_safety_profile="simulation_demo_visible",
    )
    assert corr_stats[first_id].applied_count >= 1
    pose_evaluator = CpuTcpPoseEvaluator(
        load_robot_model_spec(root / "config" / "robots" / "mycobot_280_m5.yml")
    )
    nominal = np.asarray(bundle["trajectories"][first_id]["position_rad"], dtype=float)
    biased = np.asarray(
        biased_payload["trajectories"][first_id]["position_rad"], dtype=float
    )
    corrected = np.asarray(
        corrected_payload["trajectories"][first_id]["position_rad"], dtype=float
    )
    mid = nominal.shape[0] // 2
    tcp_nom = np.asarray(
        pose_evaluator.evaluate(tuple(float(v) for v in nominal[mid])).position_m,
        dtype=float,
    )
    tcp_bias = np.asarray(
        pose_evaluator.evaluate(tuple(float(v) for v in biased[mid])).position_m,
        dtype=float,
    )
    tcp_corr = np.asarray(
        pose_evaluator.evaluate(tuple(float(v) for v in corrected[mid])).position_m,
        dtype=float,
    )
    err_bias = float(np.linalg.norm(tcp_bias - tcp_nom))
    err_corr = float(np.linalg.norm(tcp_corr - tcp_nom))
    assert err_bias > 0.004
    assert err_corr < err_bias * 0.6
