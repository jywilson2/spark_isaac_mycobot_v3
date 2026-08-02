"""Unit tests for Phase 7.5 incremental target-population contracts."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import pytest
import yaml

from mycobot_curobo.errors import ConfigurationError
from mycobot_curobo.incremental_population import (
    IncrementalPopulationRunner,
    PopulationStopReason,
    build_incremental_artifact_base_name,
    format_dz_label,
    format_populate_candidate,
    format_suite_table,
    sample_mean_std,
)
from mycobot_curobo.multi_target import (
    INCREMENTAL_FORBIDDEN_KEYS,
    MultiTargetFailureCategory,
    OptimisticTipContactDetector,
    TargetPopulation,
    deserialize_episode,
    load_multi_target_suite_config,
    serialize_episode,
)
from mycobot_curobo.planner import NominalPlan, PlanningFailure, PlanningOutcome
from mycobot_curobo.trajectory import JointTrajectory
from mycobot_curobo.validation import ValidatedPlan, ValidationMetrics, ValidationReport

ROOT = Path(__file__).parents[2]


def _trajectory(positions: list[list[float]]) -> JointTrajectory:
    array = np.asarray(positions, dtype=float)
    zeros = np.zeros_like(array)
    return JointTrajectory(
        joint_names=(
            "joint2_to_joint1",
            "joint3_to_joint2",
            "joint4_to_joint3",
            "joint5_to_joint4",
            "joint6_to_joint5",
            "joint6output_to_joint6",
        ),
        position_rad=array,
        dt_s=0.05,
        velocity_rad_s=zeros,
        acceleration_rad_s2=zeros,
        jerk_rad_s3=zeros,
    )


def _plan(request_id: str, seed: int) -> NominalPlan:
    approach = _trajectory([[0.0] * 6, [0.1] * 6])
    terminal = _trajectory([[0.1] * 6, [0.2] * 6])
    combined = _trajectory([[0.0] * 6, [0.1] * 6, [0.2] * 6])
    return NominalPlan(
        request_id=request_id,
        selected_goal_index=0,
        selected_roll_rad=0.0,
        approach_trajectory=approach,
        terminal_trajectory=terminal,
        combined_trajectory=combined,
        planner_status="success",
        planner_timings_s={"total": 0.01},
        curobo_version="0.8.0",
        scene_revision="test",
        planner_profile="benchmark_reproducible",
        random_seed=seed,
    )


def _metrics() -> ValidationMetrics:
    return ValidationMetrics(
        max_lateral_error_m=0.0,
        max_approach_axis_error_rad=0.0,
        max_roll_error_rad=0.0,
        terminal_position_error_m=0.0,
        terminal_orientation_error_rad=0.0,
        max_progress_regression_m=0.0,
        minimum_joint_limit_margin_rad=0.0,
        minimum_self_collision_clearance_m=0.0,
        minimum_world_collision_clearance_m=0.0,
    )


def _validated(plan: NominalPlan) -> ValidatedPlan:
    return ValidatedPlan(
        nominal_plan=plan,
        report=ValidationReport(
            request_id=plan.request_id,
            profile_name="simulation_initial",
            valid=True,
            violations=(),
            metrics=_metrics(),
        ),
        validation_status="passed",
        executable=True,
    )


class _FakePlanner:
    def __init__(self, outcomes: list[PlanningOutcome]) -> None:
        self._outcomes = list(outcomes)
        self.calls = 0
        self.scene_cuboid_counts: list[int] = []

    def plan(self, request: Any) -> PlanningOutcome:
        self.calls += 1
        if not self._outcomes:
            raise AssertionError("unexpected plan call")
        return self._outcomes.pop(0)


def _ok() -> PlanningOutcome:
    return PlanningOutcome(plan=_plan("ok", 1), failure=None)


def _fail() -> PlanningOutcome:
    return PlanningOutcome(
        plan=None,
        failure=PlanningFailure("planning_infeasible", "plan_failed", "failed"),
    )


def _base_incremental_yaml(**overrides: Any) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "target_population": "incremental",
        "episode_count": 1,
        "root_seed": 7,
        "frame": "g_base",
        "placement": "random",
        "retain_targets_after_contact": True,
        "max_consecutive_target_failures": 3,
        "min_targets_per_episode": 1,
        "max_placement_attempts": 5000,
        "delta_z_m": 0.30,
        "tip_allow_link_names": ["joint6_flange"],
        "field_aabb": {
            "minimum_m": [-0.22, -0.22, 0.08],
            "maximum_m": [0.22, 0.22, 0.38],
        },
        "keep_outs": [{"minimum_m": [-0.05, -0.05, 0.08], "maximum_m": [0.05, 0.05, 0.38]}],
        "arm_z_motion_range_m": 0.28,
        "target_edge_m": 0.014,
        "outward_normal_base": [0.0, 0.0, 1.0],
        "flange_diameter_assumption_m": 0.031,
        "start_joint_position_rad": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "roll_candidates_deg": [0, 90, 180, 270],
        "planner_profile": "benchmark_reproducible",
        "validation_profile": "simulation_initial",
        "scene_revision_prefix": "phase7_5-variable",
        "artifact_path": "artifacts/phase7_5",
        "minimum_self_collision_clearance_m": 0.0,
        "minimum_world_collision_clearance_m": 0.006,
        "lighting": {
            "dome_intensity": 400.0,
            "distant_intensity": 1000.0,
            "distant_angle_deg": [45.0, -30.0, 0.0],
            "color": [1.0, 0.95, 0.9],
        },
        "pre_approach_distance_m": 0.01,
    }
    payload.update(overrides)
    return payload


def _write_config(tmp_path: Path, payload: dict[str, Any]) -> Path:
    path = tmp_path / "suite.yml"
    path.write_text(yaml.safe_dump(payload), encoding="utf-8")
    return path


def test_example_config_loads_incremental() -> None:
    config = load_multi_target_suite_config(ROOT / "config/phase7_5_variable_targets_dz_0_30.yml")
    assert config.target_population is TargetPopulation.INCREMENTAL
    assert config.target_count == 0
    assert config.retain_targets_after_contact is True
    assert config.placement.value == "random"
    assert config.max_consecutive_target_failures == 5
    assert config.min_targets_per_episode == 1
    assert config.delta_z_m == pytest.approx(0.30)
    assert config.require_tip_ik is False
    assert config.max_field_regenerations == 0


@pytest.mark.parametrize("forbidden_key", sorted(INCREMENTAL_FORBIDDEN_KEYS))
def test_incremental_forbids_fixed_mode_keys(tmp_path: Path, forbidden_key: str) -> None:
    payload = _base_incremental_yaml()
    if forbidden_key == "target_count":
        payload["target_count"] = 10
    elif forbidden_key == "order":
        payload["order"] = "z_desc"
    elif forbidden_key == "require_tip_ik":
        payload["require_tip_ik"] = True
    else:
        payload[forbidden_key] = 1
    path = _write_config(tmp_path, payload)
    with pytest.raises(ConfigurationError, match=forbidden_key):
        load_multi_target_suite_config(path)


def test_incremental_requires_retain_true(tmp_path: Path) -> None:
    payload = _base_incremental_yaml(retain_targets_after_contact=False)
    path = _write_config(tmp_path, payload)
    with pytest.raises(ConfigurationError, match="retain_targets_after_contact"):
        load_multi_target_suite_config(path)


def test_incremental_requires_random_placement(tmp_path: Path) -> None:
    payload = _base_incremental_yaml(placement="grid")
    path = _write_config(tmp_path, payload)
    with pytest.raises(ConfigurationError, match="placement: random"):
        load_multi_target_suite_config(path)


def test_artifact_name_from_achieved_counts() -> None:
    assert format_dz_label(0.30) == "dz0_30"
    name = build_incremental_artifact_base_name(
        scene_revision_prefix="phase7_5-variable",
        delta_z_m=0.30,
        accepted_counts=(14, 11, 16),
        root_seed=4242,
    )
    assert name == "phase7_5-variable_dz0_30_n14-11-16_seed4242"


def test_sample_mean_std_uses_sample_variance() -> None:
    mean, std = sample_mean_std([2.0, 4.0])
    assert mean == pytest.approx(3.0)
    assert std == pytest.approx(math_sqrt2 := (2.0**0.5))
    del math_sqrt2
    mean_one, std_one = sample_mean_std([5.0])
    assert mean_one == pytest.approx(5.0)
    assert std_one is None


def test_streak_reset_and_stop_at_consecutive_failures(tmp_path: Path) -> None:
    path = _write_config(
        tmp_path,
        _base_incremental_yaml(
            max_consecutive_target_failures=3,
            min_targets_per_episode=1,
            max_targets_per_episode=0,
        ),
    )
    config = load_multi_target_suite_config(path)
    # Accept, fail, accept (resets streak), then three fails → stop.
    outcomes = [_ok(), _fail(), _ok(), _fail(), _fail(), _fail()]
    planner = _FakePlanner(outcomes)
    scene_counts: list[int] = []

    def planner_factory(seed: int, scene_model: dict, links: tuple[str, ...]) -> _FakePlanner:
        del seed, links
        scene_counts.append(len(scene_model.get("cuboid") or {}))
        return planner

    def validator(plan: NominalPlan, request: Any, clearance: Any, cube: Any) -> ValidatedPlan:
        del request, clearance, cube
        return _validated(plan)

    lines: list[str] = []
    runner = IncrementalPopulationRunner(
        planner_factory=planner_factory,
        validator=validator,
        contact_detector_factory=lambda _ep, to_id: OptimisticTipContactDetector(to_id),
        console_log=lines.append,
        apply_reach_prefilter=False,
    )
    suite = runner.run_suite(config, root_seed=7, episode_count=1)
    result = suite.results[0]
    extra = suite.extras[0]
    assert result.succeeded
    assert len(result.contacted_ids) == 2
    assert extra.stop_reason is PopulationStopReason.CONSECUTIVE_FAILURES
    assert extra.consecutive_failures_at_stop == 3
    assert extra.total_target_failures == 4
    # World grows: first plan has 0 neighbors (active omitted), second has 1, etc.
    assert scene_counts[0] == 0
    assert scene_counts[2] == 1  # third plan after two accepts
    assert any("streak 0/3" in line for line in lines)
    assert any("streak 3/3" in line and "0 more failures" in line for line in lines)
    assert suite.artifact_base_name.endswith("_n2_seed7")


def test_stop_at_max_targets_and_total_failures(tmp_path: Path) -> None:
    path = _write_config(
        tmp_path,
        _base_incremental_yaml(
            max_consecutive_target_failures=10,
            max_targets_per_episode=2,
            max_total_target_failures=0,
        ),
    )
    config = load_multi_target_suite_config(path)
    planner = _FakePlanner([_ok(), _ok(), _ok()])

    def planner_factory(seed: int, scene_model: dict, links: tuple[str, ...]) -> _FakePlanner:
        del seed, scene_model, links
        return planner

    runner = IncrementalPopulationRunner(
        planner_factory=planner_factory,
        validator=lambda plan, *_a: _validated(plan),
        contact_detector_factory=lambda _ep, to_id: OptimisticTipContactDetector(to_id),
        console_log=lambda _m: None,
        apply_reach_prefilter=False,
    )
    suite = runner.run_suite(config, root_seed=11, episode_count=1)
    assert suite.extras[0].stop_reason is PopulationStopReason.MAX_TARGETS
    assert len(suite.results[0].contacted_ids) == 2

    total_path = tmp_path / "total_fail.yml"
    total_path.write_text(
        yaml.safe_dump(
            _base_incremental_yaml(
                max_consecutive_target_failures=50,
                max_total_target_failures=2,
                min_targets_per_episode=1,
            )
        ),
        encoding="utf-8",
    )
    config2 = load_multi_target_suite_config(total_path)
    planner2 = _FakePlanner([_ok(), _fail(), _fail()])
    runner2 = IncrementalPopulationRunner(
        planner_factory=lambda *_a: planner2,
        validator=lambda plan, *_a: _validated(plan),
        contact_detector_factory=lambda _ep, to_id: OptimisticTipContactDetector(to_id),
        console_log=lambda _m: None,
        apply_reach_prefilter=False,
    )
    suite2 = runner2.run_suite(config2, root_seed=12, episode_count=1)
    assert suite2.extras[0].stop_reason is PopulationStopReason.TOTAL_FAILURES
    assert suite2.extras[0].total_target_failures == 2
    assert len(suite2.results[0].contacted_ids) == 1


def test_insufficient_targets_fails_episode(tmp_path: Path) -> None:
    path = _write_config(
        tmp_path,
        _base_incremental_yaml(
            max_consecutive_target_failures=2,
            min_targets_per_episode=3,
        ),
    )
    config = load_multi_target_suite_config(path)
    planner = _FakePlanner([_fail(), _fail()])
    runner = IncrementalPopulationRunner(
        planner_factory=lambda *_a: planner,
        validator=lambda plan, *_a: _validated(plan),
        contact_detector_factory=lambda _ep, to_id: OptimisticTipContactDetector(to_id),
        console_log=lambda _m: None,
        apply_reach_prefilter=False,
    )
    suite = runner.run_suite(config, root_seed=3, episode_count=1)
    assert not suite.results[0].succeeded
    assert suite.results[0].failure_category is MultiTargetFailureCategory.INSUFFICIENT_TARGETS
    assert not suite.suite_accepted


def test_geometric_full_is_normal_stop(tmp_path: Path) -> None:
    path = _write_config(
        tmp_path,
        _base_incremental_yaml(
            max_consecutive_target_failures=50,
            max_placement_attempts=3,
            min_targets_per_episode=1,
            # Force tiny field so almost every draw fails separation after one accept.
            keep_outs=[
                {"minimum_m": [-0.22, -0.22, 0.08], "maximum_m": [0.22, 0.22, 0.38]},
            ],
        ),
    )
    config = load_multi_target_suite_config(path)
    # keep-out covers entire field → geometric rejects only.
    planner = _FakePlanner([])
    runner = IncrementalPopulationRunner(
        planner_factory=lambda *_a: planner,
        validator=lambda plan, *_a: _validated(plan),
        contact_detector_factory=lambda _ep, to_id: OptimisticTipContactDetector(to_id),
        console_log=lambda _m: None,
        apply_reach_prefilter=False,
    )
    suite = runner.run_suite(config, root_seed=99, episode_count=1)
    assert suite.extras[0].stop_reason is PopulationStopReason.GEOMETRIC_FULL
    assert planner.calls == 0
    assert suite.results[0].failure_category is MultiTargetFailureCategory.INSUFFICIENT_TARGETS


def test_deterministic_candidate_stream_with_fake_oracle(tmp_path: Path) -> None:
    path = _write_config(
        tmp_path,
        _base_incremental_yaml(max_consecutive_target_failures=2, max_targets_per_episode=3),
    )
    config = load_multi_target_suite_config(path)

    def run_once() -> tuple[tuple[tuple[float, float, float], ...], str]:
        planner = _FakePlanner([_ok(), _ok(), _ok()])
        runner = IncrementalPopulationRunner(
            planner_factory=lambda *_a: planner,
            validator=lambda plan, *_a: _validated(plan),
            contact_detector_factory=lambda _ep, to_id: OptimisticTipContactDetector(to_id),
            console_log=lambda _m: None,
            apply_reach_prefilter=False,
        )
        suite = runner.run_suite(config, root_seed=4242, episode_count=1)
        centers = tuple(target.center_m for target in suite.results[0].episode.field.targets)
        return centers, suite.artifact_base_name

    centers_a, name_a = run_once()
    centers_b, name_b = run_once()
    assert centers_a == centers_b
    assert name_a == name_b
    assert len(centers_a) == 3


def test_empty_accepted_field_roundtrips_serialize(tmp_path: Path) -> None:
    path = _write_config(
        tmp_path,
        _base_incremental_yaml(max_consecutive_target_failures=2, min_targets_per_episode=1),
    )
    config = load_multi_target_suite_config(path)
    planner = _FakePlanner([_fail(), _fail()])
    runner = IncrementalPopulationRunner(
        planner_factory=lambda *_a: planner,
        validator=lambda plan, *_a: _validated(plan),
        contact_detector_factory=lambda _ep, to_id: OptimisticTipContactDetector(to_id),
        console_log=lambda _m: None,
        apply_reach_prefilter=False,
    )
    suite = runner.run_suite(config, root_seed=5, episode_count=1)
    payload = serialize_episode(suite.results[0].episode)
    restored = deserialize_episode(payload)
    assert restored.field.targets == ()
    assert restored.max_consecutive_unplanned_targets == 0


def test_populate_candidate_line_mentions_remaining_failures() -> None:
    line = format_populate_candidate(
        episode_index=1,
        episode_count=3,
        accepted=12,
        candidate_index=18,
        center_m=(0.1, 0.15, 0.331),
        plan_ok=False,
        plan_duration_s=22.1,
        failure_reason="plan_failed",
        streak=3,
        threshold=5,
    )
    assert line.startswith("phase7_5_populate: ep 2/3 |")
    assert "accepted 12" in line
    assert "streak 3/5 — 2 more failures end episode" in line


def test_suite_table_includes_artifact_name() -> None:
    # Smoke the formatter with empty-ish synthetic data via a minimal run.
    path = ROOT / "config/phase7_5_variable_targets_dz_0_30.yml"
    config = load_multi_target_suite_config(path)
    planner = _FakePlanner([_ok(), _fail(), _fail(), _fail(), _fail(), _fail()])
    runner = IncrementalPopulationRunner(
        planner_factory=lambda *_a: planner,
        validator=lambda plan, *_a: _validated(plan),
        contact_detector_factory=lambda _ep, to_id: OptimisticTipContactDetector(to_id),
        console_log=lambda _m: None,
        apply_reach_prefilter=False,
    )
    # One episode only for speed.
    from dataclasses import replace

    config = replace(config, episode_count=1, max_consecutive_target_failures=5)
    suite = runner.run_suite(config, root_seed=4242, episode_count=1)
    table = format_suite_table(
        results=suite.results,
        extras=suite.extras,
        artifact_base_name=suite.artifact_base_name,
    )
    assert "phase7_5_suite:" in table
    assert suite.artifact_base_name in table
    assert "total" in table
