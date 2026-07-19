"""Phase 6 deterministic sampling, taxonomy, replay, and report tests."""

from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path

import numpy as np

from mycobot_curobo.benchmark import (
    BenchmarkResult,
    BenchmarkRunner,
    FailureCategory,
    aggregate_results,
    deserialize_request,
    load_benchmark_config,
    load_case_fixture,
    planning_failure_category,
    sample_benchmark_cases,
    serialize_request,
    validation_failure_category,
    write_benchmark_reports,
)
from mycobot_curobo.planner import PlanningFailure, PlanningOutcome
from mycobot_curobo.validation import ValidationMetrics, ValidationViolation

ROOT = Path(__file__).resolve().parents[2]


class FailingPlanner:
    def plan(self, request):
        return PlanningOutcome(
            plan=None,
            failure=PlanningFailure(
                category="planning_infeasible",
                reason="no valid ik solution",
                planner_status="NO_IK_SOLUTION",
            ),
        )


def _violation(metric: str) -> ValidationViolation:
    return ValidationViolation(
        metric=metric,
        waypoint_index=0,
        measured=1.0,
        threshold=0.5,
        reason=metric,
    )


def test_sampling_is_reproducible_from_root_seed() -> None:
    config = load_benchmark_config(ROOT / "config/benchmark_workspace.yml")
    first = sample_benchmark_cases(config, root_seed=20260719, stage="smoke")
    second = sample_benchmark_cases(config, root_seed=20260719, stage="smoke")
    changed = sample_benchmark_cases(config, root_seed=20260720, stage="smoke")

    assert first == second
    assert first != changed
    assert len(first) == 20
    assert {case.planner_seed for case in first} == set(config.planner_seed_sweep)
    assert all(np.isclose(np.linalg.norm(case.surface_normal_base), 1.0) for case in first)


def test_request_serialization_round_trips_exact_domain_values() -> None:
    config = load_benchmark_config(ROOT / "config/benchmark_workspace.yml")
    request = sample_benchmark_cases(config, root_seed=7, stage="smoke", count=1)[0].to_request()

    rebuilt = deserialize_request(serialize_request(request))

    assert serialize_request(rebuilt) == serialize_request(request)


def test_failure_taxonomy_maps_planner_and_validation_causes() -> None:
    def planning(category: str, reason: str, status: str = "") -> FailureCategory:
        return planning_failure_category(PlanningFailure(category, reason, status))

    assert planning("planning_infeasible", "NO_IK solution") is FailureCategory.NO_REACHABLE_IK
    assert planning("planning_infeasible", "collision") is FailureCategory.COLLISION_INFEASIBILITY
    assert (
        planning("planning_infeasible", "optimizer failed")
        is FailureCategory.TRAJECTORY_OPTIMIZATION_FAILURE
    )
    assert planning("backend_error", "CUDA error") is FailureCategory.NUMERICAL_FAILURE
    assert (
        planning("invalid_planner_result", "bad model")
        is FailureCategory.CONFIGURATION_MODEL_FAILURE
    )
    assert (
        validation_failure_category((_violation("lateral_error"),))
        is FailureCategory.TERMINAL_LINE_VALIDATION_FAILURE
    )
    assert (
        validation_failure_category((_violation("approach_axis_error"),))
        is FailureCategory.ORIENTATION_VALIDATION_FAILURE
    )
    assert (
        validation_failure_category((_violation("self_collision_clearance"),))
        is FailureCategory.COLLISION_INFEASIBILITY
    )


def test_runner_preserves_failure_and_serialized_replay() -> None:
    config = load_benchmark_config(ROOT / "config/benchmark_workspace.yml")
    case = sample_benchmark_cases(config, root_seed=11, stage="smoke", count=1)[0]
    runner = BenchmarkRunner(
        planner_factory=lambda seed: FailingPlanner(),
        validator=lambda plan, request: None,
        repeat_count=2,
    )

    result = runner.run((case,))[0]

    assert not result.succeeded
    assert result.failure_category is FailureCategory.NO_REACHABLE_IK
    assert result.raw_planner_status == "NO_IK_SOLUTION"
    assert deserialize_request(result.replay_request).request_id == case.case_id
    assert not result.repeat_disagreed


def test_aggregation_and_reports_count_all_failures(tmp_path: Path) -> None:
    config = load_benchmark_config(ROOT / "config/benchmark_workspace.yml")
    cases = sample_benchmark_cases(config, root_seed=13, stage="smoke", count=2)
    base = BenchmarkRunner(
        planner_factory=lambda seed: FailingPlanner(),
        validator=lambda plan, request: None,
        repeat_count=2,
    ).run(cases)
    first = replace(base[0], repeat_disagreed=True)
    results = (first, base[1])

    summary = aggregate_results(results, root_seed=13, stage="smoke")
    json_path, markdown_path = write_benchmark_reports(summary, results, tmp_path)
    payload = json.loads(json_path.read_text(encoding="utf-8"))

    assert summary.total_cases == 2
    assert summary.successes == 0
    assert summary.failure_category_counts == {"no_reachable_ik": 2}
    assert summary.repeat_disagreement_rate == 0.5
    assert len(payload["failed_replay_requests"]) == 2
    assert "Planning success: 0.000%" in markdown_path.read_text(encoding="utf-8")


def test_frozen_fixture_integrity_and_parameters_only() -> None:
    config = load_benchmark_config(ROOT / "config/benchmark_workspace.yml")
    smoke = load_case_fixture(ROOT / "tests/data/benchmark_smoke_cases.json")
    regression = load_case_fixture(ROOT / "tests/data/benchmark_regression_cases.json")

    assert smoke == sample_benchmark_cases(config, root_seed=6006, stage="smoke")
    assert regression == sample_benchmark_cases(config, root_seed=60100, stage="regression")
    assert len(smoke) >= 20
    assert len(regression) >= 100
    fixture_text = (ROOT / "tests/data/benchmark_regression_cases.json").read_text(
        encoding="utf-8"
    )
    for forbidden in ("trajectory", "validation_metrics", "planner_timings"):
        assert forbidden not in fixture_text


def test_distribution_metrics_accept_valid_values() -> None:
    metrics = ValidationMetrics(
        max_lateral_error_m=0.001,
        max_approach_axis_error_rad=0.01,
        max_roll_error_rad=0.02,
        terminal_position_error_m=0.001,
        terminal_orientation_error_rad=0.01,
        max_progress_regression_m=0.0,
        minimum_joint_limit_margin_rad=0.1,
        minimum_self_collision_clearance_m=0.02,
        minimum_world_collision_clearance_m=0.03,
    )
    config = load_benchmark_config(ROOT / "config/benchmark_workspace.yml")
    case = sample_benchmark_cases(config, root_seed=5, stage="smoke", count=1)[0]
    result = BenchmarkResult(
        case=case,
        planning_succeeded=True,
        validation_passed=True,
        failure_category=None,
        failure_reason=None,
        raw_planner_status="success",
        selected_roll_rad=0.0,
        planner_timings_s={"backend_planning_time": 0.2},
        validation_metrics=metrics,
        validation_violations=(),
        execution_attempted=True,
        execution_completed=False,
        execution_failure_category="safety_rejected",
        repeat_disagreed=False,
        replay_request=serialize_request(case.to_request()),
    )

    summary = aggregate_results((result,), root_seed=5, stage="smoke")

    assert summary.success_rate == 1.0
    assert summary.execution_failure_count == 1
    assert summary.failure_category_counts == {}
