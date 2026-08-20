"""Phase 8 residual mapping and comparison report tests."""

from __future__ import annotations

from pathlib import Path

from mycobot_curobo.benchmark import BenchmarkCase, BenchmarkResult, aggregate_results
from mycobot_curobo.residual import CartesianResidual
from mycobot_curobo.residual_compare import (
    build_residual_comparison_report,
    write_residual_comparison_report,
)
from mycobot_curobo.residual_mapping import (
    FiniteDifferenceResidualMapper,
    FixedJointDeltaMapper,
)
from mycobot_curobo.robot_model import JOINT_NAMES, load_robot_model_spec


def test_fixed_joint_delta_mapper_returns_zero_for_zero_residual() -> None:
    mapper = FixedJointDeltaMapper((0.01, 0.0, 0.0, 0.0, 0.0, 0.0))
    delta = mapper.map_to_joint_delta(CartesianResidual.zero(), [0.0] * 6)
    assert delta == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def test_finite_difference_mapper_clamps_joint_delta() -> None:
    spec = load_robot_model_spec()
    mapper = FiniteDifferenceResidualMapper(
        robot_spec=spec,
        max_joint_delta_rad=0.01,
    )
    # Large Cartesian residual should still produce a clamped Δq.
    delta = mapper.map_to_joint_delta(
        CartesianResidual.create([0.05, 0.0, 0.0], [0.0, 0.0, 0.0]),
        tuple(float(value) for value in spec.default_joint_position_rad),
    )
    assert len(delta) == len(JOINT_NAMES)
    assert max(abs(value) for value in delta) <= 0.01 + 1.0e-12


def _fake_result(*, completed: bool) -> BenchmarkResult:
    case = BenchmarkCase(
        case_id="c0",
        root_seed=6006,
        sample_index=0,
        region_label="unit",
        normal_bin_label="unit",
        start_joint_label="unit",
        position_base_m=(0.15, 0.0, 0.2),
        surface_normal_base=(0.0, 0.0, -1.0),
        tangent_hint_base=(1.0, 0.0, 0.0),
        start_joint_position_rad=(0.0,) * 6,
        pre_approach_distance_m=0.05,
        planner_seed=6006,
        fixed_roll_rad=0.0,
        roll_candidates_rad=(),
        scene_revision="empty-v1",
        planner_profile="development_fast",
    )
    return BenchmarkResult(
        case=case,
        planning_succeeded=True,
        validation_passed=True,
        failure_category=None,
        failure_reason=None,
        raw_planner_status="success",
        selected_roll_rad=0.0,
        planner_timings_s={},
        validation_metrics=None,
        validation_violations=(),
        execution_attempted=True,
        execution_completed=completed,
        execution_failure_category=None if completed else "residual_validation_failed",
        repeat_disagreed=False,
        replay_request={},
    )


def test_residual_comparison_report_is_labeled_sim_only(tmp_path: Path) -> None:
    off = (_fake_result(completed=True),)
    on = (_fake_result(completed=True),)
    report = build_residual_comparison_report(
        off_results=off,
        on_results=on,
        root_seed=6006,
        stage="smoke",
    )
    assert report.sim_only is True
    assert report.residual_mode_off == "off"
    assert report.residual_mode_on == "on"
    json_path, md_path = write_residual_comparison_report(report, tmp_path)
    text = json_path.read_text(encoding="utf-8")
    assert '"sim_only": true' in text
    assert "simulation_only_residual_comparison" in text
    assert "simulation only" in md_path.read_text(encoding="utf-8").lower()
    summary = aggregate_results(off, root_seed=6006, stage="smoke")
    assert summary.execution_failure_count == 0
