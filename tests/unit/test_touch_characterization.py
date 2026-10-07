"""Unit tests for the opt-in touch-failure characterization mode."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from mycobot_curobo.cube_scene import CubeGeometry
from mycobot_curobo.errors import ConfigurationError
from mycobot_curobo.multi_target import (
    MultiTargetEpisode,
    MultiTargetEpisodeRunner,
    NumberedTarget,
    OptimisticTipContactDetector,
    OrderPolicy,
    PlacementPolicy,
    TargetField,
    load_multi_target_suite_config,
)
from mycobot_curobo.planner import (
    NominalPlan,
    PlanningFailure,
    PlanningOutcome,
)
from mycobot_curobo.robot_model import JOINT_NAMES
from mycobot_curobo.touch_characterization import (
    CHARACTERIZATION_TAG,
    LinkSphere,
    TipErrorComponents,
    TouchOutcomeBin,
    attribute_target_outcome,
    characterize_episode_results,
    compare_placement_policies,
    decompose_tip_error_m,
    false_infeasible_rate,
    flange_approach_blocked,
    format_characterization_summary,
    geometric_field_precheck,
    grazing_obstacle_for_pose,
    subsample_link_spheres,
    summarize_attributions,
    write_characterization_figures,
)
from mycobot_curobo.trajectory import JointTrajectory
from mycobot_curobo.validation import ValidatedPlan, ValidationMetrics, ValidationReport

ROOT = Path(__file__).resolve().parents[2]


def _target(target_id: str, center: tuple[float, float, float]) -> NumberedTarget:
    return NumberedTarget(
        target_id=target_id,
        center_m=center,
        edge_m=0.014,
        outward_normal_base=(0.0, 0.0, 1.0),
        pre_approach_distance_m=0.05,
    )


def _episode(
    targets: tuple[NumberedTarget, ...], index: int = 0, *, retain: bool = False
) -> MultiTargetEpisode:
    field = TargetField(
        targets=targets,
        placement=PlacementPolicy.MANUAL,
        order=OrderPolicy.LISTED,
        retain_targets_after_contact=retain,
        contact_order_ids=tuple(item.target_id for item in targets),
    )
    return MultiTargetEpisode(
        episode_index=index,
        root_seed=1,
        episode_seed=11,
        order_seed=12,
        field=field,
        start_position_rad=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        planner_profile="benchmark_reproducible",
        tip_allow_link_names=("joint6_flange",),
        max_planning_failure_per_target=1,
        max_target_failures=1,
        max_reconsider_passes=1,
        max_failed_episodes=1,
        max_consecutive_unplanned_targets=0,
        scene_revision_prefix="touch-test",
        retain_targets_after_contact=retain,
    )


def _trajectory() -> JointTrajectory:
    position = np.zeros((2, 6), dtype=float)
    return JointTrajectory(JOINT_NAMES, position, None, None, None, 0.05)


def _plan(request_id: str) -> NominalPlan:
    trajectory = _trajectory()
    return NominalPlan(
        request_id=request_id,
        selected_goal_index=0,
        selected_roll_rad=0.0,
        approach_trajectory=trajectory,
        terminal_trajectory=trajectory,
        combined_trajectory=trajectory,
        planner_status="ok",
        planner_timings_s={},
        curobo_version="test",
        scene_revision="rev",
        planner_profile="benchmark_reproducible",
        random_seed=1,
    )


def _validated(plan: NominalPlan, request_id: str, *, valid: bool) -> ValidatedPlan:
    metrics = ValidationMetrics(None, None, None, None, None, None, None, None, None)
    report = ValidationReport(
        request_id=request_id,
        profile_name="test",
        valid=valid,
        violations=(),
        metrics=metrics,
    )
    return ValidatedPlan(
        nominal_plan=plan,
        report=report,
        validation_status="valid" if valid else "invalid",
        executable=valid,
    )


def test_close_neighbors_block_flange_and_far_neighbors_do_not() -> None:
    close_a = _target("a", (0.20, 0.00, 0.15))
    close_b = _target("b", (0.22, 0.00, 0.15))
    blocked, separation = flange_approach_blocked(close_a, (close_b,), flange_diameter_m=0.031)
    assert blocked is True
    assert separation is not None and separation < 0.031

    far_b = _target("b", (0.20, 0.20, 0.15))
    clear, far_separation = flange_approach_blocked(close_a, (far_b,), flange_diameter_m=0.031)
    assert clear is False
    assert far_separation is not None and far_separation > 0.1


def test_tip_error_splits_lateral_and_approach() -> None:
    error = decompose_tip_error_m(
        (0.01, 0.0, 0.02),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, -1.0),
    )
    assert error.lateral_m == pytest.approx(0.01)
    assert error.along_approach_m == pytest.approx(-0.02)
    assert error.total_m == pytest.approx((0.01**2 + 0.02**2) ** 0.5)


def test_attribution_precedence_uses_phase6_ik_token() -> None:
    geometric, _phase6 = attribute_target_outcome(
        geometric_blocked=True,
        planning_succeeded=False,
        validation_passed=False,
        planner_status="ik_fail",
        failure_reason="ik_fail",
        coarse_clearance_m=0.01,
        fine_clearance_m=-0.01,
    )
    assert geometric is TouchOutcomeBin.GEOMETRIC_INFEASIBILITY

    ik_bin, phase6 = attribute_target_outcome(
        geometric_blocked=False,
        planning_succeeded=False,
        validation_passed=False,
        planner_status="ik_fail",
        failure_reason="no reachable ik",
        coarse_clearance_m=None,
        fine_clearance_m=None,
    )
    assert ik_bin is TouchOutcomeBin.IK_FAILURE
    assert phase6 == "no_reachable_ik"

    trajopt, phase6_collision = attribute_target_outcome(
        geometric_blocked=False,
        planning_succeeded=False,
        validation_passed=False,
        planner_status="Start state in collision",
        failure_reason="world_coll",
        coarse_clearance_m=None,
        fine_clearance_m=None,
    )
    assert trajopt is TouchOutcomeBin.TRAJOPT_PLAN_GRASP_FAILURE
    assert phase6_collision == "collision_infeasibility"

    rejected, _ignored = attribute_target_outcome(
        geometric_blocked=False,
        planning_succeeded=True,
        validation_passed=False,
        planner_status="ok",
        failure_reason="lateral_error",
        coarse_clearance_m=None,
        fine_clearance_m=None,
    )
    assert rejected is TouchOutcomeBin.VALIDATION_REJECTION

    blind, _blind_phase = attribute_target_outcome(
        geometric_blocked=False,
        planning_succeeded=True,
        validation_passed=True,
        planner_status="ok",
        failure_reason=None,
        coarse_clearance_m=0.01,
        fine_clearance_m=-0.002,
    )
    assert blind is TouchOutcomeBin.WORLD_MODEL_BLINDNESS

    success, _success_phase = attribute_target_outcome(
        geometric_blocked=False,
        planning_succeeded=True,
        validation_passed=True,
        planner_status="ok",
        failure_reason=None,
        coarse_clearance_m=0.01,
        fine_clearance_m=0.004,
    )
    assert success is TouchOutcomeBin.SUCCESS


def test_runner_legs_are_attributed_without_a_second_planner() -> None:
    far = (
        _target("1", (0.20, 0.00, 0.15)),
        _target("2", (0.20, 0.18, 0.15)),
    )
    episode = _episode(far)

    class _IkPlanner:
        def plan(self, request: object) -> PlanningOutcome:
            del request
            return PlanningOutcome(
                plan=None,
                failure=PlanningFailure(
                    category="ik_fail",
                    reason="ik_fail",
                    planner_status="ik_fail",
                ),
            )

    runner = MultiTargetEpisodeRunner(
        planner_factory=lambda _seed, _scene, _links: _IkPlanner(),
        validator=lambda plan, request, _geometries, _cube: _validated(
            plan, request.request_id, valid=True
        ),
        contact_detector_factory=lambda _episode, target_id: OptimisticTipContactDetector(
            target_id
        ),
    )
    results = runner.run((episode,), max_field_regenerations=0)
    rows = characterize_episode_results(results, flange_diameter_m=0.031)
    assert rows
    assert all(row.outcome_bin is TouchOutcomeBin.IK_FAILURE for row in rows)
    summary = summarize_attributions(rows)
    line = format_characterization_summary(summary)
    assert line.startswith(CHARACTERIZATION_TAG)
    assert "ik_failure=" in line

    close = (
        _target("1", (0.20, 0.00, 0.15)),
        _target("2", (0.215, 0.00, 0.15)),
    )

    class _SuccessPlanner:
        def plan(self, request: object) -> PlanningOutcome:
            request_id = str(getattr(request, "request_id"))
            return PlanningOutcome(plan=_plan(request_id), failure=None)

    success_runner = MultiTargetEpisodeRunner(
        planner_factory=lambda _seed, _scene, _links: _SuccessPlanner(),
        validator=lambda plan, request, _geometries, _cube: _validated(
            plan, request.request_id, valid=True
        ),
        contact_detector_factory=lambda _episode, target_id: OptimisticTipContactDetector(
            target_id
        ),
    )
    success_results = success_runner.run(
        (_episode(close, retain=True),), max_field_regenerations=0
    )
    success_rows = characterize_episode_results(
        success_results,
        flange_diameter_m=0.031,
        clearance_fn=lambda _q, _cubes: (0.01, -0.001),
    )
    assert success_rows
    assert all(row.outcome_bin is TouchOutcomeBin.GEOMETRIC_INFEASIBILITY for row in success_rows)


def test_sphere_density_false_infeasible_rises_when_the_grazing_ball_is_included() -> None:
    small = [
        LinkSphere("joint6_flange", (0.0, 0.0, 0.4 + 0.01 * index), 0.001) for index in range(19)
    ]
    grazing = LinkSphere("joint6_flange", (0.0, 0.0, 0.10), 0.02)
    spheres = tuple([*small, grazing])
    frames = {"joint6_flange": np.eye(4)}
    obstacle = grazing_obstacle_for_pose(spheres, frames)
    assert obstacle is not None
    from mycobot_curobo.touch_characterization import KnownValidQuery

    query = KnownValidQuery(label="home", link_frames=frames, obstacle=obstacle)
    sparse = subsample_link_spheres(spheres, 0.05)
    assert len(sparse) == 1
    assert false_infeasible_rate((query,), sparse) == 0.0
    assert false_infeasible_rate((query,), spheres) == 1.0
    assert grazing_obstacle_for_pose((), frames) is None


def test_workspace_map_draws_are_reproducible_and_independent_of_smoke_configs() -> None:
    centers = np.array([[0.0, 0.0, 0.0]], dtype=float)
    radii = np.array([0.05], dtype=float)
    workspace = [(0.15, 0.00, 0.16), (0.08, 0.05, 0.16)]
    first, second = compare_placement_policies(
        seed=7,
        draw_count=40,
        field_minimum_m=(-0.24, -0.24, 0.12),
        field_maximum_m=(0.24, 0.24, 0.22),
        edge_m=0.014,
        max_target_radial_m=0.20,
        workspace_centers_m=workspace,
        start_sphere_centers_m=centers,
        start_sphere_radii_m=radii,
    )
    again, _again_map = compare_placement_policies(
        seed=7,
        draw_count=40,
        field_minimum_m=(-0.24, -0.24, 0.12),
        field_maximum_m=(0.24, 0.24, 0.22),
        edge_m=0.014,
        max_target_radial_m=0.20,
        workspace_centers_m=workspace,
        start_sphere_centers_m=centers,
        start_sphere_radii_m=radii,
    )
    assert first.policy == "forward_aabb"
    assert second.policy == "workspace_map"
    assert first.rim_failures == again.rim_failures
    assert first.rim_failures > 0
    assert second.rim_failures == 0
    assert first.start_collision_failures >= 0
    assert first.draws == 40


def test_characterization_config_loads_and_does_not_replace_integration() -> None:
    config = load_multi_target_suite_config(ROOT / "config" / "touch_characterization.yml")
    assert config.placement is PlacementPolicy.RANDOM
    assert config.planner_profile == "benchmark_reproducible"
    assert config.max_field_regenerations == 0
    integration = (ROOT / "config" / "phase7_2_multi_target_integration_2x5.yml").read_text(
        encoding="utf-8"
    )
    assert "touch_characterization" not in integration
    robot = (ROOT / "config" / "robots" / "mycobot_280_m5.yml").read_text(encoding="utf-8")
    assert "collision_sphere_overlay_role: dual" in robot
    for path in (ROOT / "scripts").rglob("*.sh"):
        if path.name == "run_touch_characterization.sh":
            continue
        assert "disable-terminal-joint-snap" not in path.read_text(encoding="utf-8")


def test_playback_snap_flag_defaults_off() -> None:
    from isaac_sim.play_multi_target_suite import parse_args

    args = parse_args(["--bundle", "bundle.json", "--output-report", "out.json"])
    assert args.disable_terminal_joint_snap is False
    enabled = parse_args(
        [
            "--bundle",
            "bundle.json",
            "--output-report",
            "out.json",
            "--disable-terminal-joint-snap",
        ]
    )
    assert enabled.disable_terminal_joint_snap is True


def test_figures_and_cpu_script_write_bins_and_sphere_curve(tmp_path: Path) -> None:
    summary = summarize_attributions(())
    summary["bin_counts"] = {
        item.value: (3 if item is TouchOutcomeBin.SUCCESS else 1) for item in TouchOutcomeBin
    }
    summary["attempt_count"] = 8
    from mycobot_curobo.touch_characterization import SphereCurvePoint

    curve = (
        SphereCurvePoint("overlay_fraction_0.05", "world", 4, 0.1, 4),
        SphereCurvePoint("overlay_fraction_1.00", "world", 40, 0.8, 4),
        SphereCurvePoint("distal_overlay", "world_distal", 12, 0.4, 4),
        SphereCurvePoint("scaffolding_self", "self", 32, 0.2, 4),
    )
    written = write_characterization_figures(
        summary,
        curve,
        tmp_path,
        open_loop_errors_m=(
            TipErrorComponents(0.001, 0.002, 0.003),
            TipErrorComponents(0.004, -0.001, 0.005),
        ),
    )
    assert any(path.suffix == ".png" for path in written)
    assert any(path.suffix == ".svg" for path in written)
    assert (tmp_path / "touch_characterization_open_loop_tip_error.png").is_file()

    import importlib.util

    script_path = ROOT / "scripts" / "run_touch_characterization.py"
    spec = importlib.util.spec_from_file_location("run_touch_characterization", script_path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    exit_code = module.main(
        [
            "--no-planner",
            "--fields",
            "1",
            "--targets",
            "2",
            "--draws",
            "30",
            "--joint-samples",
            "2",
            "--seed",
            "4242",
            "--output-dir",
            str(tmp_path / "run"),
        ]
    )
    assert exit_code == 0
    payload = json.loads((tmp_path / "run" / "touch_characterization_summary.json").read_text())
    assert payload["changes_default_smoke"] is False
    assert payload["planner"] == "not_requested"
    assert payload["geometric_precheck"]["targets"] == 2
    assert payload["sphere_curve"]
    assert len(payload["placement"]) == 2
    suite = load_multi_target_suite_config(ROOT / "config" / "touch_characterization.yml")
    assert geometric_field_precheck((), flange_diameter_m=suite.flange_diameter_assumption_m) == {
        "targets": 0,
        "geometric_infeasibility": 0,
    }


def test_compare_placement_rejects_empty_workspace_overlap() -> None:
    with pytest.raises(ConfigurationError, match="workspace-map"):
        compare_placement_policies(
            seed=1,
            draw_count=4,
            field_minimum_m=(0.2, 0.2, 0.2),
            field_maximum_m=(0.3, 0.3, 0.3),
            edge_m=0.014,
            max_target_radial_m=0.36,
            workspace_centers_m=((0.0, 0.0, 0.1),),
            start_sphere_centers_m=np.zeros((1, 3)),
            start_sphere_radii_m=np.array([0.01]),
        )


def test_decompose_tip_error_rejects_zero_approach() -> None:
    with pytest.raises(ConfigurationError, match="approach_direction"):
        decompose_tip_error_m((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    cube = CubeGeometry(center_m=(0.0, 0.0, 0.0), edge_m=0.01, name="unused")
    assert cube.edge_m == 0.01
