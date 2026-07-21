"""Unit tests for Phase 7.2 multi-target field, order, retain, and runner contracts."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from mycobot_curobo.errors import ConfigurationError
from mycobot_curobo.multi_target import (
    ContactEvent,
    ContactKind,
    GRID_Z_VARIABILITY_FRACTION,
    MultiTargetEpisodeRunner,
    MultiTargetFailureCategory,
    OptimisticTipContactDetector,
    OrderPolicy,
    PlacementPolicy,
    build_grid_centers,
    build_target_field,
    deserialize_episode,
    load_multi_target_suite_config,
    override_suite_target_count,
    sample_multi_target_episodes,
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

    def plan(self, request: Any) -> PlanningOutcome:
        self.calls += 1
        if not self._outcomes:
            raise AssertionError("unexpected plan call")
        return self._outcomes.pop(0)


def test_default_config_loads_and_shuffle_is_deterministic() -> None:
    config = load_multi_target_suite_config(ROOT / "config/phase7_2_multi_target.yml")
    assert config.placement is PlacementPolicy.MANUAL
    assert config.order is OrderPolicy.SHUFFLE
    assert config.retain_targets_after_contact is False
    assert config.max_planning_failure_per_target == 5
    assert config.target_count == 2
    assert config.max_target_failures == 3  # fixed default
    assert config.max_failed_episodes == 0
    assert config.tip_allow_link_names == ("joint6_flange",)
    field_a = build_target_field(config, order_seed=123)
    field_b = build_target_field(config, order_seed=123)
    field_c = build_target_field(config, order_seed=456)
    assert field_a.contact_order_ids == field_b.contact_order_ids
    assert field_a.contact_order_ids != field_c.contact_order_ids
    assert set(field_a.contact_order_ids) == {"1", "2"}
    centers = build_grid_centers(
        4,
        config.field_minimum_m,
        config.field_maximum_m,
        arm_z_motion_range_m=config.arm_z_motion_range_m,
    )
    assert len(centers) == 4


def test_grid_z_varies_across_half_arm_range() -> None:
    config = load_multi_target_suite_config(ROOT / "config/phase7_2_multi_target_grid.yml")
    assert config.arm_z_motion_range_m == pytest.approx(0.28)
    assert GRID_Z_VARIABILITY_FRACTION == pytest.approx(0.5)
    centers = build_grid_centers(
        config.target_count,
        config.field_minimum_m,
        config.field_maximum_m,
        arm_z_motion_range_m=config.arm_z_motion_range_m,
    )
    zs = [center[2] for center in centers]
    mid_z = 0.5 * (config.field_minimum_m[2] + config.field_maximum_m[2])
    half_band = 0.5 * GRID_Z_VARIABILITY_FRACTION * config.arm_z_motion_range_m
    assert min(zs) == pytest.approx(mid_z - half_band + half_band / config.target_count)
    assert max(zs) == pytest.approx(mid_z + half_band - half_band / config.target_count)
    assert max(zs) - min(zs) == pytest.approx(
        GRID_Z_VARIABILITY_FRACTION
        * config.arm_z_motion_range_m
        * (1.0 - 1.0 / config.target_count)
    )
    assert len(set(round(z, 9) for z in zs)) == config.target_count


def test_manual_listed_retain_config() -> None:
    config = load_multi_target_suite_config(ROOT / "config/phase7_2_multi_target_manual.yml")
    assert config.placement is PlacementPolicy.MANUAL
    assert config.order is OrderPolicy.LISTED
    assert config.retain_targets_after_contact is True
    assert config.max_target_failures == 1
    assert config.max_failed_episodes == 0
    field = build_target_field(config, order_seed=0)
    assert field.contact_order_ids == ("1", "2")
    assert len(field.active_geometries(removed_ids=(), contacted_ids=("1",))) == 2


def test_grid_listed_field_builds_four_targets() -> None:
    config = load_multi_target_suite_config(ROOT / "config/phase7_2_multi_target_grid.yml")
    assert config.placement is PlacementPolicy.GRID
    assert config.order is OrderPolicy.LISTED
    assert config.max_target_failures == 3  # fixed default
    field = build_target_field(config, order_seed=0)
    assert field.contact_order_ids == ("1", "2", "3", "4")
    assert len(field.targets) == 4


def test_episodes_replay_exactly() -> None:
    config = load_multi_target_suite_config(ROOT / "config/phase7_2_multi_target.yml")
    episodes = sample_multi_target_episodes(config, root_seed=123)
    assert len(episodes) == 1
    assert episodes == sample_multi_target_episodes(config, root_seed=123)
    rebuilt = deserialize_episode(serialize_episode(episodes[0]))
    assert rebuilt.episode_index == episodes[0].episode_index
    assert rebuilt.field.contact_order_ids == episodes[0].field.contact_order_ids
    assert rebuilt.field.targets[0].target_id == episodes[0].field.targets[0].target_id
    target = episodes[0].field.targets[0]
    surface = target.to_surface_target()
    assert surface.target_id == target.target_id
    assert surface.position_base_m.shape == (3,)


def test_runner_retries_same_target_until_planning_budget_then_target_fails() -> None:
    config = load_multi_target_suite_config(ROOT / "config/phase7_2_multi_target_manual.yml")
    episode = sample_multi_target_episodes(config, root_seed=7)[0]
    episode = replace(
        episode,
        max_planning_failure_per_target=2,
        max_target_failures=0,
    )
    fail = PlanningOutcome(
        plan=None,
        failure=PlanningFailure("planning_infeasible", "no path", "failed"),
    )
    outcomes = [fail, fail, fail]
    planner = _FakePlanner(outcomes)

    def planner_factory(seed: int, scene_model: dict, links: tuple[str, ...]) -> _FakePlanner:
        del seed, scene_model, links
        return planner

    def validator(plan: NominalPlan, request: Any, clearance: tuple) -> ValidatedPlan:
        del clearance
        return _validated(plan)

    runner = MultiTargetEpisodeRunner(
        planner_factory=planner_factory,
        validator=validator,
        contact_detector_factory=lambda ep, to_id: OptimisticTipContactDetector(to_id),
        console_log=lambda _line: None,
    )
    result = runner.run((episode,))[0]
    assert result.succeeded is False
    assert result.failure_category is MultiTargetFailureCategory.MAX_TARGET_FAILURES_EXCEEDED
    assert result.planning_failure_count == 3
    assert result.target_failure_count == 1
    assert result.failed_target_ids == (episode.field.contact_order_ids[0],)
    assert all(leg.to_id == episode.field.contact_order_ids[0] for leg in result.legs)
    assert all(leg.from_id == "start" for leg in result.legs)


def test_leg_keeps_tip_collision_and_strips_only_contact_cube() -> None:
    """Other targets stay cuboid obstacles; tip links are not globally disabled."""

    episodes = sample_multi_target_episodes(
        load_multi_target_suite_config(ROOT / "config/phase7_2_multi_target_manual.yml"),
        root_seed=7,
    )
    episode = episodes[0]
    first_id = episode.field.contact_order_ids[0]
    contact_name = episode.field.target_by_id(first_id).cube_geometry.name
    other_names = {
        target.cube_geometry.name
        for target in episode.field.targets
        if target.target_id != first_id
    }
    captured: list[tuple[Any, dict[str, Any]]] = []

    def planner_factory(seed: int, scene_model: dict, links: tuple[str, ...]) -> Any:
        del seed, links

        class _Planner:
            def plan(self, request: Any) -> PlanningOutcome:
                captured.append((request, scene_model))
                return PlanningOutcome(
                    plan=_plan(request.request_id, request.random_seed),
                    failure=None,
                )

        return _Planner()

    runner = MultiTargetEpisodeRunner(
        planner_factory=planner_factory,
        validator=lambda plan, request, clearance: _validated(plan),
        contact_detector_factory=lambda ep, to_id: OptimisticTipContactDetector(to_id),
        console_log=lambda _line: None,
    )
    runner.run((episode,))
    assert captured
    request, scene_model = captured[0]
    assert request.disable_collision_links == ()
    cuboids = set(scene_model["cuboid"])
    assert contact_name not in cuboids
    assert other_names <= cuboids


def test_suite_counts_failed_episodes_against_max_failed_episodes() -> None:
    from mycobot_curobo.multi_target import (
        MultiTargetEpisodeResult,
        aggregate_multi_target_results,
        suite_acceptance_passed,
    )

    config = load_multi_target_suite_config(ROOT / "config/phase7_2_multi_target_manual.yml")
    episode = sample_multi_target_episodes(config, root_seed=7)[0]
    failed = MultiTargetEpisodeResult(
        episode=episode,
        succeeded=False,
        failure_category=MultiTargetFailureCategory.MAX_TARGET_FAILURES_EXCEEDED,
        failure_reason="exhausted",
        planning_failure_count=5,
        target_failure_count=1,
        failed_target_ids=("1",),
        legs=(),
        contacted_ids=(),
        removed_ids=(),
        episode_duration_s=1.0,
    )
    passed = MultiTargetEpisodeResult(
        episode=replace(episode, episode_index=1),
        succeeded=True,
        failure_category=None,
        failure_reason=None,
        planning_failure_count=2,
        target_failure_count=0,
        failed_target_ids=(),
        legs=(),
        contacted_ids=("1", "2"),
        removed_ids=("1", "2"),
        episode_duration_s=1.0,
    )
    summary = aggregate_multi_target_results((failed, passed), root_seed=7)
    assert summary.failed_episodes == 1
    assert summary.total_planning_failures == 7
    assert summary.total_target_failures == 1
    assert suite_acceptance_passed(summary, max_failed_episodes=1) is True
    assert suite_acceptance_passed(summary, max_failed_episodes=0) is False


def test_runner_removes_targets_when_retain_false() -> None:
    config = load_multi_target_suite_config(ROOT / "config/phase7_2_multi_target.yml")
    config = replace(config, episode_count=1, target_count=2, max_target_failures=2)
    # Rebuild a two-target field via manual override of sampled episode.
    episodes = sample_multi_target_episodes(
        load_multi_target_suite_config(ROOT / "config/phase7_2_multi_target_manual.yml"),
        root_seed=7,
    )
    episode = replace(
        episodes[0],
        retain_targets_after_contact=False,
        field=replace(episodes[0].field, retain_targets_after_contact=False),
    )

    def planner_factory(seed: int, scene_model: dict, links: tuple[str, ...]) -> Any:
        del links
        cuboids = scene_model["cuboid"]
        plan = _plan(f"req-{seed}-{len(cuboids)}", seed)

        class _Planner:
            def plan(self, request: Any) -> PlanningOutcome:
                return PlanningOutcome(
                    plan=replace(plan, request_id=request.request_id),
                    failure=None,
                )

        return _Planner()

    scenes_seen: list[int] = []

    def validator(plan: NominalPlan, request: Any, clearance: tuple) -> ValidatedPlan:
        del clearance
        scenes_seen.append(1)
        return _validated(plan)

    runner = MultiTargetEpisodeRunner(
        planner_factory=planner_factory,
        validator=validator,
        contact_detector_factory=lambda ep, to_id: OptimisticTipContactDetector(to_id),
        console_log=lambda _line: None,
    )
    result = runner.run((episode,))[0]
    assert result.succeeded is True
    assert result.contacted_ids == episode.field.contact_order_ids
    assert result.removed_ids == episode.field.contact_order_ids
    assert len(result.legs) == 2


def test_episode_passes_without_tip_contact_on_planning_failed_targets() -> None:
    config = load_multi_target_suite_config(ROOT / "config/phase7_2_multi_target_manual.yml")
    episode = sample_multi_target_episodes(config, root_seed=7)[0]
    episode = replace(episode, max_planning_failure_per_target=1, max_target_failures=1)
    first_id = episode.field.contact_order_ids[0]
    second_id = episode.field.contact_order_ids[1]
    fail = PlanningOutcome(
        plan=None,
        failure=PlanningFailure("planning_infeasible", "no path", "failed"),
    )
    plan = _plan("ok", 1)
    outcomes = [fail, fail, PlanningOutcome(plan=plan, failure=None)]
    planner = _FakePlanner(outcomes)

    def planner_factory(seed: int, scene_model: dict, links: tuple[str, ...]) -> _FakePlanner:
        del seed, scene_model, links
        return planner

    def validator(plan: NominalPlan, request: Any, clearance: tuple) -> ValidatedPlan:
        del clearance
        return _validated(plan)

    runner = MultiTargetEpisodeRunner(
        planner_factory=planner_factory,
        validator=validator,
        contact_detector_factory=lambda ep, to_id: OptimisticTipContactDetector(to_id),
        console_log=lambda _line: None,
    )
    result = runner.run((episode,))[0]
    assert result.succeeded is True
    assert result.failed_target_ids == (first_id,)
    assert result.contacted_ids == (second_id,)
    assert result.target_failure_count == 1
    assert first_id not in result.contacted_ids


def test_tip_miss_after_successful_plan_aborts_episode() -> None:
    config = load_multi_target_suite_config(ROOT / "config/phase7_2_multi_target_manual.yml")
    episode = sample_multi_target_episodes(config, root_seed=7)[0]

    def planner_factory(seed: int, scene_model: dict, links: tuple[str, ...]) -> Any:
        del scene_model, links
        plan = _plan("tip-miss", seed)

        class _Planner:
            def plan(self, request: Any) -> PlanningOutcome:
                return PlanningOutcome(
                    plan=replace(plan, request_id=request.request_id), failure=None
                )

        return _Planner()

    def validator(plan: NominalPlan, request: Any, clearance: tuple) -> ValidatedPlan:
        del request, clearance
        return _validated(plan)

    class _NoTip:
        def classify(self) -> ContactEvent:
            return ContactEvent(ContactKind.NONE)

    runner = MultiTargetEpisodeRunner(
        planner_factory=planner_factory,
        validator=validator,
        contact_detector_factory=lambda ep, to_id: _NoTip(),
        console_log=lambda _line: None,
    )
    result = runner.run((episode,))[0]
    assert result.succeeded is False
    assert result.failure_category is MultiTargetFailureCategory.TIP_CONTACT_MISSED
    assert result.contacted_ids == ()
    assert result.legs[0].planning_succeeded is True


def test_body_contact_fails_closed() -> None:
    config = load_multi_target_suite_config(ROOT / "config/phase7_2_multi_target_manual.yml")
    episode = sample_multi_target_episodes(config, root_seed=7)[0]

    def planner_factory(seed: int, scene_model: dict, links: tuple[str, ...]) -> Any:
        del scene_model, links
        plan = _plan("body", seed)

        class _Planner:
            def plan(self, request: Any) -> PlanningOutcome:
                return PlanningOutcome(
                    plan=replace(plan, request_id=request.request_id), failure=None
                )

        return _Planner()

    def validator(plan: NominalPlan, request: Any, clearance: tuple) -> ValidatedPlan:
        del request, clearance
        return _validated(plan)

    class _Body:
        def classify(self) -> ContactEvent:
            return ContactEvent(
                ContactKind.PROHIBITED_BODY_CONTACT,
                target_id="1",
                link_name="joint3",
            )

    runner = MultiTargetEpisodeRunner(
        planner_factory=planner_factory,
        validator=validator,
        contact_detector_factory=lambda ep, to_id: _Body(),
        console_log=lambda _line: None,
    )
    result = runner.run((episode,))[0]
    assert result.succeeded is False
    assert result.failure_category is MultiTargetFailureCategory.BODY_CONTACT


def test_grid_rejects_targets_list_and_manual_count_mismatch() -> None:
    config = load_multi_target_suite_config(ROOT / "config/phase7_2_multi_target_grid.yml")
    assert config.manual_targets == ()
    with pytest.raises(ConfigurationError, match="targets list"):
        # Force the loader path by writing a temp invalid manual config.
        import tempfile

        import yaml

        payload = yaml.safe_load(
            (ROOT / "config/phase7_2_multi_target_manual.yml").read_text(encoding="utf-8")
        )
        payload["target_count"] = 3
        with tempfile.NamedTemporaryFile("w", suffix=".yml", delete=False) as handle:
            yaml.safe_dump(payload, handle)
            temp_path = Path(handle.name)
        try:
            load_multi_target_suite_config(temp_path)
        finally:
            temp_path.unlink(missing_ok=True)


def test_override_target_count_truncates_manual_list() -> None:
    config = load_multi_target_suite_config(ROOT / "config/phase7_2_multi_target.yml")
    overridden = override_suite_target_count(config, 1)
    assert overridden.target_count == 1
    assert overridden.max_target_failures == 3  # fixed default; not rescaled
    assert overridden.placement is PlacementPolicy.MANUAL
    assert len(overridden.manual_targets) == 1
    assert overridden.manual_targets[0].target_id == "1"
    field = build_target_field(overridden, order_seed=0)
    assert field.contact_order_ids == ("1",)


def test_override_target_count_switches_to_grid_when_manual_too_short() -> None:
    config = load_multi_target_suite_config(ROOT / "config/phase7_2_multi_target.yml")
    overridden = override_suite_target_count(config, 5)
    assert overridden.target_count == 5
    assert overridden.max_target_failures == 3  # fixed default; not rescaled
    assert overridden.placement is PlacementPolicy.GRID
    assert overridden.manual_targets == ()
    field = build_target_field(overridden, order_seed=0)
    assert overridden.order is OrderPolicy.SHUFFLE
    assert len(field.targets) == 5
    assert set(field.contact_order_ids) == {"1", "2", "3", "4", "5"}
    # Shuffle is seeded: same order_seed must replay the same permutation.
    assert (
        field.contact_order_ids == build_target_field(overridden, order_seed=0).contact_order_ids
    )
