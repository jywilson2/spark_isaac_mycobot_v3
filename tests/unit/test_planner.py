"""Phase 3 unit tests for nominal planner orchestration and result parsing."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pytest

from mycobot_curobo.errors import ConfigurationError
from mycobot_curobo.frames import TaskFrameConfig
from mycobot_curobo.planner import (
    NamedJointState,
    NominalPlanner,
    PlanningRequest,
    load_planner_profile,
    signed_pre_approach_offset_m,
)
from mycobot_curobo.robot_model import JOINT_NAMES
from mycobot_curobo.targets import SurfaceTarget
from mycobot_curobo.trajectory import (
    JointTrajectory,
    concatenate_trajectories,
    extract_curobo_trajectory,
)

ROOT = Path(__file__).resolve().parents[2]


@dataclass
class FakeState:
    position: np.ndarray
    joint_names: tuple[str, ...] = JOINT_NAMES
    dt: np.ndarray = field(default_factory=lambda: np.asarray([0.02]))
    velocity: np.ndarray | None = None
    acceleration: np.ndarray | None = None
    jerk: np.ndarray | None = None

    def __post_init__(self) -> None:
        if self.velocity is None:
            self.velocity = np.zeros_like(self.position)
        if self.acceleration is None:
            self.acceleration = np.zeros_like(self.position)
        if self.jerk is None:
            self.jerk = np.zeros_like(self.position)


class FakeTypes:
    def goal_set(self, goal_set):
        return goal_set

    def joint_state(self, state):
        return state


class FakeBackend:
    def __init__(self, result) -> None:
        self.result = result
        self.kwargs = None
        self.reset_count = 0
        self.warmup_count = 0

    def warmup(self, *, enable_graph: bool, num_warmup_iterations: int) -> bool:
        assert enable_graph is False
        assert num_warmup_iterations == 1
        self.warmup_count += 1
        return True

    def reset_seed(self) -> None:
        self.reset_count += 1

    def plan_grasp(self, **kwargs):
        self.kwargs = kwargs
        return self.result


class FakeResult:
    def __init__(self, *, success=True, goalset_index=1) -> None:
        approach_positions = np.array(
            [
                np.zeros(6),
                np.full(6, 0.1),
                np.full(6, 0.2),
                np.full(6, np.nan),  # padded and excluded by endpoint
            ]
        )
        terminal_positions = np.array(
            [
                np.full(6, 0.2),
                np.full(6, 0.3),
                np.full(6, np.nan),
            ]
        )
        self.success = np.asarray([success])
        self.status = "ok" if success else "No grasp in goal set was reachable."
        self.goalset_index = np.asarray([goalset_index])
        self.approach_interpolated_trajectory = FakeState(approach_positions)
        self.approach_interpolated_last_tstep = np.asarray([3])
        self.grasp_interpolated_trajectory = FakeState(terminal_positions)
        self.grasp_interpolated_last_tstep = np.asarray([2])
        self.planning_time = 0.25


def _request(*, profile: str = "development_fast") -> PlanningRequest:
    target = SurfaceTarget.create(
        position_base_m=[0.1, 0.0, 0.2],
        surface_normal_base=[0.0, 0.0, 1.0],
        tangent_hint_base=[1.0, 0.0, 0.0],
        roll_candidates_rad=[0.0, 0.5],
        pre_approach_distance_m=0.04,
        target_id="planner-unit",
    )
    return PlanningRequest(
        current_joint_state=NamedJointState.create(JOINT_NAMES, np.zeros(6)),
        surface_target=target,
        scene_revision="empty-v1",
        planner_profile=profile,
        random_seed=123,
        request_id="request-unit",
    )


def _planner(result) -> tuple[NominalPlanner, list[FakeBackend]]:
    backends: list[FakeBackend] = []

    def backend_factory() -> FakeBackend:
        backend = FakeBackend(result)
        backends.append(backend)
        return backend

    profile = load_planner_profile(
        "development_fast",
        ROOT / "config" / "planner_profiles.yml",
    )
    planner = NominalPlanner(
        backend_factory,
        profile,
        task_frame_config=TaskFrameConfig(),
        types_adapter=FakeTypes(),
    )
    return planner, backends


@pytest.mark.parametrize(("sign", "expected"), [(-1, 0.04), (1, -0.04)])
def test_signed_pre_approach_offset(sign: int, expected: float) -> None:
    assert signed_pre_approach_offset_m(0.04, sign) == expected


def test_planning_high_effort_profile_loads_with_higher_budget() -> None:
    """Suite high-effort raises trajopt/attempts; IK seeds stay at benchmark."""

    from mycobot_curobo.validation import load_validation_profile

    path = ROOT / "config" / "planner_profiles.yml"
    high = load_planner_profile("planning_high_effort", path)
    bench = load_planner_profile("benchmark_reproducible", path)
    validation = load_validation_profile(
        "simulation_initial", ROOT / "config" / "validation_profiles.yml"
    )
    # num_ik_seeds=64 regresses packing-safe 2→1 grasp planning on host GPU.
    assert high.num_ik_seeds == bench.num_ik_seeds
    assert high.num_trajopt_seeds > bench.num_trajopt_seeds
    assert high.max_plan_grasp_attempts > bench.max_plan_grasp_attempts
    assert high.orientation_tolerance_rad >= bench.orientation_tolerance_rad
    # Planner success must not exceed Phase 4 terminal/roll rejection thresholds.
    assert high.orientation_tolerance_rad <= validation.max_terminal_orientation_error_rad
    assert high.orientation_tolerance_rad <= validation.max_roll_error_rad
    assert high.optimizer_collision_activation_distance_m == pytest.approx(0.01)
    assert bench.optimizer_collision_activation_distance_m == pytest.approx(0.01)


def test_success_maps_segments_roll_and_exact_plan_grasp_options() -> None:
    planner, backends = _planner(FakeResult())

    outcome = planner.plan(_request())

    assert len(backends) == 1
    backend = backends[0]
    assert outcome.succeeded
    assert outcome.plan is not None
    assert outcome.plan.selected_goal_index == 1
    assert outcome.plan.selected_roll_rad == 0.5
    assert outcome.plan.approach_trajectory.sample_count == 3
    assert outcome.plan.terminal_trajectory.sample_count == 2
    assert outcome.plan.combined_trajectory.sample_count == 4
    assert outcome.plan.validation_status == "not_evaluated"
    assert outcome.plan.executable is False
    assert backend.kwargs["grasp_approach_axis"] == "z"
    assert backend.kwargs["grasp_approach_offset"] == -0.04
    assert backend.kwargs["grasp_approach_in_tool_frame"] is True
    assert backend.kwargs["plan_approach_to_grasp"] is True
    assert backend.kwargs["plan_grasp_to_lift"] is False
    assert backend.kwargs["disable_collision_links"] == []
    assert backend.reset_count == 2
    assert backend.warmup_count == 1


def test_each_plan_uses_a_fresh_backend() -> None:
    planner, backends = _planner(FakeResult())

    first = planner.plan(_request())
    second = planner.plan(_request())

    assert first.succeeded and second.succeeded
    assert len(backends) == 2
    assert backends[0] is not backends[1]
    assert [backend.reset_count for backend in backends] == [2, 2]
    assert [backend.warmup_count for backend in backends] == [1, 1]


def test_expected_infeasibility_returns_structured_failure() -> None:
    planner, _ = _planner(FakeResult(success=False))

    outcome = planner.plan(_request())

    assert not outcome.succeeded
    assert outcome.failure is not None
    assert outcome.failure.category == "planning_infeasible"
    assert "No grasp" in outcome.failure.planner_status


def test_invalid_goal_index_fails_closed() -> None:
    planner, _ = _planner(FakeResult(goalset_index=9))

    outcome = planner.plan(_request())

    assert outcome.failure is not None
    assert outcome.failure.category == "invalid_planner_result"
    assert "goal index" in outcome.failure.reason


def test_valid_endpoint_discards_padded_nonfinite_samples() -> None:
    state = FakeState(
        np.array(
            [
                np.zeros(6),
                np.ones(6),
                np.full(6, np.nan),
            ]
        )
    )

    trajectory = extract_curobo_trajectory(
        state,
        np.asarray([2]),
        expected_joint_names=JOINT_NAMES,
        label="unit",
    )

    assert trajectory.sample_count == 2
    assert np.all(np.isfinite(trajectory.position_rad))


def test_nonfinite_valid_sample_fails_closed() -> None:
    state = FakeState(np.array([np.zeros(6), np.full(6, np.nan)]))
    with pytest.raises(ConfigurationError, match="non-finite"):
        extract_curobo_trajectory(
            state,
            np.asarray([2]),
            expected_joint_names=JOINT_NAMES,
            label="unit",
        )


def test_discontinuous_segment_boundary_is_rejected() -> None:
    first = JointTrajectory(
        joint_names=JOINT_NAMES,
        position_rad=np.array([np.zeros(6), np.ones(6)]),
        velocity_rad_s=None,
        acceleration_rad_s2=None,
        jerk_rad_s3=None,
        dt_s=0.02,
    )
    second = JointTrajectory(
        joint_names=JOINT_NAMES,
        position_rad=np.array([np.zeros(6), np.full(6, 2.0)]),
        velocity_rad_s=None,
        acceleration_rad_s2=None,
        jerk_rad_s3=None,
        dt_s=0.02,
    )

    with pytest.raises(ConfigurationError, match="discontinuous"):
        concatenate_trajectories(first, second)


def test_request_profile_or_seed_mismatch_raises_configuration_error() -> None:
    planner, _ = _planner(FakeResult())
    with pytest.raises(ConfigurationError, match="request profile"):
        planner.plan(_request(profile="wrong"))

    request = _request()
    mismatched = PlanningRequest(
        current_joint_state=request.current_joint_state,
        surface_target=request.surface_target,
        scene_revision=request.scene_revision,
        planner_profile=request.planner_profile,
        random_seed=999,
        request_id=request.request_id,
    )
    with pytest.raises(ConfigurationError, match="random_seed"):
        planner.plan(mismatched)
