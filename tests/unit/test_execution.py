"""Phase 5 dry-run trajectory execution tests."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from mycobot_curobo.errors import ConfigurationError
from mycobot_curobo.execution import (
    InMemoryCommandAdapter,
    ReplayRobotStateProvider,
    RobotStateSample,
    TrajectoryExecutor,
    TrajectorySource,
    command_positions,
)
from mycobot_curobo.frames import TaskFrameConfig
from mycobot_curobo.planner import NamedJointState, NominalPlan, PlanningRequest
from mycobot_curobo.residual import CartesianResidual, ZeroResidualCorrector
from mycobot_curobo.robot_model import JOINT_NAMES, JointLimits, Pose
from mycobot_curobo.safety import ResidualSafetyProfile, SafetyProjector
from mycobot_curobo.targets import SurfaceTarget
from mycobot_curobo.trajectory import JointTrajectory, concatenate_trajectories
from mycobot_curobo.validation import (
    ValidatedPlan,
    ValidationMetrics,
    ValidationReport,
)

ROOT = Path(__file__).resolve().parents[2]


class FixedPoseEvaluator:
    def __init__(self, position_m: np.ndarray) -> None:
        self.position_m = position_m

    def evaluate(self, joint_position_rad: tuple[float, ...]) -> Pose:
        assert len(joint_position_rad) == 6
        return Pose(
            position_m=self.position_m.copy(),
            quaternion_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
        )


class StaleStateProvider:
    def state_at(self, command_time_s: float, nominal) -> RobotStateSample:
        return RobotStateSample(
            joint_names=JOINT_NAMES,
            position_rad=nominal.joint_position_rad,
            timestamp_s=command_time_s - 1.0,
        )


class FixedResidualCorrector:
    def correction(self, observation) -> CartesianResidual:
        return CartesianResidual.create([0.0, 0.0, 0.001], [0.0, 0.0, 0.0])


def _trajectory(positions: np.ndarray) -> JointTrajectory:
    zeros = np.zeros_like(positions)
    return JointTrajectory(
        joint_names=JOINT_NAMES,
        position_rad=positions,
        velocity_rad_s=zeros.copy(),
        acceleration_rad_s2=zeros.copy(),
        jerk_rad_s3=zeros.copy(),
        dt_s=0.02,
    )


def _case() -> tuple[ValidatedPlan, PlanningRequest]:
    target = SurfaceTarget.create(
        position_base_m=[0.1, 0.0, 0.2],
        surface_normal_base=[0.0, 0.0, -1.0],
        tangent_hint_base=[1.0, 0.0, 0.0],
        fixed_roll_rad=0.0,
        pre_approach_distance_m=0.02,
        target_id="phase5-execution",
    )
    request = PlanningRequest(
        current_joint_state=NamedJointState.create(JOINT_NAMES, np.zeros(6)),
        surface_target=target,
        scene_revision="empty-v1",
        planner_profile="development_fast",
        random_seed=123,
        request_id="phase5-execution",
    )
    approach = _trajectory(np.array([[-0.1] * 6, [0.0] * 6]))
    terminal = _trajectory(np.array([[0.0] * 6, [0.01] * 6, [0.02] * 6]))
    nominal = NominalPlan(
        request_id=request.request_id,
        selected_goal_index=0,
        selected_roll_rad=0.0,
        approach_trajectory=approach,
        terminal_trajectory=terminal,
        combined_trajectory=concatenate_trajectories(approach, terminal),
        planner_status="success",
        planner_timings_s={"adapter_wall_time": 0.1},
        curobo_version="0.8.0",
        scene_revision=request.scene_revision,
        planner_profile=request.planner_profile,
        random_seed=request.random_seed,
    )
    metrics = ValidationMetrics(
        max_lateral_error_m=0.0,
        max_approach_axis_error_rad=0.0,
        max_roll_error_rad=0.0,
        terminal_position_error_m=0.0,
        terminal_orientation_error_rad=0.0,
        max_progress_regression_m=0.0,
        minimum_joint_limit_margin_rad=1.0,
        minimum_self_collision_clearance_m=0.1,
        minimum_world_collision_clearance_m=0.1,
    )
    report = ValidationReport(
        request_id=request.request_id,
        profile_name="simulation_initial",
        valid=True,
        violations=(),
        metrics=metrics,
    )
    return (
        ValidatedPlan(
            nominal_plan=nominal,
            report=report,
            validation_status="valid",
            executable=True,
        ),
        request,
    )


def _projector() -> SafetyProjector:
    profile = ResidualSafetyProfile(
        name="unit",
        max_translation_m=0.002,
        max_rotation_rad=0.02,
        max_lateral_error_m=0.005,
        minimum_joint_limit_margin_rad=0.02,
        max_state_age_s=0.1,
        watchdog_timeout_s=0.25,
    )
    limits = JointLimits(
        names=JOINT_NAMES,
        lower_rad=np.full(6, -2.0),
        upper_rad=np.full(6, 2.0),
        velocity_rad_s=np.ones(6),
        acceleration_rad_s2=np.ones(6),
        jerk_rad_s3=np.ones(6),
    )
    return SafetyProjector(profile, limits)


def _executor(
    plan: ValidatedPlan,
    *,
    corrector=ZeroResidualCorrector(),
    state_provider=ReplayRobotStateProvider(),
) -> tuple[TrajectoryExecutor, InMemoryCommandAdapter]:
    adapter = InMemoryCommandAdapter()
    executor = TrajectoryExecutor(
        corrector=corrector,
        projector=_projector(),
        state_provider=state_provider,
        pose_evaluator=FixedPoseEvaluator(np.array([0.1, 0.0, 0.2])),
        adapter=adapter,
        task_frame_config=TaskFrameConfig(),
    )
    return executor, adapter


def test_zero_residual_reproduces_nominal_command_stream() -> None:
    plan, request = _case()
    executor, adapter = _executor(plan)

    result = executor.execute(plan, request, started_at_s=10.0)

    assert result.completed
    assert np.array_equal(
        command_positions(result),
        plan.nominal_plan.combined_trajectory.position_rad,
    )
    assert tuple(adapter.commands) == result.commands


def test_invalid_plan_never_enters_execution() -> None:
    plan, _ = _case()
    invalid = replace(plan, executable=False, validation_status="invalid")

    with pytest.raises(ConfigurationError, match="valid executable"):
        TrajectorySource(invalid)


def test_stale_state_stops_before_first_command() -> None:
    plan, request = _case()
    executor, adapter = _executor(plan, state_provider=StaleStateProvider())

    result = executor.execute(plan, request, started_at_s=10.0)

    assert not result.completed
    assert result.failure_category == "safety_rejected"
    assert result.failure_waypoint_index == 0
    assert adapter.commands == []


def test_nonzero_residual_cannot_generate_a_replacement_joint_path() -> None:
    plan, request = _case()
    executor, adapter = _executor(plan, corrector=FixedResidualCorrector())

    result = executor.execute(plan, request, started_at_s=10.0)

    assert not result.completed
    assert result.failure_category == "nonzero_residual_not_implemented"
    assert adapter.commands == []


def test_phase5_modules_have_no_physical_driver_dependency() -> None:
    sources = "\n".join(
        (ROOT / "src" / "mycobot_curobo" / name).read_text(encoding="utf-8")
        for name in ("residual.py", "safety.py", "execution.py")
    ).lower()
    for forbidden in ("pymycobot", "serial", "rclpy", "isaacsim", "isaaclab"):
        assert forbidden not in sources
