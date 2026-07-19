"""Phase 4 unit tests for independent fail-closed trajectory validation."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import numpy as np

from mycobot_curobo.frames import TaskFrameConfig, build_task_frame_candidates
from mycobot_curobo.planner import (
    NamedJointState,
    NominalPlan,
    PlanningRequest,
)
from mycobot_curobo.robot_model import JOINT_NAMES, load_robot_model_spec
from mycobot_curobo.targets import SurfaceTarget
from mycobot_curobo.trajectory import JointTrajectory, concatenate_trajectories
from mycobot_curobo.validation import (
    KinematicCollisionBatch,
    load_validation_profile,
    validate_nominal_plan,
)

ROOT = Path(__file__).resolve().parents[2]
TASK_CONFIG = TaskFrameConfig()


class FakeEvaluator:
    def __init__(self, batch: KinematicCollisionBatch) -> None:
        self.batch = batch

    def evaluate(self, position_rad: np.ndarray) -> KinematicCollisionBatch:
        assert position_rad.shape[0] == self.batch.tcp_position_m.shape[0]
        return self.batch


def _trajectory(position: np.ndarray) -> JointTrajectory:
    zeros = np.zeros_like(position)
    return JointTrajectory(
        joint_names=JOINT_NAMES,
        position_rad=position,
        velocity_rad_s=zeros.copy(),
        acceleration_rad_s2=zeros.copy(),
        jerk_rad_s3=zeros.copy(),
        dt_s=0.02,
    )


def _case() -> tuple[NominalPlan, PlanningRequest, KinematicCollisionBatch]:
    target = SurfaceTarget.create(
        position_base_m=[0.1, 0.0, 0.2],
        surface_normal_base=[0.0, 0.0, -1.0],
        tangent_hint_base=[1.0, 0.0, 0.0],
        fixed_roll_rad=0.0,
        pre_approach_distance_m=0.02,
        target_id="phase4-unit",
    )
    request = PlanningRequest(
        current_joint_state=NamedJointState.create(JOINT_NAMES, np.zeros(6)),
        surface_target=target,
        scene_revision="empty-v1",
        planner_profile="development_fast",
        random_seed=123,
        request_id="phase4-unit",
    )
    terminal_position = np.zeros((4, 6), dtype=float)
    approach_position = np.vstack([np.full(6, -0.1), terminal_position[0]])
    approach = _trajectory(approach_position)
    terminal = _trajectory(terminal_position)
    plan = NominalPlan(
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
    candidate = build_task_frame_candidates(target, TASK_CONFIG)[0]
    positions = np.array(
        [
            target.position_base_m - np.array([0.0, 0.0, 0.02]),
            target.position_base_m - np.array([0.0, 0.0, 0.012]),
            target.position_base_m - np.array([0.0, 0.0, 0.005]),
            target.position_base_m,
        ]
    )
    batch = KinematicCollisionBatch(
        tcp_position_m=positions,
        tcp_rotation_base_from_tool=np.repeat(
            candidate.rotation_base_from_tool[None, :, :],
            positions.shape[0],
            axis=0,
        ),
        self_collision_clearance_m=np.full(positions.shape[0], 0.03),
        world_collision_clearance_m=np.full(positions.shape[0], 0.10),
        world_collision_evaluated=True,
    )
    return plan, request, batch


def _validate(
    plan: NominalPlan,
    request: PlanningRequest,
    batch: KinematicCollisionBatch,
):
    return validate_nominal_plan(
        plan,
        request,
        profile=load_validation_profile(
            "simulation_initial",
            ROOT / "config" / "validation_profiles.yml",
        ),
        evaluator=FakeEvaluator(batch),
        robot_spec=load_robot_model_spec(),
        task_frame_config=TASK_CONFIG,
    )


def _metrics(result) -> set[str]:
    return {violation.metric for violation in result.report.violations}


def test_valid_normal_line_path_becomes_executable() -> None:
    plan, request, batch = _case()

    result = _validate(plan, request, batch)

    assert result.report.valid
    assert result.validation_status == "valid"
    assert result.executable
    assert result.report.violations == ()
    assert result.report.metrics.max_lateral_error_m == 0.0


def test_curved_terminal_path_fails_lateral_corridor() -> None:
    plan, request, batch = _case()
    positions = batch.tcp_position_m.copy()
    positions[2, 0] += 0.006

    result = _validate(plan, request, replace(batch, tcp_position_m=positions))

    assert not result.executable
    assert "lateral_error" in _metrics(result)
    assert (
        next(v for v in result.report.violations if v.metric == "lateral_error").waypoint_index
        == 2
    )


def test_reversed_progress_path_fails_monotonicity() -> None:
    plan, request, batch = _case()
    positions = batch.tcp_position_m.copy()
    positions[2, 2] = positions[1, 2] - 0.003

    result = _validate(plan, request, replace(batch, tcp_position_m=positions))

    assert "progress_regression" in _metrics(result)


def test_misoriented_tcp_fails_axis_roll_and_terminal_orientation() -> None:
    plan, request, batch = _case()
    angle = 0.10
    rotation_x = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, np.cos(angle), -np.sin(angle)],
            [0.0, np.sin(angle), np.cos(angle)],
        ]
    )
    rotations = batch.tcp_rotation_base_from_tool @ rotation_x

    result = _validate(
        plan,
        request,
        replace(batch, tcp_rotation_base_from_tool=rotations),
    )

    assert "approach_axis_error" in _metrics(result)
    assert "terminal_orientation_error" in _metrics(result)


def test_unevaluated_world_collision_fails_closed() -> None:
    plan, request, batch = _case()

    result = _validate(
        plan,
        request,
        replace(
            batch,
            world_collision_clearance_m=None,
            world_collision_evaluated=False,
        ),
    )

    assert "world_collision_clearance" in _metrics(result)
    assert not result.executable


def test_joint_limit_and_dynamics_violations_are_reported() -> None:
    plan, request, batch = _case()
    terminal = plan.terminal_trajectory
    positions = terminal.position_rad.copy()
    positions[1, 0] = load_robot_model_spec().limits.upper_rad[0]
    velocity = terminal.velocity_rad_s.copy()
    velocity[2, 1] = load_robot_model_spec().limits.velocity_rad_s[1] + 0.1
    invalid_terminal = replace(
        terminal,
        position_rad=positions,
        velocity_rad_s=velocity,
    )
    invalid_plan = replace(plan, terminal_trajectory=invalid_terminal)

    result = _validate(invalid_plan, request, batch)

    assert {"joint_position_margin", "joint_velocity"} <= _metrics(result)


def test_self_collision_and_nonfinite_geometry_fail_closed() -> None:
    plan, request, batch = _case()
    clearance = batch.self_collision_clearance_m.copy()
    clearance[1] = -0.001
    positions = batch.tcp_position_m.copy()
    positions[3, 0] = np.nan

    result = _validate(
        plan,
        request,
        replace(
            batch,
            tcp_position_m=positions,
            self_collision_clearance_m=clearance,
        ),
    )

    assert "kinematics_finite" in _metrics(result)
    assert not result.executable
