"""Dry-run execution seam for independently validated cuRobo trajectories."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol

import numpy as np

from mycobot_curobo.errors import ConfigurationError
from mycobot_curobo.frames import TaskFrameConfig, build_task_frame_candidates
from mycobot_curobo.planner import PlanningRequest
from mycobot_curobo.residual import ResidualCorrector, ResidualObservation
from mycobot_curobo.robot_model import (
    JOINT_NAMES,
    Pose,
    RobotModelSpec,
    forward_kinematics,
)
from mycobot_curobo.safety import SafetyDecision, SafetyProjector
from mycobot_curobo.trajectory import JointTrajectory
from mycobot_curobo.validation import ValidatedPlan


@dataclass(frozen=True)
class TrajectorySample:
    """One nominal waypoint with explicit timing and optional velocity."""

    waypoint_index: int
    time_from_start_s: float
    joint_position_rad: tuple[float, ...]
    joint_velocity_rad_s: tuple[float, ...] | None


class TrajectorySource:
    """Index a verified nominal trajectory without changing its samples."""

    def __init__(self, plan: ValidatedPlan) -> None:
        if not plan.executable or not plan.report.valid or plan.validation_status != "valid":
            raise ConfigurationError("trajectory source requires a valid executable plan")
        self._trajectory = plan.nominal_plan.combined_trajectory
        if self._trajectory.joint_names != JOINT_NAMES:
            raise ConfigurationError("trajectory source joint order is invalid")

    @property
    def sample_count(self) -> int:
        return self._trajectory.sample_count

    def sample(self, waypoint_index: int) -> TrajectorySample:
        if waypoint_index < 0 or waypoint_index >= self.sample_count:
            raise ConfigurationError("trajectory waypoint index is outside the valid range")
        velocity = self._trajectory.velocity_rad_s
        return TrajectorySample(
            waypoint_index=waypoint_index,
            time_from_start_s=waypoint_index * self._trajectory.dt_s,
            joint_position_rad=tuple(
                float(value) for value in self._trajectory.position_rad[waypoint_index]
            ),
            joint_velocity_rad_s=(
                None
                if velocity is None
                else tuple(float(value) for value in velocity[waypoint_index])
            ),
        )


@dataclass(frozen=True)
class RobotStateSample:
    """Measured named joint state and monotonic timestamp."""

    joint_names: tuple[str, ...]
    position_rad: tuple[float, ...]
    timestamp_s: float


class RobotStateProvider(Protocol):
    """Supply a measured state for one command timestamp."""

    def state_at(self, command_time_s: float, nominal: TrajectorySample) -> RobotStateSample:
        """Return a state with an explicit monotonic timestamp."""


class ReplayRobotStateProvider:
    """Deterministically replay the nominal state as the measured state."""

    def state_at(self, command_time_s: float, nominal: TrajectorySample) -> RobotStateSample:
        return RobotStateSample(
            joint_names=JOINT_NAMES,
            position_rad=nominal.joint_position_rad,
            timestamp_s=float(command_time_s),
        )


class TcpPoseEvaluator(Protocol):
    """Evaluate a base-to-TCP pose without planning."""

    def evaluate(self, joint_position_rad: tuple[float, ...]) -> Pose:
        """Return the TCP pose for an explicit joint vector."""


class CpuTcpPoseEvaluator:
    """Independent NumPy/URDF FK adapter used by the Phase 5 dry-run seam."""

    def __init__(self, robot_spec: RobotModelSpec) -> None:
        self._robot_spec = robot_spec

    def evaluate(self, joint_position_rad: tuple[float, ...]) -> Pose:
        return forward_kinematics(joint_position_rad, spec=self._robot_spec)


@dataclass(frozen=True)
class JointCommand:
    """One dry-run joint command emitted through an injected adapter."""

    request_id: str
    waypoint_index: int
    time_from_start_s: float
    joint_names: tuple[str, ...]
    position_rad: tuple[float, ...]
    velocity_rad_s: tuple[float, ...] | None
    safety: SafetyDecision


class CommandAdapter(Protocol):
    """Output boundary; Phase 5 provides only an in-memory implementation."""

    def emit(self, command: JointCommand) -> None:
        """Record one already projected and re-validated command."""


class InMemoryCommandAdapter:
    """Append-only dry-run command log with no physical driver dependency."""

    def __init__(self) -> None:
        self.commands: list[JointCommand] = []

    def emit(self, command: JointCommand) -> None:
        self.commands.append(command)


@dataclass(frozen=True)
class ExecutionResult:
    """Structured completion or fail-closed stop result."""

    completed: bool
    commands: tuple[JointCommand, ...]
    failure_category: str | None
    failure_waypoint_index: int | None
    reason: str | None


class TrajectoryExecutor:
    """Replay a validated plan through zero residual and deterministic safety checks."""

    def __init__(
        self,
        *,
        corrector: ResidualCorrector,
        projector: SafetyProjector,
        state_provider: RobotStateProvider,
        pose_evaluator: TcpPoseEvaluator,
        adapter: CommandAdapter,
        task_frame_config: TaskFrameConfig,
    ) -> None:
        self._corrector = corrector
        self._projector = projector
        self._state_provider = state_provider
        self._pose_evaluator = pose_evaluator
        self._adapter = adapter
        self._task_frame_config = task_frame_config

    def execute(
        self,
        plan: ValidatedPlan,
        request: PlanningRequest,
        *,
        started_at_s: float = 0.0,
    ) -> ExecutionResult:
        """Emit unchanged nominal commands or stop before the first unsafe command."""

        start = float(started_at_s)
        if not math.isfinite(start) or start < 0.0:
            raise ConfigurationError("execution start time must be finite and non-negative")
        if plan.nominal_plan.request_id != request.request_id:
            raise ConfigurationError("execution plan and request IDs differ")
        source = TrajectorySource(plan)
        candidates = build_task_frame_candidates(
            request.surface_target,
            self._task_frame_config,
        )
        selected_index = plan.nominal_plan.selected_goal_index
        if selected_index < 0 or selected_index >= len(candidates):
            raise ConfigurationError("execution selected goal index is invalid")
        goal = candidates[selected_index]
        emitted: list[JointCommand] = []

        for index in range(source.sample_count):
            nominal = source.sample(index)
            command_time = start + nominal.time_from_start_s
            measured = self._state_provider.state_at(command_time, nominal)
            if measured.joint_names != JOINT_NAMES:
                return self._stop(emitted, "invalid_robot_state", index, "joint names differ")
            pose = self._pose_evaluator.evaluate(nominal.joint_position_rad)
            observation = ResidualObservation.create(
                request_id=request.request_id,
                waypoint_index=index,
                command_time_s=command_time,
                measured_timestamp_s=measured.timestamp_s,
                nominal_joint_position_rad=nominal.joint_position_rad,
                measured_joint_position_rad=measured.position_rad,
                nominal_tcp_position_base_m=pose.position_m,
                goal_position_base_m=goal.position_base_m,
                approach_direction_base=goal.approach_direction_base,
            )
            residual = self._corrector.correction(observation)
            decision = self._projector.project(
                observation,
                residual,
                evaluated_at_s=command_time,
                plan_executable=plan.executable,
            )
            if not decision.accepted:
                return self._stop(
                    emitted,
                    "safety_rejected",
                    index,
                    ",".join(decision.reasons),
                )
            if decision.projected_residual is None:
                raise ConfigurationError("accepted safety decision is missing a residual")
            if not decision.projected_residual.is_zero:
                return self._stop(
                    emitted,
                    "nonzero_residual_not_implemented",
                    index,
                    "Phase 5 cannot map Cartesian residuals into joint commands",
                )

            # Phase 5's corrected command is exactly nominal. The projector has
            # independently rechecked freshness, joint feasibility, and the TCP
            # corridor for this waypoint before the adapter can observe it.
            command = JointCommand(
                request_id=request.request_id,
                waypoint_index=index,
                time_from_start_s=nominal.time_from_start_s,
                joint_names=JOINT_NAMES,
                position_rad=nominal.joint_position_rad,
                velocity_rad_s=nominal.joint_velocity_rad_s,
                safety=decision,
            )
            self._adapter.emit(command)
            emitted.append(command)

        return ExecutionResult(
            completed=True,
            commands=tuple(emitted),
            failure_category=None,
            failure_waypoint_index=None,
            reason=None,
        )

    @staticmethod
    def _stop(
        commands: list[JointCommand],
        category: str,
        waypoint_index: int,
        reason: str,
    ) -> ExecutionResult:
        return ExecutionResult(
            completed=False,
            commands=tuple(commands),
            failure_category=category,
            failure_waypoint_index=waypoint_index,
            reason=reason,
        )


def command_positions(result: ExecutionResult) -> np.ndarray:
    """Return command positions for deterministic replay comparisons."""

    if not result.commands:
        return np.empty((0, len(JOINT_NAMES)), dtype=float)
    return np.asarray([command.position_rad for command in result.commands], dtype=float)


def trajectory_positions(trajectory: JointTrajectory) -> np.ndarray:
    """Return a defensive copy of nominal positions for test/report comparisons."""

    return np.asarray(trajectory.position_rad, dtype=float).copy()
