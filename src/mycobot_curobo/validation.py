"""Independent, fail-closed trajectory validation for Phase 4.

The planner's success flag is never treated as execution authorization. This
module recomputes terminal geometry, dynamics, limits, and collision clearance
through an injected evaluator and returns machine-readable violations.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

import numpy as np
import yaml

from mycobot_curobo.errors import ConfigurationError
from mycobot_curobo.frames import TaskFrameConfig, build_task_frame_candidates
from mycobot_curobo.planner import NominalPlan, PlanningRequest
from mycobot_curobo.robot_model import JOINT_NAMES, RobotModelSpec

_AXIS_INDEX = {"x": 0, "y": 1, "z": 2}


@dataclass(frozen=True)
class ValidationProfile:
    """Validated simulation or hardware trajectory thresholds in SI units."""

    name: str
    max_lateral_error_m: float
    max_approach_axis_error_rad: float
    max_roll_error_rad: float
    max_terminal_position_error_m: float
    max_terminal_orientation_error_rad: float
    max_progress_regression_m: float
    minimum_joint_limit_margin_rad: float
    minimum_self_collision_clearance_m: float
    minimum_world_collision_clearance_m: float
    boundary_position_tolerance_rad: float


@dataclass(frozen=True)
class KinematicCollisionBatch:
    """FK and collision data for an ordered joint-position batch."""

    tcp_position_m: np.ndarray
    tcp_rotation_base_from_tool: np.ndarray
    self_collision_clearance_m: np.ndarray
    world_collision_clearance_m: np.ndarray | None
    world_collision_evaluated: bool


class TrajectoryEvaluator(Protocol):
    """Backend-neutral FK and collision interface."""

    def evaluate(self, position_rad: np.ndarray) -> KinematicCollisionBatch:
        """Evaluate all waypoints without changing their order."""


@dataclass(frozen=True)
class ValidationViolation:
    """One threshold failure with the first offending waypoint."""

    metric: str
    waypoint_index: int
    measured: float | None
    threshold: float | None
    reason: str


@dataclass(frozen=True)
class ValidationMetrics:
    """Compact metrics retained for reports and Phase 6 aggregation."""

    max_lateral_error_m: float | None
    max_approach_axis_error_rad: float | None
    max_roll_error_rad: float | None
    terminal_position_error_m: float | None
    terminal_orientation_error_rad: float | None
    max_progress_regression_m: float | None
    minimum_joint_limit_margin_rad: float | None
    minimum_self_collision_clearance_m: float | None
    minimum_world_collision_clearance_m: float | None


@dataclass(frozen=True)
class ValidationReport:
    """Fail-closed result for one nominal plan."""

    request_id: str
    profile_name: str
    valid: bool
    violations: tuple[ValidationViolation, ...]
    metrics: ValidationMetrics


@dataclass(frozen=True)
class ValidatedPlan:
    """Nominal plan plus independent evidence and execution eligibility."""

    nominal_plan: NominalPlan
    report: ValidationReport
    validation_status: str
    executable: bool


def load_validation_profile(
    name: str,
    path: Path | str = Path("config/validation_profiles.yml"),
) -> ValidationProfile:
    """Load one named threshold profile and reject unsafe values."""

    source = Path(path)
    if not source.is_file():
        raise ConfigurationError(f"validation profile config not found: {source}")
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    try:
        values = payload["profiles"][name]
    except (KeyError, TypeError) as exc:
        raise ConfigurationError(f"unknown validation profile: {name}") from exc
    profile = ValidationProfile(name=name, **values)
    for field_name, value in vars(profile).items():
        if field_name == "name":
            continue
        numeric = float(value)
        if not math.isfinite(numeric) or numeric < 0.0:
            raise ConfigurationError(
                f"validation profile {name!r} field {field_name!r} must be finite and non-negative"
            )
    return profile


def _quaternion_to_rotation_wxyz(quaternion: np.ndarray) -> np.ndarray:
    quaternion = np.asarray(quaternion, dtype=float)
    quaternion = quaternion / np.linalg.norm(quaternion, axis=-1, keepdims=True)
    w, x, y, z = np.moveaxis(quaternion, -1, 0)
    rotation = np.empty(quaternion.shape[:-1] + (3, 3), dtype=float)
    rotation[..., 0, 0] = 1.0 - 2.0 * (y * y + z * z)
    rotation[..., 0, 1] = 2.0 * (x * y - z * w)
    rotation[..., 0, 2] = 2.0 * (x * z + y * w)
    rotation[..., 1, 0] = 2.0 * (x * y + z * w)
    rotation[..., 1, 1] = 1.0 - 2.0 * (x * x + z * z)
    rotation[..., 1, 2] = 2.0 * (y * z - x * w)
    rotation[..., 2, 0] = 2.0 * (x * z - y * w)
    rotation[..., 2, 1] = 2.0 * (y * z + x * w)
    rotation[..., 2, 2] = 1.0 - 2.0 * (x * x + y * y)
    return rotation


class CuroboTrajectoryEvaluator:
    """Evaluate FK and collision spheres with a validation-only cuRobo planner.

    Empty worlds are explicitly evaluated as collision-free. A non-empty world
    requires a world-clearance adapter; until one is supplied this class reports
    that metric as unevaluated so validation fails closed.
    """

    def __init__(self, planner: Any, *, scene_is_empty: bool) -> None:
        self._planner = planner
        self._scene_is_empty = bool(scene_is_empty)

    def evaluate(self, position_rad: np.ndarray) -> KinematicCollisionBatch:
        import torch
        from curobo.types import JointState

        positions = np.asarray(position_rad, dtype=float)
        if positions.ndim != 2 or positions.shape[1] != len(JOINT_NAMES):
            raise ConfigurationError("validation positions must have shape [waypoint, 6]")
        state = JointState.from_position(
            torch.as_tensor(positions, device="cuda:0", dtype=torch.float32),
            joint_names=list(JOINT_NAMES),
        )
        result = self._planner.compute_kinematics(state)
        tool_pose = result.tool_poses.to_dict()["tcp_link"]
        tcp_position = tool_pose.position.detach().cpu().numpy().reshape(-1, 3)
        quaternion = tool_pose.quaternion.detach().cpu().numpy().reshape(-1, 4)
        rotations = _quaternion_to_rotation_wxyz(quaternion)

        spheres = (
            result.robot_spheres.detach()
            .cpu()
            .numpy()
            .reshape(
                positions.shape[0],
                -1,
                4,
            )
        )
        pairs = (
            self._planner.kinematics.get_self_collision_config()
            .collision_pairs.detach()
            .cpu()
            .numpy()
            .astype(int)
        )
        first = spheres[:, pairs[:, 0], :]
        second = spheres[:, pairs[:, 1], :]
        pair_clearance = (
            np.linalg.norm(first[..., :3] - second[..., :3], axis=-1)
            - first[..., 3]
            - second[..., 3]
        )
        self_clearance = np.min(pair_clearance, axis=1)
        world_clearance = (
            np.full(positions.shape[0], np.finfo(float).max, dtype=float)
            if self._scene_is_empty
            else None
        )
        return KinematicCollisionBatch(
            tcp_position_m=tcp_position,
            tcp_rotation_base_from_tool=rotations,
            self_collision_clearance_m=self_clearance,
            world_collision_clearance_m=world_clearance,
            world_collision_evaluated=self._scene_is_empty,
        )


def _rotation_error_rad(actual: np.ndarray, desired: np.ndarray) -> float:
    relative = desired.T @ actual
    cosine = float(np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0))
    return math.acos(cosine)


def _first_exceeding(values: np.ndarray, threshold: float) -> int | None:
    indices = np.flatnonzero(values > threshold)
    return None if indices.size == 0 else int(indices[0])


def validate_nominal_plan(
    plan: NominalPlan,
    request: PlanningRequest,
    *,
    profile: ValidationProfile,
    evaluator: TrajectoryEvaluator,
    robot_spec: RobotModelSpec,
    task_frame_config: TaskFrameConfig,
) -> ValidatedPlan:
    """Validate a Phase 3 plan and grant eligibility only when all checks pass."""

    violations: list[ValidationViolation] = []

    def fail(
        metric: str,
        waypoint: int,
        measured: float | None,
        threshold: float | None,
        reason: str,
    ) -> None:
        violations.append(
            ValidationViolation(
                metric=metric,
                waypoint_index=waypoint,
                measured=measured,
                threshold=threshold,
                reason=reason,
            )
        )

    if plan.request_id != request.request_id:
        raise ConfigurationError("plan and validation request IDs differ")
    if plan.validation_status != "not_evaluated" or plan.executable:
        raise ConfigurationError("Phase 4 expects a non-executable unevaluated nominal plan")

    candidates = build_task_frame_candidates(request.surface_target, task_frame_config)
    if plan.selected_goal_index < 0 or plan.selected_goal_index >= len(candidates):
        raise ConfigurationError("selected goal index is outside the rebuilt candidate set")
    goal = candidates[plan.selected_goal_index]
    if not math.isclose(plan.selected_roll_rad, goal.roll_rad, abs_tol=1.0e-12):
        raise ConfigurationError("selected roll does not match rebuilt candidate set")

    terminal = plan.terminal_trajectory
    if terminal.joint_names != JOINT_NAMES:
        raise ConfigurationError("terminal trajectory joint order is invalid")
    sample_count = terminal.sample_count
    if sample_count < 2:
        fail("sample_count", 0, float(sample_count), 2.0, "terminal needs at least two samples")

    boundary_error = float(
        np.max(np.abs(plan.approach_trajectory.position_rad[-1] - terminal.position_rad[0]))
    )
    if boundary_error > profile.boundary_position_tolerance_rad:
        fail(
            "boundary_position",
            0,
            boundary_error,
            profile.boundary_position_tolerance_rad,
            "approach and terminal segments are discontinuous",
        )

    arrays = [
        terminal.position_rad,
        terminal.velocity_rad_s,
        terminal.acceleration_rad_s2,
    ]
    if terminal.jerk_rad_s3 is not None:
        arrays.append(terminal.jerk_rad_s3)
    if any(array is not None and not np.all(np.isfinite(array)) for array in arrays):
        fail("finite", 0, None, None, "trajectory contains non-finite values")
    if terminal.velocity_rad_s is None:
        fail("joint_velocity", 0, None, None, "velocity metric is unevaluated")
    if terminal.acceleration_rad_s2 is None:
        fail("joint_acceleration", 0, None, None, "acceleration metric is unevaluated")

    lower_margin = terminal.position_rad - robot_spec.limits.lower_rad
    upper_margin = robot_spec.limits.upper_rad - terminal.position_rad
    joint_margin = np.minimum(lower_margin, upper_margin)
    minimum_joint_margin = float(np.min(joint_margin))
    margin_indices = np.argwhere(joint_margin < profile.minimum_joint_limit_margin_rad)
    if margin_indices.size:
        fail(
            "joint_position_margin",
            int(margin_indices[0, 0]),
            float(joint_margin[tuple(margin_indices[0])]),
            profile.minimum_joint_limit_margin_rad,
            "joint position is outside the required limit margin",
        )

    for metric, values, limits in (
        ("joint_velocity", terminal.velocity_rad_s, robot_spec.limits.velocity_rad_s),
        (
            "joint_acceleration",
            terminal.acceleration_rad_s2,
            robot_spec.limits.acceleration_rad_s2,
        ),
        ("joint_jerk", terminal.jerk_rad_s3, robot_spec.limits.jerk_rad_s3),
    ):
        if values is None:
            continue
        excess = np.abs(values) - limits
        indices = np.argwhere(excess > 1.0e-9)
        if indices.size:
            waypoint, joint = (int(value) for value in indices[0])
            fail(
                metric,
                waypoint,
                float(abs(values[waypoint, joint])),
                float(limits[joint]),
                f"{metric} exceeds configured joint limit",
            )

    geometry: KinematicCollisionBatch | None = None
    try:
        geometry = evaluator.evaluate(terminal.position_rad)
    except (RuntimeError, ValueError, ConfigurationError) as exc:
        fail("kinematics_collision", 0, None, None, f"evaluation failed: {exc}")

    max_lateral: float | None = None
    max_axis_error: float | None = None
    max_roll_error: float | None = None
    terminal_position_error: float | None = None
    terminal_orientation_error: float | None = None
    max_regression: float | None = None
    minimum_self_clearance: float | None = None
    minimum_world_clearance: float | None = None

    if geometry is not None:
        expected_shapes = (
            geometry.tcp_position_m.shape == (sample_count, 3)
            and geometry.tcp_rotation_base_from_tool.shape == (sample_count, 3, 3)
            and geometry.self_collision_clearance_m.shape == (sample_count,)
        )
        if not expected_shapes:
            fail("kinematics_shape", 0, None, None, "evaluator returned invalid shapes")
        else:
            finite_arrays = [
                geometry.tcp_position_m,
                geometry.tcp_rotation_base_from_tool,
                geometry.self_collision_clearance_m,
            ]
            if geometry.world_collision_clearance_m is not None:
                finite_arrays.append(geometry.world_collision_clearance_m)
            if not all(np.all(np.isfinite(array)) for array in finite_arrays):
                fail("kinematics_finite", 0, None, None, "FK/collision output is non-finite")
            else:
                approach = goal.approach_direction_base
                displacement = geometry.tcp_position_m - goal.position_base_m
                axial = displacement @ approach
                lateral = displacement - axial[:, None] * approach
                lateral_error = np.linalg.norm(lateral, axis=1)
                max_lateral = float(np.max(lateral_error))
                index = _first_exceeding(lateral_error, profile.max_lateral_error_m)
                if index is not None:
                    fail(
                        "lateral_error",
                        index,
                        float(lateral_error[index]),
                        profile.max_lateral_error_m,
                        "TCP left the target-normal corridor",
                    )

                axis_index = _AXIS_INDEX[task_frame_config.tool_approach_axis]
                actual_axis = (
                    geometry.tcp_rotation_base_from_tool[:, :, axis_index]
                    * task_frame_config.tool_approach_sign
                )
                axis_dot = np.clip(actual_axis @ approach, -1.0, 1.0)
                axis_error = np.arccos(axis_dot)
                max_axis_error = float(np.max(axis_error))
                index = _first_exceeding(
                    axis_error,
                    profile.max_approach_axis_error_rad,
                )
                if index is not None:
                    fail(
                        "approach_axis_error",
                        index,
                        float(axis_error[index]),
                        profile.max_approach_axis_error_rad,
                        "TCP approach axis is misaligned",
                    )

                tangent_index = (axis_index + 1) % 3
                desired_tangent = goal.rotation_base_from_tool[:, tangent_index]
                actual_tangent = geometry.tcp_rotation_base_from_tool[:, :, tangent_index]
                projected = actual_tangent - (actual_tangent @ approach)[:, None] * approach
                projected_norm = np.linalg.norm(projected, axis=1)
                if np.any(projected_norm <= 1.0e-12):
                    fail("roll_error", 0, None, profile.max_roll_error_rad, "roll undefined")
                else:
                    projected /= projected_norm[:, None]
                    roll_error = np.arccos(np.clip(projected @ desired_tangent, -1.0, 1.0))
                    max_roll_error = float(np.max(roll_error))
                    index = _first_exceeding(roll_error, profile.max_roll_error_rad)
                    if index is not None:
                        fail(
                            "roll_error",
                            index,
                            float(roll_error[index]),
                            profile.max_roll_error_rad,
                            "TCP roll differs from the selected candidate",
                        )

                progress_delta = np.diff(axial)
                regressions = np.maximum(-progress_delta, 0.0)
                max_regression = float(np.max(regressions)) if regressions.size else 0.0
                index = _first_exceeding(
                    regressions,
                    profile.max_progress_regression_m,
                )
                if index is not None:
                    fail(
                        "progress_regression",
                        index + 1,
                        float(regressions[index]),
                        profile.max_progress_regression_m,
                        "terminal path moves away from the target",
                    )

                terminal_position_error = float(
                    np.linalg.norm(geometry.tcp_position_m[-1] - goal.position_base_m)
                )
                if terminal_position_error > profile.max_terminal_position_error_m:
                    fail(
                        "terminal_position_error",
                        sample_count - 1,
                        terminal_position_error,
                        profile.max_terminal_position_error_m,
                        "terminal TCP position misses the target",
                    )
                terminal_orientation_error = _rotation_error_rad(
                    geometry.tcp_rotation_base_from_tool[-1],
                    goal.rotation_base_from_tool,
                )
                if terminal_orientation_error > profile.max_terminal_orientation_error_rad:
                    fail(
                        "terminal_orientation_error",
                        sample_count - 1,
                        terminal_orientation_error,
                        profile.max_terminal_orientation_error_rad,
                        "terminal TCP orientation misses the target",
                    )

                minimum_self_clearance = float(np.min(geometry.self_collision_clearance_m))
                self_indices = np.flatnonzero(
                    geometry.self_collision_clearance_m
                    < profile.minimum_self_collision_clearance_m
                )
                if self_indices.size:
                    index = int(self_indices[0])
                    fail(
                        "self_collision_clearance",
                        index,
                        float(geometry.self_collision_clearance_m[index]),
                        profile.minimum_self_collision_clearance_m,
                        "self-collision clearance is insufficient",
                    )

                if not geometry.world_collision_evaluated:
                    fail(
                        "world_collision_clearance",
                        0,
                        None,
                        profile.minimum_world_collision_clearance_m,
                        "world collision metric is unevaluated",
                    )
                elif geometry.world_collision_clearance_m is None:
                    fail(
                        "world_collision_clearance",
                        0,
                        None,
                        profile.minimum_world_collision_clearance_m,
                        "world clearance data is missing",
                    )
                else:
                    minimum_world_clearance = float(np.min(geometry.world_collision_clearance_m))
                    world_indices = np.flatnonzero(
                        geometry.world_collision_clearance_m
                        < profile.minimum_world_collision_clearance_m
                    )
                    if world_indices.size:
                        index = int(world_indices[0])
                        fail(
                            "world_collision_clearance",
                            index,
                            float(geometry.world_collision_clearance_m[index]),
                            profile.minimum_world_collision_clearance_m,
                            "world-collision clearance is insufficient",
                        )

    report = ValidationReport(
        request_id=request.request_id,
        profile_name=profile.name,
        valid=not violations,
        violations=tuple(violations),
        metrics=ValidationMetrics(
            max_lateral_error_m=max_lateral,
            max_approach_axis_error_rad=max_axis_error,
            max_roll_error_rad=max_roll_error,
            terminal_position_error_m=terminal_position_error,
            terminal_orientation_error_rad=terminal_orientation_error,
            max_progress_regression_m=max_regression,
            minimum_joint_limit_margin_rad=minimum_joint_margin,
            minimum_self_collision_clearance_m=minimum_self_clearance,
            minimum_world_collision_clearance_m=minimum_world_clearance,
        ),
    )
    return ValidatedPlan(
        nominal_plan=plan,
        report=report,
        validation_status="valid" if report.valid else "invalid",
        executable=report.valid,
    )
