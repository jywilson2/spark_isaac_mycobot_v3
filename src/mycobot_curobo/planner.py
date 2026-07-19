"""cuRoboV2 nominal planner adapter with structured fail-closed results.

The production backend uses public ``MotionPlanner.plan_grasp``. Dependency
injection keeps result parsing, signed-offset logic, and failure taxonomy
unit-testable without CUDA. Phase 3 plans are never executable until Phase 4
independent validation marks them valid.
"""

from __future__ import annotations

import math
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Callable, Protocol

import numpy as np
import yaml

from mycobot_curobo.errors import ConfigurationError
from mycobot_curobo.frames import TaskFrameConfig
from mycobot_curobo.goal_set import (
    SurfaceGoalSet,
    build_surface_goal_set,
    to_curobo_goal_tool_pose,
)
from mycobot_curobo.robot_model import (
    JOINT_NAMES,
    load_curobo_robot_config,
    reorder_joint_state,
)
from mycobot_curobo.targets import SurfaceTarget
from mycobot_curobo.trajectory import (
    JointTrajectory,
    concatenate_trajectories,
    extract_curobo_trajectory,
)


@dataclass(frozen=True)
class NamedJointState:
    """Current joint state in radians with explicit names."""

    names: tuple[str, ...]
    position_rad: np.ndarray

    @classmethod
    def create(cls, names: tuple[str, ...], position_rad: Any) -> NamedJointState:
        ordered = reorder_joint_state(position_rad, names, JOINT_NAMES)
        if names != JOINT_NAMES:
            raise ConfigurationError(
                "current joint names are reordered; call reorder_joint_state explicitly "
                "and construct a state in cuRobo order"
            )
        return cls(names=JOINT_NAMES, position_rad=ordered)


@dataclass(frozen=True)
class PlanningRequest:
    """Inputs needed for one reproducible nominal plan."""

    current_joint_state: NamedJointState
    surface_target: SurfaceTarget
    scene_revision: str
    planner_profile: str
    random_seed: int
    request_id: str


@dataclass(frozen=True)
class PlannerProfile:
    """Validated cuRobo planner creation parameters."""

    name: str
    self_collision_check: bool
    num_ik_seeds: int
    num_trajopt_seeds: int
    position_tolerance_m: float
    orientation_tolerance_rad: float
    use_cuda_graph: bool
    random_seed: int
    optimizer_collision_activation_distance_m: float
    max_batch_size: int
    multi_env: bool
    max_goalset: int
    warmup_iterations: int
    enable_graph_warmup: bool
    max_plan_grasp_attempts: int


@dataclass(frozen=True)
class NominalPlan:
    """Successful cuRobo plan awaiting independent Phase 4 validation."""

    request_id: str
    selected_goal_index: int
    selected_roll_rad: float
    approach_trajectory: JointTrajectory
    terminal_trajectory: JointTrajectory
    combined_trajectory: JointTrajectory
    planner_status: str
    planner_timings_s: dict[str, float]
    curobo_version: str
    scene_revision: str
    planner_profile: str
    random_seed: int
    validation_status: str = "not_evaluated"
    executable: bool = False


@dataclass(frozen=True)
class PlanningFailure:
    """Expected planning infeasibility or invalid backend result."""

    category: str
    reason: str
    planner_status: str


@dataclass(frozen=True)
class PlanningOutcome:
    """Exactly one successful plan or structured failure."""

    plan: NominalPlan | None
    failure: PlanningFailure | None

    @property
    def succeeded(self) -> bool:
        return self.plan is not None and self.failure is None


class PlannerBackend(Protocol):
    """Minimal injected surface of cuRobo ``MotionPlanner``."""

    def plan_grasp(self, **kwargs: Any) -> Any:
        """Plan free-space approach and linear terminal segments."""

    def reset_seed(self) -> None:
        """Reset backend samplers to the configured reproducible seed."""

    def warmup(
        self,
        *,
        enable_graph: bool,
        num_warmup_iterations: int,
    ) -> bool:
        """Run public planner preconditioning."""


class PlannerTypesAdapter(Protocol):
    """Convert typed domain inputs into backend-specific objects."""

    def goal_set(self, goal_set: SurfaceGoalSet) -> Any:
        """Convert a surface goal set."""

    def joint_state(self, state: NamedJointState) -> Any:
        """Convert a current joint state."""


class CuroboTypesAdapter:
    """Public cuRoboV2 tensor/type conversion."""

    def goal_set(self, goal_set: SurfaceGoalSet) -> Any:
        return to_curobo_goal_tool_pose(goal_set)

    def joint_state(self, state: NamedJointState) -> Any:
        import torch
        from curobo.types import JointState

        position = torch.as_tensor(
            state.position_rad,
            device="cuda:0",
            dtype=torch.float32,
        ).unsqueeze(0)
        return JointState.from_position(position, joint_names=list(state.names))


def signed_pre_approach_offset_m(distance_m: float, tool_approach_sign: int) -> float:
    """Return cuRobo's tool-axis offset for a desired standoff distance."""

    distance = float(distance_m)
    if not math.isfinite(distance) or distance <= 0.0:
        raise ConfigurationError("pre-approach distance must be positive and finite")
    if tool_approach_sign not in (-1, 1):
        raise ConfigurationError("tool approach sign must be -1 or +1")
    return -float(tool_approach_sign) * distance


def load_planner_profile(
    name: str,
    path: Path | str = Path("config/planner_profiles.yml"),
) -> PlannerProfile:
    """Load one named, validated planner profile."""

    source = Path(path)
    if not source.is_file():
        raise ConfigurationError(f"planner profile config not found: {source}")
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    try:
        values = payload["profiles"][name]
    except (KeyError, TypeError) as exc:
        raise ConfigurationError(f"unknown planner profile: {name}") from exc
    profile = PlannerProfile(name=name, **values)
    numeric_positive = (
        profile.num_ik_seeds,
        profile.num_trajopt_seeds,
        profile.position_tolerance_m,
        profile.orientation_tolerance_rad,
        profile.optimizer_collision_activation_distance_m,
        profile.max_batch_size,
        profile.max_goalset,
        profile.warmup_iterations,
        profile.max_plan_grasp_attempts,
    )
    if any(float(value) <= 0.0 for value in numeric_positive):
        raise ConfigurationError(f"planner profile {name!r} contains non-positive values")
    if profile.random_seed < 0:
        raise ConfigurationError("planner random_seed must be non-negative")
    return profile


def _bool_tensor(value: Any) -> bool:
    if value is None:
        return False
    current = value
    if hasattr(current, "detach"):
        current = current.detach()
    if hasattr(current, "cpu"):
        current = current.cpu()
    if hasattr(current, "numpy"):
        current = current.numpy()
    array = np.asarray(current)
    return bool(array.size and np.any(array))


def _integer_scalar(value: Any, label: str) -> int:
    current = value
    if hasattr(current, "detach"):
        current = current.detach()
    if hasattr(current, "cpu"):
        current = current.cpu()
    if hasattr(current, "numpy"):
        current = current.numpy()
    array = np.asarray(current).reshape(-1)
    if array.size != 1:
        raise ConfigurationError(f"{label} must contain one value")
    return int(array[0])


class NominalPlanner:
    """Plan through a fresh cuRobo backend for every ``plan_grasp`` call.

    cuRobo v0.8.0 mutates optimizer/tool-criteria state during ``plan_grasp``.
    Reusing that backend produced shortened or failed trajectories in Phase 3
    GPU regression tests. The factory boundary deliberately trades construction
    and warmup latency for deterministic, fail-closed behavior.
    """

    def __init__(
        self,
        backend_factory: Callable[[], PlannerBackend],
        profile: PlannerProfile,
        *,
        task_frame_config: TaskFrameConfig,
        types_adapter: PlannerTypesAdapter | None = None,
        curobo_version: str = "0.8.0",
    ) -> None:
        self._backend_factory = backend_factory
        self.profile = profile
        self.task_frame_config = task_frame_config
        self._types = CuroboTypesAdapter() if types_adapter is None else types_adapter
        self.curobo_version = curobo_version

    def warmup(self, request: PlanningRequest) -> PlanningOutcome:
        """Warm the full ``plan_grasp`` path with a representative request.

        cuRobo v0.8.0's generic ``MotionPlanner.warmup`` does not exercise the
        same state transitions as ``plan_grasp``. Applications should discard
        this result and start benchmark timing only after it succeeds.
        """

        return self.plan(request)

    def plan(self, request: PlanningRequest) -> PlanningOutcome:
        """Plan with ``plan_grasp`` and map expected failures to result data."""

        if request.planner_profile != self.profile.name:
            raise ConfigurationError(
                f"request profile {request.planner_profile!r} != session {self.profile.name!r}"
            )
        if request.random_seed != self.profile.random_seed:
            raise ConfigurationError(
                "request random_seed must match the configured planner factory"
            )
        goal_set = build_surface_goal_set(request.surface_target, self.task_frame_config)
        if len(goal_set.candidates) > self.profile.max_goalset:
            raise ConfigurationError(
                f"goal set size {len(goal_set.candidates)} exceeds profile max_goalset "
                f"{self.profile.max_goalset}"
            )
        backend_goal = self._types.goal_set(goal_set)
        backend_state = self._types.joint_state(request.current_joint_state)
        offset = signed_pre_approach_offset_m(
            request.surface_target.pre_approach_distance_m,
            self.task_frame_config.tool_approach_sign,
        )
        started = time.perf_counter()
        raw: Any = None
        attempt_count = 0
        try:
            for attempt_count in range(1, self.profile.max_plan_grasp_attempts + 1):
                # Never issue a second plan_grasp call to the same v0.8.0
                # MotionPlanner. This also applies to retries after an
                # infeasible result. Generic warmup is also mandatory: Phase 4
                # endpoint validation showed an unwarmed fresh planner could
                # report success while leaving the terminal segment at the
                # pre-approach pose.
                backend = self._backend_factory()
                backend.reset_seed()
                backend.warmup(
                    enable_graph=self.profile.enable_graph_warmup,
                    num_warmup_iterations=self.profile.warmup_iterations,
                )
                backend.reset_seed()
                raw = backend.plan_grasp(
                    grasp_poses=backend_goal,
                    current_state=backend_state,
                    grasp_approach_axis=self.task_frame_config.tool_approach_axis,
                    grasp_approach_offset=offset,
                    grasp_approach_in_tool_frame=True,
                    plan_approach_to_grasp=True,
                    plan_grasp_to_lift=False,
                    disable_collision_links=[],
                )
                if raw is not None and _bool_tensor(getattr(raw, "success", None)):
                    break
        except (RuntimeError, ValueError) as exc:
            return PlanningOutcome(
                plan=None,
                failure=PlanningFailure(
                    category="backend_error",
                    reason=str(exc),
                    planner_status="exception",
                ),
            )
        elapsed = time.perf_counter() - started
        if raw is None:
            return self._failure("planner_returned_none", "plan_grasp returned None", "")
        status = str(getattr(raw, "status", "") or "")
        if not _bool_tensor(getattr(raw, "success", None)):
            return self._failure("planning_infeasible", "cuRobo reported no success", status)
        try:
            selected_index = _integer_scalar(
                getattr(raw, "goalset_index", None),
                "goalset_index",
            )
            selected_roll = goal_set.roll_for_goal_index(selected_index)
            approach = extract_curobo_trajectory(
                getattr(raw, "approach_interpolated_trajectory", None),
                getattr(raw, "approach_interpolated_last_tstep", None),
                expected_joint_names=JOINT_NAMES,
                label="approach",
            )
            terminal = extract_curobo_trajectory(
                getattr(raw, "grasp_interpolated_trajectory", None),
                getattr(raw, "grasp_interpolated_last_tstep", None),
                expected_joint_names=JOINT_NAMES,
                label="terminal",
            )
            combined = concatenate_trajectories(approach, terminal)
        except ConfigurationError as exc:
            return self._failure("invalid_planner_result", str(exc), status)
        backend_time = float(getattr(raw, "planning_time", 0.0) or 0.0)
        return PlanningOutcome(
            plan=NominalPlan(
                request_id=request.request_id,
                selected_goal_index=selected_index,
                selected_roll_rad=selected_roll,
                approach_trajectory=approach,
                terminal_trajectory=terminal,
                combined_trajectory=combined,
                planner_status=status,
                planner_timings_s={
                    "adapter_wall_time": elapsed,
                    "backend_planning_time": backend_time,
                    "attempt_count": float(attempt_count),
                },
                curobo_version=self.curobo_version,
                scene_revision=request.scene_revision,
                planner_profile=self.profile.name,
                random_seed=request.random_seed,
            ),
            failure=None,
        )

    @staticmethod
    def _failure(category: str, reason: str, status: str) -> PlanningOutcome:
        return PlanningOutcome(
            plan=None,
            failure=PlanningFailure(
                category=category,
                reason=reason,
                planner_status=status,
            ),
        )


def create_curobo_planner(
    profile: PlannerProfile,
    *,
    robot_config_path: Path | str = Path("config/robots/mycobot_280_m5.yml"),
    scene_config_path: Path | str | None = Path("config/scenes/empty.yml"),
    warmup: bool = False,
) -> Any:
    """Create one public cuRobo planner for exactly one ``plan_grasp`` call."""

    from curobo.motion_planner import MotionPlanner, MotionPlannerCfg

    scene_model: dict[str, Any] | None = None
    if scene_config_path is not None:
        scene_payload = yaml.safe_load(Path(scene_config_path).read_text(encoding="utf-8"))
        if any(bool(value) for value in scene_payload.values()):
            scene_model = scene_payload
    config = MotionPlannerCfg.create(
        robot=load_curobo_robot_config(robot_config_path),
        scene_model=scene_model,
        self_collision_check=profile.self_collision_check,
        num_ik_seeds=profile.num_ik_seeds,
        num_trajopt_seeds=profile.num_trajopt_seeds,
        position_tolerance=profile.position_tolerance_m,
        orientation_tolerance=profile.orientation_tolerance_rad,
        use_cuda_graph=profile.use_cuda_graph,
        random_seed=profile.random_seed,
        optimizer_collision_activation_distance=(
            profile.optimizer_collision_activation_distance_m
        ),
        max_batch_size=profile.max_batch_size,
        multi_env=profile.multi_env,
        max_goalset=profile.max_goalset,
    )
    planner = MotionPlanner(config)
    if warmup:
        planner.warmup(
            enable_graph=profile.enable_graph_warmup,
            num_warmup_iterations=profile.warmup_iterations,
        )
    return planner


def planner_profile_as_dict(profile: PlannerProfile) -> dict[str, Any]:
    """Stable serialization helper for reports and replay records."""

    return asdict(profile)
