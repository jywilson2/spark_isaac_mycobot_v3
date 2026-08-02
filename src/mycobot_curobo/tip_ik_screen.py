"""Optional tip-pose IK pre-screen for multi-target placement (cuRobo-owned)."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Protocol, Sequence

import numpy as np

from mycobot_curobo.cube_scene import CubeGeometry, cube_face_center, cubes_to_curobo_scene_dict
from mycobot_curobo.errors import ConfigurationError
from mycobot_curobo.frames import TaskFrameConfig
from mycobot_curobo.goal_set import build_surface_goal_set, to_curobo_goal_tool_pose
from mycobot_curobo.planner import PlannerProfile, create_curobo_planner
from mycobot_curobo.robot_model import JOINT_NAMES, TCP_LINK
from mycobot_curobo.targets import SurfaceTarget

LogFn = Callable[[str], None]


@dataclass(frozen=True)
class IkRejectionRecord:
    """One tip-IK rejection during suite target generation."""

    episode_index: int
    center_m: tuple[float, float, float]
    reason: str = "no_feasible_tip_ik"


@dataclass
class IkRejectionBudget:
    """Per-episode tip-IK reject-and-regenerate counter.

    Default max is ``target_count`` for the episode (set by the caller).
    """

    max_rejections: int
    episode_index: int = 0
    rejections: list[IkRejectionRecord] = field(default_factory=list)
    log: LogFn | None = None

    def __post_init__(self) -> None:
        if self.max_rejections < 0:
            raise ConfigurationError("max_ik_rejections must be non-negative")

    @property
    def count(self) -> int:
        return len(self.rejections)

    def reject(
        self, center_m: Sequence[float], *, reason: str = "no_feasible_tip_ik"
    ) -> None:
        array = np.asarray(center_m, dtype=float).reshape(3)
        if array.shape != (3,) or not np.all(np.isfinite(array)):
            raise ConfigurationError("center_m must contain three finite values")
        center = (float(array[0]), float(array[1]), float(array[2]))
        record = IkRejectionRecord(
            episode_index=int(self.episode_index),
            center_m=center,
            reason=str(reason),
        )
        self.rejections.append(record)
        if self.log is not None:
            self.log(
                "phase7_4_placement: "
                f"episode={record.episode_index} reject {record.reason} "
                f"centre=({center[0]:.4f},{center[1]:.4f},{center[2]:.4f}) "
                f"ik_count={self.count}/{self.max_rejections}"
            )
        if self.count > self.max_rejections:
            raise ConfigurationError(
                "episode target generation exceeded max_ik_rejections="
                f"{self.max_rejections} (episode={record.episode_index}, "
                f"centre={center}, running_count={self.count})"
            )


class TipIkScreen(Protocol):
    """Return True when flange-normal tip IK is feasible vs accepted cubes."""

    def __call__(
        self,
        center_m: tuple[float, float, float],
        accepted_centers_m: Sequence[tuple[float, float, float]] = (),
    ) -> bool:
        ...


@dataclass
class CuroboTipIkScreen:
    """World-aware tip IK via cuRobo ``MotionPlanner.ik_solver.solve_pose``.

    Screens against a caller-supplied obstacle set (draw-order accepted cubes
    during packing; full-field omit-self after pack). Rebuilds the warmed
    MotionPlanner when that set changes.
    """

    profile: PlannerProfile
    task_frame_config: TaskFrameConfig
    start_position_rad: tuple[float, ...]
    edge_m: float
    outward_normal_base: tuple[float, float, float]
    fixed_roll_rad: float | None
    roll_candidates_rad: tuple[float, ...]
    pre_approach_distance_m: float
    robot_config_path: Path | str = Path("config/robots/mycobot_280_m5.yml")
    _backend: Any = field(default=None, init=False, repr=False)
    _accepted_key: tuple[tuple[float, float, float], ...] = field(
        default=(), init=False, repr=False
    )

    def __post_init__(self) -> None:
        if len(self.start_position_rad) != len(JOINT_NAMES):
            raise ConfigurationError(
                "start_position_rad must match JOINT_NAMES length for tip IK screen"
            )

    def _scene_for_accepted(
        self, accepted_centers_m: Sequence[tuple[float, float, float]]
    ) -> dict[str, Any] | None:
        if not accepted_centers_m:
            return None
        geometries = tuple(
            CubeGeometry(
                center_m=(float(c[0]), float(c[1]), float(c[2])),
                edge_m=float(self.edge_m),
                name=f"ik_accepted_{index}",
            )
            for index, c in enumerate(accepted_centers_m)
        )
        return cubes_to_curobo_scene_dict(geometries)

    def _ensure_backend(
        self, accepted_centers_m: Sequence[tuple[float, float, float]]
    ) -> Any:
        key = tuple(
            (float(c[0]), float(c[1]), float(c[2])) for c in accepted_centers_m
        )
        if self._backend is not None and key == self._accepted_key:
            return self._backend
        self.close()
        scene_model = self._scene_for_accepted(key)
        if scene_model is None:
            backend = create_curobo_planner(
                self.profile,
                robot_config_path=self.robot_config_path,
                scene_model=None,
                warmup=True,
            )
        else:
            backend = create_curobo_planner(
                self.profile,
                robot_config_path=self.robot_config_path,
                scene_model=scene_model,
                warmup=True,
            )
        backend.reset_seed()
        self._backend = backend
        self._accepted_key = key
        return backend

    def __call__(
        self,
        center_m: tuple[float, float, float],
        accepted_centers_m: Sequence[tuple[float, float, float]] = (),
    ) -> bool:
        import torch
        from curobo.types import JointState

        face = cube_face_center(center_m, self.edge_m, self.outward_normal_base)
        normal = np.asarray(self.outward_normal_base, dtype=float)
        position = face - 0.002 * normal
        target = SurfaceTarget.create(
            target_id="ik_screen",
            position_base_m=position,
            surface_normal_base=self.outward_normal_base,
            fixed_roll_rad=self.fixed_roll_rad,
            roll_candidates_rad=(
                None if self.fixed_roll_rad is not None else self.roll_candidates_rad
            ),
            pre_approach_distance_m=self.pre_approach_distance_m,
            tool_frame=TCP_LINK,
        )
        goal_set = build_surface_goal_set(target, self.task_frame_config)
        goal = to_curobo_goal_tool_pose(goal_set)
        backend = self._ensure_backend(accepted_centers_m)
        position_js = torch.as_tensor(
            [list(self.start_position_rad)], device="cuda:0", dtype=torch.float32
        )
        current = JointState.from_position(position_js, joint_names=list(JOINT_NAMES))
        try:
            ik_result = backend.ik_solver.solve_pose(
                goal,
                return_seeds=max(1, int(self.profile.num_ik_seeds)),
                current_state=current,
            )
        except (RuntimeError, ValueError):
            return False
        success = getattr(ik_result, "success", None)
        if success is None:
            return False
        return bool(torch.count_nonzero(success).item() > 0)

    def close(self) -> None:
        backend = self._backend
        self._backend = None
        self._accepted_key = ()
        if backend is not None and hasattr(backend, "destroy"):
            backend.destroy()
