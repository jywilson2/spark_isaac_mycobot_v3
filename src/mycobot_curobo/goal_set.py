"""Ordered goal-set construction and public cuRobo conversion."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from mycobot_curobo.errors import ConfigurationError
from mycobot_curobo.frames import (
    TaskFrameCandidate,
    TaskFrameConfig,
    build_task_frame_candidates,
)
from mycobot_curobo.targets import SurfaceTarget


@dataclass(frozen=True)
class SurfaceGoalSet:
    """Task-frame candidates with stable goal-index-to-roll mapping."""

    target_id: str
    tool_frame: str
    candidates: tuple[TaskFrameCandidate, ...]
    roll_by_goal_index_rad: tuple[float, ...]

    def roll_for_goal_index(self, index: int) -> float:
        """Return a selected roll, rejecting padded/out-of-range indices."""

        if index < 0 or index >= len(self.roll_by_goal_index_rad):
            raise ConfigurationError(
                f"goal index {index} outside [0, {len(self.roll_by_goal_index_rad)})"
            )
        return self.roll_by_goal_index_rad[index]


def build_surface_goal_set(
    target: SurfaceTarget,
    config: TaskFrameConfig = TaskFrameConfig(),
) -> SurfaceGoalSet:
    """Build deterministic candidates and retain the roll-index mapping."""

    candidates = build_task_frame_candidates(target, config)
    if not candidates:
        raise ConfigurationError("surface goal set must contain at least one candidate")
    return SurfaceGoalSet(
        target_id=target.target_id,
        tool_frame=target.tool_frame,
        candidates=candidates,
        roll_by_goal_index_rad=tuple(candidate.roll_rad for candidate in candidates),
    )


def to_curobo_goal_tool_pose(
    goal_set: SurfaceGoalSet,
    *,
    device: str = "cuda:0",
    dtype: Any | None = None,
) -> Any:
    """Convert a domain goal set using cuRobo v0.8.0 public types.

    Imports are local so Phase 2 geometry/unit tests remain CPU-only.
    """

    import torch
    from curobo.types import GoalToolPose, Pose

    torch_dtype = torch.float32 if dtype is None else dtype
    positions = torch.as_tensor(
        np.stack([candidate.position_base_m for candidate in goal_set.candidates]),
        device=device,
        dtype=torch_dtype,
    )
    quaternions = torch.as_tensor(
        np.stack([candidate.quaternion_wxyz for candidate in goal_set.candidates]),
        device=device,
        dtype=torch_dtype,
    )
    poses = {
        goal_set.tool_frame: Pose(
            position=positions,
            quaternion=quaternions,
        )
    }
    return GoalToolPose.from_poses(poses, num_goalset=len(goal_set.candidates))
