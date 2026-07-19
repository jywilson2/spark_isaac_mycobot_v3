"""GPU conversion test for Phase 2 public cuRobo goal-set types."""

from __future__ import annotations

import importlib.util

import pytest

from mycobot_curobo.goal_set import (
    build_surface_goal_set,
    to_curobo_goal_tool_pose,
)
from mycobot_curobo.targets import SurfaceTarget

pytestmark = pytest.mark.gpu


def _runtime_available() -> bool:
    if importlib.util.find_spec("curobo") is None or importlib.util.find_spec("torch") is None:
        return False
    import torch

    return bool(torch.cuda.is_available())


@pytest.mark.skipif(not _runtime_available(), reason="cuRobo v0.8.0 CUDA runtime required")
def test_multiple_rolls_convert_to_public_goal_tool_pose() -> None:
    target = SurfaceTarget.create(
        position_base_m=[0.12, -0.04, 0.23],
        surface_normal_base=[0.0, 0.0, 1.0],
        tangent_hint_base=[1.0, 0.0, 0.0],
        roll_candidates_rad=[0.0, 0.5, 1.0],
        pre_approach_distance_m=0.05,
        target_id="gpu-goalset",
    )
    domain_goal_set = build_surface_goal_set(target)

    goal = to_curobo_goal_tool_pose(domain_goal_set)

    assert goal.tool_frames == ["tcp_link"]
    assert goal.batch_size == 1
    assert goal.horizon == 1
    assert goal.num_links == 1
    assert goal.num_goalset == 3
    assert tuple(goal.position.shape) == (1, 1, 1, 3, 3)
    assert tuple(goal.quaternion.shape) == (1, 1, 1, 3, 4)
