"""GPU smoke tests for the pinned cuRoboV2 public API surface."""

from __future__ import annotations

import importlib.util

import pytest

pytestmark = pytest.mark.gpu


@pytest.mark.skipif(
    importlib.util.find_spec("curobo") is None,
    reason="cuRobo v0.8.0 is not installed in the lightweight test environment",
)
def test_curobo_v080_public_imports() -> None:
    """Import required public symbols without constructing a planner."""

    from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
    from curobo.types import GoalToolPose, JointState, Pose

    assert all(
        symbol is not None
        for symbol in (
            MotionPlanner,
            MotionPlannerCfg,
            GoalToolPose,
            JointState,
            Pose,
        )
    )

