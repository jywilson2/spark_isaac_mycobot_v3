"""GPU acceptance test for independent Phase 4 validation."""

from __future__ import annotations

import importlib.util

import numpy as np
import pytest

from mycobot_curobo.frames import TaskFrameConfig
from mycobot_curobo.planner import (
    NamedJointState,
    NominalPlanner,
    PlanningRequest,
    create_curobo_planner,
    load_planner_profile,
)
from mycobot_curobo.robot_model import (
    JOINT_NAMES,
    forward_kinematics,
    load_robot_model_spec,
)
from mycobot_curobo.targets import SurfaceTarget
from mycobot_curobo.validation import (
    CuroboTrajectoryEvaluator,
    load_validation_profile,
    validate_nominal_plan,
)

pytestmark = pytest.mark.gpu


def _runtime_available() -> bool:
    if importlib.util.find_spec("curobo") is None or importlib.util.find_spec("torch") is None:
        return False
    import torch

    return bool(torch.cuda.is_available())


def _quaternion_to_rotation(quaternion_wxyz: np.ndarray) -> np.ndarray:
    w, x, y, z = quaternion_wxyz
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


@pytest.mark.skipif(not _runtime_available(), reason="cuRobo v0.8.0 CUDA runtime required")
def test_nominal_plan_passes_independent_fk_limits_and_collision_validation() -> None:
    planner_profile = load_planner_profile("benchmark_reproducible")
    task_config = TaskFrameConfig()
    robot_spec = load_robot_model_spec()
    known_goal_q = np.array([0.3, -0.1, 0.1, 0.0, 0.0, 0.0])
    known_pose = forward_kinematics(known_goal_q, spec=robot_spec)
    rotation = _quaternion_to_rotation(known_pose.quaternion_wxyz)
    target = SurfaceTarget.create(
        position_base_m=known_pose.position_m,
        surface_normal_base=rotation[:, 2],
        tangent_hint_base=rotation[:, 0],
        fixed_roll_rad=0.0,
        pre_approach_distance_m=0.01,
        target_id="phase4-known-reachable",
    )
    request = PlanningRequest(
        current_joint_state=NamedJointState.create(JOINT_NAMES, np.zeros(6)),
        surface_target=target,
        scene_revision="empty-v1",
        planner_profile=planner_profile.name,
        random_seed=planner_profile.random_seed,
        request_id="phase4-gpu",
    )
    nominal_planner = NominalPlanner(
        lambda: create_curobo_planner(planner_profile, warmup=False),
        planner_profile,
        task_frame_config=task_config,
    )

    outcome = nominal_planner.plan(request)

    assert outcome.succeeded
    assert outcome.plan is not None
    validation_backend = create_curobo_planner(planner_profile, warmup=False)
    result = validate_nominal_plan(
        outcome.plan,
        request,
        profile=load_validation_profile("simulation_initial"),
        evaluator=CuroboTrajectoryEvaluator(validation_backend, scene_is_empty=True),
        robot_spec=robot_spec,
        task_frame_config=task_config,
    )

    assert result.report.valid, result.report.violations
    assert result.validation_status == "valid"
    assert result.executable
    assert result.report.metrics.minimum_self_collision_clearance_m is not None
    assert result.report.metrics.minimum_self_collision_clearance_m >= 0.0
