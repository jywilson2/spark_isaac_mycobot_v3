"""GPU acceptance tests for the Phase 1 cuRobo robot model."""

from __future__ import annotations

import importlib.util

import numpy as np
import pytest

from mycobot_curobo.robot_model import (
    JOINT_NAMES,
    TCP_LINK,
    forward_kinematics,
    load_curobo_robot_config,
    load_robot_model_spec,
)

pytestmark = pytest.mark.gpu


def _runtime_available() -> bool:
    if importlib.util.find_spec("curobo") is None or importlib.util.find_spec("torch") is None:
        return False
    import torch

    return bool(torch.cuda.is_available())


@pytest.mark.skipif(not _runtime_available(), reason="cuRobo v0.8.0 CUDA runtime required")
def test_robot_config_warmup_fk_and_self_collision() -> None:
    """Construct/warm the public planner and compare default FK to CPU reference."""

    from curobo.motion_planner import MotionPlanner, MotionPlannerCfg

    planner_config = MotionPlannerCfg.create(
        robot=load_curobo_robot_config(),
        self_collision_check=True,
        num_ik_seeds=8,
        num_trajopt_seeds=2,
        use_cuda_graph=False,
        random_seed=123,
        max_goalset=1,
    )
    planner = MotionPlanner(planner_config)

    assert planner.joint_names == list(JOINT_NAMES)
    assert planner.tool_frames == [TCP_LINK]
    assert planner.warmup(enable_graph=False, num_warmup_iterations=1) is True

    self_collision = planner.kinematics.get_self_collision_config()
    assert self_collision.collision_pairs.shape[0] > 0

    state = planner.default_joint_state.clone().unsqueeze(0)
    curobo_pose = planner.compute_kinematics(state).tool_poses.to_dict()[TCP_LINK]
    reference = forward_kinematics(
        load_robot_model_spec().default_joint_position_rad,
        spec=load_robot_model_spec(),
    )
    assert np.allclose(
        curobo_pose.position.detach().cpu().numpy()[0],
        reference.position_m,
        atol=1.0e-6,
    )
    curobo_quaternion = curobo_pose.quaternion.detach().cpu().numpy()[0]
    assert abs(float(np.dot(curobo_quaternion, reference.quaternion_wxyz))) > 1.0 - 1.0e-6
