"""GPU acceptance test for Phase 3 public ``plan_grasp`` integration."""

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
def test_reachable_plan_grasp_returns_two_reproducible_normal_line_segments() -> None:
    profile = load_planner_profile("benchmark_reproducible")
    backend_count = 0

    def backend_factory():
        nonlocal backend_count
        backend_count += 1
        return create_curobo_planner(profile, warmup=False)

    planner = NominalPlanner(
        backend_factory,
        profile,
        task_frame_config=TaskFrameConfig(),
    )

    model = load_robot_model_spec()
    known_goal_q = np.array([0.3, -0.1, 0.1, 0.0, 0.0, 0.0])
    known_pose = forward_kinematics(known_goal_q, spec=model)
    rotation = _quaternion_to_rotation(known_pose.quaternion_wxyz)
    target = SurfaceTarget.create(
        position_base_m=known_pose.position_m,
        surface_normal_base=rotation[:, 2],
        tangent_hint_base=rotation[:, 0],
        fixed_roll_rad=0.0,
        pre_approach_distance_m=0.01,
        target_id="phase3-known-reachable",
    )
    request = PlanningRequest(
        current_joint_state=NamedJointState.create(JOINT_NAMES, np.zeros(6)),
        surface_target=target,
        scene_revision="empty-v1",
        planner_profile=profile.name,
        random_seed=profile.random_seed,
        request_id="phase3-gpu",
    )

    warmup = planner.warmup(request)
    measured = planner.plan(request)

    assert warmup.succeeded and measured.succeeded
    assert backend_count == 2
    assert warmup.plan is not None and measured.plan is not None
    assert measured.plan.selected_goal_index == 0
    assert measured.plan.selected_roll_rad == 0.0
    assert measured.plan.approach_trajectory.sample_count > 1
    assert measured.plan.terminal_trajectory.sample_count > 1
    assert measured.plan.executable is False
    assert np.allclose(
        warmup.plan.approach_trajectory.position_rad,
        measured.plan.approach_trajectory.position_rad,
        atol=1.0e-5,
    )
    warmup_endpoint = forward_kinematics(
        warmup.plan.terminal_trajectory.position_rad[-1],
        spec=model,
    )
    measured_endpoint = forward_kinematics(
        measured.plan.terminal_trajectory.position_rad[-1],
        spec=model,
    )
    assert (
        float(np.linalg.norm(measured_endpoint.position_m - target.position_base_m))
        <= profile.position_tolerance_m
    )
    assert np.allclose(
        warmup_endpoint.position_m,
        measured_endpoint.position_m,
        atol=1.0e-4,
    )
    assert (
        abs(
            float(
                np.dot(
                    warmup_endpoint.quaternion_wxyz,
                    measured_endpoint.quaternion_wxyz,
                )
            )
        )
        > 1.0 - 1.0e-5
    )

    tcp_positions = np.stack(
        [
            forward_kinematics(position, spec=model).position_m
            for position in measured.plan.terminal_trajectory.position_rad
        ]
    )
    approach_direction = -target.surface_normal_base
    displacements = tcp_positions - target.position_base_m
    axial = displacements @ approach_direction
    lateral = displacements - axial[:, None] * approach_direction
    assert float(np.max(np.linalg.norm(lateral, axis=1))) <= 0.005
