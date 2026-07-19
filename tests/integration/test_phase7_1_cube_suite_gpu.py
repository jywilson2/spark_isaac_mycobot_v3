"""GPU acceptance coverage for Phase 7.1 cube-scene planning and validation."""

from __future__ import annotations

import importlib.util
from dataclasses import replace

import numpy as np
import pytest

from mycobot_curobo.cube_scene import cube_to_curobo_scene_dict
from mycobot_curobo.cube_suite import (
    SafeNest,
    StartMode,
    load_cube_suite_config,
    sample_cube_episodes,
)
from mycobot_curobo.frames import TaskFrameConfig
from mycobot_curobo.planner import (
    NamedJointState,
    NominalPlanner,
    RelocationPlanner,
    create_curobo_planner,
    load_planner_profile,
    plan_joint_relocation,
)
from mycobot_curobo.robot_model import JOINT_NAMES, load_robot_model_spec
from mycobot_curobo.validation import (
    CuroboTrajectoryEvaluator,
    load_validation_profile,
    validate_nominal_plan,
    validate_start_state,
)

pytestmark = pytest.mark.gpu


def _runtime_available() -> bool:
    if importlib.util.find_spec("curobo") is None or importlib.util.find_spec("torch") is None:
        return False
    import torch

    return bool(torch.cuda.is_available())


def _clear_start_episode(config):
    """Pick a seeded episode whose start clears the cube for Mode A planning."""

    for seed in range(config.root_seed, config.root_seed + 32):
        for episode in sample_cube_episodes(config, root_seed=seed, episode_count=1):
            profile = replace(
                load_planner_profile(config.planner_profile), random_seed=episode.planner_seed
            )
            scene_model = cube_to_curobo_scene_dict(episode.cube_geometry)
            report = validate_start_state(
                episode.start_position_rad,
                robot_spec=load_robot_model_spec(),
                evaluator=CuroboTrajectoryEvaluator(
                    create_curobo_planner(profile, scene_model=scene_model, warmup=False),
                    scene_is_empty=False,
                    cube_center_m=episode.cube_center_m,
                    cube_edge_m=episode.cube_edge_m,
                ),
                minimum_self_collision_clearance_m=config.minimum_self_collision_clearance_m,
                minimum_world_collision_clearance_m=config.minimum_world_collision_clearance_m,
            )
            if report.valid:
                return episode
    raise AssertionError("unable to sample a start-clear Phase 7.1 episode")


@pytest.mark.skipif(not _runtime_available(), reason="cuRobo v0.8.0 CUDA runtime required")
def test_phase7_1_cube_scene_plan_validates_with_evaluated_world_clearance() -> None:
    config = load_cube_suite_config()
    episode = _clear_start_episode(config)
    profile = replace(
        load_planner_profile(config.planner_profile), random_seed=episode.planner_seed
    )
    task_config = TaskFrameConfig()
    robot_spec = load_robot_model_spec()
    scene_model = cube_to_curobo_scene_dict(episode.cube_geometry)
    request = episode.to_planning_request()

    start_report = validate_start_state(
        request.current_joint_state.position_rad,
        robot_spec=robot_spec,
        evaluator=CuroboTrajectoryEvaluator(
            create_curobo_planner(profile, scene_model=scene_model, warmup=False),
            scene_is_empty=False,
            cube_center_m=episode.cube_center_m,
            cube_edge_m=episode.cube_edge_m,
        ),
        minimum_self_collision_clearance_m=config.minimum_self_collision_clearance_m,
        minimum_world_collision_clearance_m=config.minimum_world_collision_clearance_m,
    )
    assert start_report.valid, start_report.violations

    planner = NominalPlanner(
        lambda: create_curobo_planner(profile, scene_model=scene_model, warmup=False),
        profile,
        task_frame_config=task_config,
    )
    outcome = planner.plan(request)
    assert outcome.succeeded, outcome.failure
    assert outcome.plan is not None

    validated = validate_nominal_plan(
        outcome.plan,
        request,
        profile=load_validation_profile(config.validation_profile),
        evaluator=CuroboTrajectoryEvaluator(
            create_curobo_planner(profile, scene_model=scene_model, warmup=False),
            scene_is_empty=False,
            cube_center_m=episode.cube_center_m,
            cube_edge_m=episode.cube_edge_m,
        ),
        robot_spec=robot_spec,
        task_frame_config=task_config,
    )
    assert validated.report.valid, validated.report.violations
    assert validated.executable
    assert validated.report.metrics.minimum_world_collision_clearance_m is not None
    assert np.isfinite(validated.report.metrics.minimum_world_collision_clearance_m)


@pytest.mark.skipif(not _runtime_available(), reason="cuRobo v0.8.0 CUDA runtime required")
def test_phase7_1_mode_c_relocation_uses_curobo_plan_cspace() -> None:
    config = load_cube_suite_config()
    episode = replace(
        _clear_start_episode(config),
        start_mode=StartMode.C,
        start_label="forward_left",
        start_position_rad=(0.35, -0.45, 0.70, 0.10, 0.35, -0.20),
        safe_nest=SafeNest("safe_nest", config.safe_nest.position_rad),
        request_id="phase7_1-gpu-mode-c",
    )
    profile = replace(
        load_planner_profile(config.planner_profile), random_seed=episode.planner_seed
    )
    scene_model = cube_to_curobo_scene_dict(episode.cube_geometry)
    relocator = RelocationPlanner(
        lambda: create_curobo_planner(profile, scene_model=scene_model, warmup=False),
        profile,
    )
    relocation = plan_joint_relocation(
        relocator,
        NamedJointState.create(JOINT_NAMES, episode.safe_nest.position_rad),
        NamedJointState.create(JOINT_NAMES, episode.start_position_rad),
    )
    assert hasattr(relocation, "position_rad"), relocation
    assert relocation.sample_count >= 2
    assert relocation.joint_names == JOINT_NAMES
