"""GPU planning smoke with Option B dual overlay trial-armed."""

from __future__ import annotations

import importlib.util
from dataclasses import replace
from pathlib import Path

import pytest
import yaml

from mycobot_curobo.cube_scene import cube_to_curobo_scene_dict, cubes_to_curobo_scene_dict
from mycobot_curobo.cube_suite import load_cube_suite_config, sample_cube_episodes
from mycobot_curobo.frames import TaskFrameConfig
from mycobot_curobo.multi_target import (
    load_multi_target_suite_config,
    sample_multi_target_episodes,
)
from mycobot_curobo.planner import (
    NamedJointState,
    NominalPlanner,
    PlanningRequest,
    create_curobo_planner,
    load_planner_profile,
)
from mycobot_curobo.planning_world import leg_world_geometries, leg_world_scene_dict
from mycobot_curobo.robot_model import JOINT_NAMES, load_robot_model_spec
from mycobot_curobo.validation import (
    CuroboTrajectoryEvaluator,
    load_validation_profile,
    validate_nominal_plan,
    validate_start_state,
)

pytestmark = pytest.mark.gpu

ROOT = Path(__file__).resolve().parents[2]
ROBOT = ROOT / "config" / "robots" / "mycobot_280_m5.yml"
OVERLAY = ROOT / "config" / "robots" / "mycobot_280_m5_phase1_1_spheres.yml"
SUITE_2X5 = ROOT / "config" / "phase7_2_multi_target_integration_2x5.yml"


def _runtime_available() -> bool:
    if importlib.util.find_spec("curobo") is None or importlib.util.find_spec("torch") is None:
        return False
    import torch

    return bool(torch.cuda.is_available())


def _trial_dual() -> Path:
    trial = ROOT / "config" / "robots" / "_tmp_overlay_option_b_plan.yml"
    payload = yaml.safe_load(ROBOT.read_text(encoding="utf-8"))
    kin = payload["robot_cfg"]["kinematics"]
    kin["collision_sphere_overlay_path"] = "config/robots/mycobot_280_m5_phase1_1_spheres.yml"
    kin["collision_sphere_overlay_role"] = "dual"
    trial.write_text(yaml.safe_dump(payload), encoding="utf-8")
    return trial


@pytest.mark.skipif(not _runtime_available(), reason="cuRobo v0.8.0 CUDA runtime required")
def test_option_b_dual_phase7_1_tip_plan_succeeds() -> None:
    if not OVERLAY.is_file():
        pytest.skip("Phase 1.1 overlay missing")
    trial = _trial_dual()
    try:
        config = load_cube_suite_config()
        episode = sample_cube_episodes(config, root_seed=config.root_seed, episode_count=1)[0]
        profile = replace(
            load_planner_profile(config.planner_profile), random_seed=episode.planner_seed
        )
        tip_scene = leg_world_scene_dict(
            (episode.cube_geometry,), active_contact_name=episode.cube_geometry.name
        )
        scene_with_cube = cube_to_curobo_scene_dict(episode.cube_geometry)
        spec = load_robot_model_spec(trial)
        request = episode.to_planning_request()
        start = validate_start_state(
            request.current_joint_state.position_rad,
            robot_spec=spec,
            evaluator=CuroboTrajectoryEvaluator(
                create_curobo_planner(
                    profile, robot_config_path=trial, scene_model=scene_with_cube, warmup=False
                ),
                scene_is_empty=False,
                cube_center_m=episode.cube_center_m,
                cube_edge_m=episode.cube_edge_m,
            ),
            minimum_self_collision_clearance_m=config.minimum_self_collision_clearance_m,
            minimum_world_collision_clearance_m=config.minimum_world_collision_clearance_m,
        )
        assert start.valid, start.violations
        planner = NominalPlanner(
            lambda: create_curobo_planner(
                profile, robot_config_path=trial, scene_model=tip_scene, warmup=False
            ),
            profile,
            task_frame_config=TaskFrameConfig(),
        )
        outcome = planner.plan(request)
        assert outcome.succeeded, outcome.failure
        assert outcome.plan is not None
        validated = validate_nominal_plan(
            outcome.plan,
            request,
            profile=load_validation_profile(config.validation_profile),
            evaluator=CuroboTrajectoryEvaluator(
                create_curobo_planner(
                    profile, robot_config_path=trial, scene_model=tip_scene, warmup=False
                ),
                scene_is_empty=True,
            ),
            robot_spec=spec,
            task_frame_config=TaskFrameConfig(),
        )
        assert validated.report.valid, validated.report.violations
    finally:
        trial.unlink(missing_ok=True)


@pytest.mark.skipif(not _runtime_available(), reason="cuRobo v0.8.0 CUDA runtime required")
def test_option_b_dual_phase7_2_first_leg_plans() -> None:
    if not OVERLAY.is_file() or not SUITE_2X5.is_file():
        pytest.skip("overlay or 2x5 suite missing")
    trial = _trial_dual()
    try:
        suite = load_multi_target_suite_config(SUITE_2X5)
        episode = sample_multi_target_episodes(suite, root_seed=4242, episode_count=1)[0]
        first_id = episode.field.contact_order_ids[0]
        target = episode.field.target_by_id(first_id)
        remaining = episode.field.active_geometries()
        planning = leg_world_geometries(remaining, active_contact_name=target.cube_geometry.name)
        scene = cubes_to_curobo_scene_dict(planning)
        profile = replace(
            load_planner_profile(suite.planner_profile), random_seed=episode.episode_seed
        )
        request = PlanningRequest(
            current_joint_state=NamedJointState.create(JOINT_NAMES, episode.start_position_rad),
            surface_target=target.to_surface_target(),
            scene_revision=f"{episode.scene_revision_prefix}-option-b",
            planner_profile=suite.planner_profile,
            random_seed=episode.episode_seed,
            request_id=f"option_b_{first_id}",
            disable_collision_links=(),
        )
        planner = NominalPlanner(
            lambda: create_curobo_planner(
                profile, robot_config_path=trial, scene_model=scene, warmup=False
            ),
            profile,
            task_frame_config=TaskFrameConfig(),
        )
        outcome = planner.plan(request)
        assert outcome.succeeded, outcome.failure
        assert outcome.plan is not None

        def _world_clearance(spheres_xyzwr):
            import numpy as np

            from mycobot_curobo.cube_scene import batch_sphere_cube_clearance_m

            clearances = [
                batch_sphere_cube_clearance_m(spheres_xyzwr, cube.center_m, cube.edge_m)
                for cube in planning
            ]
            return np.min(np.stack(clearances, axis=0), axis=0)

        validated = validate_nominal_plan(
            outcome.plan,
            request,
            profile=load_validation_profile(suite.validation_profile),
            evaluator=CuroboTrajectoryEvaluator(
                create_curobo_planner(
                    profile, robot_config_path=trial, scene_model=scene, warmup=False
                ),
                scene_is_empty=len(planning) == 0,
                world_clearance_fn=None if not planning else _world_clearance,
            ),
            robot_spec=load_robot_model_spec(trial),
            task_frame_config=TaskFrameConfig(),
        )
        assert validated.report.valid, validated.report.violations
    finally:
        trial.unlink(missing_ok=True)
