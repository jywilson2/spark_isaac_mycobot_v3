#!/usr/bin/env python3
"""Plan and independently validate Phase 7.1 cube episodes (no Isaac Kit imports)."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict, replace
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
for candidate in (REPO_ROOT, REPO_ROOT / "src"):
    if str(candidate) not in sys.path:
        sys.path.insert(0, str(candidate))

from mycobot_curobo.benchmark import (  # noqa: E402
    FailureCategory,
    planning_failure_category,
    validation_failure_category,
)
from mycobot_curobo.config import load_app_config  # noqa: E402
from mycobot_curobo.cube_scene import cube_to_curobo_scene_dict  # noqa: E402
from mycobot_curobo.cube_suite import (  # noqa: E402
    CubeEpisode,
    CubeEpisodeResult,
    StartMode,
    format_episode_console_row,
    load_cube_suite_config,
    sample_cube_episodes,
    serialize_episode,
)
from mycobot_curobo.errors import ConfigurationError  # noqa: E402
from mycobot_curobo.planner import (  # noqa: E402
    NamedJointState,
    NominalPlanner,
    RelocationPlanner,
    create_curobo_planner,
    load_planner_profile,
    plan_joint_relocation,
)
from mycobot_curobo.robot_model import JOINT_NAMES, load_robot_model_spec  # noqa: E402
from mycobot_curobo.trajectory import concatenate_trajectories  # noqa: E402
from mycobot_curobo.validation import (  # noqa: E402
    CuroboTrajectoryEvaluator,
    load_validation_profile,
    validate_nominal_plan,
    validate_start_state,
)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=REPO_ROOT / "config/phase7_1_cube_suite.yml",
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--all-modes",
        action="store_true",
        help="Cycle start modes A/B/C (acceptance schedule).",
    )
    mode.add_argument(
        "--chained",
        action="store_true",
        help="Mode B only: continue from last success (no home reset).",
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=None,
        help="Override config episode_count (positive integer).",
    )
    parser.add_argument("--output-bundle", type=Path, required=True)
    return parser.parse_args(argv)


def _serialize_trajectory(trajectory: Any) -> dict[str, Any]:
    return {
        "joint_names": list(trajectory.joint_names),
        "dt_s": float(trajectory.dt_s),
        "position_rad": trajectory.position_rad.tolist(),
        "velocity_rad_s": (
            None if trajectory.velocity_rad_s is None else trajectory.velocity_rad_s.tolist()
        ),
        "acceleration_rad_s2": (
            None
            if trajectory.acceleration_rad_s2 is None
            else trajectory.acceleration_rad_s2.tolist()
        ),
        "jerk_rad_s3": (
            None if trajectory.jerk_rad_s3 is None else trajectory.jerk_rad_s3.tolist()
        ),
    }


def plan_and_validate(
    episodes: tuple[CubeEpisode, ...],
    validation_profile_name: str,
    *,
    chained_failure_policy: str,
) -> tuple[tuple[CubeEpisodeResult, ...], dict[str, Any]]:
    """Run a fresh cube-scene cuRobo planner and independent validator per episode."""

    app = load_app_config()
    base_profile = load_planner_profile(episodes[0].planner_profile)
    validation_profile = load_validation_profile(validation_profile_name)
    robot_spec = load_robot_model_spec(app.robot_config_path)
    results: list[CubeEpisodeResult] = []
    trajectories: dict[str, Any] = {}
    last_success: tuple[float, ...] | None = None
    for episode in episodes:
        current_episode = episode
        if episode.start_mode is StartMode.B:
            if last_success is not None:
                current_episode = replace(
                    episode,
                    start_label="chained_last_success",
                    start_position_rad=last_success,
                )
            elif chained_failure_policy == "terminate":
                results.append(
                    CubeEpisodeResult(
                        episode,
                        False,
                        False,
                        FailureCategory.CONFIGURATION_MODEL_FAILURE,
                        "Mode B terminated without a prior successful endpoint",
                        "chained_terminate",
                    )
                )
                break
        profile = replace(base_profile, random_seed=current_episode.planner_seed)
        request = current_episode.to_planning_request()
        scene_model = cube_to_curobo_scene_dict(current_episode.cube_geometry)
        try:
            start_evaluator = CuroboTrajectoryEvaluator(
                create_curobo_planner(
                    profile,
                    robot_config_path=app.robot_config_path,
                    scene_model=scene_model,
                    warmup=False,
                ),
                scene_is_empty=False,
                cube_center_m=current_episode.cube_center_m,
                cube_edge_m=current_episode.cube_edge_m,
            )
            start_report = validate_start_state(
                request.current_joint_state.position_rad,
                robot_spec=robot_spec,
                evaluator=start_evaluator,
            )
            if not start_report.valid:
                results.append(
                    CubeEpisodeResult(
                        current_episode,
                        False,
                        False,
                        FailureCategory.CONFIGURATION_MODEL_FAILURE,
                        "; ".join(item.reason for item in start_report.violations),
                        "start_preflight",
                    )
                )
                continue
            relocation = None
            if current_episode.start_mode is StartMode.C:
                if current_episode.safe_nest is None:
                    raise ConfigurationError("Mode C requires a configured safe nest")
                scene_for_factory = scene_model
                profile_for_factory = profile
                relocation_planner = RelocationPlanner(
                    lambda scene=scene_for_factory, prof=profile_for_factory: (
                        create_curobo_planner(
                            prof,
                            robot_config_path=app.robot_config_path,
                            scene_model=scene,
                        )
                    ),
                    profile,
                )
                relocation = plan_joint_relocation(
                    relocation_planner,
                    NamedJointState.create(JOINT_NAMES, current_episode.safe_nest.position_rad),
                    NamedJointState.create(JOINT_NAMES, current_episode.start_position_rad),
                )
                if not hasattr(relocation, "position_rad"):
                    results.append(
                        CubeEpisodeResult(
                            current_episode,
                            False,
                            False,
                            FailureCategory.TRAJECTORY_OPTIMIZATION_FAILURE,
                            getattr(relocation, "reason", "cuRobo relocation failed"),
                            getattr(relocation, "planner_status", "relocation_failed"),
                        )
                    )
                    continue
                request = replace(
                    request,
                    current_joint_state=NamedJointState.create(
                        JOINT_NAMES, current_episode.safe_nest.position_rad
                    ),
                )
            scene_for_factory = scene_model
            profile_for_factory = profile

            def _approach_backend(scene=scene_for_factory, prof=profile_for_factory):
                return create_curobo_planner(
                    prof,
                    robot_config_path=app.robot_config_path,
                    scene_model=scene,
                )

            planner = NominalPlanner(
                _approach_backend,
                profile,
                task_frame_config=app.task_frame,
            )
            outcome = planner.plan(request)
            if not outcome.succeeded or outcome.plan is None:
                failure = outcome.failure
                results.append(
                    CubeEpisodeResult(
                        current_episode,
                        False,
                        False,
                        (
                            FailureCategory.TRAJECTORY_OPTIMIZATION_FAILURE
                            if failure is None
                            else planning_failure_category(failure)
                        ),
                        None if failure is None else failure.reason,
                        "" if failure is None else failure.planner_status,
                    )
                )
                continue
            evaluator_backend = create_curobo_planner(
                profile, robot_config_path=app.robot_config_path, scene_model=scene_model
            )
            validated = validate_nominal_plan(
                outcome.plan,
                request,
                profile=validation_profile,
                evaluator=CuroboTrajectoryEvaluator(
                    evaluator_backend,
                    scene_is_empty=False,
                    cube_center_m=current_episode.cube_center_m,
                    cube_edge_m=current_episode.cube_edge_m,
                ),
                robot_spec=robot_spec,
                task_frame_config=app.task_frame,
            )
            valid = validated.report.valid and validated.executable
            if valid:
                trajectories[current_episode.request_id] = (
                    outcome.plan.combined_trajectory
                    if relocation is None
                    else concatenate_trajectories(relocation, outcome.plan.combined_trajectory)
                )
                last_success = tuple(
                    float(value) for value in outcome.plan.combined_trajectory.position_rad[-1]
                )
            results.append(
                CubeEpisodeResult(
                    current_episode,
                    True,
                    valid,
                    None if valid else validation_failure_category(validated.report.violations),
                    None
                    if valid
                    else "; ".join(item.reason for item in validated.report.violations),
                    outcome.plan.planner_status,
                    validated.report.metrics,
                    final_joint_position_rad=(
                        tuple(
                            float(value)
                            for value in outcome.plan.combined_trajectory.position_rad[-1]
                        )
                        if valid
                        else None
                    ),
                )
            )
        except (ConfigurationError, RuntimeError, ValueError) as exc:
            results.append(
                CubeEpisodeResult(
                    current_episode,
                    False,
                    False,
                    FailureCategory.CONFIGURATION_MODEL_FAILURE,
                    str(exc),
                    "exception",
                )
            )
    return tuple(results), trajectories


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.episodes is not None and args.episodes <= 0:
        raise ConfigurationError("--episodes must be a positive integer")
    config = load_cube_suite_config(args.config)
    if args.all_modes:
        force_modes: tuple[str, ...] | None = ("A", "B", "C", "D")
    elif args.chained:
        force_modes = ("B", "D")
    else:
        force_modes = None
    episodes = sample_cube_episodes(
        config,
        root_seed=config.root_seed,
        episode_count=args.episodes,
        force_modes=force_modes,
    )
    results, trajectories = plan_and_validate(
        episodes,
        config.validation_profile,
        chained_failure_policy=config.chained_failure_policy.value,
    )
    for result in results:
        print(format_episode_console_row(result, count=len(results)), flush=True)
    payload = {
        "schema_version": 1,
        "root_seed": config.root_seed,
        "max_isaac_prohibited_contacts": config.max_isaac_prohibited_contacts,
        "lighting": config.lighting,
        "results": [asdict(result) for result in results],
        "frozen_requests": [serialize_episode(result.episode) for result in results],
        "trajectories": {
            request_id: _serialize_trajectory(trajectory)
            for request_id, trajectory in trajectories.items()
        },
    }
    args.output_bundle.parent.mkdir(parents=True, exist_ok=True)
    args.output_bundle.write_text(
        json.dumps(payload, indent=2, sort_keys=True, default=str) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({"bundle": str(args.output_bundle), "episodes": len(results)}), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
