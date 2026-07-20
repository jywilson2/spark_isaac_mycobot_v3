#!/usr/bin/env python3
"""Plan and independently validate Phase 7.2 multi-target episodes (no Isaac Kit)."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict, replace
from enum import Enum
from pathlib import Path
from typing import Any

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
for candidate in (REPO_ROOT, REPO_ROOT / "src"):
    if str(candidate) not in sys.path:
        sys.path.insert(0, str(candidate))

from mycobot_curobo.config import load_app_config  # noqa: E402
from mycobot_curobo.cube_scene import batch_sphere_cube_clearance_m  # noqa: E402
from mycobot_curobo.errors import ConfigurationError  # noqa: E402
from mycobot_curobo.multi_target import (  # noqa: E402
    MultiTargetEpisodeRunner,
    OptimisticTipContactDetector,
    aggregate_multi_target_results,
    format_episode_console_row,
    format_suite_summary,
    load_multi_target_suite_config,
    override_suite_target_count,
    sample_multi_target_episodes,
    serialize_episode,
    suite_acceptance_passed,
)
from mycobot_curobo.planner import (  # noqa: E402
    NominalPlan,
    NominalPlanner,
    PlanningRequest,
    create_curobo_planner,
    load_planner_profile,
)
from mycobot_curobo.robot_model import load_robot_model_spec  # noqa: E402
from mycobot_curobo.validation import (  # noqa: E402
    CuroboTrajectoryEvaluator,
    ValidatedPlan,
    load_validation_profile,
    validate_nominal_plan,
)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=REPO_ROOT / "config/phase7_2_multi_target.yml",
    )
    parser.add_argument("--episodes", type=int, default=None)
    parser.add_argument(
        "--targets",
        type=int,
        default=None,
        help="Override config target_count (positive). Manual lists shorter than N "
        "switch to grid placement in the configured field AABB.",
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


def _multi_cube_clearance_fn(geometries: tuple[Any, ...]):
    def clearance(spheres: np.ndarray) -> np.ndarray:
        if not geometries:
            return np.full(spheres.shape[0], np.finfo(float).max, dtype=float)
        clearances = [
            batch_sphere_cube_clearance_m(spheres, geometry.center_m, geometry.edge_m)
            for geometry in geometries
        ]
        return np.min(np.vstack(clearances), axis=0)

    return clearance


def plan_and_validate(
    episodes: tuple[Any, ...],
    *,
    validation_profile_name: str,
    warn_planning_duration_s: float | None,
) -> tuple[tuple[Any, ...], dict[str, Any]]:
    """Run the multi-target runner with optimistic tip contact (planning process)."""

    app = load_app_config()
    base_profile = load_planner_profile(episodes[0].planner_profile)
    validation_profile = load_validation_profile(validation_profile_name)
    robot_spec = load_robot_model_spec(app.robot_config_path)
    trajectories: dict[str, Any] = {}

    def planner_factory(seed: int, scene_model: dict[str, Any], links: tuple[str, ...]) -> Any:
        del links
        profile = replace(base_profile, random_seed=seed)
        empty_world = not scene_model.get("cuboid")

        def backend_factory(model=scene_model, prof=profile, empty=empty_world):
            if empty:
                return create_curobo_planner(
                    prof,
                    robot_config_path=app.robot_config_path,
                    scene_config_path=REPO_ROOT / "config/scenes/empty.yml",
                )
            return create_curobo_planner(
                prof,
                robot_config_path=app.robot_config_path,
                scene_model=model,
            )

        return NominalPlanner(backend_factory, profile, task_frame_config=app.task_frame)

    def validator(
        plan: NominalPlan, request: PlanningRequest, clearance_geometries: tuple[Any, ...]
    ) -> ValidatedPlan:
        evaluator_backend = create_curobo_planner(
            replace(base_profile, random_seed=request.random_seed),
            robot_config_path=app.robot_config_path,
            scene_config_path=REPO_ROOT / "config/scenes/empty.yml",
        )
        # World clearance uses only non-contact cubes; tip may occupy the goal face.
        return validate_nominal_plan(
            plan,
            request,
            profile=validation_profile,
            evaluator=CuroboTrajectoryEvaluator(
                evaluator_backend,
                scene_is_empty=len(clearance_geometries) == 0,
                world_clearance_fn=(
                    None
                    if not clearance_geometries
                    else _multi_cube_clearance_fn(clearance_geometries)
                ),
            ),
            robot_spec=robot_spec,
            task_frame_config=app.task_frame,
        )

    def plan_sink(plan: NominalPlan) -> None:
        trajectories[plan.request_id] = plan.combined_trajectory

    runner = MultiTargetEpisodeRunner(
        planner_factory=planner_factory,
        validator=validator,
        contact_detector_factory=lambda _episode, to_id: OptimisticTipContactDetector(to_id),
        plan_sink=plan_sink,
        warn_planning_duration_s=warn_planning_duration_s,
    )
    results = runner.run(episodes)
    return results, trajectories


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.episodes is not None and args.episodes <= 0:
        raise ConfigurationError("--episodes must be a positive integer")
    if args.targets is not None and args.targets <= 0:
        raise ConfigurationError("--targets must be a positive integer")
    config = load_multi_target_suite_config(args.config)
    if args.targets is not None:
        before = config
        config = override_suite_target_count(config, args.targets)
        if config.placement is not before.placement:
            print(
                f"phase7_2_plan: --targets {args.targets} exceeded manual list "
                f"({len(before.manual_targets)}); using grid placement",
                flush=True,
            )
    episodes = sample_multi_target_episodes(
        config, root_seed=config.root_seed, episode_count=args.episodes
    )
    results, trajectories = plan_and_validate(
        episodes,
        validation_profile_name=config.validation_profile,
        warn_planning_duration_s=config.warn_planning_duration_s,
    )
    summary = aggregate_multi_target_results(results, root_seed=config.root_seed)
    for result in results:
        print(format_episode_console_row(result, count=len(results)), flush=True)
    print(format_suite_summary(summary), flush=True)
    payload = {
        "schema_version": 1,
        "root_seed": config.root_seed,
        "tip_allow_link_names": list(config.tip_allow_link_names),
        "retain_targets_after_contact": config.retain_targets_after_contact,
        "lighting": config.lighting,
        "summary": asdict(summary),
        "results": [asdict(result) for result in results],
        "frozen_requests": [serialize_episode(result.episode) for result in results],
        "trajectories": {
            request_id: _serialize_trajectory(trajectory)
            for request_id, trajectory in trajectories.items()
        },
    }
    args.output_bundle.parent.mkdir(parents=True, exist_ok=True)
    args.output_bundle.write_text(
        json.dumps(
            payload,
            indent=2,
            sort_keys=True,
            default=lambda value: value.value if isinstance(value, Enum) else value,
        )
        + "\n",
        encoding="utf-8",
    )
    accepted = suite_acceptance_passed(summary, max_failed_episodes=config.max_failed_episodes)
    fully_succeeded = summary.successes == summary.total_episodes
    print(
        json.dumps(
            {
                "bundle": str(args.output_bundle),
                "episodes": len(results),
                "suite_accepted": accepted,
                "fully_succeeded": fully_succeeded,
                "failed_episodes": summary.failed_episodes,
                "max_failed_episodes": config.max_failed_episodes,
                "total_planning_failures": summary.total_planning_failures,
                "total_target_failures": summary.total_target_failures,
            }
        ),
        flush=True,
    )
    # Playback bundles require every episode to succeed so trajectories exist.
    # Suite acceptance (failed-episode budget) is reported separately.
    return 0 if fully_succeeded else 1


if __name__ == "__main__":
    raise SystemExit(main())
