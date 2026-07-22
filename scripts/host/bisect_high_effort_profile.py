#!/usr/bin/env python3
"""One-knob bisect of planning_high_effort vs benchmark on a 1×2 packing-safe field."""

from __future__ import annotations

import json
import sys
import time
from dataclasses import replace
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
for candidate in (REPO_ROOT, REPO_ROOT / "src"):
    if str(candidate) not in sys.path:
        sys.path.insert(0, str(candidate))


def main() -> int:
    import numpy as np

    from mycobot_curobo.config import load_app_config
    from mycobot_curobo.cube_scene import (
        batch_sphere_cube_clearance_m,
        cubes_to_curobo_scene_dict,
    )
    from mycobot_curobo.frames import TaskFrameConfig
    from mycobot_curobo.multi_target import (
        load_multi_target_suite_config,
        sample_multi_target_episodes,
    )
    from mycobot_curobo.planner import (
        NamedJointState,
        NominalPlanner,
        PlannerProfile,
        PlanningRequest,
        create_curobo_planner,
        load_planner_profile,
    )
    from mycobot_curobo.robot_model import JOINT_NAMES
    from mycobot_curobo.validation import (
        CuroboTrajectoryEvaluator,
        load_validation_profile,
        validate_nominal_plan,
    )

    suite_path = Path("/tmp/he_bisect/suite.yml")
    if not suite_path.is_file():
        raise SystemExit(f"missing suite config {suite_path}")
    suite = load_multi_target_suite_config(suite_path)
    episodes = sample_multi_target_episodes(suite, root_seed=suite.root_seed, episode_count=1)
    episode = episodes[0]
    # Contact order from shuffle — plan first two legs: start->first, first->second
    order = episode.field.contact_order_ids
    print(f"bisect: order={order} seed={episode.episode_seed}", flush=True)

    app = load_app_config()
    path = REPO_ROOT / "config/planner_profiles.yml"
    bench = load_planner_profile("benchmark_reproducible", path)
    variants: dict[str, PlannerProfile] = {
        "benchmark_reproducible": bench,
        "he_ik64": replace(bench, name="he_ik64", num_ik_seeds=64),
        "he_traj8": replace(bench, name="he_traj8", num_trajopt_seeds=8),
        "he_attempts4": replace(bench, name="he_attempts4", max_plan_grasp_attempts=4),
        "he_ik64_traj8": replace(
            bench, name="he_ik64_traj8", num_ik_seeds=64, num_trajopt_seeds=8
        ),
        "he_full": replace(
            bench,
            name="he_full",
            num_ik_seeds=64,
            num_trajopt_seeds=8,
            max_plan_grasp_attempts=4,
        ),
        "planning_high_effort": load_planner_profile("planning_high_effort", path),
    }
    validation = load_validation_profile(suite.validation_profile)
    task_frame = TaskFrameConfig()
    empty = REPO_ROOT / "config/scenes/empty.yml"
    summary: dict[str, object] = {}

    for name, profile in variants.items():
        print(f"=== {name} ===", flush=True)
        joints = tuple(float(v) for v in episode.start_position_rad)
        removed: set[str] = set()
        leg_rows: list[dict[str, object]] = []
        from_id = "start"
        ok_all = True
        for to_id in order:
            target = episode.field.target_by_id(to_id)
            geometries = episode.field.active_geometries(removed_ids=frozenset(removed))
            planning_geometries = tuple(
                g for g in geometries if g.name != target.cube_geometry.name
            )
            scene_model = cubes_to_curobo_scene_dict(planning_geometries)
            prof = replace(profile, random_seed=episode.episode_seed)
            request = PlanningRequest(
                current_joint_state=NamedJointState.create(JOINT_NAMES, joints),
                surface_target=target.to_surface_target(),
                scene_revision=f"bisect-{name}-{from_id}-{to_id}",
                planner_profile=prof.name,
                random_seed=prof.random_seed,
                request_id=f"{name}_{from_id}_to_{to_id}",
            )

            def factory(model: dict = scene_model, p: PlannerProfile = prof) -> object:
                if not model.get("cuboid"):
                    return create_curobo_planner(
                        p, robot_config_path=app.robot_config_path, scene_config_path=empty
                    )
                return create_curobo_planner(
                    p, robot_config_path=app.robot_config_path, scene_model=model
                )

            planner = NominalPlanner(factory, prof, task_frame_config=task_frame)
            started = time.perf_counter()
            outcome = planner.plan(request)
            elapsed = time.perf_counter() - started
            row: dict[str, object] = {
                "from_id": from_id,
                "to_id": to_id,
                "planning_succeeded": bool(outcome.succeeded),
                "planning_duration_s": elapsed,
                "planner_status": (
                    ""
                    if outcome.failure is None
                    else outcome.failure.planner_status
                    if outcome.plan is None
                    else outcome.plan.planner_status
                ),
                "failure_reason": None if outcome.failure is None else outcome.failure.reason,
                "validation_passed": False,
            }
            if outcome.succeeded and outcome.plan is not None:
                row["planner_status"] = outcome.plan.planner_status
                clearance = tuple(g for g in geometries if g.name != target.cube_geometry.name)
                evaluator_backend = create_curobo_planner(
                    replace(prof, random_seed=request.random_seed),
                    robot_config_path=app.robot_config_path,
                    scene_config_path=empty,
                )

                def clearance_fn(spheres: np.ndarray, geos=clearance) -> np.ndarray:
                    if not geos:
                        return np.full(spheres.shape[0], np.finfo(float).max)
                    vals = [
                        batch_sphere_cube_clearance_m(spheres, g.center_m, g.edge_m) for g in geos
                    ]
                    return np.min(np.vstack(vals), axis=0)

                validated = validate_nominal_plan(
                    outcome.plan,
                    request,
                    profile=validation,
                    evaluator=CuroboTrajectoryEvaluator(
                        evaluator_backend,
                        scene_is_empty=len(clearance) == 0,
                        world_clearance_fn=None if not clearance else clearance_fn,
                    ),
                    robot_spec=__import__(
                        "mycobot_curobo.robot_model", fromlist=["load_robot_model_spec"]
                    ).load_robot_model_spec(app.robot_config_path),
                    task_frame_config=task_frame,
                )
                row["validation_passed"] = bool(validated.report.valid)
                if validated.report.valid:
                    joints = tuple(
                        float(v) for v in outcome.plan.combined_trajectory.position_rad[-1]
                    )
                    removed.add(to_id)
                    from_id = to_id
                else:
                    ok_all = False
                    row["failure_reason"] = "; ".join(
                        v.reason for v in validated.report.violations
                    )
            else:
                ok_all = False
            leg_rows.append(row)
            print(
                f"  {row['from_id']}->{to_id} "
                f"plan={row['planning_succeeded']} valid={row['validation_passed']} "
                f"t={elapsed:.2f} status={row['planner_status']!r} "
                f"reason={row['failure_reason']!r}",
                flush=True,
            )
            if not row["planning_succeeded"] or not row["validation_passed"]:
                ok_all = False
                break
        summary[name] = {"succeeded": ok_all, "legs": leg_rows}
        print(f"  => episode_ok={ok_all}", flush=True)

    out = Path("/tmp/he_bisect/bisect_summary.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"bisect: wrote {out}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
