#!/usr/bin/env python3
"""Diagnose Option A armed-overlay start/end collisions against target cubes.

Reproduces the Phase 7.1 / 7.2 GPU planning regression with the Option A
(full-replace) overlay and reports which spheres / links / cubes penetrate.
Run on the host:

  ./scripts/host/spark_host_exec.sh python \\
    scripts/host/diagnose_phase1_1_option_a_regression.py
"""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from mycobot_curobo.cube_scene import (  # noqa: E402
    batch_sphere_cube_clearance_m,
    cube_to_curobo_scene_dict,
    cubes_to_curobo_scene_dict,
    sphere_aabb_clearance_m,
)
from mycobot_curobo.cube_suite import load_cube_suite_config, sample_cube_episodes  # noqa: E402
from mycobot_curobo.multi_target import (  # noqa: E402
    load_multi_target_suite_config,
    sample_multi_target_episodes,
)
from mycobot_curobo.planner import (  # noqa: E402
    NominalPlanner,
    create_curobo_planner,
    load_planner_profile,
)
from mycobot_curobo.robot_model import (  # noqa: E402
    JOINT_NAMES,
    load_curobo_robot_config,
    load_robot_model_spec,  # noqa: E402
)
from mycobot_curobo.validation import (  # noqa: E402
    CuroboTrajectoryEvaluator,
    validate_start_state,
)

ROBOT = ROOT / "config" / "robots" / "mycobot_280_m5.yml"
TRIAL = ROOT / "config" / "robots" / "_tmp_diagnose_option_a.yml"
SUITE_2X5 = ROOT / "config" / "phase7_2_multi_target_integration_2x5.yml"


def _trial_option_a() -> Path:
    payload = yaml.safe_load(ROBOT.read_text(encoding="utf-8"))
    kin = payload["robot_cfg"]["kinematics"]
    kin["collision_sphere_overlay_path"] = "config/robots/mycobot_280_m5_phase1_1_spheres.yml"
    # Force historical Option A full-replace for diagnosis (default role is dual).
    kin["collision_sphere_overlay_role"] = "replace"
    TRIAL.write_text(yaml.safe_dump(payload), encoding="utf-8")
    return TRIAL


def _sphere_link_offsets(kinematics: dict) -> list[tuple[str, int, int]]:
    offsets: list[tuple[str, int, int]] = []
    cursor = 0
    for link in kinematics["collision_link_names"]:
        count = len(kinematics["collision_spheres"][link])
        offsets.append((str(link), cursor, count))
        cursor += count
    return offsets


def _worst_sphere_hits(
    spheres_xyzwr: np.ndarray,
    cube_center: tuple[float, float, float],
    cube_edge: float,
    offsets: list[tuple[str, int, int]],
    top_n: int = 8,
) -> list[tuple[str, int, float]]:
    half = (cube_edge * 0.5,) * 3
    centers = spheres_xyzwr[:, :3]
    radii = spheres_xyzwr[:, 3]
    hits: list[tuple[str, int, float]] = []
    for link, start, count in offsets:
        for local in range(count):
            idx = start + local
            clear = float(
                sphere_aabb_clearance_m(
                    centers[idx : idx + 1], radii[idx : idx + 1], cube_center, half
                )
            )
            if clear < 0.0:
                hits.append((link, local, clear))
    hits.sort(key=lambda item: item[2])
    return hits[:top_n]


def _fk_spheres(planner, joints: np.ndarray) -> np.ndarray:
    import torch
    from curobo.types import JointState

    state = JointState.from_position(
        torch.as_tensor(joints.reshape(1, -1), device="cuda:0", dtype=torch.float32),
        joint_names=list(JOINT_NAMES),
    )
    kin = planner.compute_kinematics(state)
    return kin.robot_spheres.detach().cpu().numpy().reshape(-1, 4)


def diagnose_7_1(trial: Path, offsets: list[tuple[str, int, int]]) -> None:
    print("\n=== Phase 7.1 (single cube stays in planning world) ===")
    config = load_cube_suite_config()
    episode = sample_cube_episodes(config, root_seed=config.root_seed, episode_count=1)[0]
    profile = replace(
        load_planner_profile(config.planner_profile), random_seed=episode.planner_seed
    )
    scene = cube_to_curobo_scene_dict(episode.cube_geometry)
    planner = create_curobo_planner(
        profile, robot_config_path=trial, scene_model=scene, warmup=False
    )
    spec = load_robot_model_spec(trial)
    start_q = np.asarray(episode.start_position_rad, dtype=float)
    report = validate_start_state(
        start_q,
        robot_spec=spec,
        evaluator=CuroboTrajectoryEvaluator(
            planner,
            scene_is_empty=False,
            cube_center_m=episode.cube_center_m,
            cube_edge_m=episode.cube_edge_m,
        ),
        minimum_self_collision_clearance_m=config.minimum_self_collision_clearance_m,
        minimum_world_collision_clearance_m=config.minimum_world_collision_clearance_m,
    )
    spheres = _fk_spheres(planner, start_q)
    start_clear = float(
        batch_sphere_cube_clearance_m(
            spheres.reshape(1, -1, 4), episode.cube_center_m, episode.cube_edge_m
        )[0]
    )
    print(
        f"mode={episode.start_mode} seed={episode.planner_seed} "
        f"start_valid={report.valid} clearance={start_clear:.6f} m"
    )
    if not report.valid:
        print(f"  violations: {report.violations}")
    print(
        "  worst start hits:",
        _worst_sphere_hits(spheres, episode.cube_center_m, episode.cube_edge_m, offsets),
    )

    from mycobot_curobo.frames import TaskFrameConfig

    task = TaskFrameConfig()
    nominal = NominalPlanner(
        backend_factory=lambda: create_curobo_planner(
            profile, robot_config_path=trial, scene_model=scene, warmup=False
        ),
        profile=profile,
        task_frame_config=task,
    )
    request = episode.to_planning_request()
    outcome = nominal.plan(request)
    if outcome.succeeded and outcome.plan is not None:
        print(f"  plan success=True status={outcome.plan.planner_status}")
        end_q = np.asarray(outcome.plan.combined_trajectory.position_rad[-1], dtype=float)
        end_spheres = _fk_spheres(planner, end_q)
        end_clear = float(
            batch_sphere_cube_clearance_m(
                end_spheres.reshape(1, -1, 4), episode.cube_center_m, episode.cube_edge_m
            )[0]
        )
        print(f"  end clearance={end_clear:.6f} m")
        print(
            "  worst end hits:",
            _worst_sphere_hits(end_spheres, episode.cube_center_m, episode.cube_edge_m, offsets),
        )
    else:
        failure = outcome.failure
        print(
            f"  plan success=False category={getattr(failure, 'category', None)} "
            f"reason={getattr(failure, 'reason', None)} "
            f"status={getattr(failure, 'planner_status', None)}"
        )


def diagnose_7_2(trial: Path, offsets: list[tuple[str, int, int]]) -> None:
    print("\n=== Phase 7.2 integration 2x5 (active omitted; neighbors remain) ===")
    suite = load_multi_target_suite_config(SUITE_2X5)
    episode = sample_multi_target_episodes(suite, root_seed=4242, episode_count=1)[0]
    profile = load_planner_profile(suite.planner_profile)
    first_id = episode.field.contact_order_ids[0]
    target = episode.field.target_by_id(first_id)
    remaining = episode.field.active_geometries()
    planning = [g for g in remaining if g.name != target.cube_geometry.name]
    scene = cubes_to_curobo_scene_dict(planning)
    planner = create_curobo_planner(
        profile, robot_config_path=trial, scene_model=scene, warmup=False
    )
    start_q = np.asarray(episode.start_position_rad, dtype=float)
    spheres = _fk_spheres(planner, start_q)
    print(f"active={first_id} neighbors={len(planning)}")
    for geom in planning:
        clear = float(
            batch_sphere_cube_clearance_m(spheres.reshape(1, -1, 4), geom.center_m, geom.edge_m)[0]
        )
        hits = _worst_sphere_hits(spheres, geom.center_m, geom.edge_m, offsets, top_n=4)
        flag = "HIT" if clear < 0.0 else "ok"
        print(f"  start vs {geom.name}: {flag} clearance={clear:.6f} m hits={hits}")

    spec = load_robot_model_spec(trial)
    report = validate_start_state(
        start_q,
        robot_spec=spec,
        evaluator=CuroboTrajectoryEvaluator(planner, scene_is_empty=len(planning) == 0),
        minimum_self_collision_clearance_m=suite.minimum_self_collision_clearance_m,
        minimum_world_collision_clearance_m=suite.minimum_world_collision_clearance_m,
    )
    print(f"start_valid={report.valid} violations={report.violations}")


def main() -> int:
    import torch

    if not torch.cuda.is_available():
        print("ERROR: CUDA required", file=sys.stderr)
        return 2
    trial = _trial_option_a()
    try:
        cfg = load_curobo_robot_config(trial)
        kin = cfg["robot_cfg"]["kinematics"]
        offsets = _sphere_link_offsets(kin)
        n = sum(c for _, _, c in offsets)
        print(f"Option A trial: {n} spheres on {len(offsets)} links (full replace)")
        diagnose_7_1(trial, offsets)
        diagnose_7_2(trial, offsets)
    finally:
        trial.unlink(missing_ok=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
