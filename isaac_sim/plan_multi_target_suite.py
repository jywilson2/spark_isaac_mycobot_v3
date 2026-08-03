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
from mycobot_curobo.cube_scene import (  # noqa: E402
    CubeGeometry,
    batch_sphere_cube_clearance_m,
    flange_disk_cube_clearance_m,
    flange_disk_face_overhang_m,
)
from mycobot_curobo.errors import ConfigurationError  # noqa: E402
from mycobot_curobo.incremental_population import (  # noqa: E402
    IncrementalPopulationRunner,
    format_suite_table,
    format_z_dist_header,
)
from mycobot_curobo.multi_target import (  # noqa: E402
    MultiTargetEpisode,
    MultiTargetEpisodeRunner,
    OptimisticTipContactDetector,
    TargetPopulation,
    aggregate_multi_target_results,
    format_episode_console_row,
    format_suite_summary,
    load_multi_target_suite_config,
    override_suite_target_count,
    regenerate_episode_field,
    resolve_invocation_root_seed,
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
from mycobot_curobo.tip_ik_screen import CuroboTipIkScreen  # noqa: E402
from mycobot_curobo.validation import (  # noqa: E402
    CuroboTrajectoryEvaluator,
    ValidatedPlan,
    ValidationReport,
    ValidationViolation,
    load_validation_profile,
    validate_nominal_plan,
    validate_retreat_segment,
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
    parser.add_argument(
        "--root-seed",
        type=int,
        default=None,
        help="Suite root seed for reproducible placement and episode planner "
        "seeds (episode_seed = root_seed + 1009*(i+1)). Omit to draw a "
        "distinct random seed for each episode (maximizes coverage). "
        "YAML root_seed is not used unless this flag is set to that value.",
    )
    parser.add_argument(
        "--app-config",
        type=Path,
        default=None,
        help="Optional app.yml override (e.g. trial-armed Option B robot path). "
        "Defaults to config/app.yml.",
    )
    parser.add_argument("--output-bundle", type=Path, required=True)
    parser.add_argument(
        "--skip-physx-gate",
        action="store_true",
        help="Skip post-episode PhysX accept/regen (not for Phase 7.5 host smokes).",
    )
    parser.add_argument(
        "--usd",
        type=Path,
        default=None,
        help="Prepared robot USD for the PhysX gate (defaults to prepared MyCobot USD).",
    )
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


def _build_planner_and_validator(
    *,
    planner_profile_name: str,
    validation_profile_name: str,
    minimum_self_collision_clearance_m: float,
    minimum_world_collision_clearance_m: float,
    flange_diameter_assumption_m: float | None,
    require_flange_face_containment: bool,
    flange_face_overhang_tolerance_m: float,
    app_config_path: Path | None,
) -> tuple[Any, Any, Any]:
    """Return ``(app, planner_factory, validator)`` shared by fixed/incremental hosts."""

    app = load_app_config() if app_config_path is None else load_app_config(app_config_path)
    base_profile = load_planner_profile(planner_profile_name)
    validation_profile = replace(
        load_validation_profile(validation_profile_name),
        minimum_self_collision_clearance_m=float(minimum_self_collision_clearance_m),
        minimum_world_collision_clearance_m=float(minimum_world_collision_clearance_m),
    )
    robot_spec = load_robot_model_spec(app.robot_config_path)
    flange_diameter = (
        None if flange_diameter_assumption_m is None else float(flange_diameter_assumption_m)
    )
    overhang_tol = float(flange_face_overhang_tolerance_m)

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
        plan: NominalPlan,
        request: PlanningRequest,
        clearance_geometries: tuple[Any, ...],
        contact_cube: CubeGeometry,
    ) -> ValidatedPlan:
        evaluator_backend = create_curobo_planner(
            replace(base_profile, random_seed=request.random_seed),
            robot_config_path=app.robot_config_path,
            scene_config_path=REPO_ROOT / "config/scenes/empty.yml",
        )
        evaluator = CuroboTrajectoryEvaluator(
            evaluator_backend,
            scene_is_empty=len(clearance_geometries) == 0,
            world_clearance_fn=(
                None
                if not clearance_geometries
                else _multi_cube_clearance_fn(clearance_geometries)
            ),
        )
        # World clearance uses only non-contact cubes; tip may occupy the goal face.
        validated = validate_nominal_plan(
            plan,
            request,
            profile=validation_profile,
            evaluator=evaluator,
            robot_spec=robot_spec,
            task_frame_config=app.task_frame,
        )
        violations: list[ValidationViolation] = []
        if (
            validated.report.valid
            and plan.retreat_trajectory is not None
            and request.plan_grasp_to_lift
        ):
            violations.extend(
                validate_retreat_segment(
                    plan.retreat_trajectory,
                    request_id=request.request_id,
                    profile=validation_profile,
                    evaluator=evaluator,
                    robot_spec=robot_spec,
                    contact_terminal=plan.terminal_trajectory,
                )
            )
            if violations:
                report = ValidationReport(
                    request_id=request.request_id,
                    profile_name=validated.report.profile_name,
                    valid=False,
                    violations=validated.report.violations + tuple(violations),
                    metrics=validated.report.metrics,
                )
                return ValidatedPlan(
                    nominal_plan=plan,
                    report=report,
                    validation_status="invalid",
                    executable=False,
                )
        if not validated.report.valid or flange_diameter is None:
            return validated

        # Transit anti-graze: flange-radius sphere at every TCP vs neighbor cubes.
        # Score approach+contact only; retreat is validated separately for world
        # clearance and must not dilute the contact-path flange check.
        if clearance_geometries:
            if plan.retreat_trajectory is not None:
                from mycobot_curobo.trajectory import concatenate_trajectories

                transit_positions = concatenate_trajectories(
                    plan.approach_trajectory, plan.terminal_trajectory
                ).position_rad
            else:
                transit_positions = plan.combined_trajectory.position_rad
            geom_all = evaluator.evaluate(transit_positions)
            min_clear = float("inf")
            min_wp = 0
            min_name = clearance_geometries[0].name
            for wp_index, tcp in enumerate(geom_all.tcp_position_m):
                for cube in clearance_geometries:
                    clear = flange_disk_cube_clearance_m(tcp, flange_diameter, cube)
                    if clear < min_clear:
                        min_clear = float(clear)
                        min_wp = int(wp_index)
                        min_name = cube.name
            # Penetration-only for the conservative flange-sphere model; robot
            # sphere world clearance already enforces the suite margin.
            world_tol = 0.0
            if min_clear < world_tol:
                violations.append(
                    ValidationViolation(
                        metric="flange_neighbor_clearance",
                        waypoint_index=min_wp,
                        measured=min_clear,
                        threshold=world_tol,
                        reason=(
                            "flange disk grazes/penetrates a non-contact target "
                            f"(cube={min_name}; clearance_m={min_clear:.4f}; "
                            f"tol_m={world_tol:.4f}; flange_diameter_m={flange_diameter})"
                        ),
                    )
                )

        if require_flange_face_containment:
            geometry = evaluator.evaluate(plan.terminal_trajectory.position_rad)
            tcp = geometry.tcp_position_m[-1]
            overhang = flange_disk_face_overhang_m(
                tcp,
                request.surface_target.surface_normal_base,
                flange_diameter,
                contact_cube,
            )
            if overhang > overhang_tol:
                violations.append(
                    ValidationViolation(
                        metric="flange_face_containment",
                        waypoint_index=int(plan.terminal_trajectory.sample_count - 1),
                        measured=float(overhang),
                        threshold=overhang_tol,
                        reason=(
                            "flange disk overhangs the contact face "
                            f"(overhang_m={overhang:.4f}; tol_m={overhang_tol:.4f}; "
                            f"edge_m={contact_cube.edge_m}; "
                            f"flange_diameter_m={flange_diameter})"
                        ),
                    )
                )

        if not violations:
            return validated
        report = ValidationReport(
            request_id=request.request_id,
            profile_name=validated.report.profile_name,
            valid=False,
            violations=validated.report.violations + tuple(violations),
            metrics=validated.report.metrics,
        )
        return ValidatedPlan(
            nominal_plan=plan,
            report=report,
            validation_status="invalid",
            executable=False,
        )

    return app, planner_factory, validator


def plan_and_validate(
    episodes: tuple[Any, ...],
    *,
    validation_profile_name: str,
    warn_planning_duration_s: float | None,
    minimum_self_collision_clearance_m: float = 0.0,
    minimum_world_collision_clearance_m: float = 0.0,
    flange_diameter_assumption_m: float | None = None,
    require_flange_face_containment: bool = False,
    flange_face_overhang_tolerance_m: float = 0.005,
    app_config_path: Path | None = None,
    regenerate_field: Any | None = None,
    max_field_regenerations: int = 0,
) -> tuple[tuple[Any, ...], dict[str, Any]]:
    """Run the multi-target runner with optimistic tip contact (planning process)."""

    _app, planner_factory, validator = _build_planner_and_validator(
        planner_profile_name=episodes[0].planner_profile,
        validation_profile_name=validation_profile_name,
        minimum_self_collision_clearance_m=minimum_self_collision_clearance_m,
        minimum_world_collision_clearance_m=minimum_world_collision_clearance_m,
        flange_diameter_assumption_m=flange_diameter_assumption_m,
        require_flange_face_containment=require_flange_face_containment,
        flange_face_overhang_tolerance_m=flange_face_overhang_tolerance_m,
        app_config_path=app_config_path,
    )
    trajectories: dict[str, Any] = {}

    def plan_sink(plan: NominalPlan) -> None:
        trajectories[plan.request_id] = plan.combined_trajectory

    runner = MultiTargetEpisodeRunner(
        planner_factory=planner_factory,
        validator=validator,
        contact_detector_factory=lambda _episode, to_id: OptimisticTipContactDetector(to_id),
        plan_sink=plan_sink,
        warn_planning_duration_s=warn_planning_duration_s,
    )
    results = runner.run(
        episodes,
        regenerate_field=regenerate_field,
        max_field_regenerations=max_field_regenerations,
    )
    return results, trajectories


def plan_incremental(
    config: Any,
    *,
    root_seed: int | None,
    episode_count: int | None,
    independent_random_episode_seeds: bool,
    app_config_path: Path | None = None,
    enable_physx_gate: bool = True,
    usd: Path | None = None,
) -> Any:
    """Run Phase 7.5 incremental population with optimistic tip contact.

    When ``enable_physx_gate`` is true and ``max_physx_regenerations > 0``, each
    populated episode is headless-PhysX-smoked before acceptance (host subprocess).
    """

    import gc

    import torch
    from curobo.types import JointState

    from mycobot_curobo.robot_model import JOINT_NAMES, load_robot_model_spec

    app, planner_factory, validator = _build_planner_and_validator(
        planner_profile_name=config.planner_profile,
        validation_profile_name=config.validation_profile,
        minimum_self_collision_clearance_m=config.minimum_self_collision_clearance_m,
        minimum_world_collision_clearance_m=config.minimum_world_collision_clearance_m,
        flange_diameter_assumption_m=config.flange_diameter_assumption_m,
        require_flange_face_containment=config.require_flange_face_containment,
        flange_face_overhang_tolerance_m=config.flange_face_overhang_tolerance_m,
        app_config_path=app_config_path,
    )
    trajectories: dict[str, Any] = {}
    base_profile = load_planner_profile(config.planner_profile)

    def _make_sphere_backend() -> Any:
        return create_curobo_planner(
            replace(base_profile, random_seed=int(config.root_seed)),
            robot_config_path=app.robot_config_path,
            scene_config_path=REPO_ROOT / "config/scenes/empty.yml",
        )

    sphere_backend: Any = _make_sphere_backend()

    def plan_sink(plan: NominalPlan) -> None:
        trajectories[plan.request_id] = plan.combined_trajectory

    def waypoint_spheres_fn(positions: np.ndarray) -> np.ndarray:
        nonlocal sphere_backend
        if sphere_backend is None:
            raise ConfigurationError("sphere backend released during PhysX gate")
        positions_arr = np.asarray(positions, dtype=float)
        if positions_arr.ndim != 2 or positions_arr.shape[1] != len(JOINT_NAMES):
            raise ConfigurationError("waypoint positions must have shape [N, 6]")
        state = JointState.from_position(
            torch.as_tensor(positions_arr, device="cuda:0", dtype=torch.float32),
            joint_names=list(JOINT_NAMES),
        )
        result = sphere_backend.compute_kinematics(state)
        return result.robot_spheres.detach().cpu().numpy().reshape(positions_arr.shape[0], -1, 4)

    physx_gate_fn = None
    if enable_physx_gate and int(config.max_physx_regenerations) > 0:
        from isaac_sim.physx_episode_gate import (
            build_sphere_clearance_fn,
            collision_pairs_from_planner,
            resolve_prepared_usd,
            run_headless_physx_gate,
            sphere_link_names_from_counts,
        )

        prepared_usd = resolve_prepared_usd(REPO_ROOT, usd)
        pairs = collision_pairs_from_planner(sphere_backend)
        robot_spec = load_robot_model_spec(app.robot_config_path)
        link_names = sphere_link_names_from_counts(robot_spec.collision_sphere_count_by_link)
        # Option B dual overlay may emit more sphere slots than scaffolding links;
        # pass names only when lengths match the FK sphere count.
        sample = waypoint_spheres_fn(np.zeros((1, len(JOINT_NAMES)), dtype=float))
        sphere_names = link_names if len(link_names) == int(sample.shape[1]) else None
        sphere_fn = build_sphere_clearance_fn(
            waypoint_spheres_fn=waypoint_spheres_fn,
            collision_pairs=pairs,
            sphere_link_names=sphere_names,
        )
        _z_frag, _z_width, _z_lo, _z_hi = format_z_dist_header(config)
        z_density = {
            "delta_z_m": float(_z_width),
            "z_band_fraction": config.z_band_fraction,
            "z_lo_m": float(_z_lo),
            "z_hi_m": float(_z_hi),
        }

        def _release_cuda_for_kit() -> None:
            """Drop cuRobo GPU objects so Kit can allocate during the gate."""

            nonlocal sphere_backend
            print("phase7_5_physx_regen: releasing cuRobo CUDA before Kit gate", flush=True)
            sphere_backend = None
            gc.collect()
            try:
                torch.cuda.empty_cache()
            except Exception as exc:  # noqa: BLE001
                print(f"phase7_5_physx_regen: cuda empty_cache skipped ({exc})", flush=True)

        def _restore_cuda_after_kit() -> None:
            nonlocal sphere_backend
            print("phase7_5_physx_regen: restoring cuRobo CUDA after Kit gate", flush=True)
            sphere_backend = _make_sphere_backend()

        def physx_gate_fn(result, extra, episode_traj):  # type: ignore[no-untyped-def]
            from dataclasses import replace as dc_replace

            _release_cuda_for_kit()
            try:
                # Sphere clearance needs cuRobo CUDA — evaluate after restore.
                passed, discard = run_headless_physx_gate(
                    result=result,
                    extra=extra,
                    trajectories=episode_traj,
                    tip_allow_link_names=config.tip_allow_link_names,
                    lighting=config.lighting,
                    z_density=z_density,
                    root_seed=root_seed if root_seed is not None else config.root_seed,
                    max_failed_episodes=config.max_failed_episodes,
                    usd=prepared_usd,
                    repo_root=REPO_ROOT,
                    sphere_clearance_fn=None,
                )
            finally:
                _restore_cuda_after_kit()
            if discard is not None and discard.q_rad is not None:
                try:
                    clearance, pair = sphere_fn(np.asarray(discard.q_rad, dtype=float))
                    discard = dc_replace(
                        discard,
                        sphere_clearance_m=float(clearance),
                        sphere_pair=pair,
                    )
                except Exception as exc:  # noqa: BLE001
                    print(
                        f"phase7_5_physx_regen: sphere clearance eval failed: {exc}",
                        flush=True,
                    )
            return passed, discard

        print(
            "phase7_5_physx_regen: enabled "
            f"max_physx_regenerations={config.max_physx_regenerations} "
            f"usd={prepared_usd}",
            flush=True,
        )
    else:
        print("phase7_5_physx_regen: skipped (disabled or max_physx_regenerations=0)", flush=True)

    runner = IncrementalPopulationRunner(
        planner_factory=planner_factory,
        validator=validator,
        contact_detector_factory=lambda _episode, to_id: OptimisticTipContactDetector(to_id),
        plan_sink=plan_sink,
        warn_planning_duration_s=config.warn_planning_duration_s,
        console_log=lambda message: print(message, flush=True),
        waypoint_spheres_fn=waypoint_spheres_fn,
    )
    suite = runner.run_suite(
        config,
        root_seed=root_seed,
        episode_count=episode_count,
        independent_random_episode_seeds=independent_random_episode_seeds,
        trajectories_out=trajectories,
        physx_gate_fn=physx_gate_fn,
    )
    return suite


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.episodes is not None and args.episodes <= 0:
        raise ConfigurationError("--episodes must be a positive integer")
    if args.targets is not None and args.targets <= 0:
        raise ConfigurationError("--targets must be a positive integer")
    independent_episode_seeds = args.root_seed is None
    root_seed: int | None
    if independent_episode_seeds:
        root_seed = None
        print(
            "phase7_2_plan: episode seeds=independent_random (no --root-seed)",
            flush=True,
        )
    else:
        root_seed = resolve_invocation_root_seed(args.root_seed)
        print(f"phase7_2_plan: root_seed={root_seed} (cli)", flush=True)
    config = load_multi_target_suite_config(args.config)
    if config.target_population is TargetPopulation.INCREMENTAL:
        if args.targets is not None:
            raise ConfigurationError(
                "--targets is invalid for target_population=incremental "
                "(achieved count is an outcome)"
            )
        print("phase7_5_populate: target_population=incremental", flush=True)
        suite = plan_incremental(
            config,
            root_seed=root_seed,
            episode_count=args.episodes,
            independent_random_episode_seeds=independent_episode_seeds,
            app_config_path=args.app_config,
            enable_physx_gate=not bool(args.skip_physx_gate),
            usd=args.usd,
        )
        print(
            format_suite_table(
                results=suite.results,
                extras=suite.extras,
                artifact_base_name=suite.artifact_base_name,
            ),
            flush=True,
        )
        fully_succeeded = suite.summary.successes == suite.summary.total_episodes
        _z_frag, _z_width, _z_lo, _z_hi = format_z_dist_header(config)
        # Write the caller-requested path (smoke env) and the normative named
        # artifact beside it so logs/filenames both carry achieved counts.
        requested_bundle = args.output_bundle
        named_bundle = requested_bundle.with_name(f"{suite.artifact_base_name}.bundle.json")
        payload = {
            "schema_version": 1,
            "target_population": TargetPopulation.INCREMENTAL.value,
            "artifact_base_name": suite.artifact_base_name,
            "root_seed": suite.root_seed if suite.root_seed is not None else root_seed,
            "seed_mode": (
                "independent_random_per_episode" if independent_episode_seeds else "cli_root"
            ),
            "episode_seeds": list(suite.episode_seeds),
            "max_failed_episodes": int(config.max_failed_episodes),
            "suite_accepted": bool(suite.suite_accepted),
            "fully_succeeded": bool(fully_succeeded),
            "tip_allow_link_names": list(config.tip_allow_link_names),
            "retain_targets_after_contact": True,
            "lighting": config.lighting,
            "z_density": {
                "delta_z_m": float(_z_width),
                "z_band_fraction": config.z_band_fraction,
                "z_lo_m": float(_z_lo),
                "z_hi_m": float(_z_hi),
            },
            "incremental_episodes": [
                {
                    "stop_reason": extra.stop_reason.value,
                    "consecutive_failures_at_stop": extra.consecutive_failures_at_stop,
                    "total_target_failures": extra.total_target_failures,
                    "accepted_plan_durations_s": list(extra.accepted_plan_durations_s),
                    "failed_plan_durations_s": list(extra.failed_plan_durations_s),
                    "draws": extra.draws,
                    "planned_candidates": extra.planned_candidates,
                    "geometric_rejects": asdict(extra.geometric_rejects),
                    "z_band_lo_m": extra.z_band_lo_m,
                    "z_band_hi_m": extra.z_band_hi_m,
                    "wall_duration_s": extra.wall_duration_s,
                    "populate_duration_s": extra.wall_duration_s,
                    "tip_contacts": len(result.contacted_ids),
                    "candidate_failures": [asdict(record) for record in extra.candidate_failures],
                    "retreat_distance_m": float(config.retreat_distance_m),
                    "physx_discards": [record.to_dict() for record in extra.physx_discards],
                    "physx_acceptance": extra.physx_acceptance,
                    "physx_regen_attempts": int(extra.physx_regen_attempts),
                }
                for extra, result in zip(suite.extras, suite.results)
            ],
            "physx_regeneration_exhausted": bool(suite.physx_regeneration_exhausted),
            "max_physx_regenerations": int(config.max_physx_regenerations),
            "placement_generation": None,
            "summary": asdict(suite.summary),
            "results": [asdict(result) for result in suite.results],
            "frozen_requests": [serialize_episode(result.episode) for result in suite.results],
            "trajectories": {
                request_id: _serialize_trajectory(trajectory)
                for request_id, trajectory in suite.trajectories.items()
            },
        }
        text = (
            json.dumps(
                payload,
                indent=2,
                sort_keys=True,
                default=lambda value: value.value if isinstance(value, Enum) else value,
            )
            + "\n"
        )
        requested_bundle.parent.mkdir(parents=True, exist_ok=True)
        requested_bundle.write_text(text, encoding="utf-8")
        if named_bundle.resolve() != requested_bundle.resolve():
            named_bundle.write_text(text, encoding="utf-8")
        print(
            json.dumps(
                {
                    "bundle": str(named_bundle),
                    "bundle_alias": str(requested_bundle),
                    "artifact_base_name": suite.artifact_base_name,
                    "episodes": len(suite.results),
                    "accepted_counts": [len(r.contacted_ids) for r in suite.results],
                    "suite_accepted": suite.suite_accepted,
                    "fully_succeeded": fully_succeeded,
                    "failed_episodes": suite.summary.failed_episodes,
                    "max_failed_episodes": config.max_failed_episodes,
                    "total_planning_failures": suite.summary.total_planning_failures,
                    "total_target_failures": suite.summary.total_target_failures,
                }
            ),
            flush=True,
        )
        return 0 if suite.suite_accepted else 1

    if args.targets is not None:
        before = config
        config = override_suite_target_count(config, args.targets)
        if config.placement is not before.placement:
            print(
                f"phase7_2_plan: --targets {args.targets} exceeded manual list "
                f"({len(before.manual_targets)}); using grid placement",
                flush=True,
            )
    placement_stats: list = []
    app = load_app_config() if args.app_config is None else load_app_config(args.app_config)
    tip_ik_screen: CuroboTipIkScreen | None = None
    tip_ik_fn = None
    if config.require_tip_ik:
        tip_ik_screen = CuroboTipIkScreen(
            profile=load_planner_profile(config.planner_profile),
            task_frame_config=app.task_frame,
            start_position_rad=config.start_joint_position_rad,
            edge_m=config.target_edge_m,
            outward_normal_base=config.outward_normal_base,
            fixed_roll_rad=config.fixed_roll_rad,
            roll_candidates_rad=config.roll_candidates_rad,
            pre_approach_distance_m=config.pre_approach_distance_m,
            robot_config_path=app.robot_config_path,
        )
        tip_ik_fn = tip_ik_screen
        max_ik = (
            int(config.max_ik_rejections)
            if config.max_ik_rejections is not None
            else int(config.target_count)
        )
        print(
            f"phase7_2_plan: tip IK pre-screen enabled (max_ik_rejections_per_episode={max_ik})",
            flush=True,
        )
    field_regens_used = 0
    try:
        episodes = sample_multi_target_episodes(
            config,
            root_seed=root_seed,
            episode_count=args.episodes,
            independent_random_episode_seeds=independent_episode_seeds,
            tip_ik_fn=tip_ik_fn,
            log=lambda message: print(message, flush=True),
            placement_stats_out=placement_stats,
        )
        for episode in episodes:
            print(
                f"phase7_2_plan: episode={episode.episode_index} "
                f"episode_seed={episode.episode_seed} order_seed={episode.order_seed}",
                flush=True,
            )
            for target in episode.field.targets:
                c = target.center_m
                print(
                    f"phase7_4_placement: episode={episode.episode_index} "
                    f"target={target.target_id} "
                    f"centre=({c[0]:.4f},{c[1]:.4f},{c[2]:.4f})",
                    flush=True,
                )

        def regenerate_field(
            episode: MultiTargetEpisode, regen_index: int
        ) -> MultiTargetEpisode | None:
            nonlocal field_regens_used
            try:
                replacement = regenerate_episode_field(
                    config,
                    episode,
                    tip_ik_fn=tip_ik_fn,
                    log=lambda message: print(message, flush=True),
                    regen_index=regen_index,
                )
            except ConfigurationError as exc:
                print(
                    f"phase7_2_plan: field regeneration failed: {exc}",
                    flush=True,
                )
                return None
            field_regens_used += 1
            for target in replacement.field.targets:
                c = target.center_m
                print(
                    f"phase7_4_placement: episode={replacement.episode_index} "
                    f"regen={regen_index + 1} target={target.target_id} "
                    f"centre=({c[0]:.4f},{c[1]:.4f},{c[2]:.4f})",
                    flush=True,
                )
            return replacement

        results, trajectories = plan_and_validate(
            episodes,
            validation_profile_name=config.validation_profile,
            warn_planning_duration_s=config.warn_planning_duration_s,
            minimum_self_collision_clearance_m=config.minimum_self_collision_clearance_m,
            minimum_world_collision_clearance_m=config.minimum_world_collision_clearance_m,
            flange_diameter_assumption_m=config.flange_diameter_assumption_m,
            require_flange_face_containment=config.require_flange_face_containment,
            flange_face_overhang_tolerance_m=config.flange_face_overhang_tolerance_m,
            app_config_path=args.app_config,
            regenerate_field=regenerate_field,
            max_field_regenerations=config.max_field_regenerations,
        )
    finally:
        if tip_ik_screen is not None:
            tip_ik_screen.close()

    # Prefer frozen episode seeds from results (may differ after field regen).
    result_episodes = tuple(result.episode for result in results)
    summary_seed = (
        root_seed
        if root_seed is not None
        else int(result_episodes[0].episode_seed if result_episodes else 0)
    )
    summary = aggregate_multi_target_results(results, root_seed=summary_seed)
    for result in results:
        print(format_episode_console_row(result, count=len(results)), flush=True)
    print(format_suite_summary(summary), flush=True)
    placement = placement_stats[0] if placement_stats else None
    accepted = suite_acceptance_passed(summary, max_failed_episodes=config.max_failed_episodes)
    fully_succeeded = summary.successes == summary.total_episodes
    payload = {
        "schema_version": 1,
        "target_population": TargetPopulation.FIXED.value,
        "root_seed": root_seed,
        "seed_mode": (
            "independent_random_per_episode" if independent_episode_seeds else "cli_root"
        ),
        "episode_seeds": [int(episode.episode_seed) for episode in result_episodes],
        "max_failed_episodes": int(config.max_failed_episodes),
        "suite_accepted": bool(accepted),
        "fully_succeeded": bool(fully_succeeded),
        "tip_allow_link_names": list(config.tip_allow_link_names),
        "retain_targets_after_contact": config.retain_targets_after_contact,
        "lighting": config.lighting,
        "placement_generation": (
            None
            if placement is None
            else {
                "generation_duration_s": placement.generation_duration_s,
                "max_reach_rejections": placement.max_reach_rejections,
                "reach_rejections": [asdict(item) for item in placement.reach_rejections],
                "max_ik_rejections_per_episode": placement.max_ik_rejections_per_episode,
                "ik_rejections": [asdict(item) for item in placement.ik_rejections],
                "max_field_regenerations": config.max_field_regenerations,
                "field_regenerations": field_regens_used,
                "require_tip_ik": config.require_tip_ik,
            }
        ),
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
                "field_regenerations": field_regens_used,
            }
        ),
        flush=True,
    )
    # Exit 0 when within max_failed_episodes. Incomplete trajectories may still
    # exist for failed episodes; playback skips those legs.
    return 0 if accepted else 1


if __name__ == "__main__":
    raise SystemExit(main())
