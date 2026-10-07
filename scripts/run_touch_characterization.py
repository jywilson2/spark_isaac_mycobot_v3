#!/usr/bin/env python3
"""Opt-in multi-target touch-failure characterization (not a smoke gate).

CPU measurements (geometric precheck, sphere-density curve, workspace-map
comparison) always run. ``--with-planner`` adds seeded planning through
``MultiTargetEpisodeRunner`` and the configured planner profile. Default
smoke scripts do not call this entry point.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict, replace
from enum import Enum
from pathlib import Path
from typing import Any, Sequence

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
for candidate in (REPO_ROOT, REPO_ROOT / "src"):
    if str(candidate) not in sys.path:
        sys.path.insert(0, str(candidate))

from mycobot_curobo.errors import ConfigurationError  # noqa: E402
from mycobot_curobo.multi_target import (  # noqa: E402
    MultiTargetEpisodeRunner,
    OptimisticTipContactDetector,
    load_multi_target_suite_config,
    sample_multi_target_episodes,
)
from mycobot_curobo.robot_model import (  # noqa: E402
    link_transforms_base,
    load_robot_model_spec,
)
from mycobot_curobo.touch_characterization import (  # noqa: E402
    CHARACTERIZATION_TAG,
    PROXY_LABEL,
    KnownValidQuery,
    SphereCurvePoint,
    TipErrorComponents,
    characterize_episode_results,
    compare_placement_policies,
    format_characterization_summary,
    geometric_field_precheck,
    grazing_obstacle_for_pose,
    load_overlay_spheres,
    load_scaffolding_spheres,
    load_workspace_success_centers,
    sphere_cover_curve,
    summarize_attributions,
    transform_spheres_to_base,
    write_characterization_figures,
)

DEFAULT_CONFIG = REPO_ROOT / "config" / "touch_characterization.yml"
DEFAULT_ROBOT = REPO_ROOT / "config" / "robots" / "mycobot_280_m5.yml"
DEFAULT_OVERLAY = REPO_ROOT / "config" / "robots" / "mycobot_280_m5_phase1_1_spheres.yml"
DEFAULT_WORKSPACE = REPO_ROOT / "artifacts" / "workspace" / "tip_contact_workspace_v1.json"


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--robot-config", type=Path, default=DEFAULT_ROBOT)
    parser.add_argument("--overlay", type=Path, default=DEFAULT_OVERLAY)
    parser.add_argument("--workspace", type=Path, default=DEFAULT_WORKSPACE)
    parser.add_argument("--output-dir", type=Path, default=REPO_ROOT / "docs" / "figures")
    parser.add_argument("--fields", type=int, default=20)
    parser.add_argument("--targets", type=int, default=2)
    parser.add_argument("--seed", type=int, default=4242)
    parser.add_argument("--draws", type=int, default=200)
    parser.add_argument("--joint-samples", type=int, default=8)
    parser.add_argument("--with-planner", action="store_true", default=False)
    parser.add_argument("--no-planner", action="store_true", default=False)
    parser.add_argument("--bundle-out", type=Path, default=None)
    return parser.parse_args(list(argv) if argv is not None else None)


def _json_default(value: object) -> object:
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(f"cannot serialize {type(value)!r}")


def _sample_joint_states(spec: Any, count: int, seed: int) -> tuple[tuple[float, ...], ...]:
    if count < 1:
        raise ConfigurationError("--joint-samples must be positive")
    rng = np.random.default_rng(seed)
    lower = np.asarray(spec.limits.lower_rad, dtype=float)
    upper = np.asarray(spec.limits.upper_rad, dtype=float)
    states = [tuple(float(v) for v in spec.default_joint_position_rad)]
    span = upper - lower
    for _ in range(count - 1):
        draw = lower + 0.15 * span + rng.random(6) * 0.70 * span
        draw = np.minimum(np.maximum(draw, lower + 1.0e-4), upper - 1.0e-4)
        states.append(tuple(float(v) for v in draw))
    return tuple(states)


def _build_queries(
    spec: Any,
    overlay: Any,
    joint_states: Sequence[Sequence[float]],
) -> tuple[KnownValidQuery, ...]:
    queries: list[KnownValidQuery] = []
    for index, position in enumerate(joint_states):
        frames = link_transforms_base(position, spec=spec)
        obstacle = grazing_obstacle_for_pose(overlay, frames)
        if obstacle is None:
            continue
        queries.append(
            KnownValidQuery(label=f"q{index:02d}", link_frames=frames, obstacle=obstacle)
        )
    return tuple(queries)


def _clearance_fn(scaffolding: Any, overlay: Any, spec: Any):
    def clearance(
        position_rad: tuple[float, ...], cubes: tuple[Any, ...]
    ) -> tuple[float | None, float | None]:
        frames = link_transforms_base(position_rad, spec=spec)
        from mycobot_curobo.touch_characterization import sphere_set_clearance_m

        coarse = sphere_set_clearance_m(scaffolding, frames, cubes)
        fine = sphere_set_clearance_m(overlay, frames, cubes)
        return coarse, fine

    return clearance


def _run_planner(
    config: Any,
    episodes: Any,
    *,
    bundle_out: Path | None,
) -> tuple[tuple[Any, ...], str | None]:
    from isaac_sim.plan_multi_target_suite import (
        _build_planner_and_validator,
        _serialize_trajectory,
    )

    _app, planner_factory, validator = _build_planner_and_validator(
        planner_profile_name=config.planner_profile,
        validation_profile_name=config.validation_profile,
        minimum_self_collision_clearance_m=config.minimum_self_collision_clearance_m,
        minimum_world_collision_clearance_m=config.minimum_world_collision_clearance_m,
        flange_diameter_assumption_m=config.flange_diameter_assumption_m,
        require_flange_face_containment=config.require_flange_face_containment,
        flange_face_overhang_tolerance_m=config.flange_face_overhang_tolerance_m,
        app_config_path=None,
    )
    trajectories: dict[str, Any] = {}

    def plan_sink(plan: Any) -> None:
        trajectories[plan.request_id] = plan.combined_trajectory

    runner = MultiTargetEpisodeRunner(
        planner_factory=planner_factory,
        validator=validator,
        contact_detector_factory=lambda _episode, target_id: OptimisticTipContactDetector(
            target_id
        ),
        plan_sink=plan_sink,
        warn_planning_duration_s=config.warn_planning_duration_s,
    )
    results = runner.run(episodes, max_field_regenerations=0)
    if bundle_out is not None and trajectories:
        payload = {
            "schema_version": 1,
            "target_population": "fixed",
            "root_seed": int(config.root_seed),
            "seed_mode": "cli_root",
            "episode_seeds": [int(result.episode.episode_seed) for result in results],
            "max_failed_episodes": int(config.max_failed_episodes),
            "tip_allow_link_names": list(config.tip_allow_link_names),
            "retain_targets_after_contact": bool(config.retain_targets_after_contact),
            "lighting": config.lighting,
            "results": [asdict(result) for result in results],
            "trajectories": {
                request_id: _serialize_trajectory(trajectory)
                for request_id, trajectory in trajectories.items()
            },
            "characterization": True,
        }
        bundle_out.parent.mkdir(parents=True, exist_ok=True)
        bundle_out.write_text(
            json.dumps(payload, indent=2, sort_keys=True, default=_json_default) + "\n",
            encoding="utf-8",
        )
    return results, None


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if args.with_planner and args.no_planner:
        raise SystemExit("pass only one of --with-planner or --no-planner")
    use_planner = bool(args.with_planner) and not bool(args.no_planner)
    blockers: list[str] = []
    config = load_multi_target_suite_config(args.config)
    config = replace(
        config,
        episode_count=int(args.fields),
        target_count=int(args.targets),
        root_seed=int(args.seed),
        max_planning_failure_per_target=1,
        max_reconsider_passes=1,
        max_field_regenerations=0,
    )
    episodes = sample_multi_target_episodes(
        config, root_seed=int(args.seed), episode_count=int(args.fields)
    )
    precheck = geometric_field_precheck(
        episodes, flange_diameter_m=config.flange_diameter_assumption_m
    )
    spec = load_robot_model_spec(args.robot_config)
    scaffolding = load_scaffolding_spheres(args.robot_config)
    overlay = load_overlay_spheres(args.overlay)
    joint_states = _sample_joint_states(spec, int(args.joint_samples), int(args.seed))
    queries = _build_queries(spec, overlay, joint_states)
    curve: tuple[SphereCurvePoint, ...] = ()
    if not queries:
        blockers.append(
            "sphere curve: no grazing cuboid kept the reference centres clear; "
            "false-infeasible rate was not computed"
        )
    else:
        curve = sphere_cover_curve(
            queries, overlay_spheres=overlay, scaffolding_spheres=scaffolding
        )
    start_frames = link_transforms_base(config.start_joint_position_rad, spec=spec)
    start_centers, start_radii = transform_spheres_to_base(scaffolding, start_frames)
    workspace_centers = load_workspace_success_centers(args.workspace)
    placement = compare_placement_policies(
        seed=int(args.seed),
        draw_count=int(args.draws),
        field_minimum_m=config.field_minimum_m,
        field_maximum_m=config.field_maximum_m,
        edge_m=config.target_edge_m,
        max_target_radial_m=float(config.max_target_radial_m or 0.36),
        workspace_centers_m=workspace_centers,
        start_sphere_centers_m=start_centers,
        start_sphere_radii_m=start_radii,
    )
    attributions: tuple[Any, ...] = ()
    planner_note = "not_requested"
    if use_planner:
        try:
            results, planner_error = _run_planner(config, episodes, bundle_out=args.bundle_out)
            if planner_error:
                blockers.append(planner_error)
                planner_note = "failed"
            else:
                attributions = characterize_episode_results(
                    results,
                    flange_diameter_m=config.flange_diameter_assumption_m,
                    clearance_fn=_clearance_fn(scaffolding, overlay, spec),
                )
                planner_note = "ran"
        except Exception as exc:
            blockers.append(f"planner attribution unavailable: {type(exc).__name__}: {exc}")
            planner_note = "unavailable"
    else:
        blockers.append(
            "planner attribution not requested (--with-planner). "
            "Geometric precheck, sphere curve, and placement comparison are CPU-only."
        )
    summary = summarize_attributions(attributions)
    print(format_characterization_summary(summary), flush=True)
    print(
        f"{CHARACTERIZATION_TAG} geometric_precheck "
        f"targets={precheck['targets']} "
        f"geometric_infeasibility={precheck['geometric_infeasibility']}",
        flush=True,
    )
    for point in curve:
        print(
            f"{CHARACTERIZATION_TAG} sphere_curve label={point.label} "
            f"role={point.role} spheres={point.sphere_count} "
            f"false_infeasible_rate={point.false_infeasible_rate:.4f} "
            f"queries={point.query_count}",
            flush=True,
        )
    for item in placement:
        print(
            f"{CHARACTERIZATION_TAG} placement policy={item.policy} "
            f"draws={item.draws} rim_failures={item.rim_failures} "
            f"start_collision_failures={item.start_collision_failures} "
            f"accepted={item.accepted}",
            flush=True,
        )
    open_loop: list[TipErrorComponents] = []
    report = {
        "schema_version": 1,
        "mode": "opt_in_touch_characterization",
        "changes_default_smoke": False,
        "proxy_label": PROXY_LABEL,
        "proxy_note": (
            "World-model blindness is the dense-overlay versus scaffolding-32 "
            "clearance proxy. It is not a PhysX replay and is not sub-millimeter "
            "hardware accuracy."
        ),
        "seed": int(args.seed),
        "fields": int(args.fields),
        "targets_per_field": int(args.targets),
        "planner_profile": config.planner_profile,
        "planner": planner_note,
        "geometric_precheck": precheck,
        "attribution_summary": summary,
        "attributions": [asdict(row) for row in attributions],
        "sphere_curve": [asdict(point) for point in curve],
        "placement": [asdict(item) for item in placement],
        "open_loop_tip_errors": [],
        "blockers": blockers,
    }
    figures = write_characterization_figures(
        summary, curve, args.output_dir, open_loop_errors_m=open_loop
    )
    summary_path = args.output_dir / "touch_characterization_summary.json"
    args.output_dir.mkdir(parents=True, exist_ok=True)
    summary_path.write_text(
        json.dumps(report, indent=2, sort_keys=True, default=_json_default) + "\n",
        encoding="utf-8",
    )
    print(
        f"{CHARACTERIZATION_TAG} summary={summary_path} figures={len(figures)}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
