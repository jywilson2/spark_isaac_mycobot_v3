#!/usr/bin/env python3
"""Measure +Z tip-contact plan reachability on a rim/keep-out sample grid (host GPU).

Writes a versioned candidate-region artifact. This is evidence for later field
expansion — not a full dexterous-workspace claim.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import replace
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
for candidate in (REPO_ROOT, REPO_ROOT / "src"):
    if str(candidate) not in sys.path:
        sys.path.insert(0, str(candidate))


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=REPO_ROOT / "artifacts/workspace/tip_contact_workspace_v1.json",
    )
    parser.add_argument("--grid-step-m", type=float, default=None)
    parser.add_argument("--z-layers", type=int, default=None)
    parser.add_argument("--max-samples", type=int, default=None)
    parser.add_argument("--planner-profile", type=str, default=None)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    from mycobot_curobo.config import load_app_config
    from mycobot_curobo.frames import TaskFrameConfig
    from mycobot_curobo.planner import (
        NamedJointState,
        NominalPlanner,
        PlanningRequest,
        create_curobo_planner,
        load_planner_profile,
    )
    from mycobot_curobo.robot_model import JOINT_NAMES
    from mycobot_curobo.tip_contact_workspace import (
        TipContactSampleResult,
        build_tip_contact_sample_centers,
        default_tip_contact_workspace_config,
        serialize_tip_contact_workspace,
        surface_target_for_sample,
        write_tip_contact_workspace_artifact,
    )

    config = default_tip_contact_workspace_config()
    if args.grid_step_m is not None:
        config = replace(config, grid_step_m=float(args.grid_step_m))
    if args.z_layers is not None:
        config = replace(config, z_layers=int(args.z_layers))
    if args.planner_profile is not None:
        config = replace(config, planner_profile=str(args.planner_profile))

    samples = build_tip_contact_sample_centers(config)
    if args.max_samples is not None:
        samples = samples[: max(0, int(args.max_samples))]
    print(
        f"tip_contact_workspace: samples={len(samples)} "
        f"step={config.grid_step_m} z_layers={config.z_layers} "
        f"profile={config.planner_profile}",
        flush=True,
    )

    app = load_app_config()
    base_profile = load_planner_profile(config.planner_profile)
    task_frame = TaskFrameConfig()
    empty_scene = REPO_ROOT / "config/scenes/empty.yml"
    results: list[TipContactSampleResult] = []

    for sample_index, sample in enumerate(samples):
        surface = surface_target_for_sample(sample, config)
        start = config.start_joint_bank_rad[0]
        profile = replace(base_profile, random_seed=config.random_seed + sample_index)
        request = PlanningRequest(
            current_joint_state=NamedJointState.create(JOINT_NAMES, start),
            surface_target=surface,
            scene_revision="tip-contact-workspace-empty-v1",
            planner_profile=profile.name,
            random_seed=profile.random_seed,
            request_id=sample.sample_id,
            disable_collision_links=(),
        )

        def backend_factory(prof: object = profile) -> object:
            return create_curobo_planner(
                prof,
                robot_config_path=app.robot_config_path,
                scene_config_path=empty_scene,
            )

        planner = NominalPlanner(backend_factory, profile, task_frame_config=task_frame)
        started = time.perf_counter()
        outcome = planner.plan(request)
        elapsed = time.perf_counter() - started
        if outcome.succeeded and outcome.plan is not None:
            results.append(
                TipContactSampleResult(
                    sample_id=sample.sample_id,
                    center_m=sample.center_m,
                    start_label="zeros",
                    succeeded=True,
                    planner_status=outcome.plan.planner_status,
                    failure_reason=None,
                    planning_duration_s=elapsed,
                )
            )
            status = "OK"
        else:
            failure = outcome.failure
            results.append(
                TipContactSampleResult(
                    sample_id=sample.sample_id,
                    center_m=sample.center_m,
                    start_label="zeros",
                    succeeded=False,
                    planner_status="" if failure is None else failure.planner_status,
                    failure_reason=None if failure is None else failure.reason,
                    planning_duration_s=elapsed,
                )
            )
            status = "FAIL"
        if sample_index % 10 == 0 or sample_index + 1 == len(samples):
            print(
                f"  [{sample_index + 1}/{len(samples)}] {sample.sample_id} "
                f"{sample.center_m} {status} {elapsed:.2f}s",
                flush=True,
            )

    payload = serialize_tip_contact_workspace(config=config, results=results)
    out = write_tip_contact_workspace_artifact(args.output, payload)
    summary = payload["summary"]
    print(
        f"tip_contact_workspace: wrote {out} "
        f"successes={summary['successes']}/{summary['total_samples']} "
        f"rate={summary['success_rate']:.1%}",
        flush=True,
    )
    if summary.get("success_aabb_m") is not None:
        print(f"tip_contact_workspace: success_aabb={summary['success_aabb_m']}", flush=True)
    return 0 if summary["successes"] > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
