#!/usr/bin/env python3
"""Play planned Phase 7.1 cube episodes in Isaac Sim without importing cuRobo.

Planning must run in a separate process (``plan_cube_suite.py``). Importing
cuRobo/Warp before ``SimulationApp`` breaks Kit extension startup.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import traceback
from dataclasses import asdict, replace
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
for candidate in (REPO_ROOT, REPO_ROOT / "src"):
    if str(candidate) not in sys.path:
        sys.path.insert(0, str(candidate))

# Kit needs a few post-open updates before stage queries are reliable.
STAGE_SETTLE_UPDATES = 10
GUI_VIEWPORT_SETTLE_STEPS = 30


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, required=True)
    parser.add_argument("--repo-root", type=Path, default=REPO_ROOT)
    parser.add_argument("--usd", type=Path)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--headless", action="store_false", dest="gui")
    mode.add_argument("--gui", action="store_true", dest="gui")
    parser.set_defaults(gui=False)
    exit_mode = parser.add_mutually_exclusive_group()
    exit_mode.add_argument("--auto-exit", action="store_true", default=True)
    exit_mode.add_argument("--no-auto-exit", action="store_false", dest="auto_exit")
    parser.add_argument("--output-report", type=Path, required=True)
    parser.add_argument("--hold-s", type=float, default=None)
    return parser.parse_args(argv)


def _empty_report_payload() -> dict[str, Any]:
    return {
        "schema_version": 1,
        "lighting_ready": False,
        "stage_lighting_mode": False,
        "joint_playback_completed": False,
        "tip_metrics_status": "not_evaluated",
        "tip_position_error_m": None,
        "tip_orientation_error_rad": None,
        "summary": {"total_episodes": 0, "successes": 0, "success_rate": 0.0},
        "results": [],
        "frozen_requests": [],
        "error": None,
    }


def _write_report(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True, default=str) + "\n",
        encoding="utf-8",
    )


def _find_articulation_root(stage: Any) -> str:
    from pxr import UsdPhysics

    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return str(prim.GetPath())
    raise RuntimeError("USD contains no articulation root")


def _drive_targets(robot: Any, targets: Any) -> None:
    setter = getattr(robot, "set_joint_position_targets", None)
    if callable(setter):
        setter(targets)
        return
    from isaacsim.core.utils.types import ArticulationAction

    robot.apply_action(ArticulationAction(joint_positions=targets))


def _physics_steps_for_duration(duration_s: float, physics_dt_s: float) -> int:
    return max(1, int(math.ceil(duration_s / physics_dt_s)))


def _resolve_prepared_usd(repo: Path, usd_arg: Path | None) -> Path:
    from isaac_sim.urdf_utils import default_prepared_urdf

    default_usd = default_prepared_urdf(repo).with_suffix(".usd")
    nested_usda = default_usd.with_suffix("") / default_usd.with_suffix(".usda").name
    usd = (usd_arg or (default_usd if default_usd.is_file() else nested_usda)).resolve()
    if not usd.is_file():
        raise FileNotFoundError(f"prepared robot USD not found: {usd}")
    return usd


def _play_validated_episodes(
    *,
    app: Any,
    args: argparse.Namespace,
) -> dict[str, Any]:
    """Import Kit-dependent modules only after SimulationApp construction."""

    import numpy as np
    import omni.usd
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation

    from isaac_sim.articulation_playback import articulation_position_targets
    from isaac_sim.contact_monitor import ProhibitedContactMonitor
    from isaac_sim.scene_setup import (
        IsaacLightingConfig,
        add_cube_prim,
        configure_kit_for_stage_lighting,
        prepare_illuminated_stage,
        stage_lighting_mode_active,
    )
    from mycobot_curobo.benchmark import FailureCategory
    from mycobot_curobo.cube_suite import (
        CubeEpisodeResult,
        aggregate_cube_results,
        deserialize_episode,
        format_episode_console_row,
        serialize_episode,
    )
    from mycobot_curobo.trajectory import JointTrajectory
    from mycobot_curobo.validation import ValidationMetrics

    def load_result(item: dict[str, Any]) -> CubeEpisodeResult:
        metrics = item.get("validation_metrics")
        return CubeEpisodeResult(
            episode=deserialize_episode(item["episode"]),
            planning_succeeded=bool(item["planning_succeeded"]),
            validation_passed=bool(item["validation_passed"]),
            failure_category=(
                None
                if item.get("failure_category") is None
                else FailureCategory(item["failure_category"])
            ),
            failure_reason=item.get("failure_reason"),
            planner_status=str(item.get("planner_status", "")),
            validation_metrics=(None if metrics is None else ValidationMetrics(**metrics)),
            isaac_prohibited_contacts=item.get("isaac_prohibited_contacts"),
            final_joint_position_rad=(
                None
                if item.get("final_joint_position_rad") is None
                else tuple(item["final_joint_position_rad"])
            ),
        )

    def load_trajectory(item: dict[str, Any]) -> JointTrajectory:
        return JointTrajectory(
            joint_names=tuple(item["joint_names"]),
            position_rad=np.asarray(item["position_rad"], dtype=float),
            dt_s=float(item["dt_s"]),
            velocity_rad_s=(
                None
                if item.get("velocity_rad_s") is None
                else np.asarray(item["velocity_rad_s"], dtype=float)
            ),
            acceleration_rad_s2=(
                None
                if item.get("acceleration_rad_s2") is None
                else np.asarray(item["acceleration_rad_s2"], dtype=float)
            ),
            jerk_rad_s3=(
                None
                if item.get("jerk_rad_s3") is None
                else np.asarray(item["jerk_rad_s3"], dtype=float)
            ),
        )

    print("phase7_1_playback: kit ready", flush=True)
    bundle = json.loads(args.bundle.read_text(encoding="utf-8"))
    results = [load_result(item) for item in bundle["results"]]
    trajectories = {
        request_id: load_trajectory(item)
        for request_id, item in bundle.get("trajectories", {}).items()
    }
    max_contacts = int(bundle["max_isaac_prohibited_contacts"])
    lighting_config = IsaacLightingConfig.from_mapping(bundle["lighting"])
    root_seed = int(bundle["root_seed"])
    usd = _resolve_prepared_usd(args.repo_root.resolve(), args.usd)

    # Disable auto light-rig before open; otherwise Kit warns "No lights found"
    # and applies Default, which hides later UsdLux prims.
    configure_kit_for_stage_lighting()
    context = omni.usd.get_context()
    if not context.open_stage(str(usd)):
        raise RuntimeError(f"failed to open USD stage: {usd}")
    for _ in range(STAGE_SETTLE_UPDATES):
        app.update()
    stage = context.get_stage()
    _lighting_paths, lighting_ok = prepare_illuminated_stage(stage, lighting_config)
    if not lighting_ok:
        raise RuntimeError("Phase 7.1 lighting prims were not created")
    print(
        f"phase7_1_playback: lighting_ready stage_lighting_mode={stage_lighting_mode_active()}",
        flush=True,
    )

    world = World(stage_units_in_meters=1.0)
    robot_root = _find_articulation_root(stage)
    robot = world.scene.add(SingleArticulation(prim_path=robot_root, name="mycobot_phase7_1"))
    world.reset()
    # World.reset can restore camera/rig lighting; re-enable stage lights.
    prepare_illuminated_stage(stage, lighting_config)
    robot.initialize()
    if args.gui:
        for _ in range(GUI_VIEWPORT_SETTLE_STEPS):
            world.step(render=True)
        # Viewport may exist only after settle; re-assert stage lighting mode.
        prepare_illuminated_stage(stage, lighting_config)
        print(
            "phase7_1_playback: GUI viewport settled "
            f"(DISPLAY={os.environ.get('DISPLAY', '')!r} "
            f"stage_lighting_mode={stage_lighting_mode_active()})",
            flush=True,
        )
    dof_names = tuple(str(name) for name in robot.dof_names)
    physics_dt_s = world.get_physics_dt()

    for index, result in enumerate(results):
        if not result.succeeded:
            print(format_episode_console_row(result, count=len(results)), flush=True)
            continue
        episode = result.episode
        cube_path = f"/World/Phase7_1/Cubes/{episode.cube_name}"
        add_cube_prim(
            stage,
            prim_path=cube_path,
            center_m=episode.cube_center_m,
            edge_m=episode.cube_edge_m,
        )
        reset_targets = articulation_position_targets(
            episode.start_position_rad, dof_names, robot.get_joint_positions()
        )
        print(
            f"[{index + 1}/{len(results)}] simulator reset to {episode.start_label}",
            flush=True,
        )
        robot.set_joint_positions(reset_targets)
        world.step(render=args.gui)
        monitor = ProhibitedContactMonitor()
        monitor.start(stage, cube_path, robot_root)
        current = reset_targets
        try:
            trajectory = trajectories[result.episode.request_id]
            waypoint_steps = _physics_steps_for_duration(trajectory.dt_s, physics_dt_s)
            for waypoint in trajectory.position_rad:
                targets = articulation_position_targets(waypoint, dof_names, current)
                _drive_targets(robot, targets)
                current = targets
                for _ in range(waypoint_steps):
                    world.step(render=args.gui)
            for _ in range(_physics_steps_for_duration(args.hold_s, physics_dt_s)):
                world.step(render=args.gui)
        finally:
            monitor.stop()
        contacts = int(monitor.poll())
        failed_contact = contacts > max_contacts
        results[index] = replace(
            result,
            isaac_prohibited_contacts=contacts,
            failure_category=(
                FailureCategory.COLLISION_INFEASIBILITY
                if failed_contact
                else result.failure_category
            ),
            failure_reason=(
                f"prohibited Isaac contacts {contacts} exceed {max_contacts}"
                if failed_contact
                else result.failure_reason
            ),
            validation_passed=result.validation_passed and not failed_contact,
        )
        print(format_episode_console_row(results[index], count=len(results)), flush=True)

    if not args.auto_exit:
        print(
            "phase7_1_playback: holding GUI open (--no-auto-exit); close the window to finish",
            flush=True,
        )
        while app.is_running():
            world.step(render=True)

    summary = aggregate_cube_results(results, root_seed=root_seed)
    executable = [
        result for result in results if result.planning_succeeded and result.validation_passed
    ]
    return {
        "schema_version": 1,
        "lighting_ready": lighting_ok,
        "stage_lighting_mode": stage_lighting_mode_active(),
        "joint_playback_completed": (
            all(result.isaac_prohibited_contacts is not None for result in executable)
            if executable
            else True
        ),
        "tip_metrics_status": "not_evaluated",
        "tip_position_error_m": None,
        "tip_orientation_error_rad": None,
        "summary": asdict(summary),
        "results": [asdict(result) for result in results],
        "frozen_requests": [serialize_episode(result.episode) for result in results],
        "error": None,
    }


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.hold_s is None:
        args.hold_s = 2.0 if args.gui else 0.5
    if args.hold_s < 0.0 or not math.isfinite(args.hold_s):
        raise ValueError("--hold-s must be finite and non-negative")
    if not args.bundle.is_file():
        raise FileNotFoundError(f"Phase 7.1 plan bundle not found: {args.bundle}")

    payload = _empty_report_payload()
    lighting_ok = False
    # Launch Kit before importing project modules that might pull CUDA/Warp helpers.
    from isaacsim import SimulationApp

    if args.gui:
        display = os.environ.get("DISPLAY", "")
        print(
            f"phase7_1_playback: opening Isaac Sim GUI on DISPLAY={display!r}",
            flush=True,
        )
    app = SimulationApp(
        {
            "headless": not args.gui,
            "width": 1280,
            "height": 720,
        }
    )
    exit_code = 1
    try:
        payload = _play_validated_episodes(app=app, args=args)
        lighting_ok = bool(payload.get("lighting_ready"))
    except Exception as exc:
        payload["error"] = f"{type(exc).__name__}: {exc}"
        traceback.print_exc()
    finally:
        # Write evidence before close(): Kit shutdown can terminate the process
        # before statements after close() run.
        if payload.get("error"):
            exit_code = 2
        else:
            summary = payload.get("summary") or {}
            exit_code = (
                0
                if summary.get("successes") == summary.get("total_episodes") and lighting_ok
                else 1
            )
        payload["exit_code"] = exit_code
        _write_report(args.output_report, payload)
        print(
            json.dumps({"summary": payload.get("summary"), "report": str(args.output_report)}),
            flush=True,
        )
        try:
            app.close()
        except Exception as close_exc:  # noqa: BLE001
            print(f"warning: SimulationApp.close failed: {close_exc}", flush=True)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
