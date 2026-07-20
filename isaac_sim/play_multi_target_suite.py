#!/usr/bin/env python3
"""Play planned Phase 7.2 multi-target episodes in Isaac Sim without importing cuRobo.

Planning must run in a separate process (``plan_multi_target_suite.py``).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import traceback
from dataclasses import asdict, replace
from enum import Enum
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
for candidate in (REPO_ROOT, REPO_ROOT / "src"):
    if str(candidate) not in sys.path:
        sys.path.insert(0, str(candidate))

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
    exit_mode.add_argument("--auto-exit", action="store_true", dest="auto_exit")
    exit_mode.add_argument("--no-auto-exit", action="store_false", dest="auto_exit")
    parser.set_defaults(auto_exit=True)
    parser.add_argument("--output-report", type=Path, required=True)
    parser.add_argument("--hold-s", type=float, default=None)
    return parser.parse_args(argv)


def _empty_report_payload() -> dict[str, Any]:
    return {
        "schema_version": 1,
        "lighting_ready": False,
        "stage_lighting_mode": False,
        "joint_playback_completed": False,
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


def _play_validated_episodes(*, app: Any, args: argparse.Namespace) -> dict[str, Any]:
    import numpy as np
    import omni.usd
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation

    from isaac_sim.articulation_playback import articulation_position_targets
    from isaac_sim.scene_setup import (
        BODY_CONTACT_COLOR_RGBA,
        DEFAULT_TARGET_COLOR_RGBA,
        TIP_CONTACT_COLOR_RGBA,
        IsaacLightingConfig,
        add_cube_prim,
        add_target_label,
        configure_kit_for_stage_lighting,
        prepare_illuminated_stage,
        remove_prim,
        set_cube_color,
        stage_lighting_mode_active,
    )
    from isaac_sim.tip_body_contact import TipBodyContactMonitor
    from mycobot_curobo.multi_target import (
        ContactEvent,
        ContactKind,
        MultiTargetEpisodeResult,
        MultiTargetFailureCategory,
        MultiTargetLegResult,
        aggregate_multi_target_results,
        deserialize_episode,
        format_episode_console_row,
        format_leg_console_row,
        serialize_episode,
    )
    from mycobot_curobo.trajectory import JointTrajectory
    from mycobot_curobo.validation import ValidationMetrics

    def enum_or_none(enum_cls: type[Enum], value: Any) -> Any:
        return None if value is None else enum_cls(value)

    def load_metrics(payload: dict[str, Any] | None) -> ValidationMetrics | None:
        if payload is None:
            return None
        return ValidationMetrics(**payload)

    def load_leg(item: dict[str, Any]) -> MultiTargetLegResult:
        return MultiTargetLegResult(
            from_id=str(item["from_id"]),
            to_id=str(item["to_id"]),
            planning_succeeded=bool(item["planning_succeeded"]),
            validation_passed=bool(item["validation_passed"]),
            contact_kind=enum_or_none(ContactKind, item.get("contact_kind")),
            failure_category=enum_or_none(
                MultiTargetFailureCategory, item.get("failure_category")
            ),
            failure_reason=item.get("failure_reason"),
            planner_status=str(item.get("planner_status", "")),
            planning_duration_s=item.get("planning_duration_s"),
            motion_duration_s=item.get("motion_duration_s"),
            time_to_contact_s=item.get("time_to_contact_s"),
            request_id=item.get("request_id"),
            scene_revision=item.get("scene_revision"),
            validation_metrics=load_metrics(item.get("validation_metrics")),
            final_joint_position_rad=(
                None
                if item.get("final_joint_position_rad") is None
                else tuple(item["final_joint_position_rad"])
            ),
            attempt_index=int(item.get("attempt_index", 0)),
        )

    def load_result(item: dict[str, Any]) -> MultiTargetEpisodeResult:
        return MultiTargetEpisodeResult(
            episode=deserialize_episode(item["episode"]),
            succeeded=bool(item["succeeded"]),
            failure_category=enum_or_none(
                MultiTargetFailureCategory, item.get("failure_category")
            ),
            failure_reason=item.get("failure_reason"),
            planning_failure_count=int(
                item.get(
                    "planning_failure_count",
                    item.get(
                        "intra_episode_plan_failures",
                        item.get("failed_plan_count", 0),
                    ),
                )
            ),
            target_failure_count=int(item.get("target_failure_count", 0)),
            failed_target_ids=tuple(item.get("failed_target_ids", ())),
            legs=tuple(load_leg(leg) for leg in item["legs"]),
            contacted_ids=tuple(item["contacted_ids"]),
            removed_ids=tuple(item["removed_ids"]),
            episode_duration_s=item.get("episode_duration_s"),
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

    def post_message(message: str) -> None:
        print(message, flush=True)
        try:
            import carb

            carb.log_warn(message)
        except ImportError:
            pass

    def _tip_pose_m(robot: Any, stage: Any) -> np.ndarray | None:
        """Best-effort flange/TCP world position from the articulation stage."""
        try:
            from pxr import UsdGeom

            tip_prim = next(
                (
                    prim
                    for prim in stage.Traverse()
                    if prim.GetName() in ("tcp_link", "joint6_flange")
                ),
                None,
            )
            if tip_prim is None:
                return None
            xform = UsdGeom.Xformable(tip_prim)
            world = xform.ComputeLocalToWorldTransform(0.0)
            return np.asarray(
                [
                    world.ExtractTranslation()[0],
                    world.ExtractTranslation()[1],
                    world.ExtractTranslation()[2],
                ],
                dtype=float,
            )
        except Exception:
            return None

    def _classify_leg_contact(
        *,
        monitor: TipBodyContactMonitor,
        robot: Any,
        target: Any,
        to_id: str,
    ):
        physx = monitor.classify()
        if physx.kind is ContactKind.PROHIBITED_BODY_CONTACT:
            return physx
        tip = _tip_pose_m(robot, stage)
        face = np.asarray(target.to_surface_target().position_base_m, dtype=float)
        if tip is not None and float(np.linalg.norm(tip - face)) <= 0.012:
            return ContactEvent(
                ContactKind.ALLOWED_TIP_CONTACT, target_id=to_id, link_name="joint6_flange"
            )
        if physx.kind is ContactKind.ALLOWED_TIP_CONTACT:
            return physx
        return ContactEvent(ContactKind.NONE)

    print("phase7_2_playback: kit ready", flush=True)
    bundle = json.loads(args.bundle.read_text(encoding="utf-8"))
    results = [load_result(item) for item in bundle["results"]]
    trajectories = {
        request_id: load_trajectory(item)
        for request_id, item in bundle.get("trajectories", {}).items()
    }
    tip_links = tuple(bundle["tip_allow_link_names"])
    retain = bool(bundle["retain_targets_after_contact"])
    lighting_config = IsaacLightingConfig.from_mapping(bundle["lighting"])
    root_seed = int(bundle["root_seed"])
    usd = _resolve_prepared_usd(args.repo_root.resolve(), args.usd)

    configure_kit_for_stage_lighting()
    context = omni.usd.get_context()
    if not context.open_stage(str(usd)):
        raise RuntimeError(f"failed to open USD stage: {usd}")
    for _ in range(STAGE_SETTLE_UPDATES):
        app.update()
    stage = context.get_stage()
    _lighting_paths, lighting_ok = prepare_illuminated_stage(stage, lighting_config)
    if not lighting_ok:
        raise RuntimeError("Phase 7.2 lighting prims were not created")
    print(
        f"phase7_2_playback: lighting_ready stage_lighting_mode={stage_lighting_mode_active()}",
        flush=True,
    )

    world = World(stage_units_in_meters=1.0)
    robot_root = _find_articulation_root(stage)
    robot = world.scene.add(SingleArticulation(prim_path=robot_root, name="mycobot_phase7_2"))
    world.reset()
    prepare_illuminated_stage(stage, lighting_config)
    robot.initialize()
    if args.gui:
        for _ in range(GUI_VIEWPORT_SETTLE_STEPS):
            world.step(render=True)
        prepare_illuminated_stage(stage, lighting_config)
        print(
            "phase7_2_playback: GUI viewport settled "
            f"(DISPLAY={os.environ.get('DISPLAY', '')!r} "
            f"stage_lighting_mode={stage_lighting_mode_active()})",
            flush=True,
        )
    dof_names = tuple(str(name) for name in robot.dof_names)
    physics_dt_s = world.get_physics_dt()
    planned_results = tuple(results)

    class _PlaybackStopped(Exception):
        """Raised when the Kit window closes during playback/replay."""

    def _play_one_episode(
        *,
        episode_index: int,
        planned: MultiTargetEpisodeResult,
        episode_count: int,
        pass_label: str,
    ) -> MultiTargetEpisodeResult:
        playable = any(leg.planning_succeeded and leg.validation_passed for leg in planned.legs)
        if not playable:
            print(
                f"phase7_2_playback: {pass_label} skip episode {episode_index + 1} "
                "(no validated trajectories)",
                flush=True,
            )
            print(format_episode_console_row(planned, count=episode_count), flush=True)
            return planned

        episode = planned.episode
        target_paths = {
            target.target_id: f"/World/Phase7_2/Targets/target_{target.target_id}"
            for target in episode.field.targets
        }
        for path in target_paths.values():
            remove_prim(stage, path)
        for target in episode.field.targets:
            path = target_paths[target.target_id]
            add_cube_prim(
                stage,
                prim_path=path,
                center_m=target.center_m,
                edge_m=target.edge_m,
                color_rgba=DEFAULT_TARGET_COLOR_RGBA,
            )
            add_target_label(
                stage,
                prim_path=path,
                target_id=target.target_id,
                center_m=target.center_m,
            )
        reset_targets = articulation_position_targets(
            episode.start_position_rad, dof_names, robot.get_joint_positions()
        )
        robot.set_joint_positions(reset_targets)
        world.step(render=args.gui)
        if not app.is_running():
            raise _PlaybackStopped()
        monitor = TipBodyContactMonitor()
        monitor.start(
            stage,
            target_paths=target_paths,
            robot_root_path=robot_root,
            tip_allow_link_names=tip_links,
        )
        updated_legs: list[MultiTargetLegResult] = []
        contacted: list[str] = []
        removed: list[str] = []
        episode_failed = False
        failure_category = planned.failure_category
        failure_reason = planned.failure_reason
        # Preserve planning-side episode failures (e.g. max_target_failures) while
        # still animating any validated legs for visualization / replay.
        plan_already_failed = not planned.succeeded
        current = reset_targets
        try:
            for leg in planned.legs:
                if not app.is_running():
                    raise _PlaybackStopped()
                if not leg.planning_succeeded or not leg.validation_passed:
                    updated_legs.append(leg)
                    continue
                if leg.request_id is None or leg.request_id not in trajectories:
                    updated_legs.append(
                        replace(
                            leg,
                            failure_category=MultiTargetFailureCategory.CONFIGURATION_MODEL_FAILURE,
                            failure_reason="missing trajectory for validated leg",
                        )
                    )
                    episode_failed = True
                    failure_category = MultiTargetFailureCategory.CONFIGURATION_MODEL_FAILURE
                    failure_reason = "missing trajectory for validated leg"
                    break
                monitor.reset()
                trajectory = trajectories[leg.request_id]
                motion_started = time.perf_counter()
                tip_seen_at: float | None = None
                waypoint_steps = _physics_steps_for_duration(trajectory.dt_s, physics_dt_s)
                for waypoint in trajectory.position_rad:
                    if not app.is_running():
                        raise _PlaybackStopped()
                    targets = articulation_position_targets(waypoint, dof_names, current)
                    try:
                        _drive_targets(robot, targets)
                    except AttributeError as exc:
                        # Kit teardown after window close leaves articulation views None.
                        if not app.is_running():
                            raise _PlaybackStopped() from exc
                        raise
                    current = targets
                    for _ in range(waypoint_steps):
                        if not app.is_running():
                            raise _PlaybackStopped()
                        world.step(render=args.gui)
                        classification = monitor.classify()
                        if (
                            classification.kind is ContactKind.PROHIBITED_BODY_CONTACT
                            or classification.kind is ContactKind.ALLOWED_TIP_CONTACT
                        ):
                            if tip_seen_at is None:
                                tip_seen_at = time.perf_counter()
                            if classification.kind is ContactKind.PROHIBITED_BODY_CONTACT:
                                tip_seen_at = time.perf_counter()
                                break
                    if monitor.classify().kind is ContactKind.PROHIBITED_BODY_CONTACT:
                        break
                motion_duration_s = time.perf_counter() - motion_started
                for _ in range(_physics_steps_for_duration(args.hold_s, physics_dt_s)):
                    if not app.is_running():
                        raise _PlaybackStopped()
                    world.step(render=args.gui)
                target_obj = episode.field.target_by_id(leg.to_id)
                contact = _classify_leg_contact(
                    monitor=monitor, robot=robot, target=target_obj, to_id=leg.to_id
                )
                planning_s = (
                    0.0 if leg.planning_duration_s is None else float(leg.planning_duration_s)
                )
                if contact.kind is ContactKind.PROHIBITED_BODY_CONTACT:
                    set_cube_color(stage, target_paths[leg.to_id], BODY_CONTACT_COLOR_RGBA)
                    message = (
                        f"BODY CONTACT {leg.from_id}->{leg.to_id} "
                        f"plan_s={planning_s:.3f} motion_s={motion_duration_s:.3f}"
                    )
                    post_message(message)
                    updated = replace(
                        leg,
                        contact_kind=contact.kind,
                        motion_duration_s=motion_duration_s,
                        time_to_contact_s=planning_s + motion_duration_s,
                        failure_category=MultiTargetFailureCategory.BODY_CONTACT,
                        failure_reason=message,
                    )
                    updated_legs.append(updated)
                    print(
                        format_leg_console_row(
                            updated, episode_index=episode_index, episode_count=episode_count
                        ),
                        flush=True,
                    )
                    episode_failed = True
                    failure_category = MultiTargetFailureCategory.BODY_CONTACT
                    failure_reason = message
                    break
                if contact.kind is ContactKind.ALLOWED_TIP_CONTACT:
                    set_cube_color(stage, target_paths[leg.to_id], TIP_CONTACT_COLOR_RGBA)
                    ttc = planning_s + (
                        (tip_seen_at - motion_started)
                        if tip_seen_at is not None
                        else motion_duration_s
                    )
                    message = (
                        f"TIP CONTACT {leg.from_id}->{leg.to_id} "
                        f"plan_s={planning_s:.3f} motion_s={motion_duration_s:.3f} "
                        f"ttc_s={ttc:.3f}"
                    )
                    post_message(message)
                    contacted.append(leg.to_id)
                    if not retain:
                        remove_prim(stage, target_paths[leg.to_id])
                        removed.append(leg.to_id)
                    updated = replace(
                        leg,
                        contact_kind=contact.kind,
                        motion_duration_s=motion_duration_s,
                        time_to_contact_s=ttc,
                    )
                    updated_legs.append(updated)
                    print(
                        format_leg_console_row(
                            updated, episode_index=episode_index, episode_count=episode_count
                        ),
                        flush=True,
                    )
                    continue
                message = f"TIP CONTACT MISSED {leg.from_id}->{leg.to_id}"
                post_message(message)
                updated = replace(
                    leg,
                    contact_kind=ContactKind.NONE,
                    motion_duration_s=motion_duration_s,
                    failure_category=MultiTargetFailureCategory.TIP_CONTACT_MISSED,
                    failure_reason=message,
                )
                updated_legs.append(updated)
                episode_failed = True
                failure_category = MultiTargetFailureCategory.TIP_CONTACT_MISSED
                failure_reason = message
                break
        finally:
            monitor.stop()

        if not episode_failed and not plan_already_failed:
            expected = set(episode.field.contact_order_ids) - set(planned.failed_target_ids)
            if set(contacted) != expected:
                episode_failed = True
                failure_category = MultiTargetFailureCategory.TARGETS_INCOMPLETE
                failure_reason = (
                    f"contacted {sorted(contacted)} != required {sorted(expected)} "
                    f"(failed_targets={list(planned.failed_target_ids)})"
                )
        if plan_already_failed:
            episode_failed = True
            failure_category = planned.failure_category
            failure_reason = planned.failure_reason
        updated_result = replace(
            planned,
            succeeded=not episode_failed,
            failure_category=failure_category if episode_failed else None,
            failure_reason=failure_reason if episode_failed else None,
            legs=tuple(updated_legs) if updated_legs else planned.legs,
            contacted_ids=tuple(contacted),
            removed_ids=tuple(removed),
            failed_target_ids=planned.failed_target_ids,
            planning_failure_count=planned.planning_failure_count,
            target_failure_count=planned.target_failure_count,
        )
        print(format_episode_console_row(updated_result, count=episode_count), flush=True)
        return updated_result

    pass_index = 0
    while True:
        pass_label = "pass-1" if pass_index == 0 else f"replay-{pass_index}"
        if pass_index > 0:
            print(
                f"phase7_2_playback: replaying episodes ({pass_label}); "
                "close the Kit window or Ctrl+C to stop",
                flush=True,
            )
        for episode_index, planned in enumerate(planned_results):
            if pass_index > 0 and not app.is_running():
                break
            try:
                updated = _play_one_episode(
                    episode_index=episode_index,
                    planned=planned,
                    episode_count=len(planned_results),
                    pass_label=pass_label,
                )
            except _PlaybackStopped:
                print(
                    "phase7_2_playback: Kit window closed; stopping playback/replay",
                    flush=True,
                )
                pass_index = -1  # signal outer break
                break
            if pass_index == 0:
                results[episode_index] = updated
        if pass_index < 0:
            break
        if args.auto_exit:
            break
        if not app.is_running():
            break
        pass_index += 1
        try:
            # Brief settle between replays so the viewport remains responsive.
            for _ in range(GUI_VIEWPORT_SETTLE_STEPS):
                if not app.is_running():
                    break
                world.step(render=True)
        except (KeyboardInterrupt, AttributeError):
            print("phase7_2_playback: replay settle interrupted; stopping", flush=True)
            break

    summary = aggregate_multi_target_results(results, root_seed=root_seed)
    return {
        "schema_version": 1,
        "lighting_ready": lighting_ok,
        "stage_lighting_mode": stage_lighting_mode_active(),
        "joint_playback_completed": True,
        "summary": asdict(summary),
        "results": [asdict(result) for result in results],
        "frozen_requests": [serialize_episode(result.episode) for result in results],
        "error": None,
    }


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.hold_s is None:
        args.hold_s = 1.0 if args.gui else 0.25
    if args.hold_s < 0.0 or not math.isfinite(args.hold_s):
        raise ValueError("--hold-s must be finite and non-negative")
    if not args.bundle.is_file():
        raise FileNotFoundError(f"Phase 7.2 plan bundle not found: {args.bundle}")

    payload = _empty_report_payload()
    from isaacsim import SimulationApp

    if args.gui:
        print(
            "phase7_2_playback: opening Isaac Sim GUI on "
            f"DISPLAY={os.environ.get('DISPLAY', '')!r}",
            flush=True,
        )
    app = SimulationApp({"headless": not args.gui, "width": 1280, "height": 720})
    exit_code = 1
    try:
        payload = _play_validated_episodes(app=app, args=args)
        summary = payload.get("summary") or {}
        successes = int(summary.get("successes", 0))
        total = int(summary.get("total_episodes", -1))
        exit_code = 0 if successes == total else 1
    except Exception as exc:
        payload["error"] = f"{type(exc).__name__}: {exc}"
        traceback.print_exc()
        exit_code = 1
    finally:
        _write_report(args.output_report, payload)
        app.close()
    print(json.dumps({"report": str(args.output_report), "exit_code": exit_code}), flush=True)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
