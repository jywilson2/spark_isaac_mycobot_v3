#!/usr/bin/env python3
"""Play one independently validated joint plan in Isaac Sim 6.x."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
if str(REPO_ROOT / "src") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "src"))

from isaac_sim.articulation_playback import articulation_position_targets  # noqa: E402
from isaac_sim.sim_metrics import orientation_error_rad, tip_position_error_m  # noqa: E402
from isaac_sim.urdf_utils import default_prepared_urdf  # noqa: E402
from mycobot_curobo.plan_io import load_playback_plan, require_executable_plan  # noqa: E402


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--usd", type=Path)
    parser.add_argument("--repo-root", type=Path, default=REPO_ROOT)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--headless", action="store_true", default=True)
    mode.add_argument("--gui", action="store_true")
    parser.add_argument("--auto-exit", action="store_true", default=True)
    parser.add_argument("--no-auto-exit", action="store_false", dest="auto_exit")
    parser.add_argument("--hold-s", type=float, default=0.5)
    parser.add_argument("--output-metrics", type=Path, required=True)
    return parser.parse_args(argv)


def _find_articulation_root(stage: Any) -> str:
    from pxr import UsdPhysics

    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return str(prim.GetPath())
    raise RuntimeError("USD contains no articulation root")


def _tip_pose_wxyz(stage: Any) -> tuple[list[float], list[float]] | None:
    from pxr import Usd, UsdGeom

    tip_prim = next((prim for prim in stage.Traverse() if prim.GetName() == "tcp_link"), None)
    if tip_prim is None:
        return None
    transform = UsdGeom.Xformable(tip_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    translation = transform.ExtractTranslation()
    quaternion = transform.ExtractRotationQuat()
    imaginary = quaternion.GetImaginary()
    return (
        [float(translation[0]), float(translation[1]), float(translation[2])],
        [
            float(quaternion.GetReal()),
            float(imaginary[0]),
            float(imaginary[1]),
            float(imaginary[2]),
        ],
    )


def _write_metrics(path: Path, metrics: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    plan = load_playback_plan(args.plan)
    require_executable_plan(plan)
    if args.hold_s < 0.0 or not math.isfinite(args.hold_s):
        raise ValueError("--hold-s must be finite and non-negative")
    repo = args.repo_root.resolve()
    default_usd = default_prepared_urdf(repo).with_suffix(".usd")
    nested_usda = default_usd.with_suffix("") / default_usd.with_suffix(".usda").name
    usd = (args.usd or (default_usd if default_usd.is_file() else nested_usda)).resolve()
    if not usd.is_file():
        raise FileNotFoundError(f"prepared robot USD not found: {usd}")

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": not args.gui})
    metrics: dict[str, Any] = {
        "schema_version": 1,
        "request_id": plan.request_id,
        "joint_playback_completed": False,
        "waypoints_played": 0,
        "tip_metrics_status": "not_evaluated",
        "tip_position_error_m": None,
        "tip_orientation_error_rad": None,
    }
    try:
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation

        context = omni.usd.get_context()
        if not context.open_stage(str(usd)):
            raise RuntimeError(f"failed to open USD stage: {usd}")
        for _ in range(10):
            app.update()
        stage = context.get_stage()
        articulation_path = _find_articulation_root(stage)
        world = World(stage_units_in_meters=1.0)
        robot = world.scene.add(
            SingleArticulation(prim_path=articulation_path, name="mycobot_phase7")
        )
        world.reset()
        robot.initialize()
        dof_names = tuple(str(name) for name in robot.dof_names)
        current = robot.get_joint_positions()
        if current is None:
            raise RuntimeError("articulation returned no joint positions")
        for waypoint in plan.position_rad:
            targets = articulation_position_targets(waypoint, dof_names, current)
            robot.set_joint_positions(targets)
            current = targets
            steps = max(1, int(math.ceil(plan.dt_s / world.get_physics_dt())))
            for _ in range(steps):
                world.step(render=args.gui)
            metrics["waypoints_played"] += 1
        hold_steps = int(math.ceil(args.hold_s / world.get_physics_dt()))
        for _ in range(hold_steps):
            world.step(render=args.gui)
        metrics["joint_playback_completed"] = True
        tip_pose = _tip_pose_wxyz(stage)
        if tip_pose is not None:
            position, quaternion = tip_pose
            metrics["tip_position_error_m"] = tip_position_error_m(
                position, plan.goal_position_base_m
            )
            metrics["tip_orientation_error_rad"] = orientation_error_rad(
                quaternion, plan.goal_quaternion_wxyz
            )
            metrics["tip_metrics_status"] = "evaluated"
        _write_metrics(args.output_metrics, metrics)
        print(json.dumps(metrics, sort_keys=True))
        if not args.auto_exit:
            while app.is_running():
                world.step(render=True)
        return 0
    except Exception as exc:
        metrics["error"] = f"{type(exc).__name__}: {exc}"
        _write_metrics(args.output_metrics, metrics)
        print(metrics["error"], file=sys.stderr)
        sys.stderr.flush()
        os._exit(1)
    finally:
        app.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"{type(exc).__name__}: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc
