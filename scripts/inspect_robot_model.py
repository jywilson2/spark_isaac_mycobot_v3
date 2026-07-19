#!/usr/bin/env python3
"""Inspect the Phase 1 robot model without issuing physical commands."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence

from mycobot_curobo.robot_model import (
    forward_kinematics,
    load_curobo_robot_config,
    load_robot_model_spec,
)


def inspect_model(config_path: Path, *, gpu: bool, warmup_iterations: int) -> dict[str, object]:
    """Return machine-readable model facts and optionally exercise cuRobo."""

    spec = load_robot_model_spec(config_path)
    cpu_pose = forward_kinematics(spec.default_joint_position_rad, spec=spec)
    result: dict[str, object] = {
        "config_path": str(spec.config_path),
        "urdf_path": str(spec.urdf_path),
        "base_link": spec.base_link,
        "flange_link": spec.flange_link,
        "tcp_link": spec.tcp_link,
        "joint_names": list(spec.joint_names),
        "tool_frames": list(spec.tool_frames),
        "default_joint_position_rad": spec.default_joint_position_rad.tolist(),
        "default_fk": {
            "position_base_m": cpu_pose.position_m.tolist(),
            "quaternion_wxyz": cpu_pose.quaternion_wxyz.tolist(),
        },
        "collision_sphere_count_by_link": spec.collision_sphere_count_by_link,
        "gpu_checked": False,
    }
    if gpu:
        from curobo.motion_planner import MotionPlanner, MotionPlannerCfg

        config = MotionPlannerCfg.create(
            robot=load_curobo_robot_config(config_path),
            self_collision_check=True,
            num_ik_seeds=8,
            num_trajopt_seeds=2,
            use_cuda_graph=False,
            max_goalset=1,
        )
        planner = MotionPlanner(config)
        planner.warmup(enable_graph=False, num_warmup_iterations=warmup_iterations)
        collision = planner.kinematics.get_self_collision_config()
        result.update(
            {
                "gpu_checked": True,
                "curobo_joint_names": list(planner.joint_names),
                "curobo_tool_frames": list(planner.tool_frames),
                "self_collision_pair_count": int(collision.collision_pairs.shape[0]),
                "warmup_iterations": warmup_iterations,
            }
        )
    return result


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("config/robots/mycobot_280_m5.yml"),
    )
    parser.add_argument("--gpu", action="store_true", help="construct and warm cuRobo planner")
    parser.add_argument("--warmup-iterations", type=int, default=1)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    if args.warmup_iterations < 1:
        raise ValueError("--warmup-iterations must be >= 1")
    print(
        json.dumps(
            inspect_model(
                args.config,
                gpu=bool(args.gpu),
                warmup_iterations=int(args.warmup_iterations),
            ),
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
