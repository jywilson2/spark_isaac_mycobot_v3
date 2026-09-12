#!/usr/bin/env python3
"""Apply Phase 8 residual+noise corrections or tip-bias injection to a frozen bundle."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

from mycobot_curobo.residual_playback import (
    apply_cartesian_tip_bias_to_bundle,
    apply_residual_noise_to_bundle,
)
from mycobot_curobo.residual_train import train_offline_residual_policy


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def main() -> int:
    # Ensure relative config/asset lookups work when invoked from $HOME.
    os.chdir(_repo_root())

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, required=True)
    parser.add_argument("--output-bundle", type=Path, required=True)
    parser.add_argument(
        "--checkpoint",
        type=Path,
        default=None,
        help="Residual policy checkpoint (required unless --inject-tip-bias-only)",
    )
    parser.add_argument("--train-if-missing", action="store_true")
    parser.add_argument(
        "--force-retrain",
        action="store_true",
        help="Retrain the checkpoint even if it already exists",
    )
    parser.add_argument(
        "--inject-tip-bias-only",
        action="store_true",
        help="Inject Cartesian tip bias into joints (no policy); for residual-off A/B",
    )
    parser.add_argument("--measurement-noise-std-rad", type=float, default=0.01)
    parser.add_argument("--noise-seed", type=int, default=8008)
    parser.add_argument(
        "--tip-bias-m",
        type=float,
        nargs=3,
        default=(0.001, 0.0, 0.0),
        help="Tip bias: inject (+bias) and/or train policy to cancel (−bias)",
    )
    parser.add_argument(
        "--residual-safety-profile",
        type=str,
        default="simulation_bounded_residual",
    )
    parser.add_argument(
        "--actuator-noise-profile",
        type=str,
        default="simulation_default",
        help=(
            "Joint actuator/servo noise profile from config/actuator_noise.yml "
            "(distinct from tip bias and measurement noise)"
        ),
    )
    args = parser.parse_args()

    tip_bias = tuple(float(value) for value in args.tip_bias_m)
    bundle = json.loads(args.bundle.read_text(encoding="utf-8"))

    if args.inject_tip_bias_only:
        payload, stats = apply_cartesian_tip_bias_to_bundle(
            bundle,
            tip_bias_m=tip_bias,
            residual_safety_profile=args.residual_safety_profile,
            measurement_noise_std_rad=0.0,
            noise_seed=args.noise_seed,
            actuator_noise_profile=args.actuator_noise_profile,
        )
        mode = "inject_tip_bias"
    else:
        if args.checkpoint is None:
            raise SystemExit("--checkpoint is required unless --inject-tip-bias-only")
        if args.force_retrain or (args.train_if_missing and not args.checkpoint.is_file()):
            summary = train_offline_residual_policy(
                output_path=str(args.checkpoint),
                sample_count=96,
                seed=args.noise_seed,
                tip_bias_m=tip_bias,
                actuator_noise_profile=args.actuator_noise_profile,
            )
            print(
                json.dumps(
                    {"trained": True, "tip_bias_m": list(tip_bias), **summary.__dict__},
                    indent=2,
                    sort_keys=True,
                )
            )
        payload, stats = apply_residual_noise_to_bundle(
            bundle,
            checkpoint_path=str(args.checkpoint),
            measurement_noise_std_rad=args.measurement_noise_std_rad,
            noise_seed=args.noise_seed,
            residual_safety_profile=args.residual_safety_profile,
            actuator_noise_profile=args.actuator_noise_profile,
        )
        mode = "residual_correct"

    args.output_bundle.parent.mkdir(parents=True, exist_ok=True)
    args.output_bundle.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    applied = sum(item.applied_count for item in stats.values())
    nonzero = sum(item.residual_nonzero_count for item in stats.values())
    max_delta = max(
        (item.max_abs_joint_delta_rad for item in stats.values()),
        default=0.0,
    )
    actuator_meta = (payload.get("residual_playback") or {}).get("actuator_noise") or {}
    print(
        json.dumps(
            {
                "mode": mode,
                "output_bundle": str(args.output_bundle),
                "trajectories": len(stats),
                "residual_nonzero_waypoints": nonzero,
                "residual_applied_waypoints": applied,
                "max_abs_joint_delta_rad": max_delta,
                "residual_safety_profile": args.residual_safety_profile,
                "actuator_noise_profile": args.actuator_noise_profile,
                "actuator_noise_applied": bool(actuator_meta.get("applied", False)),
                "tip_bias_m": list(tip_bias),
                "sim_only": True,
                "measurement_noise_std_rad": (
                    0.0 if args.inject_tip_bias_only else args.measurement_noise_std_rad
                ),
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
