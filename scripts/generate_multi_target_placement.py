#!/usr/bin/env python3
"""CPU-only multi-target suite placement generation with streamed logging."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict, replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from mycobot_curobo.errors import ConfigurationError  # noqa: E402
from mycobot_curobo.multi_target import (  # noqa: E402
    load_multi_target_suite_config,
    override_suite_target_count,
    resolve_invocation_root_seed,
    sample_multi_target_episodes,
)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=ROOT / "config/phase7_4_multi_target_standard_2x20_delta_z_0_30.yml",
    )
    parser.add_argument("--root-seed", type=int, default=None)
    parser.add_argument("--episodes", type=int, default=None)
    parser.add_argument("--targets", type=int, default=None)
    parser.add_argument(
        "--output-json",
        type=Path,
        default=None,
        help="Optional JSON dump of centres + reach rejections + timing.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    config = load_multi_target_suite_config(args.config)
    if args.targets is not None:
        config = override_suite_target_count(config, args.targets)
    # CPU placement stream has no cuRobo tip-IK screen; host plan_multi_target
    # enforces require_tip_ik. Skip here so densest YAML still generates centres.
    if config.require_tip_ik:
        print(
            "phase7_4_placement: skipping tip IK pre-screen "
            "(CPU placement-only; use plan_multi_target_suite on host GPU)",
            flush=True,
        )
        config = replace(config, require_tip_ik=False)
    independent = args.root_seed is None
    root_seed = None if independent else resolve_invocation_root_seed(args.root_seed)
    stats_out: list = []

    def _log(message: str) -> None:
        print(message, flush=True)

    episodes = sample_multi_target_episodes(
        config,
        root_seed=root_seed,
        episode_count=args.episodes,
        independent_random_episode_seeds=independent,
        log=_log,
        placement_stats_out=stats_out,
    )
    for episode in episodes:
        for target in episode.field.targets:
            c = target.center_m
            print(
                f"phase7_4_placement: episode={episode.episode_index} "
                f"target={target.target_id} "
                f"centre=({c[0]:.4f},{c[1]:.4f},{c[2]:.4f})",
                flush=True,
            )
    stats = stats_out[0]
    summary = {
        "config": str(args.config),
        "episodes": len(episodes),
        "targets_per_episode": config.target_count,
        "generation_duration_s": stats.generation_duration_s,
        "max_reach_rejections": stats.max_reach_rejections,
        "reach_rejection_count": len(stats.reach_rejections),
        "reach_rejections": [asdict(item) for item in stats.reach_rejections],
        "fields": [
            {
                "episode_index": episode.episode_index,
                "episode_seed": episode.episode_seed,
                "centres_m": [list(t.center_m) for t in episode.field.targets],
            }
            for episode in episodes
        ],
    }
    print(
        "phase7_4_placement: suite target generation summary "
        f"duration_s={stats.generation_duration_s:.3f} "
        f"rejections={len(stats.reach_rejections)}/{stats.max_reach_rejections}",
        flush=True,
    )
    print(json.dumps({"placement_ok": True, **summary}, sort_keys=True), flush=True)
    if args.output_json is not None:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ConfigurationError as exc:
        print(f"ERROR: {exc}", file=sys.stderr, flush=True)
        raise SystemExit(2) from exc
