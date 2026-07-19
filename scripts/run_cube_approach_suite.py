#!/usr/bin/env python3
"""Sample and print Phase 7.1 cube episodes without importing Isaac Sim."""

from __future__ import annotations

import argparse

from mycobot_curobo.cube_suite import load_cube_suite_config, sample_cube_episodes


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config/phase7_1_cube_suite.yml")
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--episodes", type=int, default=None)
    args = parser.parse_args()
    config = load_cube_suite_config(args.config)
    seed = config.root_seed if args.seed is None else args.seed
    for episode in sample_cube_episodes(config, root_seed=seed, episode_count=args.episodes):
        print(
            f"[{episode.episode_index + 1}] {episode.request_id} "
            f"start={episode.start_mode.value}/{episode.start_label} "
            f"cube={episode.cube_center_m} normal={episode.normal_bin_label} "
            f"scene={episode.scene_revision}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
