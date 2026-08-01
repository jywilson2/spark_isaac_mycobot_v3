#!/usr/bin/env python3
"""Write a temporary Option B dual-armed robot YAML and app.yml."""

from __future__ import annotations

import argparse
from pathlib import Path

import yaml


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, required=True)
    parser.add_argument("--trial-robot", type=Path, required=True)
    parser.add_argument("--trial-app", type=Path, required=True)
    args = parser.parse_args()
    root = args.repo_root.resolve()
    src = root / "config" / "robots" / "mycobot_280_m5.yml"
    payload = yaml.safe_load(src.read_text(encoding="utf-8"))
    kin = payload["robot_cfg"]["kinematics"]
    kin["collision_sphere_overlay_path"] = "config/robots/mycobot_280_m5_phase1_1_spheres.yml"
    kin["collision_sphere_overlay_role"] = "dual"
    args.trial_robot.parent.mkdir(parents=True, exist_ok=True)
    args.trial_robot.write_text(yaml.safe_dump(payload), encoding="utf-8")
    app = yaml.safe_load((root / "config" / "app.yml").read_text(encoding="utf-8"))
    # Store a path relative to config/ so load_app_config resolves against repo root.
    rel_robot = args.trial_robot.resolve().relative_to(root)
    app["robot_config_path"] = str(rel_robot)
    args.trial_app.parent.mkdir(parents=True, exist_ok=True)
    args.trial_app.write_text(yaml.safe_dump(app), encoding="utf-8")
    print(f"option_b_trial: robot={args.trial_robot}")
    print(f"option_b_trial: app={args.trial_app}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
