"""Unit test for Option B trial app/robot writer."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "host" / "write_option_b_trial_app.py"


def test_write_option_b_trial_app_arms_dual_overlay(tmp_path: Path) -> None:
    trial_robot = tmp_path / "trial_robot.yml"
    trial_app = tmp_path / "trial_app.yml"
    proc = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--repo-root",
            str(ROOT),
            "--trial-robot",
            str(trial_robot),
            "--trial-app",
            str(trial_app),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    assert "option_b_trial" in proc.stdout
    robot = yaml.safe_load(trial_robot.read_text(encoding="utf-8"))
    kin = robot["robot_cfg"]["kinematics"]
    assert kin["collision_sphere_overlay_role"] == "dual"
    assert "phase1_1_spheres.yml" in kin["collision_sphere_overlay_path"]
    app = yaml.safe_load(trial_app.read_text(encoding="utf-8"))
    assert Path(app["robot_config_path"]).name == trial_robot.name
