import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from isaac_sim.articulation_playback import (
    REVOLUTE_JOINT_NAMES,
    articulation_position_targets,
    revolute_dof_indices,
)

ROOT = Path(__file__).parents[2]
SMOKE = ROOT / "scripts" / "host" / "smoke_isaac_viz.sh"
VERIFY = ROOT / "scripts" / "run_verification.sh"
PLAYER = ROOT / "isaac_sim" / "play_nominal_plan.py"
PLAN = ROOT / "tests" / "data" / "phase7_validated_plan.json"


def test_articulation_mapping_handles_extra_dofs() -> None:
    names = ("gripper", *REVOLUTE_JOINT_NAMES)
    assert revolute_dof_indices(names) == (1, 2, 3, 4, 5, 6)
    targets = articulation_position_targets(np.arange(6), names, np.zeros(7))
    assert np.array_equal(targets, [0.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0])
    with pytest.raises(ValueError, match="missing"):
        revolute_dof_indices(REVOLUTE_JOINT_NAMES[:-1])


def test_smoke_and_verification_wire_required_gui_gate() -> None:
    smoke = SMOKE.read_text(encoding="utf-8")
    verification = VERIFY.read_text(encoding="utf-8")
    assert "spark_host_check_prereqs" in smoke or "check_prereqs.sh" in smoke
    assert "convert_urdf_to_usd.sh" in smoke
    assert "play_nominal_plan.py" in smoke
    assert "phase7_validated_plan.json" in smoke
    assert "PHASE7_" + "NOT_IMPLEMENTED" not in smoke
    assert "--gui --auto-exit" in verification
    assert "spark_host_exec.sh" in verification
    assert "SPARK_RUN_ISAAC_GUI_SMOKE" not in verification


def test_player_refuses_invalid_plan_before_isaac_import(tmp_path: Path) -> None:
    payload = json.loads(PLAN.read_text(encoding="utf-8"))
    payload["executable"] = False
    invalid = tmp_path / "invalid.json"
    invalid.write_text(json.dumps(payload), encoding="utf-8")
    result = subprocess.run(
        [
            sys.executable,
            str(PLAYER),
            "--plan",
            str(invalid),
            "--output-metrics",
            str(tmp_path / "metrics.json"),
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode != 0
    assert "not executable" in result.stderr
    assert "isaacsim" not in result.stderr
