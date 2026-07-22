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
CHAINED_GUI = ROOT / "scripts" / "host" / "run_phase7_1_chained_gui.sh"
VERIFY = ROOT / "scripts" / "run_verification.sh"
PLAYER = ROOT / "isaac_sim" / "play_nominal_plan.py"
PHASE7_1_PLAYER = ROOT / "isaac_sim" / "play_cube_suite.py"
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
    assert "smoke_phase7_1_cube_suite.sh --gui --auto-exit --all-modes" in verification
    assert "smoke_phase7_2_multi_target.sh --gui --auto-exit" in verification
    assert "--with-integration-smoke" in verification
    assert "smoke_phase7_2_integration_2x5.sh" in verification
    assert "SPARK_RUN_INTEGRATION_SMOKE" in verification
    smoke72 = (ROOT / "scripts/host/smoke_phase7_2_multi_target.sh").read_text(encoding="utf-8")
    smoke72_int = (ROOT / "scripts/host/smoke_phase7_2_integration_2x5.sh").read_text(
        encoding="utf-8"
    )
    gpu72 = (ROOT / "scripts/host/run_phase7_2_gpu.sh").read_text(encoding="utf-8")
    assert "test_phase7_2_multi_target_gpu.py" in gpu72
    assert "--targets" in smoke72
    assert 'plan_args+=(--targets "${targets}")' in smoke72
    assert "--episodes" in smoke72
    assert 'plan_args+=(--episodes "${episodes}")' in smoke72
    assert "--config" in smoke72
    assert "plan_status" in smoke72
    assert "phase7_2_multi_target_integration_2x5.yml" in smoke72_int
    assert "--targets 5" in smoke72_int
    assert "--episodes 2" in smoke72_int
    player72 = (ROOT / "isaac_sim" / "play_multi_target_suite.py").read_text(encoding="utf-8")
    assert "replaying episodes" in player72
    assert "Kit window closed" in player72
    assert "spark_host_exec.sh" in verification
    assert "SPARK_RUN_ISAAC_GUI_SMOKE" not in verification
    player = PLAYER.read_text(encoding="utf-8")
    assert "prepare_illuminated_stage" in player
    assert "configure_kit_for_stage_lighting" in player
    assert "stage_lighting_mode" in player
    assert "scene_setup" in player
    phase71 = PHASE7_1_PLAYER.read_text(encoding="utf-8")
    assert "set_joint_position_targets" in phase71
    assert "prepare_illuminated_stage" in phase71
    assert "configure_kit_for_stage_lighting" in phase71
    scene_setup = (ROOT / "isaac_sim" / "scene_setup.py").read_text(encoding="utf-8")
    assert "enable_viewport_stage_lighting" in scene_setup
    assert "SetLightingMenuModeCommand" in scene_setup
    assert "autoLightRig/enabled" in scene_setup
    assert "label_digit_segment_boxes" in scene_setup
    assert "label_parent_local_offset_m" in scene_setup
    assert "PENDING_CONTACT_COLOR_RGBA" in scene_setup
    assert "TIP_CONTACT_FAILED_COLOR_RGBA" in scene_setup
    assert "add_target_label" in player72
    assert "PENDING_CONTACT_COLOR_RGBA" in player72
    assert "TIP_CONTACT_FAILED_COLOR_RGBA" in player72
    assert "frame_viewport_on_arm" in scene_setup
    assert "compute_viewport_framing" in scene_setup
    assert "frame_viewport_on_arm" in player72
    assert "compute_viewport_framing" in player72
    assert "frame_viewport_on_arm" in phase71


def test_chained_gui_host_script_wires_mode_b_and_episodes() -> None:
    script = CHAINED_GUI.read_text(encoding="utf-8")
    planner = (ROOT / "isaac_sim" / "plan_cube_suite.py").read_text(encoding="utf-8")
    assert CHAINED_GUI.exists()
    assert "spark_host_require_native_shell" in script
    assert "spark_require_gui_display" in script
    assert "plan_cube_suite.py" in script
    assert "play_cube_suite.py" in script
    assert "--GUI|--gui" in script
    assert 'mode="--gui"' in script
    assert "--chained" in script
    assert "--episodes" in script
    assert "episodes=20" in script
    assert '"--chained"' in planner or "chained" in planner
    assert 'force_modes = ("B", "D")' in planner


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
