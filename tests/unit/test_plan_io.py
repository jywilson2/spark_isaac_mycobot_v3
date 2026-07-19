import json
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from mycobot_curobo.errors import ConfigurationError
from mycobot_curobo.frames import TaskFrameConfig
from mycobot_curobo.plan_io import (
    load_playback_plan,
    playback_plan_from_dict,
    require_executable_plan,
    validated_plan_to_playback_dict,
)
from mycobot_curobo.planner import NamedJointState, NominalPlan, PlanningRequest
from mycobot_curobo.robot_model import JOINT_NAMES
from mycobot_curobo.targets import SurfaceTarget
from mycobot_curobo.trajectory import JointTrajectory
from mycobot_curobo.validation import (
    ValidatedPlan,
    ValidationMetrics,
    ValidationReport,
)

DATA = Path(__file__).parents[1] / "data" / "phase7_validated_plan.json"


def test_phase7_fixture_loads_with_exact_joint_order() -> None:
    plan = load_playback_plan(DATA)
    require_executable_plan(plan)
    assert plan.joint_names == JOINT_NAMES
    assert plan.position_rad.shape == (6, 6)
    assert plan.velocity_rad_s is not None


def test_non_executable_plan_is_refused() -> None:
    payload = json.loads(DATA.read_text(encoding="utf-8"))
    payload["executable"] = False
    plan = playback_plan_from_dict(payload)
    with pytest.raises(ConfigurationError, match="not executable"):
        require_executable_plan(plan)


def test_loader_rejects_reordered_joints_and_nonfinite_values() -> None:
    payload = json.loads(DATA.read_text(encoding="utf-8"))
    payload["joint_names"][0], payload["joint_names"][1] = (
        payload["joint_names"][1],
        payload["joint_names"][0],
    )
    with pytest.raises(ConfigurationError, match="joint_names"):
        playback_plan_from_dict(payload)
    payload = json.loads(DATA.read_text(encoding="utf-8"))
    payload["position_rad"][0][0] = float("nan")
    with pytest.raises(ConfigurationError, match="finite"):
        playback_plan_from_dict(payload)


def test_validated_plan_conversion_preserves_typed_contract() -> None:
    positions = np.zeros((2, 6))
    trajectory = JointTrajectory(
        joint_names=JOINT_NAMES,
        position_rad=positions,
        velocity_rad_s=np.zeros_like(positions),
        acceleration_rad_s2=np.zeros_like(positions),
        jerk_rad_s3=np.zeros_like(positions),
        dt_s=0.02,
    )
    target = SurfaceTarget.create(
        position_base_m=[0.1, 0.0, 0.2],
        surface_normal_base=[0.0, 0.0, 1.0],
        fixed_roll_rad=0.0,
        target_id="phase7-conversion",
    )
    request = PlanningRequest(
        current_joint_state=NamedJointState.create(JOINT_NAMES, np.zeros(6)),
        surface_target=target,
        scene_revision="empty",
        planner_profile="development_fast",
        random_seed=7,
        request_id="phase7-conversion",
    )
    nominal = NominalPlan(
        request_id=request.request_id,
        selected_goal_index=0,
        selected_roll_rad=0.0,
        approach_trajectory=trajectory,
        terminal_trajectory=trajectory,
        combined_trajectory=trajectory,
        planner_status="success",
        planner_timings_s={},
        curobo_version="0.8.0",
        scene_revision="empty",
        planner_profile="development_fast",
        random_seed=7,
    )
    metric_values = {field: 0.0 for field in ValidationMetrics.__dataclass_fields__}
    report = ValidationReport(
        request_id=request.request_id,
        profile_name="simulation_initial",
        valid=True,
        violations=(),
        metrics=ValidationMetrics(**metric_values),
    )
    validated = ValidatedPlan(nominal, report, "valid", True)
    payload = validated_plan_to_playback_dict(validated, request, TaskFrameConfig())
    loaded = playback_plan_from_dict(payload)
    assert loaded.executable
    assert loaded.request_id == request.request_id
    assert np.array_equal(loaded.position_rad, positions)

    invalid = replace(validated, executable=False, validation_status="invalid")
    assert not validated_plan_to_playback_dict(invalid, request, TaskFrameConfig())["executable"]
