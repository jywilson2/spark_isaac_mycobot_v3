"""GPU checks for Phase 1.1 Option A overlay self-clear and detectability."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
import pytest
import yaml

from mycobot_curobo.cube_scene import batch_sphere_cube_clearance_m
from mycobot_curobo.planner import create_curobo_planner, load_planner_profile
from mycobot_curobo.robot_model import JOINT_NAMES, load_robot_model_spec
from mycobot_curobo.validation import CuroboTrajectoryEvaluator, validate_start_state

pytestmark = pytest.mark.gpu

ROOT = Path(__file__).resolve().parents[2]
ROBOT_CONFIG = ROOT / "config" / "robots" / "mycobot_280_m5.yml"
OVERLAY = ROOT / "config" / "robots" / "mycobot_280_m5_phase1_1_spheres.yml"
E = 0.014
# Seeded mid-reach interior posture from Phase 6 smoke fixtures.
MID_REACH = np.asarray([0.3, -0.1, 0.1, 0.0, 0.0, 0.0], dtype=float)


def _runtime_available() -> bool:
    if importlib.util.find_spec("curobo") is None or importlib.util.find_spec("torch") is None:
        return False
    import torch

    return bool(torch.cuda.is_available())


def _trial_robot_with_overlay() -> Path:
    trial = ROOT / "config" / "robots" / "_tmp_overlay_option_a.yml"
    payload = yaml.safe_load(ROBOT_CONFIG.read_text(encoding="utf-8"))
    payload["robot_cfg"]["kinematics"]["collision_sphere_overlay_path"] = (
        "config/robots/mycobot_280_m5_phase1_1_spheres.yml"
    )
    trial.write_text(yaml.safe_dump(payload), encoding="utf-8")
    return trial


@pytest.mark.skipif(not _runtime_available(), reason="cuRobo v0.8.0 CUDA runtime required")
def test_default_scaffolding_planner_loads_and_zero_pose_is_self_clear() -> None:
    import torch
    from curobo.types import JointState

    profile = load_planner_profile("benchmark_reproducible")
    spec = load_robot_model_spec(ROBOT_CONFIG)
    assert sum(spec.collision_sphere_count_by_link.values()) == 32

    planner = create_curobo_planner(
        profile, robot_config_path=ROBOT_CONFIG, scene_model=None, warmup=False
    )
    q = np.zeros((1, 6), dtype=float)
    state = JointState.from_position(
        torch.as_tensor(q, device="cuda:0", dtype=torch.float32),
        joint_names=list(JOINT_NAMES),
    )
    kin = planner.compute_kinematics(state)
    spheres = kin.robot_spheres.detach().cpu().numpy().reshape(1, -1, 4)
    assert spheres.shape[1] == 32

    report = validate_start_state(
        q[0],
        robot_spec=spec,
        evaluator=CuroboTrajectoryEvaluator(planner, scene_is_empty=True),
        minimum_self_collision_clearance_m=0.0,
        minimum_world_collision_clearance_m=0.0,
    )
    assert report.valid, report.violations

    clip = tuple(float(x) for x in spheres[0, spheres.shape[1] // 2, :3])
    assert float(batch_sphere_cube_clearance_m(spheres, clip, E)[0]) <= 0.0


@pytest.mark.skipif(not _runtime_available(), reason="cuRobo v0.8.0 CUDA runtime required")
def test_phase1_1_option_a_overlay_self_clear_zero_and_mid_reach() -> None:
    """Option A thickness-capped cover must pass the self-collision hard gate."""

    if not OVERLAY.is_file():
        pytest.skip("Phase 1.1 Option A overlay missing")

    trial = _trial_robot_with_overlay()
    try:
        profile = load_planner_profile("benchmark_reproducible")
        spec = load_robot_model_spec(trial)
        total = sum(spec.collision_sphere_count_by_link.values())
        assert 32 < total <= 2048
        planner = create_curobo_planner(
            profile, robot_config_path=trial, scene_model=None, warmup=False
        )
        evaluator = CuroboTrajectoryEvaluator(planner, scene_is_empty=True)
        for joints in (np.zeros(6, dtype=float), MID_REACH):
            report = validate_start_state(
                joints,
                robot_spec=spec,
                evaluator=evaluator,
                minimum_self_collision_clearance_m=0.0,
                minimum_world_collision_clearance_m=0.0,
            )
            assert report.valid, (joints.tolist(), report.violations)
    finally:
        trial.unlink(missing_ok=True)


@pytest.mark.skipif(not _runtime_available(), reason="cuRobo v0.8.0 CUDA runtime required")
def test_phase1_1_option_a_detects_body_clip_cube_of_edge_e() -> None:
    """Edge-E cube clipped into a link sphere must report non-positive clearance."""

    if not OVERLAY.is_file():
        pytest.skip("Phase 1.1 Option A overlay missing")

    import torch
    from curobo.types import JointState

    trial = _trial_robot_with_overlay()
    try:
        profile = load_planner_profile("benchmark_reproducible")
        planner = create_curobo_planner(
            profile, robot_config_path=trial, scene_model=None, warmup=False
        )
        q = np.zeros((1, 6), dtype=float)
        state = JointState.from_position(
            torch.as_tensor(q, device="cuda:0", dtype=torch.float32),
            joint_names=list(JOINT_NAMES),
        )
        kin = planner.compute_kinematics(state)
        spheres = kin.robot_spheres.detach().cpu().numpy().reshape(1, -1, 4)
        assert spheres.shape[1] > 32
        clip = tuple(float(x) for x in spheres[0, spheres.shape[1] // 2, :3])
        clearance = float(batch_sphere_cube_clearance_m(spheres, clip, E)[0])
        assert clearance <= 0.0
    finally:
        trial.unlink(missing_ok=True)
