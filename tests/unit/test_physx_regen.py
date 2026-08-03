"""Unit tests for Phase 7.5 PhysX accept/regen helpers and suite loop."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from mycobot_curobo.incremental_population import IncrementalPopulationRunner
from mycobot_curobo.multi_target import (
    MultiTargetFailureCategory,
    OptimisticTipContactDetector,
    load_multi_target_suite_config,
)
from mycobot_curobo.physx_regen import (
    PhysxDiscardRecord,
    discard_from_physx_failure_payload,
    format_physx_regen_discard_line,
    min_self_sphere_clearance,
    physx_retry_episode_seed,
)
from mycobot_curobo.planner import NominalPlan, PlanningFailure, PlanningOutcome
from mycobot_curobo.trajectory import JointTrajectory
from mycobot_curobo.validation import ValidatedPlan, ValidationMetrics, ValidationReport

ROOT = Path(__file__).parents[2]


def _trajectory(positions: list[list[float]]) -> JointTrajectory:
    array = np.asarray(positions, dtype=float)
    zeros = np.zeros_like(array)
    return JointTrajectory(
        joint_names=(
            "joint2_to_joint1",
            "joint3_to_joint2",
            "joint4_to_joint3",
            "joint5_to_joint4",
            "joint6_to_joint5",
            "joint6output_to_joint6",
        ),
        position_rad=array,
        dt_s=0.05,
        velocity_rad_s=zeros,
        acceleration_rad_s2=zeros,
        jerk_rad_s3=zeros,
    )


def _plan(request_id: str, seed: int) -> NominalPlan:
    contact = 0.2
    retreat_end = contact + 0.05
    approach = _trajectory([[0.0] * 6, [0.1] * 6])
    terminal = _trajectory([[0.1] * 6, [contact] * 6])
    retreat = _trajectory([[contact] * 6, [retreat_end] * 6])
    combined = _trajectory([[0.0] * 6, [0.1] * 6, [contact] * 6, [retreat_end] * 6])
    return NominalPlan(
        request_id=request_id,
        selected_goal_index=0,
        selected_roll_rad=0.0,
        approach_trajectory=approach,
        terminal_trajectory=terminal,
        combined_trajectory=combined,
        planner_status="success",
        planner_timings_s={"total": 0.01},
        curobo_version="0.8.0",
        scene_revision="test",
        planner_profile="benchmark_reproducible",
        random_seed=seed,
        retreat_trajectory=retreat,
    )


def _metrics() -> ValidationMetrics:
    return ValidationMetrics(
        max_lateral_error_m=0.0,
        max_approach_axis_error_rad=0.0,
        max_roll_error_rad=0.0,
        terminal_position_error_m=0.0,
        terminal_orientation_error_rad=0.0,
        max_progress_regression_m=0.0,
        minimum_joint_limit_margin_rad=0.0,
        minimum_self_collision_clearance_m=0.0,
        minimum_world_collision_clearance_m=0.0,
    )


def _validated(plan: NominalPlan) -> ValidatedPlan:
    return ValidatedPlan(
        nominal_plan=plan,
        report=ValidationReport(
            request_id=plan.request_id,
            profile_name="simulation_initial",
            valid=True,
            violations=(),
            metrics=_metrics(),
        ),
        validation_status="passed",
        executable=True,
    )


def _ok() -> PlanningOutcome:
    return PlanningOutcome(plan=_plan("ok", 0), failure=None)


def _fail() -> PlanningOutcome:
    return PlanningOutcome(
        plan=None,
        failure=PlanningFailure("planning_infeasible", "fake", "fail"),
    )


class _FakePlanner:
    def __init__(self, outcomes: list[PlanningOutcome]) -> None:
        self._outcomes = list(outcomes)

    def plan(self, request: Any) -> PlanningOutcome:
        del request
        if not self._outcomes:
            return _fail()
        return self._outcomes.pop(0)


def test_physx_retry_episode_seed_is_base_plus_attempt() -> None:
    assert physx_retry_episode_seed(4242, 1) == 4243
    assert physx_retry_episode_seed(4242, 3) == 4245
    with pytest.raises(ValueError):
        physx_retry_episode_seed(1, 0)


def test_min_self_sphere_clearance_and_pair_label() -> None:
    spheres = np.asarray(
        [
            [0.0, 0.0, 0.0, 0.05],
            [0.2, 0.0, 0.0, 0.05],
            [0.05, 0.0, 0.0, 0.05],
        ],
        dtype=float,
    )
    pairs = np.asarray([[0, 1], [0, 2]], dtype=int)
    clearance, label = min_self_sphere_clearance(spheres, pairs, sphere_link_names=("a", "b", "c"))
    # pair 0-2: distance 0.05 - 0.05 - 0.05 = -0.05
    assert clearance == pytest.approx(-0.05)
    assert label == "a@0↔c@2"


def test_discard_log_line_includes_sphere_cover_fields() -> None:
    discard = PhysxDiscardRecord(
        episode_index=2,
        regen_attempt=1,
        episode_seed=5251,
        category="self_collision",
        leg_from_id="1",
        leg_to_id="6",
        request_id="ep002_1_to_6_attempt0",
        links="joint2↔joint5",
        target_id=None,
        waypoint_index=12,
        waypoint_count=48,
        u=0.25,
        t_s=0.6,
        q_rad=(0.1, 0.2, 0.3, 0.4, 0.5, 0.6),
        sphere_clearance_m=0.0021,
        sphere_pair="joint2@2↔joint5@0",
        reason="mesh fold not covered by spheres",
    )
    line = format_physx_regen_discard_line(
        episode_index=2,
        episode_count=3,
        regen_attempt=1,
        max_physx_regenerations=3,
        discard=discard,
    )
    assert line.startswith("phase7_5_physx_regen: ep 3/3 regen 1/3 DISCARD")
    assert "links joint2↔joint5" in line
    assert "waypoint 12/48 u=0.250 t_s=0.600" in line
    assert "q_rad=[" in line
    assert "sphere_clearance_m=0.0021" in line
    assert "sphere_pair=joint2@2↔joint5@0" in line


def test_discard_from_failure_payload_round_trip() -> None:
    record = discard_from_physx_failure_payload(
        episode_index=0,
        regen_attempt=2,
        episode_seed=100,
        failure={
            "category": "body_contact",
            "leg_from_id": "start",
            "leg_to_id": "3",
            "request_id": "r1",
            "links": "joint3",
            "target_id": "3",
            "waypoint_index": 4,
            "waypoint_count": 10,
            "u": 0.444,
            "t_s": 0.2,
            "q_rad": [0.0, 0.1, 0.2, 0.3, 0.4, 0.5],
            "reason": "BODY CONTACT",
        },
        sphere_clearance_m=0.01,
        sphere_pair="sphere0↔sphere1",
    )
    assert record.category == "body_contact"
    assert record.target_id == "3"
    assert record.q_rad == (0.0, 0.1, 0.2, 0.3, 0.4, 0.5)
    assert record.to_dict()["sphere_pair"] == "sphere0↔sphere1"


def test_physx_gate_regenerates_until_pass(tmp_path: Path) -> None:
    import yaml

    payload = yaml.safe_load(
        (ROOT / "config/phase7_5_variable_targets_dz_0_30.yml").read_text(encoding="utf-8")
    )
    payload.update(
        {
            "max_physx_regenerations": 3,
            "max_targets_per_episode": 1,
            "min_targets_per_episode": 1,
            "max_total_target_failures": 5,
            "max_consecutive_target_failures": 0,
            "max_placement_attempts": 1000,
        }
    )
    path = tmp_path / "suite.yml"
    path.write_text(yaml.safe_dump(payload), encoding="utf-8")
    config = load_multi_target_suite_config(path)

    # Enough OK outcomes for several population attempts (1 target each).
    planner = _FakePlanner([_ok() for _ in range(10)])

    def planner_factory(seed: int, scene_model: dict, links: tuple[str, ...]) -> _FakePlanner:
        del seed, scene_model, links
        return planner

    calls: list[int] = []

    def gate(result, extra, traj):  # type: ignore[no-untyped-def]
        del extra, traj
        calls.append(int(result.episode.episode_seed))
        if len(calls) < 3:
            discard = PhysxDiscardRecord(
                episode_index=0,
                regen_attempt=len(calls),
                episode_seed=int(result.episode.episode_seed),
                category="self_collision",
                leg_from_id="start",
                leg_to_id=result.contacted_ids[0] if result.contacted_ids else "1",
                request_id="r",
                links="joint2↔joint5",
                target_id=None,
                waypoint_index=1,
                waypoint_count=4,
                u=0.333,
                t_s=0.05,
                q_rad=(0.0, 0.1, 0.2, 0.3, 0.4, 0.5),
                sphere_clearance_m=0.001,
                sphere_pair="joint2@0↔joint5@0",
                reason="fake self collision",
            )
            return False, discard
        return True, None

    lines: list[str] = []
    runner = IncrementalPopulationRunner(
        planner_factory=planner_factory,
        validator=lambda plan, request, clearance, cube: _validated(plan),
        contact_detector_factory=lambda _ep, to_id: OptimisticTipContactDetector(to_id),
        console_log=lines.append,
        apply_reach_prefilter=False,
    )
    suite = runner.run_suite(config, root_seed=4242, episode_count=1, physx_gate_fn=gate)
    assert suite.suite_accepted
    assert not suite.physx_regeneration_exhausted
    assert len(calls) == 3
    # base seed = 4242 + 1009*1 = 5251; retries +1, +2
    assert calls[0] == 5251
    assert calls[1] == 5252
    assert calls[2] == 5253
    assert suite.extras[0].physx_acceptance == "pass"
    assert suite.extras[0].physx_regen_attempts == 2
    assert len(suite.extras[0].physx_discards) == 2
    assert any("DISCARD" in line and "links joint2↔joint5" in line for line in lines)
    assert any("ACCEPT | physx_regen_attempts=2" in line for line in lines)


def test_sanitized_gate_env_strips_kit_injected_vars() -> None:
    from isaac_sim.physx_episode_gate import sanitized_gate_env

    base = {
        "LD_PRELOAD": "/isaacsim/kit/libcarb.so",
        "CARB_APP_PATH": "/isaacsim/kit",
        "OMNI_KIT_ALLOW_ROOT": "1",
        "EXP_PATH": "/isaacsim/apps",
        "ISAAC_PATH": "/isaacsim",
        "isaac_sim_package_path": "/isaacsim",
        "PYTHONPATH": "/repo/src:/repo:/repo/src:/repo:/isaacsim/kit/python",
        "PYTHONHOME": "/isaacsim/kit/python",
        "LD_LIBRARY_PATH": "/isaacsim/kit:/opt/ros/jazzy/lib",
        "ISAACSIM_PATH": "/isaacsim",
        "ISAACSIM_PYTHON_EXE": "/isaacsim/python.sh",
        "PATH": "/usr/bin",
        "HOME": "/home/user",
        "DISPLAY": ":1",
    }
    env = sanitized_gate_env(base, repo_root=Path("/repo"))
    for kit_var in (
        "LD_PRELOAD",
        "CARB_APP_PATH",
        "OMNI_KIT_ALLOW_ROOT",
        "EXP_PATH",
        "ISAAC_PATH",
        "isaac_sim_package_path",
        "PYTHONHOME",
        "LD_LIBRARY_PATH",
    ):
        assert kit_var not in env
    # PYTHONPATH is reset to exactly the repo entries (no Kit paths, no dupes).
    assert env["PYTHONPATH"] == "/repo/src:/repo"
    # python.sh discovery and generic host vars survive.
    assert env["ISAACSIM_PATH"] == "/isaacsim"
    assert env["ISAACSIM_PYTHON_EXE"] == "/isaacsim/python.sh"
    assert env["PATH"] == "/usr/bin"
    assert env["HOME"] == "/home/user"
    assert env["DISPLAY"] == ":1"


def test_resolve_gate_python_prefers_isaacsim_python_sh(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    from isaac_sim.physx_episode_gate import resolve_gate_python

    # Explicit argument always wins.
    assert resolve_gate_python("/custom/python") == "/custom/python"

    python_sh = tmp_path / "python.sh"
    python_sh.write_text("#!/bin/bash\n", encoding="utf-8")
    monkeypatch.setenv("ISAACSIM_PYTHON_EXE", str(python_sh))
    assert resolve_gate_python() == str(python_sh)

    # Fall back to ISAACSIM_PATH/python.sh when the exe var is absent.
    monkeypatch.delenv("ISAACSIM_PYTHON_EXE", raising=False)
    monkeypatch.setenv("ISAACSIM_PATH", str(tmp_path))
    assert resolve_gate_python() == str(python_sh)

    # Never nest under Kit implicitly: last resort is sys.executable.
    monkeypatch.delenv("ISAACSIM_PATH", raising=False)
    assert resolve_gate_python() == sys.executable


def test_gate_timeout_fails_closed(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """A hung gate playback is killed and discarded instead of wedging the suite."""

    import yaml

    from isaac_sim import physx_episode_gate as gate_mod

    payload = yaml.safe_load(
        (ROOT / "config/phase7_5_variable_targets_dz_0_30.yml").read_text(encoding="utf-8")
    )
    payload.update(
        {
            "max_physx_regenerations": 1,
            "max_targets_per_episode": 1,
            "min_targets_per_episode": 1,
            "max_total_target_failures": 5,
            "max_placement_attempts": 1000,
        }
    )
    config_path = tmp_path / "suite.yml"
    config_path.write_text(yaml.safe_dump(payload), encoding="utf-8")
    config = load_multi_target_suite_config(config_path)

    # Populate one real episode to obtain result/extra/trajectories payloads.
    captured: dict[str, Any] = {}

    def capture_gate(result, extra, traj):  # type: ignore[no-untyped-def]
        captured["result"] = result
        captured["extra"] = extra
        captured["traj"] = dict(traj)
        return True, None

    runner = IncrementalPopulationRunner(
        planner_factory=lambda seed, scene, links: _FakePlanner([_ok() for _ in range(10)]),
        validator=lambda plan, request, clearance, cube: _validated(plan),
        contact_detector_factory=lambda _ep, to_id: OptimisticTipContactDetector(to_id),
        console_log=lambda _line: None,
        apply_reach_prefilter=False,
    )
    runner.run_suite(config, root_seed=4242, episode_count=1, physx_gate_fn=capture_gate)
    assert "result" in captured

    launches: dict[str, Any] = {}

    class _HungProc:
        pid = 999999
        returncode = None

        def __init__(self) -> None:
            self._timed_out_once = False

        def communicate(self, timeout: float | None = None) -> tuple[str, None]:
            if timeout is not None and not self._timed_out_once:
                self._timed_out_once = True
                raise subprocess.TimeoutExpired(cmd="gate", timeout=timeout)
            return "kit bootstrap noise", None

    def fake_popen(cmd, **kwargs):  # type: ignore[no-untyped-def]
        launches["cmd"] = list(cmd)
        launches["env"] = kwargs.get("env")
        launches["start_new_session"] = kwargs.get("start_new_session")
        return _HungProc()

    killed: list[int] = []
    monkeypatch.setattr(gate_mod.subprocess, "Popen", fake_popen)
    monkeypatch.setattr(gate_mod.os, "killpg", lambda pid, sig: killed.append(pid))

    passed, discard = gate_mod.run_headless_physx_gate(
        result=captured["result"],
        extra=captured["extra"],
        trajectories=captured["traj"],
        tip_allow_link_names=config.tip_allow_link_names,
        lighting={},
        z_density={},
        root_seed=4242,
        max_failed_episodes=0,
        usd=tmp_path / "robot.usda",
        python_exe="/fake/python.sh",
        timeout_s=1.5,
    )
    assert passed is False
    assert discard is not None
    assert discard.category == "physx_overlap"
    assert "gate_timeout" in discard.reason
    assert killed == [999999]
    # Child launch is isolated: explicit exe, new session, sanitized env.
    assert launches["cmd"][0] == "/fake/python.sh"
    assert launches["start_new_session"] is True
    child_env = launches["env"]
    assert child_env is not None
    assert "LD_PRELOAD" not in child_env
    assert not any(key.startswith(("CARB_", "OMNI_")) for key in child_env)
    assert child_env["PYTHONPATH"].count(str(ROOT)) == 2  # src + repo root only


def test_physx_gate_exhaustion_fails_closed(tmp_path: Path) -> None:
    import yaml

    payload = yaml.safe_load(
        (ROOT / "config/phase7_5_variable_targets_dz_0_30.yml").read_text(encoding="utf-8")
    )
    payload.update(
        {
            "max_physx_regenerations": 2,
            "max_targets_per_episode": 1,
            "min_targets_per_episode": 1,
            "max_total_target_failures": 5,
            "max_placement_attempts": 1000,
        }
    )
    path = tmp_path / "suite.yml"
    path.write_text(yaml.safe_dump(payload), encoding="utf-8")
    config = load_multi_target_suite_config(path)
    planner = _FakePlanner([_ok() for _ in range(10)])

    def gate(result, extra, traj):  # type: ignore[no-untyped-def]
        del extra, traj
        return False, PhysxDiscardRecord(
            episode_index=0,
            regen_attempt=1,
            episode_seed=int(result.episode.episode_seed),
            category="self_collision",
            leg_from_id="start",
            leg_to_id="1",
            request_id="r",
            links="joint2↔joint4",
            target_id=None,
            waypoint_index=0,
            waypoint_count=2,
            u=0.0,
            t_s=0.0,
            q_rad=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            sphere_clearance_m=None,
            sphere_pair=None,
            reason="always fail",
        )

    runner = IncrementalPopulationRunner(
        planner_factory=lambda seed, scene, links: planner,
        validator=lambda plan, request, clearance, cube: _validated(plan),
        contact_detector_factory=lambda _ep, to_id: OptimisticTipContactDetector(to_id),
        apply_reach_prefilter=False,
    )
    suite = runner.run_suite(config, root_seed=7, episode_count=1, physx_gate_fn=gate)
    assert not suite.suite_accepted
    assert suite.physx_regeneration_exhausted
    assert suite.results[0].failure_category is (
        MultiTargetFailureCategory.PHYSX_REGENERATION_EXHAUSTED
    )
    assert suite.extras[0].physx_acceptance == "exhausted"
    assert suite.extras[0].physx_regen_attempts == 2
