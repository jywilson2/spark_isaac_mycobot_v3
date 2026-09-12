"""Apply bounded residual corrections to nominal joint trajectories (sim-only).

Used to exercise Phase 8 residual code before Isaac playback. Does not invoke a
planner or replace cuRobo trajectories: each waypoint remains a local
correction of the validated nominal sample.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np

from mycobot_curobo.actuator_noise import (
    JointActuatorNoiseModel,
    load_actuator_noise_profile,
)
from mycobot_curobo.errors import ConfigurationError
from mycobot_curobo.execution import CpuTcpPoseEvaluator
from mycobot_curobo.residual import (
    CartesianResidual,
    FixedResidualCorrector,
    PolicyResidualCorrector,
    ResidualCorrector,
    ResidualObservation,
    ZeroResidualCorrector,
)
from mycobot_curobo.residual_mapping import FiniteDifferenceResidualMapper, ResidualJointMapper
from mycobot_curobo.robot_model import JOINT_NAMES, RobotModelSpec, load_robot_model_spec
from mycobot_curobo.safety import (
    SafetyProjector,
    lateral_distance_m,
    load_residual_safety_profile,
)


@dataclass(frozen=True)
class ResidualApplyStats:
    """Counts from one residual+noise pass over a trajectory."""

    waypoint_count: int
    residual_nonzero_count: int
    applied_count: int
    fallback_nominal_count: int
    safety_reject_count: int
    max_abs_joint_delta_rad: float
    measurement_noise_std_rad: float
    sim_only: bool = True


class NoisyMeasuredStateProvider:
    """Add seeded Gaussian joint noise to the nominal measured state."""

    def __init__(
        self,
        *,
        noise_std_rad: float,
        seed: int,
        clamp_lower_rad: np.ndarray | None = None,
        clamp_upper_rad: np.ndarray | None = None,
    ) -> None:
        if not math.isfinite(noise_std_rad) or noise_std_rad < 0.0:
            raise ConfigurationError("noise_std_rad must be finite and non-negative")
        self._noise_std_rad = float(noise_std_rad)
        self._rng = np.random.default_rng(seed)
        self._clamp_lower = (
            None if clamp_lower_rad is None else np.asarray(clamp_lower_rad, dtype=float)
        )
        self._clamp_upper = (
            None if clamp_upper_rad is None else np.asarray(clamp_upper_rad, dtype=float)
        )

    def noisy_joints(self, nominal_joint_position_rad: Sequence[float]) -> tuple[float, ...]:
        nominal = np.asarray(nominal_joint_position_rad, dtype=float)
        if self._noise_std_rad == 0.0:
            measured = nominal
        else:
            noise = self._rng.normal(0.0, self._noise_std_rad, size=nominal.shape)
            measured = nominal + noise
        if self._clamp_lower is not None and self._clamp_upper is not None:
            measured = np.clip(measured, self._clamp_lower, self._clamp_upper)
        return tuple(float(value) for value in measured)


def apply_residual_to_joint_positions(
    position_rad: np.ndarray,
    *,
    goal_position_base_m: Sequence[float] | None,
    approach_direction_base: Sequence[float],
    corrector: ResidualCorrector,
    projector: SafetyProjector,
    joint_mapper: ResidualJointMapper,
    pose_evaluator: CpuTcpPoseEvaluator,
    measurement_noise_std_rad: float = 0.0,
    noise_seed: int = 0,
    request_id: str = "residual-playback",
    use_waypoint_tcp_as_goal: bool = False,
) -> tuple[np.ndarray, ResidualApplyStats]:
    """Return corrected joint positions and apply statistics."""

    positions = np.asarray(position_rad, dtype=float)
    if positions.ndim != 2 or positions.shape[1] != len(JOINT_NAMES):
        raise ConfigurationError(
            f"position_rad must have shape (N, {len(JOINT_NAMES)}), got {positions.shape}"
        )
    approach = np.asarray(approach_direction_base, dtype=float)
    norm = float(np.linalg.norm(approach))
    if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1.0e-8):
        raise ConfigurationError("approach_direction_base must be unit length")
    goal: tuple[float, float, float] | None
    if use_waypoint_tcp_as_goal:
        goal = None
    else:
        if goal_position_base_m is None:
            raise ConfigurationError(
                "goal_position_base_m is required unless use_waypoint_tcp_as_goal"
            )
        goal = tuple(float(value) for value in goal_position_base_m)
        if len(goal) != 3:
            raise ConfigurationError("goal_position_base_m must have length 3")

    noisy = NoisyMeasuredStateProvider(noise_std_rad=measurement_noise_std_rad, seed=noise_seed)
    corrected = positions.copy()
    nonzero = 0
    applied = 0
    fallback = 0
    rejected = 0
    max_abs_delta = 0.0
    fallback_mode = projector.profile.residual_fallback

    for index, nominal_row in enumerate(positions):
        nominal = tuple(float(value) for value in nominal_row)
        measured = noisy.noisy_joints(nominal)
        pose = pose_evaluator.evaluate(nominal)
        waypoint_goal = tuple(float(value) for value in pose.position_m) if goal is None else goal
        observation = ResidualObservation.create(
            request_id=request_id,
            waypoint_index=index,
            command_time_s=float(index) * 0.02,
            measured_timestamp_s=float(index) * 0.02,
            nominal_joint_position_rad=nominal,
            measured_joint_position_rad=measured,
            nominal_tcp_position_base_m=pose.position_m,
            goal_position_base_m=waypoint_goal,
            approach_direction_base=approach,
        )
        residual = corrector.correction(observation)
        decision = projector.project(
            observation,
            residual,
            evaluated_at_s=observation.command_time_s,
            plan_executable=True,
        )
        if not decision.accepted or decision.projected_residual is None:
            rejected += 1
            if fallback_mode == "stop":
                # Keep already-corrected prefix; leave remainder nominal.
                break
            fallback += 1
            continue
        projected = decision.projected_residual
        if projected.is_zero:
            continue
        nonzero += 1
        try:
            delta = joint_mapper.map_to_joint_delta(projected, nominal)
        except ConfigurationError:
            fallback += 1
            continue
        if any(abs(value) > projector.profile.max_joint_delta_rad + 1.0e-12 for value in delta):
            fallback += 1
            continue
        candidate = tuple(
            float(joint + offset) for joint, offset in zip(nominal, delta, strict=True)
        )
        lower = projector.joint_limits.lower_rad + projector.profile.minimum_joint_limit_margin_rad
        upper = projector.joint_limits.upper_rad - projector.profile.minimum_joint_limit_margin_rad
        candidate_arr = np.asarray(candidate, dtype=float)
        if np.any(candidate_arr < lower) or np.any(candidate_arr > upper):
            fallback += 1
            continue
        try:
            corrected_pose = pose_evaluator.evaluate(candidate)
        except ConfigurationError:
            fallback += 1
            continue
        lateral = lateral_distance_m(
            np.asarray(corrected_pose.position_m, dtype=float),
            np.asarray(waypoint_goal, dtype=float),
            approach,
        )
        if lateral > projector.profile.max_lateral_error_m:
            fallback += 1
            continue
        corrected[index] = candidate_arr
        applied += 1
        max_abs_delta = max(max_abs_delta, float(np.max(np.abs(delta))))

    stats = ResidualApplyStats(
        waypoint_count=int(positions.shape[0]),
        residual_nonzero_count=nonzero,
        applied_count=applied,
        fallback_nominal_count=fallback,
        safety_reject_count=rejected,
        max_abs_joint_delta_rad=max_abs_delta,
        measurement_noise_std_rad=float(measurement_noise_std_rad),
        sim_only=True,
    )
    return corrected, stats


def _repo_root() -> Path:
    """Return the repository root containing ``config/`` (cwd-independent)."""

    # residual_playback.py lives at src/mycobot_curobo/residual_playback.py
    return Path(__file__).resolve().parents[2]


def build_default_residual_stack(
    *,
    checkpoint_path: str | None = None,
    residual_safety_profile: str = "simulation_bounded_residual",
    robot_spec: RobotModelSpec | None = None,
) -> tuple[ResidualCorrector, SafetyProjector, ResidualJointMapper, CpuTcpPoseEvaluator]:
    """Assemble corrector/projector/mapper/FK for residual playback."""

    from mycobot_curobo.residual import load_residual_policy_checkpoint

    repo = _repo_root()
    if robot_spec is None:
        spec = load_robot_model_spec(repo / "config" / "robots" / "mycobot_280_m5.yml")
    else:
        spec = robot_spec
    profile = load_residual_safety_profile(
        residual_safety_profile,
        path=repo / "config" / "residual_safety.yml",
    )
    projector = SafetyProjector(profile, spec.limits)
    pose_evaluator = CpuTcpPoseEvaluator(spec)
    mapper = FiniteDifferenceResidualMapper(
        robot_spec=spec,
        max_joint_delta_rad=profile.max_joint_delta_rad,
        pose_evaluator=pose_evaluator,
    )
    if checkpoint_path is None:
        corrector: ResidualCorrector = ZeroResidualCorrector()
    else:
        corrector = PolicyResidualCorrector(load_residual_policy_checkpoint(checkpoint_path))
    return corrector, projector, mapper, pose_evaluator


def _goal_approach_by_request(
    payload: dict,
) -> dict[str, tuple[tuple[float, float, float], tuple[float, float, float]]]:
    """Map request_id -> (contact goal, approach) from episode legs + targets."""

    goal_by_request: dict[str, tuple[tuple[float, float, float], tuple[float, float, float]]] = {}
    for result in payload.get("results", []):
        episode = result.get("episode") or {}
        targets = {
            str(target["target_id"]): target
            for target in (episode.get("field") or {}).get("targets", [])
        }
        for leg in result.get("legs", []):
            request_id = leg.get("request_id")
            to_id = str(leg.get("to_id"))
            if not request_id or to_id not in targets:
                continue
            target = targets[to_id]
            center = tuple(float(v) for v in target["center_m"])
            normal = np.asarray(target["outward_normal_base"], dtype=float)
            normal_norm = float(np.linalg.norm(normal))
            if normal_norm <= 0.0:
                continue
            # Approach into the face (against outward normal).
            approach = tuple(float(v) for v in (-normal / normal_norm))
            # Contact goal approximates the near face center along the normal.
            half = 0.5 * float(target["edge_m"])
            goal = tuple(float(c - half * n) for c, n in zip(center, normal / normal_norm))
            goal_by_request[str(request_id)] = (goal, approach)
    return goal_by_request


def _stats_payload(
    stats_by_id: dict[str, ResidualApplyStats],
) -> dict[str, dict[str, float | int]]:
    return {
        request_id: {
            "waypoint_count": stats.waypoint_count,
            "residual_nonzero_count": stats.residual_nonzero_count,
            "applied_count": stats.applied_count,
            "fallback_nominal_count": stats.fallback_nominal_count,
            "safety_reject_count": stats.safety_reject_count,
            "max_abs_joint_delta_rad": stats.max_abs_joint_delta_rad,
        }
        for request_id, stats in stats_by_id.items()
    }


def _apply_actuator_noise_to_payload(
    payload: dict,
    *,
    actuator_noise_profile: str,
    noise_seed: int,
    projector: SafetyProjector,
) -> dict[str, object]:
    """Disturb commanded joints in-place; return actuator metadata."""

    profile = load_actuator_noise_profile(actuator_noise_profile)
    model = JointActuatorNoiseModel(profile)
    meta: dict[str, object] = {
        **profile.to_dict(),
        "applied": False,
        "trajectories_disturbed": [],
    }
    if not profile.enabled:
        return meta

    lower = projector.joint_limits.lower_rad
    upper = projector.joint_limits.upper_rad
    touched: list[str] = []
    trajectories = payload.get("trajectories") or {}
    for index, (request_id, trajectory) in enumerate(trajectories.items()):
        positions = np.asarray(trajectory["position_rad"], dtype=float)
        disturbed = model.disturb_joint_positions(
            positions,
            seed=noise_seed + 17_000 + index,
            lower_rad=lower if profile.clamp_to_limits else None,
            upper_rad=upper if profile.clamp_to_limits else None,
        )
        trajectory["position_rad"] = disturbed.tolist()
        touched.append(str(request_id))
    meta["applied"] = True
    meta["trajectories_disturbed"] = sorted(touched)
    return meta


def _apply_corrector_to_bundle_trajectories(
    payload: dict,
    *,
    corrector: ResidualCorrector,
    projector: SafetyProjector,
    mapper: ResidualJointMapper,
    pose_evaluator: CpuTcpPoseEvaluator,
    measurement_noise_std_rad: float,
    noise_seed: int,
) -> dict[str, ResidualApplyStats]:
    """Mutate ``payload['trajectories']`` in place; return per-request stats."""

    goal_by_request = _goal_approach_by_request(payload)
    stats_by_id: dict[str, ResidualApplyStats] = {}
    trajectories = payload.get("trajectories") or {}
    for request_id, trajectory in trajectories.items():
        if request_id not in goal_by_request:
            continue
        goal, approach = goal_by_request[request_id]
        positions = np.asarray(trajectory["position_rad"], dtype=float)
        corrected, stats = apply_residual_to_joint_positions(
            positions,
            goal_position_base_m=goal,
            approach_direction_base=approach,
            corrector=corrector,
            projector=projector,
            joint_mapper=mapper,
            pose_evaluator=pose_evaluator,
            measurement_noise_std_rad=measurement_noise_std_rad,
            noise_seed=noise_seed + abs(hash(request_id)) % 10_000,
            request_id=str(request_id),
            use_waypoint_tcp_as_goal=True,
        )
        trajectory["position_rad"] = corrected.tolist()
        stats_by_id[str(request_id)] = stats
    return stats_by_id


def apply_cartesian_tip_bias_to_bundle(
    bundle: dict,
    *,
    tip_bias_m: Sequence[float],
    residual_safety_profile: str = "simulation_bounded_residual",
    measurement_noise_std_rad: float = 0.0,
    noise_seed: int = 8008,
    actuator_noise_profile: str = "simulation_none",
) -> tuple[dict, dict[str, ResidualApplyStats]]:
    """Return a copy of a Phase 7.2 bundle with a fixed Cartesian tip bias injected.

    Used for residual-off demo playback: EE is intentionally offset so residual-on
    can cancel the same bias. Does not load a policy checkpoint.
    """

    import copy

    bias = np.asarray(tip_bias_m, dtype=float)
    if bias.shape != (3,) or not np.all(np.isfinite(bias)):
        raise ConfigurationError("tip_bias_m must be a finite length-3 vector")
    if float(np.linalg.norm(bias)) <= 0.0:
        raise ConfigurationError("tip_bias_m must be non-zero for bias injection")

    payload = copy.deepcopy(bundle)
    _, projector, mapper, pose_evaluator = build_default_residual_stack(
        checkpoint_path=None,
        residual_safety_profile=residual_safety_profile,
    )
    corrector: ResidualCorrector = FixedResidualCorrector(
        CartesianResidual.create(tuple(float(v) for v in bias), (0.0, 0.0, 0.0))
    )
    stats_by_id = _apply_corrector_to_bundle_trajectories(
        payload,
        corrector=corrector,
        projector=projector,
        mapper=mapper,
        pose_evaluator=pose_evaluator,
        measurement_noise_std_rad=measurement_noise_std_rad,
        noise_seed=noise_seed,
    )
    actuator_meta = _apply_actuator_noise_to_payload(
        payload,
        actuator_noise_profile=actuator_noise_profile,
        noise_seed=noise_seed,
        projector=projector,
    )
    payload["residual_playback"] = {
        "sim_only": True,
        "mode": "inject_tip_bias",
        "tip_bias_m": [float(v) for v in bias],
        "checkpoint_path": None,
        "measurement_noise_std_rad": measurement_noise_std_rad,
        "noise_seed": noise_seed,
        "residual_safety_profile": residual_safety_profile,
        "actuator_noise": actuator_meta,
        "trajectories_touched": sorted(stats_by_id.keys()),
        "stats": _stats_payload(stats_by_id),
    }
    return payload, stats_by_id


def apply_residual_noise_to_bundle(
    bundle: dict,
    *,
    checkpoint_path: str,
    measurement_noise_std_rad: float,
    noise_seed: int = 8008,
    residual_safety_profile: str = "simulation_bounded_residual",
    actuator_noise_profile: str = "simulation_default",
) -> tuple[dict, dict[str, ResidualApplyStats]]:
    """Return a copy of a Phase 7.2 bundle with residual-corrected trajectories."""

    import copy

    payload = copy.deepcopy(bundle)
    corrector, projector, mapper, pose_evaluator = build_default_residual_stack(
        checkpoint_path=checkpoint_path,
        residual_safety_profile=residual_safety_profile,
    )
    stats_by_id = _apply_corrector_to_bundle_trajectories(
        payload,
        corrector=corrector,
        projector=projector,
        mapper=mapper,
        pose_evaluator=pose_evaluator,
        measurement_noise_std_rad=measurement_noise_std_rad,
        noise_seed=noise_seed,
    )
    actuator_meta = _apply_actuator_noise_to_payload(
        payload,
        actuator_noise_profile=actuator_noise_profile,
        noise_seed=noise_seed,
        projector=projector,
    )
    payload["residual_playback"] = {
        "sim_only": True,
        "mode": "residual_correct",
        "checkpoint_path": checkpoint_path,
        "measurement_noise_std_rad": measurement_noise_std_rad,
        "noise_seed": noise_seed,
        "residual_safety_profile": residual_safety_profile,
        "actuator_noise": actuator_meta,
        "trajectories_touched": sorted(stats_by_id.keys()),
        "stats": _stats_payload(stats_by_id),
    }
    return payload, stats_by_id
