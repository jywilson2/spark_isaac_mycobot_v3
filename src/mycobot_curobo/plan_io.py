"""Isaac-neutral JSON contract for validated joint-plan playback."""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from mycobot_curobo.errors import ConfigurationError
from mycobot_curobo.frames import TaskFrameConfig, build_task_frame_candidates
from mycobot_curobo.planner import PlanningRequest
from mycobot_curobo.robot_model import JOINT_NAMES
from mycobot_curobo.validation import ValidatedPlan

PLAYBACK_SCHEMA_VERSION = 1


@dataclass(frozen=True)
class PlaybackPlan:
    """Validated, finite joint trajectory and target metadata for simulation."""

    schema_version: int
    request_id: str
    executable: bool
    validation_status: str
    joint_names: tuple[str, ...]
    dt_s: float
    position_rad: np.ndarray
    velocity_rad_s: np.ndarray | None
    goal_position_base_m: np.ndarray
    goal_quaternion_wxyz: np.ndarray
    approach_direction_base: np.ndarray
    selected_roll_rad: float
    metrics: dict[str, float | None]


def _finite_array(value: Any, shape: tuple[int, ...], label: str) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.shape != shape:
        raise ConfigurationError(f"{label} must have shape {shape}, got {array.shape}")
    if not np.all(np.isfinite(array)):
        raise ConfigurationError(f"{label} must contain only finite values")
    return array.copy()


def require_executable_plan(plan: PlaybackPlan | Mapping[str, Any]) -> None:
    """Reject playback unless independent validation explicitly authorized it."""

    executable = plan.executable if isinstance(plan, PlaybackPlan) else plan.get("executable")
    if executable is not True:
        raise ConfigurationError("playback plan is not executable")


def validated_plan_to_playback_dict(
    validated_plan: ValidatedPlan,
    request: PlanningRequest,
    task_frame_config: TaskFrameConfig,
) -> dict[str, Any]:
    """Serialize a typed validated plan without importing Isaac APIs."""

    if validated_plan.nominal_plan.request_id != request.request_id:
        raise ConfigurationError("validated plan and request IDs differ")
    candidates = build_task_frame_candidates(request.surface_target, task_frame_config)
    index = validated_plan.nominal_plan.selected_goal_index
    if index < 0 or index >= len(candidates):
        raise ConfigurationError("selected goal index is outside the task-frame candidates")
    goal = candidates[index]
    trajectory = validated_plan.nominal_plan.combined_trajectory
    if trajectory.joint_names != JOINT_NAMES:
        raise ConfigurationError("playback trajectory joint order is invalid")
    return {
        "schema_version": PLAYBACK_SCHEMA_VERSION,
        "request_id": request.request_id,
        "executable": validated_plan.executable,
        "validation_status": validated_plan.validation_status,
        "joint_names": list(trajectory.joint_names),
        "dt_s": trajectory.dt_s,
        "position_rad": trajectory.position_rad.tolist(),
        "velocity_rad_s": (
            None if trajectory.velocity_rad_s is None else trajectory.velocity_rad_s.tolist()
        ),
        "goal_position_base_m": goal.position_base_m.tolist(),
        "goal_quaternion_wxyz": goal.quaternion_wxyz.tolist(),
        "approach_direction_base": goal.approach_direction_base.tolist(),
        "selected_roll_rad": validated_plan.nominal_plan.selected_roll_rad,
        "metrics": asdict(validated_plan.report.metrics),
    }


def playback_plan_from_dict(payload: Mapping[str, Any]) -> PlaybackPlan:
    """Validate and load the versioned playback JSON mapping."""

    try:
        schema_version = int(payload["schema_version"])
        request_id = str(payload["request_id"])
        executable = payload["executable"]
        validation_status = str(payload["validation_status"])
        joint_names = tuple(str(name) for name in payload["joint_names"])
        dt_s = float(payload["dt_s"])
        positions = np.asarray(payload["position_rad"], dtype=float)
        velocity_value = payload.get("velocity_rad_s")
        metrics_value = payload["metrics"]
    except (KeyError, TypeError, ValueError) as exc:
        raise ConfigurationError(f"invalid playback plan fields: {exc}") from exc
    if schema_version != PLAYBACK_SCHEMA_VERSION:
        raise ConfigurationError(f"unsupported playback schema_version: {schema_version}")
    if not request_id.strip():
        raise ConfigurationError("request_id must be non-empty")
    if not isinstance(executable, bool):
        raise ConfigurationError("executable must be a boolean")
    if joint_names != JOINT_NAMES:
        raise ConfigurationError(f"joint_names must exactly equal {JOINT_NAMES!r}")
    if not math.isfinite(dt_s) or dt_s <= 0.0:
        raise ConfigurationError("dt_s must be positive and finite")
    if positions.ndim != 2 or positions.shape[0] < 1 or positions.shape[1] != len(JOINT_NAMES):
        raise ConfigurationError("position_rad must have shape [waypoint, 6]")
    if not np.all(np.isfinite(positions)):
        raise ConfigurationError("position_rad must contain only finite values")
    velocity = None
    if velocity_value is not None:
        velocity = _finite_array(velocity_value, positions.shape, "velocity_rad_s")
    quaternion = _finite_array(payload["goal_quaternion_wxyz"], (4,), "goal_quaternion_wxyz")
    if not math.isclose(float(np.linalg.norm(quaternion)), 1.0, abs_tol=1.0e-8):
        raise ConfigurationError("goal_quaternion_wxyz must be unit length")
    approach = _finite_array(payload["approach_direction_base"], (3,), "approach_direction_base")
    if not math.isclose(float(np.linalg.norm(approach)), 1.0, abs_tol=1.0e-8):
        raise ConfigurationError("approach_direction_base must be unit length")
    selected_roll = float(payload["selected_roll_rad"])
    if not math.isfinite(selected_roll):
        raise ConfigurationError("selected_roll_rad must be finite")
    if not isinstance(metrics_value, Mapping):
        raise ConfigurationError("metrics must be a mapping")
    metrics: dict[str, float | None] = {}
    for key, value in metrics_value.items():
        numeric = None if value is None else float(value)
        if numeric is not None and not math.isfinite(numeric):
            raise ConfigurationError(f"metric {key!r} must be finite or null")
        metrics[str(key)] = numeric
    return PlaybackPlan(
        schema_version=schema_version,
        request_id=request_id,
        executable=executable,
        validation_status=validation_status,
        joint_names=joint_names,
        dt_s=dt_s,
        position_rad=positions.copy(),
        velocity_rad_s=velocity,
        goal_position_base_m=_finite_array(
            payload["goal_position_base_m"], (3,), "goal_position_base_m"
        ),
        goal_quaternion_wxyz=quaternion,
        approach_direction_base=approach,
        selected_roll_rad=selected_roll,
        metrics=metrics,
    )


def load_playback_plan(path: Path | str) -> PlaybackPlan:
    """Load a playback plan from JSON."""

    source = Path(path)
    try:
        payload = json.loads(source.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ConfigurationError(f"could not load playback plan {source}: {exc}") from exc
    if not isinstance(payload, dict):
        raise ConfigurationError("playback plan root must be a mapping")
    return playback_plan_from_dict(payload)


def write_playback_plan(path: Path | str, payload: Mapping[str, Any]) -> Path:
    """Validate and atomically write a playback plan JSON file."""

    plan = playback_plan_from_dict(payload)
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    serialized = {
        **dict(payload),
        "position_rad": plan.position_rad.tolist(),
        "velocity_rad_s": (None if plan.velocity_rad_s is None else plan.velocity_rad_s.tolist()),
    }
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    temporary.write_text(json.dumps(serialized, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(destination)
    return destination
