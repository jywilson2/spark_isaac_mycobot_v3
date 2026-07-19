"""Validated project configuration loaders.

Configuration is converted to immutable typed values at startup. Core planning
modules consume these values and do not read YAML or infer missing frames.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import yaml

from mycobot_curobo.errors import ConfigurationError
from mycobot_curobo.frames import TaskFrameConfig
from mycobot_curobo.robot_model import TCP_LINK
from mycobot_curobo.targets import normalize_angle_rad


@dataclass(frozen=True)
class AppConfig:
    """Validated application defaults for Phases 1–6."""

    source_path: Path
    robot_config_path: Path
    scene_config_path: Path
    tool_frame: str
    task_frame: TaskFrameConfig
    default_roll_candidates_rad: tuple[float, ...]
    normal_epsilon: float
    min_pre_approach_m: float
    max_pre_approach_m: float
    planner_profile_name: str
    validation_profile_name: str
    output_path: Path
    project_random_seed: int
    logging_level: str


def _positive_float(payload: dict[str, object], key: str) -> float:
    value = float(payload[key])
    if not math.isfinite(value) or value <= 0.0:
        raise ConfigurationError(f"{key} must be a positive finite value")
    return value


def load_app_config(path: Path | str = Path("config/app.yml")) -> AppConfig:
    """Load project-relative paths and task-frame defaults from YAML."""

    source = Path(path).resolve()
    if not source.is_file():
        raise ConfigurationError(f"app config not found: {source}")
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ConfigurationError("app config root must be a mapping")
    project_root = source.parent.parent
    tool_frame = str(payload["tool_frame"])
    if tool_frame != TCP_LINK:
        raise ConfigurationError(f"tool_frame must be {TCP_LINK!r}")
    rolls = tuple(
        normalize_angle_rad(math.radians(float(value)))
        for value in payload["default_roll_candidates_deg"]
    )
    if not rolls or len({round(value, 12) for value in rolls}) != len(rolls):
        raise ConfigurationError("default roll candidates must be non-empty and unique")
    minimum = _positive_float(payload, "min_pre_approach_m")
    maximum = _positive_float(payload, "max_pre_approach_m")
    if minimum >= maximum:
        raise ConfigurationError("min_pre_approach_m must be less than max_pre_approach_m")
    normal_epsilon = _positive_float(payload, "normal_epsilon")
    tangent_epsilon = _positive_float(payload, "tangent_epsilon")
    seed = int(payload["project_random_seed"])
    if seed < 0:
        raise ConfigurationError("project_random_seed must be non-negative")
    return AppConfig(
        source_path=source,
        robot_config_path=(project_root / str(payload["robot_config_path"])).resolve(),
        scene_config_path=(project_root / str(payload["scene_config_path"])).resolve(),
        tool_frame=tool_frame,
        task_frame=TaskFrameConfig(
            tool_approach_axis=str(payload["tool_approach_axis"]),
            tool_approach_sign=int(payload["tool_approach_sign"]),
            approach_against_outward_normal=bool(payload["approach_against_outward_normal"]),
            tangent_epsilon=tangent_epsilon,
        ),
        default_roll_candidates_rad=rolls,
        normal_epsilon=normal_epsilon,
        min_pre_approach_m=minimum,
        max_pre_approach_m=maximum,
        planner_profile_name=str(payload["planner_profile_name"]),
        validation_profile_name=str(payload["validation_profile_name"]),
        output_path=(project_root / str(payload["output_path"])).resolve(),
        project_random_seed=seed,
        logging_level=str(payload["logging_level"]),
    )
