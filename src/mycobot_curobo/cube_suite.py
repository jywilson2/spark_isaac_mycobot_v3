"""Deterministic Phase 7.1 cube-approach sampling, replay, and reporting."""

from __future__ import annotations

import json
import math
from collections import Counter
from dataclasses import asdict, dataclass, replace
from enum import Enum
from pathlib import Path
from typing import Any, Callable, Sequence

import numpy as np
import yaml

from mycobot_curobo.benchmark import (
    FailureCategory,
    NormalBin,
    StartJointConfiguration,
    WorkspaceRegion,
    planning_failure_category,
    validation_failure_category,
)
from mycobot_curobo.cube_scene import (
    CubeGeometry,
    cube_approach_target_position,
    cube_scene_revision,
)
from mycobot_curobo.errors import ConfigurationError
from mycobot_curobo.planner import (
    JOINT_NAMES,
    NamedJointState,
    NominalPlan,
    PlanningOutcome,
    PlanningRequest,
)
from mycobot_curobo.robot_model import TCP_LINK, forward_kinematics, load_robot_model_spec
from mycobot_curobo.targets import SurfaceTarget
from mycobot_curobo.validation import ValidatedPlan, ValidationMetrics

# Rejection budget while FK-mapping goal-bank tips into declared Mode D AABBs.
GOAL_REGION_SAMPLE_ATTEMPTS = 64


def _quaternion_to_rotation(quaternion_wxyz: np.ndarray) -> np.ndarray:
    w, x, y, z = quaternion_wxyz
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _point_in_region(point_m: Sequence[float], region: WorkspaceRegion) -> bool:
    return all(
        float(lo) <= float(value) <= float(hi)
        for value, lo, hi in zip(point_m, region.minimum_m, region.maximum_m)
    )


class StartMode(str, Enum):
    A = "A"
    B = "B"
    C = "C"


class GoalMode(str, Enum):
    D = "D"


class ChainedFailurePolicy(str, Enum):
    USE_LAST_SUCCESS = "use_last_success"
    TERMINATE = "terminate"


@dataclass(frozen=True)
class SafeNest:
    label: str
    position_rad: tuple[float, ...]


@dataclass(frozen=True)
class CubeSuiteConfig:
    episode_count: int
    root_seed: int
    frame: str
    enabled_start_modes: tuple[StartMode, ...]
    goal_mode_d_enabled: bool
    chained_failure_policy: ChainedFailurePolicy
    safe_nest: SafeNest
    cube_edge_m: float
    flange_diameter_assumption_m: float
    terminal_standoff_m: float
    regions: tuple[WorkspaceRegion, ...]
    normal_bins: tuple[NormalBin, ...]
    start_sampling: str
    start_joint_bank: tuple[StartJointConfiguration, ...]
    goal_joint_bank: tuple[StartJointConfiguration, ...]
    planner_profile: str
    validation_profile: str
    scene_revision_prefix: str
    artifact_path: str
    minimum_self_collision_clearance_m: float
    minimum_world_collision_clearance_m: float
    max_isaac_prohibited_contacts: int
    lighting: dict[str, Any]
    pre_approach_distance_m: float
    roll_candidates_rad: tuple[float, ...]


def _tuple3(value: Any, label: str) -> tuple[float, float, float]:
    array = np.asarray(value, dtype=float)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ConfigurationError(f"{label} must contain three finite values")
    return tuple(float(item) for item in array)


def load_cube_suite_config(
    path: Path | str = Path("config/phase7_1_cube_suite.yml"),
) -> CubeSuiteConfig:
    """Load and validate the declarative suite without simulator dependencies."""

    source = Path(path)
    if not source.is_file():
        raise ConfigurationError(f"cube suite config not found: {source}")
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(payload, dict) or payload.get("frame") != "g_base":
        raise ConfigurationError("cube suite must explicitly use frame g_base")
    try:
        modes = payload["modes"]
        enabled = tuple(
            StartMode(item) for item in ("A", "B", "C") if modes[item]["enabled_by_default"]
        )
        goal_enabled = bool(modes["D"]["enabled_by_default"])
        regions = tuple(
            WorkspaceRegion(
                str(item["label"]),
                _tuple3(item["minimum_m"], "region minimum_m"),
                _tuple3(item["maximum_m"], "region maximum_m"),
            )
            for item in payload["regions"]
        )
        bins = []
        for item in payload["normal_bins"]:
            direction = np.asarray(
                _tuple3(item["direction_base"], "normal direction"), dtype=float
            )
            norm = float(np.linalg.norm(direction))
            angle = math.radians(float(item["cone_half_angle_deg"]))
            if norm <= 1.0e-12 or not 0.0 <= angle < math.pi / 2.0:
                raise ConfigurationError(
                    "normal bins require non-zero direction and angle in [0,90)"
                )
            bins.append(NormalBin(str(item["label"]), tuple(direction / norm), angle))
        bank = tuple(
            StartJointConfiguration(
                str(item["label"]), tuple(float(x) for x in item["position_rad"])
            )
            for item in payload["start_joint_bank"]
        )
        goal_bank = tuple(
            StartJointConfiguration(
                str(item["label"]), tuple(float(x) for x in item["position_rad"])
            )
            for item in payload["goal_joint_bank"]
        )
        nest = SafeNest(
            str(payload["safe_nest_label"]),
            tuple(float(x) for x in payload["safe_nest_joint_position_rad"]),
        )
        config = CubeSuiteConfig(
            episode_count=int(payload["episode_count"]),
            root_seed=int(payload["root_seed"]),
            frame="g_base",
            enabled_start_modes=enabled,
            goal_mode_d_enabled=goal_enabled,
            chained_failure_policy=ChainedFailurePolicy(payload["chained_failure_policy"]),
            safe_nest=nest,
            cube_edge_m=float(payload["cube_edge_m"]),
            flange_diameter_assumption_m=float(payload["flange_diameter_assumption_m"]),
            terminal_standoff_m=float(payload["terminal_standoff_m"]),
            regions=regions,
            normal_bins=tuple(bins),
            start_sampling=str(payload["start_sampling"]),
            start_joint_bank=bank,
            goal_joint_bank=goal_bank,
            planner_profile=str(payload["planner_profile"]),
            validation_profile=str(payload["validation_profile"]),
            scene_revision_prefix=str(payload["scene_revision_prefix"]),
            artifact_path=str(payload["artifact_path"]),
            minimum_self_collision_clearance_m=float(
                payload["minimum_self_collision_clearance_m"]
            ),
            minimum_world_collision_clearance_m=float(
                payload["minimum_world_collision_clearance_m"]
            ),
            max_isaac_prohibited_contacts=int(payload["max_isaac_prohibited_contacts"]),
            lighting=dict(payload["lighting"]),
            pre_approach_distance_m=float(payload["pre_approach_distance_m"]),
            roll_candidates_rad=tuple(
                math.radians(float(x)) for x in payload["roll_candidates_deg"]
            ),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ConfigurationError(f"invalid cube suite configuration: {exc}") from exc
    if config.episode_count <= 0 or config.root_seed < 0 or not config.goal_mode_d_enabled:
        raise ConfigurationError("episode_count/root_seed/D default mode are invalid")
    if config.enabled_start_modes != (StartMode.A,):
        raise ConfigurationError("only Mode A may be enabled by default")
    if len(config.regions) == 0 or any(
        a >= b for r in config.regions for a, b in zip(r.minimum_m, r.maximum_m)
    ):
        raise ConfigurationError("cube regions must be non-empty valid AABBs")
    if len(config.start_joint_bank) < 4 or any(
        len(s.position_rad) != len(JOINT_NAMES) for s in config.start_joint_bank
    ):
        raise ConfigurationError("cube suite requires at least four six-joint bank starts")
    if len(config.goal_joint_bank) < 3 or any(
        len(s.position_rad) != len(JOINT_NAMES) for s in config.goal_joint_bank
    ):
        raise ConfigurationError("cube suite requires at least three six-joint goal poses")
    if config.start_sampling != "bank" or not any(
        np.any(s.position_rad) for s in config.start_joint_bank
    ):
        raise ConfigurationError("cube suite start sampling must be a diverse bank")
    if len(config.safe_nest.position_rad) != len(JOINT_NAMES):
        raise ConfigurationError("safe nest must contain six joint positions")
    positives = (
        config.cube_edge_m,
        config.flange_diameter_assumption_m,
        config.terminal_standoff_m,
        config.pre_approach_distance_m,
    )
    if any(not math.isfinite(v) or v <= 0.0 for v in positives):
        raise ConfigurationError("cube dimensions and distances must be positive and finite")
    if (
        config.minimum_self_collision_clearance_m < 0
        or config.minimum_world_collision_clearance_m < 0
    ):
        raise ConfigurationError("minimum clearances must be non-negative")
    if config.max_isaac_prohibited_contacts < 0 or not config.roll_candidates_rad:
        raise ConfigurationError("Isaac contact limit and roll candidates are invalid")
    return config


@dataclass(frozen=True)
class CubeEpisode:
    episode_index: int
    root_seed: int
    start_mode: StartMode
    start_label: str
    start_position_rad: tuple[float, ...]
    cube_center_m: tuple[float, float, float]
    cube_edge_m: float
    cube_name: str
    normal_bin_label: str
    surface_normal_base: tuple[float, float, float]
    tangent_hint_base: tuple[float, float, float] | None
    fixed_roll_rad: float | None
    terminal_standoff_m: float
    pre_approach_distance_m: float
    planner_seed: int
    scene_revision: str
    planner_profile: str
    request_id: str
    safe_nest: SafeNest | None
    roll_candidates_rad: tuple[float, ...]
    goal_label: str

    @property
    def cube_geometry(self) -> CubeGeometry:
        return CubeGeometry(self.cube_center_m, self.cube_edge_m, name=self.cube_name)

    def to_planning_request(self) -> PlanningRequest:
        return PlanningRequest(
            current_joint_state=NamedJointState.create(JOINT_NAMES, self.start_position_rad),
            surface_target=SurfaceTarget.create(
                position_base_m=cube_approach_target_position(
                    self.cube_center_m,
                    self.cube_edge_m,
                    self.surface_normal_base,
                    self.terminal_standoff_m,
                ),
                surface_normal_base=self.surface_normal_base,
                tangent_hint_base=self.tangent_hint_base,
                fixed_roll_rad=self.fixed_roll_rad,
                roll_candidates_rad=(
                    None if self.fixed_roll_rad is not None else self.roll_candidates_rad
                ),
                pre_approach_distance_m=self.pre_approach_distance_m,
                tool_frame=TCP_LINK,
                target_id=self.request_id,
            ),
            scene_revision=self.scene_revision,
            planner_profile=self.planner_profile,
            random_seed=self.planner_seed,
            request_id=self.request_id,
        )


def sample_cube_episodes(
    config: CubeSuiteConfig,
    *,
    root_seed: int,
    episode_count: int | None = None,
    force_modes: Sequence[StartMode | str] | None = None,
) -> tuple[CubeEpisode, ...]:
    """Sample frozen replay inputs; acceptance force modes cycle A/B/C and retain D goals."""

    count = config.episode_count if episode_count is None else int(episode_count)
    if root_seed < 0 or count <= 0:
        raise ConfigurationError("root_seed must be non-negative and episode_count positive")
    requested = (
        None
        if force_modes is None
        else {str(item.value if isinstance(item, Enum) else item) for item in force_modes}
    )
    if requested is None:
        modes = config.enabled_start_modes or (StartMode.A,)
    else:
        unknown = requested - {"A", "B", "C", "D"}
        if unknown or "D" not in requested:
            raise ConfigurationError("forced modes must be a subset containing D")
        modes = tuple(mode for mode in StartMode if mode.value in requested) or (StartMode.A,)
    if not config.goal_mode_d_enabled:
        raise ConfigurationError("Mode D goal sampling must remain enabled for Phase 7.1")
    rng = np.random.default_rng(root_seed)
    robot_spec = load_robot_model_spec()
    episodes = []
    for index in range(count):
        start_mode = modes[index % len(modes)]
        start = config.start_joint_bank[int(rng.integers(len(config.start_joint_bank)))]
        # Mode D: FK-align cube centres from the goal bank into declared AABBs.
        cube = None
        goal = None
        outward = None
        tangent = None
        region_label = ""
        for _ in range(GOAL_REGION_SAMPLE_ATTEMPTS):
            candidate = config.goal_joint_bank[int(rng.integers(len(config.goal_joint_bank)))]
            pose = forward_kinematics(np.asarray(candidate.position_rad), spec=robot_spec)
            rotation = _quaternion_to_rotation(pose.quaternion_wxyz)
            candidate_outward = rotation[:, 2] / float(np.linalg.norm(rotation[:, 2]))
            cube_center = (
                pose.position_m
                - (config.terminal_standoff_m + 0.5 * config.cube_edge_m) * candidate_outward
            )
            matching = [
                region for region in config.regions if _point_in_region(cube_center, region)
            ]
            if not matching:
                continue
            cube = CubeGeometry(
                tuple(float(value) for value in cube_center),
                config.cube_edge_m,
                name=f"cube_{index:03d}",
            )
            goal = candidate
            outward = candidate_outward
            tangent = rotation[:, 0] / float(np.linalg.norm(rotation[:, 0]))
            region_label = matching[0].label
            break
        if cube is None or goal is None or outward is None or tangent is None:
            raise ConfigurationError(
                "unable to sample Mode D cube centres inside declared regions from goal bank"
            )
        # Keep normal bins in the frozen payload for replay taxonomy; FK tool +z wins.
        normal_bin = config.normal_bins[int(rng.integers(len(config.normal_bins)))]
        revision = f"{config.scene_revision_prefix}-{cube_scene_revision(cube)}"
        episodes.append(
            CubeEpisode(
                index,
                root_seed,
                start_mode,
                start.label,
                start.position_rad,
                cube.center_m,
                cube.edge_m,
                cube.name,
                f"{normal_bin.label}:{region_label}:{goal.label}",
                tuple(float(value) for value in outward),
                tuple(float(value) for value in tangent),
                0.0,
                config.terminal_standoff_m,
                config.pre_approach_distance_m,
                root_seed + index,
                revision,
                config.planner_profile,
                f"phase7_1-{root_seed}-{index:04d}",
                config.safe_nest if start_mode is StartMode.C else None,
                config.roll_candidates_rad,
                goal.label,
            )
        )
    return tuple(episodes)


def serialize_episode(episode: CubeEpisode) -> dict[str, Any]:
    """Serialize every frozen field, including cube geometry, for exact replay."""
    return asdict(episode)


def deserialize_episode(payload: dict[str, Any]) -> CubeEpisode:
    """Rebuild a frozen episode from its JSON-compatible representation."""
    data = dict(payload)
    data["start_mode"] = StartMode(data["start_mode"])
    data["start_position_rad"] = tuple(data["start_position_rad"])
    data["cube_center_m"] = tuple(data["cube_center_m"])
    data["surface_normal_base"] = tuple(data["surface_normal_base"])
    if data.get("tangent_hint_base") is not None:
        data["tangent_hint_base"] = tuple(data["tangent_hint_base"])
    data["roll_candidates_rad"] = tuple(data["roll_candidates_rad"])
    if data["safe_nest"] is not None:
        data["safe_nest"] = SafeNest(
            data["safe_nest"]["label"], tuple(data["safe_nest"]["position_rad"])
        )
    data.setdefault("goal_label", "legacy")
    data.setdefault("tangent_hint_base", None)
    data.setdefault("fixed_roll_rad", None)
    return CubeEpisode(**data)


@dataclass(frozen=True)
class CubeEpisodeResult:
    episode: CubeEpisode
    planning_succeeded: bool
    validation_passed: bool
    failure_category: FailureCategory | None
    failure_reason: str | None
    planner_status: str
    validation_metrics: ValidationMetrics | None = None
    isaac_prohibited_contacts: int | None = None
    final_joint_position_rad: tuple[float, ...] | None = None

    @property
    def succeeded(self) -> bool:
        return self.planning_succeeded and self.validation_passed


@dataclass(frozen=True)
class CubeSuiteSummary:
    root_seed: int
    total_episodes: int
    successes: int
    success_rate: float
    failure_category_counts: dict[str, int]
    lateral_error_m: dict[str, float | None]
    approach_axis_error_rad: dict[str, float | None]


def _distribution(values: list[float]) -> dict[str, float | None]:
    if not values:
        return {"p50": None, "p95": None}
    return {"p50": float(np.percentile(values, 50)), "p95": float(np.percentile(values, 95))}


def aggregate_cube_results(
    results: Sequence[CubeEpisodeResult], *, root_seed: int
) -> CubeSuiteSummary:
    if not results:
        raise ConfigurationError("cannot aggregate an empty cube suite")
    failures = Counter(r.failure_category.value for r in results if r.failure_category is not None)
    metrics = [r.validation_metrics for r in results if r.validation_metrics is not None]
    return CubeSuiteSummary(
        root_seed=root_seed,
        total_episodes=len(results),
        successes=sum(r.succeeded for r in results),
        success_rate=sum(r.succeeded for r in results) / len(results),
        failure_category_counts=dict(sorted(failures.items())),
        lateral_error_m=_distribution(
            [m.max_lateral_error_m for m in metrics if m.max_lateral_error_m is not None]
        ),
        approach_axis_error_rad=_distribution(
            [
                m.max_approach_axis_error_rad
                for m in metrics
                if m.max_approach_axis_error_rad is not None
            ]
        ),
    )


def format_episode_console_row(result: CubeEpisodeResult, *, count: int | None = None) -> str:
    metrics = result.validation_metrics
    lateral = (
        "n/a"
        if metrics is None or metrics.max_lateral_error_m is None
        else f"{metrics.max_lateral_error_m:.4f}"
    )
    axis = (
        "n/a"
        if metrics is None or metrics.max_approach_axis_error_rad is None
        else f"{metrics.max_approach_axis_error_rad:.4f}"
    )
    prefix = (
        f"{result.episode.episode_index + 1}/{count}"
        if count
        else str(result.episode.episode_index + 1)
    )
    failure = None if result.failure_category is None else result.failure_category.value
    return (
        f"[{prefix}] {result.episode.start_mode.value}/{result.episode.start_label} "
        f"{result.episode.normal_bin_label} plan={result.planning_succeeded} "
        f"valid={result.validation_passed} lateral={lateral} axis={axis} "
        f"isaac_contacts={result.isaac_prohibited_contacts} failure={failure}"
    )


def format_suite_summary(summary: CubeSuiteSummary) -> str:
    return (
        f"Phase 7.1: {summary.successes}/{summary.total_episodes} "
        f"({summary.success_rate:.1%}) failures={summary.failure_category_counts} "
        f"lateral_p50/p95={summary.lateral_error_m['p50']}/{summary.lateral_error_m['p95']} "
        f"axis_p50/p95={summary.approach_axis_error_rad['p50']}/"
        f"{summary.approach_axis_error_rad['p95']}"
    )


def write_cube_suite_report(
    summary: CubeSuiteSummary, results: Sequence[CubeEpisodeResult], output_dir: Path | str
) -> Path:
    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    path = destination / f"phase7_1_cube_seed_{summary.root_seed}.json"
    path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "summary": asdict(summary),
                "results": [asdict(r) for r in results],
            },
            indent=2,
            default=lambda value: value.value if isinstance(value, Enum) else value,
        ),
        encoding="utf-8",
    )
    return path


class CubeSuiteRunner:
    """Run plan/validate episodes; Mode B chains results and C relocates through cuRobo."""

    def __init__(
        self,
        *,
        planner_factory: Callable[[int], Any],
        validator: Callable[[NominalPlan, PlanningRequest], ValidatedPlan],
        relocator: Callable[[Any, NamedJointState], PlanningOutcome] | None = None,
        start_preflight: Callable[[CubeEpisode], bool] | None = None,
    ) -> None:
        self._planner_factory, self._validator = planner_factory, validator
        self._relocator, self._start_preflight = relocator, start_preflight

    def run(self, episodes: Sequence[CubeEpisode]) -> tuple[CubeEpisodeResult, ...]:
        last_success: tuple[float, ...] | None = None
        results: list[CubeEpisodeResult] = []
        for episode in episodes:
            current = episode
            if episode.start_mode is StartMode.B and last_success is not None:
                current = replace(
                    episode, start_label="chained_last_success", start_position_rad=last_success
                )
            if self._start_preflight is not None and not self._start_preflight(current):
                results.append(
                    CubeEpisodeResult(
                        current,
                        False,
                        False,
                        FailureCategory.CONFIGURATION_MODEL_FAILURE,
                        "invalid start state",
                        "start_preflight",
                    )
                )
                continue
            request = current.to_planning_request()
            try:
                planner = self._planner_factory(current.planner_seed)
                if current.start_mode is StartMode.C:
                    if self._relocator is None:
                        raise ConfigurationError("Mode C requires a cuRobo relocation planner")
                    relocation = self._relocator(
                        planner,
                        NamedJointState.create(JOINT_NAMES, current.safe_nest.position_rad),
                    )
                    if not relocation.succeeded:
                        results.append(
                            CubeEpisodeResult(
                                current,
                                False,
                                False,
                                FailureCategory.TRAJECTORY_OPTIMIZATION_FAILURE,
                                "relocation failed",
                                "relocation_failed",
                            )
                        )
                        continue
                    request = replace(
                        request,
                        current_joint_state=NamedJointState.create(
                            JOINT_NAMES, current.safe_nest.position_rad
                        ),
                    )
                outcome = planner.plan(request)
                if not outcome.succeeded or outcome.plan is None:
                    failure = outcome.failure
                    results.append(
                        CubeEpisodeResult(
                            current,
                            False,
                            False,
                            FailureCategory.TRAJECTORY_OPTIMIZATION_FAILURE
                            if failure is None
                            else planning_failure_category(failure),
                            None if failure is None else failure.reason,
                            "" if failure is None else failure.planner_status,
                        )
                    )
                    continue
                validated = self._validator(outcome.plan, request)
                category = (
                    None
                    if validated.report.valid
                    else validation_failure_category(validated.report.violations)
                )
                final = (
                    tuple(float(x) for x in outcome.plan.combined_trajectory.position_rad[-1])
                    if validated.report.valid
                    else None
                )
                result = CubeEpisodeResult(
                    current,
                    True,
                    validated.report.valid,
                    category,
                    None
                    if validated.report.valid
                    else "; ".join(v.reason for v in validated.report.violations),
                    outcome.plan.planner_status,
                    validated.report.metrics,
                    final_joint_position_rad=final,
                )
                results.append(result)
                if result.succeeded:
                    last_success = final
            except (ConfigurationError, RuntimeError, ValueError) as exc:
                results.append(
                    CubeEpisodeResult(
                        current,
                        False,
                        False,
                        FailureCategory.CONFIGURATION_MODEL_FAILURE,
                        str(exc),
                        "exception",
                    )
                )
        return tuple(results)
