"""Deterministic Phase 6 workspace benchmarking and replay reports."""

from __future__ import annotations

import json
import math
from collections import Counter, defaultdict
from dataclasses import asdict, dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Callable, Protocol, Sequence

import numpy as np
import yaml

from mycobot_curobo.errors import ConfigurationError
from mycobot_curobo.execution import ExecutionResult
from mycobot_curobo.planner import (
    NamedJointState,
    NominalPlan,
    PlanningFailure,
    PlanningOutcome,
    PlanningRequest,
)
from mycobot_curobo.robot_model import JOINT_NAMES, TCP_LINK
from mycobot_curobo.targets import SurfaceTarget
from mycobot_curobo.validation import (
    ValidatedPlan,
    ValidationMetrics,
    ValidationViolation,
)


class FailureCategory(str, Enum):
    """Stable Phase 6 failure taxonomy."""

    NO_REACHABLE_IK = "no_reachable_ik"
    COLLISION_INFEASIBILITY = "collision_infeasibility"
    TRAJECTORY_OPTIMIZATION_FAILURE = "trajectory_optimization_failure"
    TERMINAL_LINE_VALIDATION_FAILURE = "terminal_line_validation_failure"
    ORIENTATION_VALIDATION_FAILURE = "orientation_validation_failure"
    NUMERICAL_FAILURE = "numerical_failure"
    CONFIGURATION_MODEL_FAILURE = "configuration_model_failure"


@dataclass(frozen=True)
class WorkspaceRegion:
    label: str
    minimum_m: tuple[float, float, float]
    maximum_m: tuple[float, float, float]


@dataclass(frozen=True)
class NormalBin:
    label: str
    direction_base: tuple[float, float, float]
    cone_half_angle_rad: float


@dataclass(frozen=True)
class StartJointConfiguration:
    label: str
    position_rad: tuple[float, ...]


@dataclass(frozen=True)
class BenchmarkConfig:
    """Validated deterministic benchmark sampling contract."""

    frame: str
    workspace_declaration: str
    regions: tuple[WorkspaceRegion, ...]
    normal_bins: tuple[NormalBin, ...]
    start_joint_bank: tuple[StartJointConfiguration, ...]
    pre_approach_range_m: tuple[float, float]
    planner_seed_sweep: tuple[int, ...]
    fixed_roll_probability: float
    fixed_roll_candidates_rad: tuple[float, ...]
    roll_candidates_rad: tuple[float, ...]
    stage_sizes: dict[str, int]
    scene_revision: str
    planner_profile: str
    validation_profile: str
    residual_safety_profile: str
    repeat_count: int


@dataclass(frozen=True)
class BenchmarkCase:
    """Compact sampled parameters sufficient to rebuild a planning request."""

    case_id: str
    root_seed: int
    sample_index: int
    region_label: str
    normal_bin_label: str
    start_joint_label: str
    position_base_m: tuple[float, float, float]
    surface_normal_base: tuple[float, float, float]
    tangent_hint_base: tuple[float, float, float] | None
    start_joint_position_rad: tuple[float, ...]
    pre_approach_distance_m: float
    planner_seed: int
    fixed_roll_rad: float | None
    roll_candidates_rad: tuple[float, ...]
    scene_revision: str
    planner_profile: str

    def to_request(self) -> PlanningRequest:
        """Rebuild the exact typed request represented by this case."""

        target = SurfaceTarget.create(
            position_base_m=self.position_base_m,
            surface_normal_base=self.surface_normal_base,
            tangent_hint_base=self.tangent_hint_base,
            fixed_roll_rad=self.fixed_roll_rad,
            roll_candidates_rad=(
                None if self.fixed_roll_rad is not None else self.roll_candidates_rad
            ),
            pre_approach_distance_m=self.pre_approach_distance_m,
            tool_frame=TCP_LINK,
            target_id=self.case_id,
        )
        return PlanningRequest(
            current_joint_state=NamedJointState.create(JOINT_NAMES, self.start_joint_position_rad),
            surface_target=target,
            scene_revision=self.scene_revision,
            planner_profile=self.planner_profile,
            random_seed=self.planner_seed,
            request_id=self.case_id,
        )


@dataclass(frozen=True)
class BenchmarkResult:
    """One case result with raw status and exact replay request."""

    case: BenchmarkCase
    planning_succeeded: bool
    validation_passed: bool
    failure_category: FailureCategory | None
    failure_reason: str | None
    raw_planner_status: str
    selected_roll_rad: float | None
    planner_timings_s: dict[str, float]
    validation_metrics: ValidationMetrics | None
    validation_violations: tuple[ValidationViolation, ...]
    execution_attempted: bool
    execution_completed: bool | None
    execution_failure_category: str | None
    repeat_disagreed: bool
    replay_request: dict[str, Any]

    @property
    def succeeded(self) -> bool:
        return self.planning_succeeded and self.validation_passed


@dataclass(frozen=True)
class BenchmarkSummary:
    """Aggregate metrics that never remove or rewrite failed attempts."""

    root_seed: int
    stage: str
    total_cases: int
    planning_successes: int
    validation_passes: int
    successes: int
    planning_success_rate: float
    validation_pass_rate: float
    success_rate: float
    success_by_region: dict[str, dict[str, float | int]]
    success_by_normal_bin: dict[str, dict[str, float | int]]
    failure_category_counts: dict[str, int]
    selected_roll_counts: dict[str, int]
    planner_time_s: dict[str, float | None]
    lateral_error_m: dict[str, float | None]
    orientation_error_rad: dict[str, float | None]
    minimum_clearance_m: dict[str, float | None]
    repeat_disagreement_count: int
    repeat_disagreement_rate: float
    execution_failure_count: int


class BenchmarkPlanner(Protocol):
    def plan(self, request: PlanningRequest) -> PlanningOutcome:
        """Return a typed nominal planning outcome."""


class BenchmarkValidator(Protocol):
    def __call__(self, plan: NominalPlan, request: PlanningRequest) -> ValidatedPlan:
        """Independently validate a nominal plan."""


PlannerFactory = Callable[[int], BenchmarkPlanner]
Executor = Callable[[ValidatedPlan, PlanningRequest], ExecutionResult]


def _tuple3(value: Sequence[float], label: str) -> tuple[float, float, float]:
    vector = np.asarray(value, dtype=float)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise ConfigurationError(f"{label} must contain three finite SI values")
    return tuple(float(item) for item in vector)


def load_benchmark_config(
    path: Path | str = Path("config/benchmark_workspace.yml"),
) -> BenchmarkConfig:
    """Load and validate the declared Phase 6 sampling space."""

    source = Path(path)
    if not source.is_file():
        raise ConfigurationError(f"benchmark config not found: {source}")
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ConfigurationError("benchmark config root must be a mapping")
    if payload.get("frame") != "g_base":
        raise ConfigurationError("benchmark frame must be explicitly g_base")

    regions = tuple(
        WorkspaceRegion(
            label=str(item["label"]),
            minimum_m=_tuple3(item["minimum_m"], "region minimum_m"),
            maximum_m=_tuple3(item["maximum_m"], "region maximum_m"),
        )
        for item in payload["regions"]
    )
    for region in regions:
        if any(low >= high for low, high in zip(region.minimum_m, region.maximum_m)):
            raise ConfigurationError(f"region {region.label!r} has an invalid AABB")

    bins: list[NormalBin] = []
    for item in payload["normal_bins"]:
        direction = np.asarray(_tuple3(item["direction_base"], "normal direction"), dtype=float)
        norm = float(np.linalg.norm(direction))
        angle = math.radians(float(item["cone_half_angle_deg"]))
        if norm <= 1.0e-12 or not 0.0 <= angle < math.pi / 2.0:
            raise ConfigurationError("normal bins require a direction and half-angle in [0, 90)")
        bins.append(
            NormalBin(
                label=str(item["label"]),
                direction_base=tuple(float(value) for value in direction / norm),
                cone_half_angle_rad=angle,
            )
        )

    bank = tuple(
        StartJointConfiguration(
            label=str(item["label"]),
            position_rad=tuple(float(value) for value in item["position_rad"]),
        )
        for item in payload["start_joint_bank"]
    )
    if any(len(item.position_rad) != len(JOINT_NAMES) for item in bank):
        raise ConfigurationError("each benchmark start state must contain six radians values")

    pre_approach = tuple(float(value) for value in payload["pre_approach_range_m"])
    if len(pre_approach) != 2 or not 0.01 <= pre_approach[0] < pre_approach[1] <= 0.15:
        raise ConfigurationError("pre_approach_range_m must be an ordered safe pair")
    seeds = tuple(int(value) for value in payload["planner_seed_sweep"])
    stages = {str(key): int(value) for key, value in payload["stage_sizes"].items()}
    if not seeds or any(seed < 0 for seed in seeds):
        raise ConfigurationError("planner_seed_sweep must contain non-negative seeds")
    if stages.get("smoke", 0) < 20 or stages.get("regression", 0) < 100:
        raise ConfigurationError("smoke/regression benchmark sizes are below Phase 6 minimums")
    if stages.get("exploratory", 0) < 1000:
        raise ConfigurationError("exploratory benchmark size is below Phase 6 minimum")
    repeat_count = int(payload["repeat_count"])
    if repeat_count < 2:
        raise ConfigurationError("repeat_count must be at least two")
    roll = payload["roll_policy"]
    probability = float(roll["fixed_probability"])
    if not 0.0 <= probability <= 1.0:
        raise ConfigurationError("fixed roll probability must be within [0, 1]")

    return BenchmarkConfig(
        frame="g_base",
        workspace_declaration=str(payload["workspace_declaration"]),
        regions=regions,
        normal_bins=tuple(bins),
        start_joint_bank=bank,
        pre_approach_range_m=(pre_approach[0], pre_approach[1]),
        planner_seed_sweep=seeds,
        fixed_roll_probability=probability,
        fixed_roll_candidates_rad=tuple(
            math.radians(float(value)) for value in roll["fixed_candidates_deg"]
        ),
        roll_candidates_rad=tuple(
            math.radians(float(value)) for value in roll["candidate_set_deg"]
        ),
        stage_sizes=stages,
        scene_revision=str(payload["scene_revision"]),
        planner_profile=str(payload["planner_profile"]),
        validation_profile=str(payload["validation_profile"]),
        residual_safety_profile=str(payload["residual_safety_profile"]),
        repeat_count=repeat_count,
    )


def _sample_normal(rng: np.random.Generator, normal_bin: NormalBin) -> np.ndarray:
    axis = np.asarray(normal_bin.direction_base)
    helper = np.eye(3)[int(np.argmin(np.abs(axis)))]
    tangent = np.cross(axis, helper)
    tangent /= np.linalg.norm(tangent)
    bitangent = np.cross(axis, tangent)
    cosine = rng.uniform(math.cos(normal_bin.cone_half_angle_rad), 1.0)
    sine = math.sqrt(max(0.0, 1.0 - cosine * cosine))
    azimuth = rng.uniform(0.0, 2.0 * math.pi)
    return cosine * axis + sine * (math.cos(azimuth) * tangent + math.sin(azimuth) * bitangent)


def sample_benchmark_cases(
    config: BenchmarkConfig,
    *,
    root_seed: int,
    stage: str,
    count: int | None = None,
) -> tuple[BenchmarkCase, ...]:
    """Generate cases reproducibly from only the root seed and config."""

    if root_seed < 0:
        raise ConfigurationError("benchmark root seed must be non-negative")
    if stage not in config.stage_sizes:
        raise ConfigurationError(f"unknown benchmark stage: {stage}")
    case_count = config.stage_sizes[stage] if count is None else int(count)
    if case_count <= 0:
        raise ConfigurationError("benchmark case count must be positive")
    rng = np.random.default_rng(root_seed)
    cases: list[BenchmarkCase] = []
    for index in range(case_count):
        region = config.regions[int(rng.integers(len(config.regions)))]
        normal_bin = config.normal_bins[int(rng.integers(len(config.normal_bins)))]
        start = config.start_joint_bank[int(rng.integers(len(config.start_joint_bank)))]
        position = rng.uniform(region.minimum_m, region.maximum_m)
        normal = _sample_normal(rng, normal_bin)
        fixed = rng.random() < config.fixed_roll_probability
        fixed_roll = (
            config.fixed_roll_candidates_rad[
                int(rng.integers(len(config.fixed_roll_candidates_rad)))
            ]
            if fixed
            else None
        )
        cases.append(
            BenchmarkCase(
                case_id=f"phase6-{stage}-{root_seed}-{index:05d}",
                root_seed=root_seed,
                sample_index=index,
                region_label=region.label,
                normal_bin_label=normal_bin.label,
                start_joint_label=start.label,
                position_base_m=tuple(float(value) for value in position),
                surface_normal_base=tuple(float(value) for value in normal),
                tangent_hint_base=None,
                start_joint_position_rad=start.position_rad,
                pre_approach_distance_m=float(rng.uniform(*config.pre_approach_range_m)),
                planner_seed=config.planner_seed_sweep[index % len(config.planner_seed_sweep)],
                fixed_roll_rad=fixed_roll,
                roll_candidates_rad=() if fixed else config.roll_candidates_rad,
                scene_revision=config.scene_revision,
                planner_profile=config.planner_profile,
            )
        )
    return tuple(cases)


def planning_failure_category(failure: PlanningFailure) -> FailureCategory:
    """Map planner categories/status text without discarding the raw status."""

    text = " ".join((failure.category, failure.reason, failure.planner_status)).lower()
    if any(token in text for token in ("collision", "self_coll", "world_coll")):
        return FailureCategory.COLLISION_INFEASIBILITY
    if any(token in text for token in ("no_ik", "ik_fail", "ik infeasible", "reachable ik")):
        return FailureCategory.NO_REACHABLE_IK
    if any(
        token in text
        for token in ("nan", "non-finite", "infinite", "numerical", "cuda", "backend_error")
    ):
        return FailureCategory.NUMERICAL_FAILURE
    if any(
        token in text
        for token in ("configuration", "invalid_planner_result", "model", "joint name")
    ):
        return FailureCategory.CONFIGURATION_MODEL_FAILURE
    return FailureCategory.TRAJECTORY_OPTIMIZATION_FAILURE


_LINE_METRICS = {
    "lateral_error",
    "progress_regression",
    "terminal_position_error",
}
_ORIENTATION_METRICS = {
    "approach_axis_error",
    "roll_error",
    "terminal_orientation_error",
}
_COLLISION_METRICS = {"self_collision_clearance", "world_collision_clearance"}
_NUMERICAL_METRICS = {"finite", "kinematics_finite"}
_CONFIGURATION_METRICS = {"kinematics_collision", "kinematics_shape", "sample_count"}


def validation_failure_category(
    violations: Sequence[ValidationViolation],
) -> FailureCategory:
    """Map all violations by deterministic safety-oriented precedence."""

    metrics = {item.metric for item in violations}
    if metrics & _NUMERICAL_METRICS:
        return FailureCategory.NUMERICAL_FAILURE
    if metrics & _CONFIGURATION_METRICS:
        return FailureCategory.CONFIGURATION_MODEL_FAILURE
    if metrics & _COLLISION_METRICS:
        return FailureCategory.COLLISION_INFEASIBILITY
    if metrics & _LINE_METRICS:
        return FailureCategory.TERMINAL_LINE_VALIDATION_FAILURE
    if metrics & _ORIENTATION_METRICS:
        return FailureCategory.ORIENTATION_VALIDATION_FAILURE
    return FailureCategory.TRAJECTORY_OPTIMIZATION_FAILURE


def serialize_request(request: PlanningRequest) -> dict[str, Any]:
    """Serialize every request field needed for exact failed-case replay."""

    target = request.surface_target
    return {
        "request_id": request.request_id,
        "scene_revision": request.scene_revision,
        "planner_profile": request.planner_profile,
        "random_seed": request.random_seed,
        "current_joint_state": {
            "names": list(request.current_joint_state.names),
            "position_rad": request.current_joint_state.position_rad.tolist(),
        },
        "surface_target": {
            "target_id": target.target_id,
            "position_base_m": target.position_base_m.tolist(),
            "surface_normal_base": target.surface_normal_base.tolist(),
            "tangent_hint_base": (
                None if target.tangent_hint_base is None else target.tangent_hint_base.tolist()
            ),
            "fixed_roll_rad": target.fixed_roll_rad,
            "roll_candidates_rad": list(target.roll_candidates_rad),
            "pre_approach_distance_m": target.pre_approach_distance_m,
            "tool_frame": target.tool_frame,
        },
    }


def deserialize_request(payload: dict[str, Any]) -> PlanningRequest:
    """Rebuild a typed request from a report replay object."""

    state = payload["current_joint_state"]
    target = payload["surface_target"]
    return PlanningRequest(
        current_joint_state=NamedJointState.create(
            tuple(str(value) for value in state["names"]), state["position_rad"]
        ),
        surface_target=SurfaceTarget.create(
            position_base_m=target["position_base_m"],
            surface_normal_base=target["surface_normal_base"],
            tangent_hint_base=target["tangent_hint_base"],
            fixed_roll_rad=target["fixed_roll_rad"],
            roll_candidates_rad=(
                None if target["fixed_roll_rad"] is not None else target["roll_candidates_rad"]
            ),
            pre_approach_distance_m=target["pre_approach_distance_m"],
            tool_frame=target["tool_frame"],
            target_id=target["target_id"],
        ),
        scene_revision=str(payload["scene_revision"]),
        planner_profile=str(payload["planner_profile"]),
        random_seed=int(payload["random_seed"]),
        request_id=str(payload["request_id"]),
    )


class BenchmarkRunner:
    """Run plan → validate → optional zero-residual replay without hiding failures."""

    def __init__(
        self,
        *,
        planner_factory: PlannerFactory,
        validator: BenchmarkValidator,
        repeat_count: int = 2,
        executor: Executor | None = None,
    ) -> None:
        if repeat_count < 1:
            raise ConfigurationError("repeat_count must be positive")
        self._planner_factory = planner_factory
        self._validator = validator
        self._repeat_count = repeat_count
        self._executor = executor

    def run(self, cases: Sequence[BenchmarkCase]) -> tuple[BenchmarkResult, ...]:
        return tuple(self._run_case(case) for case in cases)

    def _attempt(self, case: BenchmarkCase) -> tuple[BenchmarkResult, tuple[Any, ...]]:
        request = case.to_request()
        replay = serialize_request(request)
        try:
            outcome = self._planner_factory(case.planner_seed).plan(request)
            if not outcome.succeeded:
                if outcome.failure is None:
                    raise ConfigurationError("failed planning outcome is missing failure data")
                category = planning_failure_category(outcome.failure)
                result = BenchmarkResult(
                    case=case,
                    planning_succeeded=False,
                    validation_passed=False,
                    failure_category=category,
                    failure_reason=outcome.failure.reason,
                    raw_planner_status=outcome.failure.planner_status,
                    selected_roll_rad=None,
                    planner_timings_s={},
                    validation_metrics=None,
                    validation_violations=(),
                    execution_attempted=False,
                    execution_completed=None,
                    execution_failure_category=None,
                    repeat_disagreed=False,
                    replay_request=replay,
                )
                return result, (False, category.value, outcome.failure.planner_status)
            if outcome.plan is None:
                raise ConfigurationError("successful planning outcome is missing a plan")
            validated = self._validator(outcome.plan, request)
            violations = validated.report.violations
            category = None if validated.report.valid else validation_failure_category(violations)
            execution: ExecutionResult | None = None
            if validated.report.valid and self._executor is not None:
                execution = self._executor(validated, request)
            result = BenchmarkResult(
                case=case,
                planning_succeeded=True,
                validation_passed=validated.report.valid,
                failure_category=category,
                failure_reason=None if not violations else "; ".join(v.reason for v in violations),
                raw_planner_status=outcome.plan.planner_status,
                selected_roll_rad=outcome.plan.selected_roll_rad,
                planner_timings_s=dict(outcome.plan.planner_timings_s),
                validation_metrics=validated.report.metrics,
                validation_violations=violations,
                execution_attempted=execution is not None,
                execution_completed=None if execution is None else execution.completed,
                execution_failure_category=(
                    None if execution is None else execution.failure_category
                ),
                repeat_disagreed=False,
                replay_request=replay,
            )
            metrics = (
                None if result.validation_metrics is None else asdict(result.validation_metrics)
            )
            signature = (
                True,
                result.validation_passed,
                None if category is None else category.value,
                result.raw_planner_status,
                result.selected_roll_rad,
                metrics,
                result.execution_completed,
                result.execution_failure_category,
            )
            return result, signature
        except ConfigurationError as exc:
            return self._exception_result(
                case, replay, FailureCategory.CONFIGURATION_MODEL_FAILURE, exc
            )
        except (ArithmeticError, FloatingPointError, RuntimeError, ValueError) as exc:
            return self._exception_result(case, replay, FailureCategory.NUMERICAL_FAILURE, exc)

    @staticmethod
    def _exception_result(
        case: BenchmarkCase,
        replay: dict[str, Any],
        category: FailureCategory,
        exc: Exception,
    ) -> tuple[BenchmarkResult, tuple[Any, ...]]:
        result = BenchmarkResult(
            case=case,
            planning_succeeded=False,
            validation_passed=False,
            failure_category=category,
            failure_reason=str(exc),
            raw_planner_status="exception",
            selected_roll_rad=None,
            planner_timings_s={},
            validation_metrics=None,
            validation_violations=(),
            execution_attempted=False,
            execution_completed=None,
            execution_failure_category=None,
            repeat_disagreed=False,
            replay_request=replay,
        )
        return result, (False, category.value, "exception", str(exc))

    def _run_case(self, case: BenchmarkCase) -> BenchmarkResult:
        attempts = [self._attempt(case) for _ in range(self._repeat_count)]
        first, first_signature = attempts[0]
        disagreed = any(signature != first_signature for _, signature in attempts[1:])
        return BenchmarkResult(**{**vars(first), "repeat_disagreed": disagreed})


def _distribution(values: Sequence[float]) -> dict[str, float | None]:
    if not values:
        return {"minimum": None, "median": None, "p95": None, "maximum": None}
    array = np.asarray(values, dtype=float)
    return {
        "minimum": float(np.min(array)),
        "median": float(np.percentile(array, 50)),
        "p95": float(np.percentile(array, 95)),
        "maximum": float(np.max(array)),
    }


def _rates(
    results: Sequence[BenchmarkResult],
    attribute: str,
) -> dict[str, dict[str, float | int]]:
    groups: dict[str, list[BenchmarkResult]] = defaultdict(list)
    for result in results:
        groups[str(getattr(result.case, attribute))].append(result)
    return {
        label: {
            "successes": sum(item.succeeded for item in items),
            "total": len(items),
            "success_rate": sum(item.succeeded for item in items) / len(items),
        }
        for label, items in sorted(groups.items())
    }


def aggregate_results(
    results: Sequence[BenchmarkResult], *, root_seed: int, stage: str
) -> BenchmarkSummary:
    """Aggregate every supplied attempt, including all failures."""

    if not results:
        raise ConfigurationError("cannot aggregate an empty benchmark")
    total = len(results)
    planning = sum(item.planning_succeeded for item in results)
    validation = sum(item.validation_passed for item in results)
    failures = Counter(
        item.failure_category.value for item in results if item.failure_category is not None
    )
    rolls = Counter(
        f"{item.selected_roll_rad:.12g}" for item in results if item.selected_roll_rad is not None
    )
    planner_times = [
        item.planner_timings_s["backend_planning_time"]
        for item in results
        if "backend_planning_time" in item.planner_timings_s
    ]
    metrics = [item.validation_metrics for item in results if item.validation_metrics is not None]
    lateral = [
        item.max_lateral_error_m for item in metrics if item.max_lateral_error_m is not None
    ]
    orientation = [
        item.max_approach_axis_error_rad
        for item in metrics
        if item.max_approach_axis_error_rad is not None
    ]
    clearances = [
        item.minimum_self_collision_clearance_m
        for item in metrics
        if item.minimum_self_collision_clearance_m is not None
    ]
    disagreements = sum(item.repeat_disagreed for item in results)
    execution_failures = sum(
        item.execution_attempted and item.execution_completed is False for item in results
    )
    return BenchmarkSummary(
        root_seed=root_seed,
        stage=stage,
        total_cases=total,
        planning_successes=planning,
        validation_passes=validation,
        successes=validation,
        planning_success_rate=planning / total,
        validation_pass_rate=validation / total,
        success_rate=validation / total,
        success_by_region=_rates(results, "region_label"),
        success_by_normal_bin=_rates(results, "normal_bin_label"),
        failure_category_counts=dict(sorted(failures.items())),
        selected_roll_counts=dict(sorted(rolls.items())),
        planner_time_s=_distribution(planner_times),
        lateral_error_m=_distribution(lateral),
        orientation_error_rad=_distribution(orientation),
        minimum_clearance_m=_distribution(clearances),
        repeat_disagreement_count=disagreements,
        repeat_disagreement_rate=disagreements / total,
        execution_failure_count=execution_failures,
    )


def _jsonable(value: Any) -> Any:
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {key: _jsonable(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(item) for item in value]
    return value


def write_benchmark_reports(
    summary: BenchmarkSummary,
    results: Sequence[BenchmarkResult],
    output_dir: Path | str = Path("artifacts/benchmarks"),
) -> tuple[Path, Path]:
    """Write matching JSON and Markdown reports, including failed-case replay."""

    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    stem = f"phase6_{summary.stage}_seed_{summary.root_seed}"
    json_path = destination / f"{stem}.json"
    markdown_path = destination / f"{stem}.md"
    payload = {
        "schema_version": 1,
        "summary": _jsonable(asdict(summary)),
        "results": [_jsonable(asdict(result)) for result in results],
        "failed_replay_requests": [
            result.replay_request for result in results if not result.succeeded
        ],
    }
    json_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    failure_lines = (
        "\n".join(
            f"- `{name}`: {count}" for name, count in summary.failure_category_counts.items()
        )
        or "- None"
    )
    markdown_path.write_text(
        "\n".join(
            (
                f"# Phase 6 benchmark: {summary.stage}",
                "",
                f"- Root seed: `{summary.root_seed}`",
                f"- Total cases: {summary.total_cases}",
                f"- Planning success: {summary.planning_success_rate:.3%}",
                f"- Validation pass / overall success: {summary.success_rate:.3%}",
                f"- Repeat disagreement: {summary.repeat_disagreement_rate:.3%}",
                f"- Execution replay failures (not planning failures): "
                f"{summary.execution_failure_count}",
                "",
                "## Failure categories",
                "",
                failure_lines,
                "",
                "Exact failed-case replay requests are embedded in the matching JSON report.",
                "",
            )
        ),
        encoding="utf-8",
    )
    return json_path, markdown_path


def write_case_fixture(cases: Sequence[BenchmarkCase], path: Path | str) -> Path:
    """Write compact frozen case parameters without trajectories."""

    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps([_jsonable(asdict(case)) for case in cases], indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return destination


def load_case_fixture(path: Path | str) -> tuple[BenchmarkCase, ...]:
    """Load compact frozen benchmark parameters."""

    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    return tuple(
        BenchmarkCase(
            **{
                **item,
                "position_base_m": tuple(item["position_base_m"]),
                "surface_normal_base": tuple(item["surface_normal_base"]),
                "tangent_hint_base": (
                    None if item["tangent_hint_base"] is None else tuple(item["tangent_hint_base"])
                ),
                "start_joint_position_rad": tuple(item["start_joint_position_rad"]),
                "roll_candidates_rad": tuple(item["roll_candidates_rad"]),
            }
        )
        for item in payload
    )
