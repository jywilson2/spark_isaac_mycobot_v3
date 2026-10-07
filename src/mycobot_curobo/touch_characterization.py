"""Opt-in measurement of multi-target tip-contact failure modes.

This module does not change default smoke, the integration 2×5 placement
policy, planner profiles, the armed collision-sphere robot, or pass/fail
thresholds. Callers opt in by constructing a characterization config and
invoking these functions. Planning, when requested, goes through
``MultiTargetEpisodeRunner`` and the existing planner profiles.

World-model blindness uses a documented proxy, not a PhysX replay:
scaffolding spheres (the 32-sphere self set) versus the Phase 1.1 dense
overlay. A positive coarse clearance and a negative dense clearance means
the coarse spheres miss a contact the overlay marks. That is not a claim
about the armed Option B planner, which already attaches the overlay for
world checks.
"""

from __future__ import annotations

import math
from collections import Counter
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Callable, Sequence

import numpy as np
import yaml

from mycobot_curobo.benchmark import (
    FailureCategory,
    planning_failure_category,
)
from mycobot_curobo.cube_scene import (
    CubeGeometry,
    flange_disk_cube_clearance_m,
    sphere_aabb_clearance_m,
)
from mycobot_curobo.errors import ConfigurationError
from mycobot_curobo.multi_target import (
    MultiTargetEpisode,
    MultiTargetEpisodeResult,
    NumberedTarget,
)
from mycobot_curobo.planner import PlanningFailure
from mycobot_curobo.robot_model import WORLD_COVER_LINK_SUFFIX
from mycobot_curobo.target_placement import approach_plane_separation_m, center_violates_rim

CHARACTERIZATION_TAG = "touch_characterization:"
PROXY_LABEL = "dense_overlay_vs_scaffolding_32"
DISTAL_LINK_NAMES: tuple[str, ...] = ("joint5", "joint6", "joint6_flange")
DENSITY_FRACTIONS: tuple[float, ...] = (0.05, 0.10, 0.25, 0.50, 1.0)
_GRAZE_GAP_FRACTION = 0.55
_GRAZE_HALF_EXTENT_M = 0.002


class TouchOutcomeBin(str, Enum):
    """Planning-outcome bins for one target attempt."""

    GEOMETRIC_INFEASIBILITY = "geometric_infeasibility"
    WORLD_MODEL_BLINDNESS = "world_model_blindness"
    IK_FAILURE = "ik_failure"
    TRAJOPT_PLAN_GRASP_FAILURE = "trajopt_plan_grasp_failure"
    VALIDATION_REJECTION = "validation_rejection"
    SUCCESS = "success"


@dataclass(frozen=True)
class TipErrorComponents:
    """Executed tip error relative to the contact point (metres)."""

    lateral_m: float
    along_approach_m: float
    total_m: float


@dataclass(frozen=True)
class LinkSphere:
    """One collision sphere in its link frame."""

    link_name: str
    center_m: tuple[float, float, float]
    radius_m: float


@dataclass(frozen=True)
class TargetAttribution:
    """One runner leg classified into the characterization bins."""

    episode_index: int
    target_id: str
    outcome_bin: TouchOutcomeBin
    phase6_category: str | None
    geometric_blocked: bool
    planner_status: str
    failure_reason: str | None
    coarse_clearance_m: float | None
    fine_clearance_m: float | None
    proxy_label: str
    approach_plane_separation_m: float | None


@dataclass(frozen=True)
class KnownValidQuery:
    """A joint pose whose reference sphere *centres* clear one cuboid.

    The cuboid is placed so at least one reference sphere *radius* intersects
    it. Marking the pose in collision is therefore a false infeasible relative
    to the centre point cloud (a mesh-sample proxy), not a PhysX result.
    """

    label: str
    link_frames: dict[str, np.ndarray]
    obstacle: CubeGeometry


@dataclass(frozen=True)
class SphereCurvePoint:
    label: str
    role: str
    sphere_count: int
    false_infeasible_rate: float
    query_count: int


@dataclass(frozen=True)
class PlacementPolicyReport:
    """Independent rim and start-collision tallies for one placement policy."""

    policy: str
    seed: int
    draws: int
    rim_failures: int
    start_collision_failures: int
    accepted: int


ClearanceFn = Callable[
    [tuple[float, ...], tuple[CubeGeometry, ...]],
    tuple[float | None, float | None],
]


def decompose_tip_error_m(
    executed_tip_m: Sequence[float],
    goal_position_m: Sequence[float],
    approach_direction_base: Sequence[float],
) -> TipErrorComponents:
    """Split tip error into lateral, along-approach, and total distance.

    ``approach_direction_base`` is the direction the tool travels toward the
    contact point (opposite the face outward normal when the task frame
    approaches against that normal). ``along_approach_m`` is signed: positive
    means the executed tip is past the goal along that direction.
    """

    executed = np.asarray(executed_tip_m, dtype=float).reshape(3)
    goal = np.asarray(goal_position_m, dtype=float).reshape(3)
    approach = np.asarray(approach_direction_base, dtype=float).reshape(3)
    if not np.all(np.isfinite(executed)) or not np.all(np.isfinite(goal)):
        raise ConfigurationError("tip and goal positions must be finite")
    norm = float(np.linalg.norm(approach))
    if not math.isfinite(norm) or norm <= 1.0e-12:
        raise ConfigurationError("approach_direction_base must be a non-zero finite vector")
    unit = approach / norm
    displacement = executed - goal
    along = float(np.dot(displacement, unit))
    lateral = displacement - along * unit
    return TipErrorComponents(
        lateral_m=float(np.linalg.norm(lateral)),
        along_approach_m=along,
        total_m=float(np.linalg.norm(displacement)),
    )


def flange_approach_blocked(
    target: NumberedTarget,
    neighbors: Sequence[NumberedTarget],
    *,
    flange_diameter_m: float,
    sample_count: int = 9,
) -> tuple[bool, float | None]:
    """Return whether a neighbor blocks the flange disk regardless of planning.

    The flange is a sphere of radius ``flange_diameter_m / 2`` swept from the
    pre-approach point to the contact point along the face outward normal.
    Negative clearance to any neighbor AABB is geometric infeasibility.
    The active contact cube is not a neighbor. Returns ``(blocked, closest
    approach-plane centre separation)``.
    """

    diameter = float(flange_diameter_m)
    if not math.isfinite(diameter) or diameter <= 0.0:
        raise ConfigurationError("flange_diameter_m must be positive finite")
    count = int(sample_count)
    if count < 2:
        raise ConfigurationError("sample_count must be at least 2")
    contact = np.asarray(target.to_surface_target().position_base_m, dtype=float)
    normal = np.asarray(target.outward_normal_base, dtype=float)
    start = contact + normal * float(target.pre_approach_distance_m)
    samples = [(1.0 - float(t)) * start + float(t) * contact for t in np.linspace(0.0, 1.0, count)]
    closest: float | None = None
    blocked = False
    for neighbor in neighbors:
        if neighbor.target_id == target.target_id:
            continue
        separation = approach_plane_separation_m(
            target.center_m, neighbor.center_m, target.outward_normal_base
        )
        closest = separation if closest is None else min(closest, separation)
        cube = neighbor.cube_geometry
        for point in samples:
            clearance = flange_disk_cube_clearance_m(
                tuple(float(v) for v in point), diameter, cube
            )
            if clearance < 0.0:
                blocked = True
    return blocked, closest


def attribute_target_outcome(
    *,
    geometric_blocked: bool,
    planning_succeeded: bool,
    validation_passed: bool,
    planner_status: str,
    failure_reason: str | None,
    coarse_clearance_m: float | None,
    fine_clearance_m: float | None,
) -> tuple[TouchOutcomeBin, str | None]:
    """Classify one attempt. Geometric blockage wins over the planner result.

    Phase 6 ``planning_failure_category`` maps IK tokens to ``ik_failure``.
    Every other planning failure, including collision infeasibility, maps to
    ``trajopt_plan_grasp_failure`` because ``plan_grasp`` refused the motion.
    The Phase 6 category is returned separately so collision infeasibility
    stays visible. World-model blindness fires only after a validated plan
    when the coarse clearance is non-negative and the dense proxy clearance
    is negative.
    """

    phase6: str | None = None
    if geometric_blocked:
        return TouchOutcomeBin.GEOMETRIC_INFEASIBILITY, phase6
    if not planning_succeeded:
        failure = PlanningFailure(
            category=failure_reason or planner_status or "planning_infeasible",
            reason=failure_reason or "",
            planner_status=planner_status or "",
        )
        category = planning_failure_category(failure)
        phase6 = category.value
        if category is FailureCategory.NO_REACHABLE_IK:
            return TouchOutcomeBin.IK_FAILURE, phase6
        return TouchOutcomeBin.TRAJOPT_PLAN_GRASP_FAILURE, phase6
    if not validation_passed:
        return TouchOutcomeBin.VALIDATION_REJECTION, phase6
    if (
        coarse_clearance_m is not None
        and fine_clearance_m is not None
        and math.isfinite(coarse_clearance_m)
        and math.isfinite(fine_clearance_m)
        and coarse_clearance_m >= 0.0
        and fine_clearance_m < 0.0
    ):
        return TouchOutcomeBin.WORLD_MODEL_BLINDNESS, phase6
    return TouchOutcomeBin.SUCCESS, phase6


def characterize_episode_results(
    results: Sequence[MultiTargetEpisodeResult],
    *,
    flange_diameter_m: float,
    clearance_fn: ClearanceFn | None = None,
    proxy_label: str = PROXY_LABEL,
) -> tuple[TargetAttribution, ...]:
    """Attribute every runner leg, tracking retain-false removals in order."""

    rows: list[TargetAttribution] = []
    for result in results:
        removed: set[str] = set()
        retain = result.episode.field.retain_targets_after_contact
        for leg in result.legs:
            target = result.episode.field.target_by_id(leg.to_id)
            neighbors = tuple(
                item
                for item in result.episode.field.targets
                if item.target_id != leg.to_id and item.target_id not in removed
            )
            blocked, separation = flange_approach_blocked(
                target, neighbors, flange_diameter_m=flange_diameter_m
            )
            coarse: float | None = None
            fine: float | None = None
            planning_ok = bool(leg.planning_succeeded)
            validation_ok = bool(leg.validation_passed)
            if (
                clearance_fn is not None
                and planning_ok
                and validation_ok
                and leg.final_joint_position_rad is not None
            ):
                cubes = tuple(item.cube_geometry for item in neighbors)
                try:
                    coarse, fine = clearance_fn(leg.final_joint_position_rad, cubes)
                except (ConfigurationError, ValueError):
                    coarse, fine = None, None
            outcome, phase6 = attribute_target_outcome(
                geometric_blocked=blocked,
                planning_succeeded=planning_ok,
                validation_passed=validation_ok,
                planner_status=leg.planner_status,
                failure_reason=leg.failure_reason,
                coarse_clearance_m=coarse,
                fine_clearance_m=fine,
            )
            rows.append(
                TargetAttribution(
                    episode_index=result.episode.episode_index,
                    target_id=leg.to_id,
                    outcome_bin=outcome,
                    phase6_category=phase6,
                    geometric_blocked=blocked,
                    planner_status=leg.planner_status,
                    failure_reason=leg.failure_reason,
                    coarse_clearance_m=coarse,
                    fine_clearance_m=fine,
                    proxy_label=proxy_label,
                    approach_plane_separation_m=separation,
                )
            )
            if (
                not retain
                and leg.failure_category is None
                and leg.validation_passed
                and leg.planning_succeeded
            ):
                removed.add(leg.to_id)
    return tuple(rows)


def geometric_field_precheck(
    episodes: Sequence[MultiTargetEpisode],
    *,
    flange_diameter_m: float,
) -> dict[str, int]:
    """Count full-field flange-sweep blocks before any planner call.

    Neighbors are every other cube in the field (no retain-false removal).
    This is an analytic census, not a pass/fail gate.
    """

    blocked = 0
    total = 0
    for episode in episodes:
        field = episode.field
        for target in field.targets:
            neighbors = tuple(item for item in field.targets if item.target_id != target.target_id)
            is_blocked, _separation = flange_approach_blocked(
                target, neighbors, flange_diameter_m=flange_diameter_m
            )
            total += 1
            if is_blocked:
                blocked += 1
    return {"targets": total, "geometric_infeasibility": blocked}


def summarize_attributions(rows: Sequence[TargetAttribution]) -> dict[str, object]:
    """Counts and rates for the six bins, plus the Phase 6 histogram."""

    total = len(rows)
    counts = Counter(row.outcome_bin.value for row in rows)
    phase6 = Counter(row.phase6_category for row in rows if row.phase6_category is not None)
    ordered = [item.value for item in TouchOutcomeBin]
    bin_counts = {name: int(counts.get(name, 0)) for name in ordered}
    rates = {name: (0.0 if total == 0 else bin_counts[name] / total) for name in ordered}
    return {
        "attempt_count": total,
        "bin_counts": bin_counts,
        "bin_rates": rates,
        "phase6_category_counts": dict(sorted(phase6.items())),
        "proxy_label": PROXY_LABEL,
    }


def format_characterization_summary(summary: dict[str, object]) -> str:
    """One stdout line using the ``touch_characterization:`` tag."""

    counts = summary.get("bin_counts")
    if not isinstance(counts, dict):
        raise ConfigurationError("summary bin_counts must be a mapping")
    names = tuple(item.value for item in TouchOutcomeBin)
    parts = [f"{key}={int(counts.get(key, 0))}" for key in names]
    attempts = int(summary.get("attempt_count", 0))
    return f"{CHARACTERIZATION_TAG} attempts={attempts} " + " ".join(parts)


def _as_center(value: object, label: str) -> tuple[float, float, float]:
    array = np.asarray(value, dtype=float).reshape(-1)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ConfigurationError(f"{label} must contain three finite values")
    return (float(array[0]), float(array[1]), float(array[2]))


def link_spheres_from_mapping(mapping: object) -> tuple[LinkSphere, ...]:
    """Parse a ``collision_spheres`` mapping of link → list[{center, radius}]."""

    if not isinstance(mapping, dict) or not mapping:
        raise ConfigurationError("collision_spheres must be a non-empty mapping")
    spheres: list[LinkSphere] = []
    for link_name in sorted(mapping):
        entries = mapping[link_name]
        if not isinstance(entries, list) or not entries:
            raise ConfigurationError(
                f"collision spheres for {link_name!r} must be a non-empty list"
            )
        for entry in entries:
            if not isinstance(entry, dict):
                raise ConfigurationError(f"sphere entry on {link_name!r} must be a mapping")
            radius = float(entry["radius"])
            if not math.isfinite(radius) or radius <= 0.0:
                raise ConfigurationError(f"sphere radius on {link_name!r} must be positive finite")
            spheres.append(
                LinkSphere(
                    link_name=str(link_name),
                    center_m=_as_center(entry["center"], f"{link_name} center"),
                    radius_m=radius,
                )
            )
    return tuple(spheres)


def load_scaffolding_spheres(robot_yaml: Path | str) -> tuple[LinkSphere, ...]:
    """Read inline scaffolding spheres from the robot YAML (no overlay merge)."""

    payload = yaml.safe_load(Path(robot_yaml).read_text(encoding="utf-8"))
    try:
        spheres = payload["robot_cfg"]["kinematics"]["collision_spheres"]
    except (TypeError, KeyError) as exc:
        raise ConfigurationError("robot YAML is missing inline collision_spheres") from exc
    return link_spheres_from_mapping(spheres)


def load_overlay_spheres(overlay_yaml: Path | str) -> tuple[LinkSphere, ...]:
    """Read the Phase 1.1 overlay file without writing it into the robot YAML."""

    payload = yaml.safe_load(Path(overlay_yaml).read_text(encoding="utf-8"))
    if not isinstance(payload, dict) or "collision_spheres" not in payload:
        raise ConfigurationError("sphere overlay must define collision_spheres")
    return link_spheres_from_mapping(payload["collision_spheres"])


def filter_link_spheres(
    spheres: Sequence[LinkSphere], link_names: Sequence[str]
) -> tuple[LinkSphere, ...]:
    allowed = set(link_names)
    return tuple(sphere for sphere in spheres if sphere.link_name in allowed)


def subsample_link_spheres(
    spheres: Sequence[LinkSphere], fraction: float
) -> tuple[LinkSphere, ...]:
    """Keep an even stride of each link's spheres. ``fraction`` is in (0, 1]."""

    portion = float(fraction)
    if not math.isfinite(portion) or portion <= 0.0 or portion > 1.0:
        raise ConfigurationError("sphere fraction must be in (0, 1]")
    grouped: dict[str, list[LinkSphere]] = {}
    for sphere in spheres:
        grouped.setdefault(sphere.link_name, []).append(sphere)
    kept: list[LinkSphere] = []
    for link_name in sorted(grouped):
        group = grouped[link_name]
        if portion == 1.0:
            kept.extend(group)
            continue
        count = min(len(group), max(1, int(round(len(group) * portion))))
        positions = np.linspace(0, len(group) - 1, count)
        chosen: list[int] = []
        for raw in positions:
            index = int(round(float(raw)))
            index = min(max(index, 0), len(group) - 1)
            if index not in chosen:
                chosen.append(index)
        for index in chosen:
            kept.append(group[index])
    return tuple(kept)


def resolve_sphere_frame(link_name: str, link_frames: dict[str, np.ndarray]) -> np.ndarray:
    """Map a sphere link, including ``*_world_cover``, onto a serial-chain frame."""

    if link_name in link_frames:
        return link_frames[link_name]
    suffix = WORLD_COVER_LINK_SUFFIX
    if link_name.endswith(suffix):
        parent = link_name[: -len(suffix)]
        if parent in link_frames:
            return link_frames[parent]
    raise ConfigurationError(f"no link frame for collision sphere link {link_name!r}")


def transform_spheres_to_base(
    spheres: Sequence[LinkSphere], link_frames: dict[str, np.ndarray]
) -> tuple[np.ndarray, np.ndarray]:
    if not spheres:
        raise ConfigurationError("sphere set is empty")
    centers = np.zeros((len(spheres), 3), dtype=float)
    radii = np.zeros(len(spheres), dtype=float)
    for index, sphere in enumerate(spheres):
        frame = resolve_sphere_frame(sphere.link_name, link_frames)
        rotation = np.asarray(frame[:3, :3], dtype=float)
        translation = np.asarray(frame[:3, 3], dtype=float)
        local = np.asarray(sphere.center_m, dtype=float)
        centers[index] = rotation @ local + translation
        radii[index] = sphere.radius_m
    return centers, radii


def sphere_set_clearance_m(
    spheres: Sequence[LinkSphere],
    link_frames: dict[str, np.ndarray],
    cubes: Sequence[CubeGeometry],
) -> float:
    """Minimum signed clearance of a sphere set against cuboid AABBs."""

    if not cubes or not spheres:
        return math.inf
    centers, radii = transform_spheres_to_base(spheres, link_frames)
    clearances = [
        sphere_aabb_clearance_m(
            centers,
            radii,
            cube.center_m,
            (0.5 * float(cube.edge_m),) * 3,
        )
        for cube in cubes
    ]
    return float(min(clearances))


def _gap_cube(center_m: np.ndarray, radius_m: float, direction: np.ndarray) -> CubeGeometry:
    half = _GRAZE_HALF_EXTENT_M
    gap = _GRAZE_GAP_FRACTION * float(radius_m)
    unit = direction / float(np.linalg.norm(direction))
    cube_center = center_m + unit * (gap + half)
    return CubeGeometry(
        center_m=(float(cube_center[0]), float(cube_center[1]), float(cube_center[2])),
        edge_m=2.0 * half,
        name="touch_char_graze",
    )


def grazing_obstacle_for_pose(
    spheres: Sequence[LinkSphere], link_frames: dict[str, np.ndarray]
) -> CubeGeometry | None:
    """Place a cuboid that misses every sphere centre and hits one radius.

    Returns ``None`` when no such cuboid is found. The resulting query is
    known-valid against the centre point cloud and falsely infeasible for any
    sphere set that includes the penetrating ball.
    """

    if not spheres:
        return None
    centers, radii = transform_spheres_to_base(spheres, link_frames)
    point_radii = np.zeros(radii.shape, dtype=float)
    directions = np.vstack((np.eye(3), -np.eye(3)))
    order = np.argsort(-radii)
    for index in order[:12]:
        radius = float(radii[int(index)])
        if radius < 1.0e-4:
            continue
        center = centers[int(index)]
        for direction in directions:
            cube = _gap_cube(center, radius, direction)
            half = (0.5 * float(cube.edge_m),) * 3
            point_clear = sphere_aabb_clearance_m(centers, point_radii, cube.center_m, half)
            sphere_clear = sphere_aabb_clearance_m(
                centers[int(index) : int(index) + 1],
                radii[int(index) : int(index) + 1],
                cube.center_m,
                half,
            )
            if point_clear > 1.0e-5 and sphere_clear < 0.0:
                return cube
    return None


def false_infeasible_rate(
    queries: Sequence[KnownValidQuery], spheres: Sequence[LinkSphere]
) -> float:
    """Fraction of known-valid queries the sphere set marks in collision."""

    if not queries:
        raise ConfigurationError("known-valid query set is empty")
    if not spheres:
        return 0.0
    hits = 0
    for query in queries:
        clearance = sphere_set_clearance_m(spheres, query.link_frames, (query.obstacle,))
        if clearance < 0.0:
            hits += 1
    return hits / len(queries)


def sphere_cover_curve(
    queries: Sequence[KnownValidQuery],
    *,
    overlay_spheres: Sequence[LinkSphere],
    scaffolding_spheres: Sequence[LinkSphere],
    distal_link_names: Sequence[str] = DISTAL_LINK_NAMES,
    fractions: Sequence[float] = DENSITY_FRACTIONS,
) -> tuple[SphereCurvePoint, ...]:
    """Sweep overlay density plus distal-only, self, and world sets.

    Sets are built in memory from the overlay and scaffolding YAML. Nothing
    here writes ``collision_sphere_overlay_role`` or arms the 1012-sphere
    cover as the default robot.
    """

    if not overlay_spheres:
        raise ConfigurationError("overlay sphere set is empty")
    points: list[SphereCurvePoint] = []
    query_count = len(queries)
    for fraction in fractions:
        subset = subsample_link_spheres(overlay_spheres, float(fraction))
        points.append(
            SphereCurvePoint(
                label=f"overlay_fraction_{fraction:.2f}",
                role="world",
                sphere_count=len(subset),
                false_infeasible_rate=false_infeasible_rate(queries, subset),
                query_count=query_count,
            )
        )
    distal = filter_link_spheres(overlay_spheres, distal_link_names)
    if distal:
        points.append(
            SphereCurvePoint(
                label="distal_overlay",
                role="world_distal",
                sphere_count=len(distal),
                false_infeasible_rate=false_infeasible_rate(queries, distal),
                query_count=query_count,
            )
        )
    if scaffolding_spheres:
        points.append(
            SphereCurvePoint(
                label="scaffolding_self",
                role="self",
                sphere_count=len(scaffolding_spheres),
                false_infeasible_rate=false_infeasible_rate(queries, scaffolding_spheres),
                query_count=query_count,
            )
        )
    points.append(
        SphereCurvePoint(
            label="overlay_world",
            role="world",
            sphere_count=len(overlay_spheres),
            false_infeasible_rate=false_infeasible_rate(queries, overlay_spheres),
            query_count=query_count,
        )
    )
    return tuple(points)


def load_workspace_success_centers(path: Path | str) -> tuple[tuple[float, float, float], ...]:
    """Successful cube centres from a tip-contact workspace artifact."""

    import json

    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    results = payload.get("results")
    if not isinstance(results, list):
        raise ConfigurationError("workspace artifact results must be a list")
    centers = [
        _as_center(item["center_m"], "workspace center")
        for item in results
        if isinstance(item, dict) and item.get("succeeded") is True
    ]
    if not centers:
        raise ConfigurationError("workspace artifact has no successful centres")
    return tuple(centers)


def _inside_aabb(
    center: Sequence[float],
    minimum_m: Sequence[float],
    maximum_m: Sequence[float],
) -> bool:
    point = np.asarray(center, dtype=float)
    low = np.asarray(minimum_m, dtype=float)
    high = np.asarray(maximum_m, dtype=float)
    return bool(np.all(point >= low) and np.all(point <= high))


def compare_placement_policies(
    *,
    seed: int,
    draw_count: int,
    field_minimum_m: Sequence[float],
    field_maximum_m: Sequence[float],
    edge_m: float,
    max_target_radial_m: float,
    workspace_centers_m: Sequence[Sequence[float]],
    start_sphere_centers_m: np.ndarray,
    start_sphere_radii_m: np.ndarray,
) -> tuple[PlacementPolicyReport, PlacementPolicyReport]:
    """Compare forward-AABB draws with workspace-map draws.

    Counts are independent: one draw can be both a rim failure and a
    start-collision failure. Accepted draws are neither. This does not
    replace the integration suite sampler.
    """

    if seed < 0:
        raise ConfigurationError("placement seed must be non-negative")
    draws = int(draw_count)
    if draws < 1:
        raise ConfigurationError("draw_count must be positive")
    edge = float(edge_m)
    if not math.isfinite(edge) or edge <= 0.0:
        raise ConfigurationError("edge_m must be positive finite")
    low = np.asarray(field_minimum_m, dtype=float)
    high = np.asarray(field_maximum_m, dtype=float)
    if low.shape != (3,) or high.shape != (3,) or np.any(low >= high):
        raise ConfigurationError("field AABB must be a finite ordered pair")
    inside = [
        tuple(float(v) for v in center)
        for center in workspace_centers_m
        if _inside_aabb(center, low, high)
    ]
    if not inside:
        raise ConfigurationError("no workspace-map centres lie inside the field AABB")
    centers = np.asarray(start_sphere_centers_m, dtype=float)
    radii = np.asarray(start_sphere_radii_m, dtype=float)
    if centers.ndim != 2 or centers.shape[1] != 3 or radii.shape != (centers.shape[0],):
        raise ConfigurationError("start spheres must have shapes [N,3] and [N]")

    def tally(
        samples: Sequence[Sequence[float]], policy: str, policy_seed: int
    ) -> PlacementPolicyReport:
        rim = 0
        start_hits = 0
        accepted = 0
        for sample in samples:
            is_rim = center_violates_rim(
                sample, edge_m=edge, max_target_radial_m=max_target_radial_m
            )
            clearance = sphere_aabb_clearance_m(centers, radii, sample, (0.5 * edge,) * 3)
            is_start = clearance < 0.0
            if is_rim:
                rim += 1
            if is_start:
                start_hits += 1
            if not is_rim and not is_start:
                accepted += 1
        return PlacementPolicyReport(
            policy=policy,
            seed=policy_seed,
            draws=len(samples),
            rim_failures=rim,
            start_collision_failures=start_hits,
            accepted=accepted,
        )

    aabb_rng = np.random.default_rng(seed)
    raw_draws = aabb_rng.uniform(low, high, size=(draws, 3))
    aabb_samples = [tuple(float(v) for v in row) for row in raw_draws]
    map_rng = np.random.default_rng(seed + 1)
    map_index = map_rng.integers(0, len(inside), size=draws)
    map_samples = [inside[int(index)] for index in map_index]
    return (
        tally(aabb_samples, "forward_aabb", seed),
        tally(map_samples, "workspace_map", seed + 1),
    )


def write_characterization_figures(
    summary: dict[str, object],
    curve: Sequence[SphereCurvePoint],
    output_dir: Path | str,
    *,
    open_loop_errors_m: Sequence[TipErrorComponents] = (),
) -> tuple[Path, ...]:
    """Write PNG and SVG figures. Requires matplotlib (Agg backend)."""

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []
    counts = summary.get("bin_counts")
    if not isinstance(counts, dict):
        raise ConfigurationError("summary bin_counts must be a mapping")
    labels = [item.value for item in TouchOutcomeBin]
    values = [int(counts.get(label, 0)) for label in labels]
    figure, axis = plt.subplots(figsize=(9.0, 4.5))
    axis.bar(range(len(labels)), values, color="#3c6e71")
    axis.set_xticks(range(len(labels)))
    axis.set_xticklabels(labels, rotation=25, ha="right")
    axis.set_ylabel("attempts")
    attempts = int(summary.get("attempt_count", 0))
    title = "Touch-failure characterization bins (simulation)"
    if attempts == 0:
        title = "Touch-failure bins (planner not run; counts are zero)"
    axis.set_title(title)
    figure.tight_layout()
    for suffix in (".png", ".svg"):
        path = destination / f"touch_characterization_failure_bins{suffix}"
        figure.savefig(path)
        written.append(path)
    plt.close(figure)

    if curve:
        density = [point for point in curve if point.label.startswith("overlay_fraction_")]
        figure, axis = plt.subplots(figsize=(7.5, 4.5))
        if density:
            axis.plot(
                [point.sphere_count for point in density],
                [point.false_infeasible_rate for point in density],
                marker="o",
                color="#1d3557",
                label="overlay density",
            )
        for point in curve:
            if point.label.startswith("overlay_fraction_"):
                continue
            axis.scatter(
                [point.sphere_count],
                [point.false_infeasible_rate],
                label=f"{point.label} ({point.role})",
            )
        axis.set_xlabel("sphere count")
        axis.set_ylabel("false-infeasible rate")
        axis.set_ylim(-0.05, 1.05)
        axis.set_title("Sphere density vs false-infeasible rate (centre-cloud proxy)")
        axis.legend(fontsize=8)
        figure.tight_layout()
        for suffix in (".png", ".svg"):
            path = destination / f"touch_characterization_sphere_curve{suffix}"
            figure.savefig(path)
            written.append(path)
        plt.close(figure)

    if open_loop_errors_m:
        figure, axis = plt.subplots(figsize=(7.5, 4.5))
        axis.hist(
            [item.total_m * 1000.0 for item in open_loop_errors_m],
            bins=min(12, max(4, len(open_loop_errors_m))),
            color="#e09f3e",
        )
        axis.set_xlabel("total tip error (mm, simulation)")
        axis.set_ylabel("legs")
        axis.set_title("Open-loop terminal tip error (snap disabled)")
        figure.tight_layout()
        for suffix in (".png", ".svg"):
            path = destination / f"touch_characterization_open_loop_tip_error{suffix}"
            figure.savefig(path)
            written.append(path)
        plt.close(figure)
    return tuple(written)
