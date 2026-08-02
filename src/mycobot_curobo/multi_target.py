"""Phase 7.2 multi-target tip-contact field, order policy, and episode runner."""

from __future__ import annotations

import json
import math
import secrets
import time
from collections import Counter
from dataclasses import asdict, dataclass, field, replace
from enum import Enum
from pathlib import Path
from typing import Any, Callable, Protocol, Sequence

import numpy as np
import yaml

from mycobot_curobo.cube_scene import (
    CubeGeometry,
    cube_face_center,
    cubes_to_curobo_scene_dict,
    multi_cube_scene_revision,
)
from mycobot_curobo.errors import ConfigurationError
from mycobot_curobo.planner import (
    JOINT_NAMES,
    NamedJointState,
    NominalPlan,
    PlanningOutcome,
    PlanningRequest,
)
from mycobot_curobo.planning_world import leg_world_geometries
from mycobot_curobo.robot_model import TCP_LINK
from mycobot_curobo.target_placement import (
    DEFAULT_DEXTEROUS_REACH,
    DEFAULT_Z_BAND_FRACTION,
    DexterousReachModel,
    KeepOutAabb,
    LayoutSpec,
    LogFn,
    ReachRejectionBudget,
    ReachRejectionRecord,
    TipIkFn,
    build_layout_centers,
    build_random_centers,
    ee_clearance_min_center_separation_m,
    parse_dexterous_reach,
    parse_keep_outs,
    parse_layout_spec,
    replace_out_of_reach_centers,
    resolve_z_band_half_m,
    validate_centers_separation,
    z_band_bounds,
)
from mycobot_curobo.targets import SurfaceTarget
from mycobot_curobo.tip_ik_screen import IkRejectionBudget, IkRejectionRecord
from mycobot_curobo.validation import ValidatedPlan, ValidationMetrics


class PlacementPolicy(str, Enum):
    GRID = "grid"
    MANUAL = "manual"
    RANDOM = "random"
    LAYOUT = "layout"


class OrderPolicy(str, Enum):
    SHUFFLE = "shuffle"
    LISTED = "listed"
    Z_DESC = "z_desc"


class TargetPopulation(str, Enum):
    """How episode target fields are obtained.

    ``fixed`` preserves Phase 7.2–7.4 whole-field placement. ``incremental``
    (Phase 7.5) grows the field one planner-verified candidate at a time.
    """

    FIXED = "fixed"
    INCREMENTAL = "incremental"


class ContactKind(str, Enum):
    ALLOWED_TIP_CONTACT = "allowed_tip_contact"
    PROHIBITED_BODY_CONTACT = "prohibited_body_contact"
    NONE = "none"


class MultiTargetFailureCategory(str, Enum):
    PLAN_FAILED = "plan_failed"
    VALIDATION_FAILED = "validation_failed"
    BODY_CONTACT = "body_contact"
    TIP_CONTACT_MISSED = "tip_contact_missed"
    MAX_PLANNING_FAILURE_PER_TARGET_EXCEEDED = "max_planning_failure_per_target_exceeded"
    MAX_TARGET_FAILURES_EXCEEDED = "max_target_failures_exceeded"  # deprecated PASS escape
    TARGETS_INCOMPLETE = "targets_incomplete"
    TARGETS_UNPLANNED = "targets_unplanned"
    MAX_RECONSIDER_PASSES_EXCEEDED = "max_reconsider_passes_exceeded"
    MAX_CONSECUTIVE_UNPLANNED_TARGETS_EXCEEDED = "max_consecutive_unplanned_targets_exceeded"
    INSUFFICIENT_TARGETS = "insufficient_targets"
    CONFIGURATION_MODEL_FAILURE = "configuration_model_failure"


# Keys that must be absent under target_population: incremental (fail closed).
INCREMENTAL_FORBIDDEN_KEYS: frozenset[str] = frozenset(
    {
        "target_count",
        "order",
        "max_planning_failure_per_target",
        "max_reconsider_passes",
        "max_consecutive_unplanned_targets",
        "max_field_regenerations",
        "require_tip_ik",
        "max_ik_rejections",
        "max_reach_rejections",
    }
)


@dataclass(frozen=True)
class ContactEvent:
    """Result of one tip/body contact classification sample."""

    kind: ContactKind
    target_id: str | None = None
    link_name: str | None = None


class ContactDetector(Protocol):
    """Report tip-allowed, body-prohibited, or no contact for a motion segment."""

    def classify(self) -> ContactEvent:
        """Return the highest-priority contact observed for the current leg."""
        ...


@dataclass(frozen=True)
class NumberedTarget:
    """One numbered surface target with identity-oriented cube collision geometry."""

    target_id: str
    center_m: tuple[float, float, float]
    edge_m: float
    outward_normal_base: tuple[float, float, float]
    fixed_roll_rad: float | None = 0.0
    roll_candidates_rad: tuple[float, ...] = ()
    pre_approach_distance_m: float = 0.05

    def __post_init__(self) -> None:
        if not self.target_id or not str(self.target_id).strip():
            raise ConfigurationError("target_id must be a non-empty string")
        center = np.asarray(self.center_m, dtype=float)
        normal = np.asarray(self.outward_normal_base, dtype=float)
        if center.shape != (3,) or not np.all(np.isfinite(center)):
            raise ConfigurationError("center_m must contain three finite values")
        if not math.isfinite(self.edge_m) or self.edge_m <= 0.0:
            raise ConfigurationError("edge_m must be positive and finite")
        norm = float(np.linalg.norm(normal))
        if normal.shape != (3,) or not np.all(np.isfinite(normal)) or norm <= 1.0e-12:
            raise ConfigurationError("outward_normal_base must be a non-zero finite 3-vector")
        object.__setattr__(self, "center_m", tuple(float(item) for item in center))
        object.__setattr__(
            self,
            "outward_normal_base",
            tuple(float(item) for item in (normal / norm)),
        )

    @property
    def cube_geometry(self) -> CubeGeometry:
        return CubeGeometry(
            center_m=self.center_m,
            edge_m=self.edge_m,
            name=f"target_{self.target_id}",
        )

    def to_surface_target(self) -> SurfaceTarget:
        """Build a flange-normal contact target just past the selected face centre.

        A millimetre-scale inward offset makes PhysX tip/face overlap reliable while
        keeping the nominal approach on the flange-normal axis.
        """

        face = cube_face_center(self.center_m, self.edge_m, self.outward_normal_base)
        normal = np.asarray(self.outward_normal_base, dtype=float)
        position = face - 0.002 * normal
        return SurfaceTarget.create(
            position_base_m=position,
            surface_normal_base=self.outward_normal_base,
            fixed_roll_rad=self.fixed_roll_rad,
            roll_candidates_rad=(
                None if self.fixed_roll_rad is not None else self.roll_candidates_rad
            ),
            pre_approach_distance_m=self.pre_approach_distance_m,
            tool_frame=TCP_LINK,
            target_id=self.target_id,
        )


@dataclass(frozen=True)
class TargetField:
    """Numbered SurfaceTarget set with placement and contact-order policy."""

    targets: tuple[NumberedTarget, ...]
    placement: PlacementPolicy
    order: OrderPolicy
    retain_targets_after_contact: bool
    contact_order_ids: tuple[str, ...]

    def __post_init__(self) -> None:
        ids = [target.target_id for target in self.targets]
        if len(set(ids)) != len(ids):
            raise ConfigurationError("target_id values must be unique")
        if set(self.contact_order_ids) != set(ids) or len(self.contact_order_ids) != len(ids):
            raise ConfigurationError("contact_order_ids must be a permutation of target ids")

    def target_by_id(self, target_id: str) -> NumberedTarget:
        for target in self.targets:
            if target.target_id == target_id:
                return target
        raise ConfigurationError(f"unknown target_id: {target_id}")

    def active_geometries(
        self, *, removed_ids: Sequence[str] = (), contacted_ids: Sequence[str] = ()
    ) -> tuple[CubeGeometry, ...]:
        """Return collision geometry for the current scene revision.

        Args:
            removed_ids: Targets already despawned when retain is false.
            contacted_ids: Unused for geometry when retain is true; retained
                targets remain obstacles after tip contact.

        Returns:
            Active cube geometries participating in cuRobo world collision.
        """

        del contacted_ids
        removed = set(removed_ids)
        return tuple(
            target.cube_geometry for target in self.targets if target.target_id not in removed
        )


@dataclass(frozen=True)
class MultiTargetSuiteConfig:
    episode_count: int
    # Fixed mode: required positive count. Incremental mode: 0 (count is an outcome).
    target_count: int
    root_seed: int
    frame: str
    placement: PlacementPolicy
    # Fixed mode: required order policy. Incremental mode: LISTED placeholder
    # (contact order is acceptance order by construction; YAML must omit ``order``).
    order: OrderPolicy
    retain_targets_after_contact: bool
    max_planning_failure_per_target: int
    max_target_failures: int  # deprecated; must not allow PASS with unplanned targets
    max_reconsider_passes: int
    max_failed_episodes: int
    # Consecutive deferred (unplanned) targets without an intervening tip
    # success. Reaching this limit fails the episode; the runner may regenerate
    # the field up to max_field_regenerations times, otherwise remaining suite
    # planning is aborted. Default is max(3, ceil(target_count/3)).
    # Value 0 means no maximum (tracking/abort disabled).
    max_consecutive_unplanned_targets: int
    tip_allow_link_names: tuple[str, ...]
    field_minimum_m: tuple[float, float, float]
    field_maximum_m: tuple[float, float, float]
    # Declared vertical envelope magnitude for grid Z variability (meters).
    # Vendor MyCobot 280 working radius is the usual declared value; not a
    # measured dexterous-workspace claim.
    arm_z_motion_range_m: float
    target_edge_m: float
    outward_normal_base: tuple[float, float, float]
    manual_targets: tuple[NumberedTarget, ...]
    start_joint_position_rad: tuple[float, ...]
    planner_profile: str
    validation_profile: str
    scene_revision_prefix: str
    artifact_path: str
    minimum_self_collision_clearance_m: float
    minimum_world_collision_clearance_m: float
    lighting: dict[str, Any]
    pre_approach_distance_m: float
    roll_candidates_rad: tuple[float, ...]
    fixed_roll_rad: float | None
    warn_planning_duration_s: float | None
    flange_diameter_assumption_m: float
    # When true, terminal tip TCP must keep the flange disk inside the contact
    # face (fail closed on overhang / flange-edge collision).
    require_flange_face_containment: bool
    # Allowed flange overhang (m); typically matches planner position tolerance
    # so small lateral IK error does not fail containment on edge≈flange faces.
    flange_face_overhang_tolerance_m: float
    # Phase 7.3 placement controls (defaults preserve Phase 7.2 behaviour).
    min_center_separation_m: float
    ee_approach_clearance_m: float
    max_target_radial_m: float | None = None
    keep_outs: tuple[KeepOutAabb, ...] = ()
    max_placement_attempts: int = 1000
    layout: LayoutSpec | None = None
    # Phase 7.4 Z band / Z-aware spacing (defaults preserve Phase 7.2/7.3 fields).
    z_band_fraction: float = DEFAULT_Z_BAND_FRACTION
    delta_z_m: float | None = None
    z_separation_gain: float = 1.0
    dexterous_reach: DexterousReachModel = DEFAULT_DEXTEROUS_REACH
    # None → default target_count × episode_count at generation time.
    max_reach_rejections: int | None = None
    # None → default target_count per episode for tip-IK screening.
    max_ik_rejections: int | None = None
    # When true, host planning must supply a tip_ik_fn (fail closed otherwise).
    require_tip_ik: bool = False
    # Suite-wide field regenerations after consecutive-unplanned abort (default 3).
    max_field_regenerations: int = 3
    # Phase 7.5: fixed (default) vs incremental population.
    target_population: TargetPopulation = TargetPopulation.FIXED
    max_consecutive_target_failures: int = 5
    max_total_target_failures: int = 0
    max_targets_per_episode: int = 0
    min_targets_per_episode: int = 1


def _tuple3(value: Any, label: str) -> tuple[float, float, float]:
    array = np.asarray(value, dtype=float)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ConfigurationError(f"{label} must contain three finite values")
    return tuple(float(item) for item in array)


def _positive_int(value: Any, label: str) -> int:
    try:
        number = int(value)
    except (TypeError, ValueError) as exc:
        raise ConfigurationError(f"{label} must be a positive integer") from exc
    if number < 1:
        raise ConfigurationError(f"{label} must be a positive integer")
    return number


def _non_negative_int(value: Any, label: str) -> int:
    try:
        number = int(value)
    except (TypeError, ValueError) as exc:
        raise ConfigurationError(f"{label} must be a non-negative integer") from exc
    if number < 0:
        raise ConfigurationError(f"{label} must be a non-negative integer")
    return number


def default_max_consecutive_unplanned_targets(target_count: int) -> int:
    """Scale consecutive-unplanned abort with suite size (min 3)."""

    count = _positive_int(target_count, "target_count")
    return max(3, (count + 2) // 3)


def _max_consecutive_unplanned_targets(value: Any) -> int:
    """Parse max_consecutive_unplanned_targets: 0 = no max; else positive."""

    try:
        number = int(value)
    except (TypeError, ValueError) as exc:
        raise ConfigurationError(
            "max_consecutive_unplanned_targets must be 0 or a positive integer"
        ) from exc
    if number < 0:
        raise ConfigurationError(
            "max_consecutive_unplanned_targets must be 0 or a positive integer"
        )
    return number


def load_multi_target_suite_config(
    path: Path | str = Path("config/phase7_2_multi_target.yml"),
) -> MultiTargetSuiteConfig:
    """Load and validate a multi-target suite without simulator dependencies."""

    source = Path(path)
    if not source.is_file():
        raise ConfigurationError(f"multi-target suite config not found: {source}")
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(payload, dict) or payload.get("frame") != "g_base":
        raise ConfigurationError("multi-target suite must explicitly use frame g_base")
    population_raw = payload.get("target_population", TargetPopulation.FIXED.value)
    try:
        target_population = TargetPopulation(str(population_raw))
    except ValueError as exc:
        raise ConfigurationError("target_population must be 'fixed' or 'incremental'") from exc
    incremental = target_population is TargetPopulation.INCREMENTAL
    if incremental:
        for key in sorted(INCREMENTAL_FORBIDDEN_KEYS):
            if key in payload:
                raise ConfigurationError(
                    f"target_population=incremental forbids key '{key}' "
                    "(count/order/deferral/regen/tip-IK machinery does not run)"
                )
        if "placement" not in payload or str(payload["placement"]) != PlacementPolicy.RANDOM.value:
            raise ConfigurationError("target_population=incremental requires placement: random")
        if "retain_targets_after_contact" not in payload:
            raise ConfigurationError(
                "target_population=incremental requires retain_targets_after_contact: true"
            )
        if not bool(payload["retain_targets_after_contact"]):
            raise ConfigurationError(
                "target_population=incremental requires retain_targets_after_contact: true"
            )
        placement = PlacementPolicy.RANDOM
        order = OrderPolicy.LISTED
        target_count = 0
        retain = True
        max_planning_failure_per_target = 1
        max_target_failures = 0
        max_consecutive_unplanned_targets = 0
        max_reconsider_passes = 0
        max_field_regenerations = 0
        require_tip_ik = False
        max_reach_rejections = None
        max_ik_rejections = None
        max_consecutive_target_failures = _positive_int(
            payload.get("max_consecutive_target_failures", 5),
            "max_consecutive_target_failures",
        )
        max_total_target_failures = _non_negative_int(
            payload.get("max_total_target_failures", 0), "max_total_target_failures"
        )
        max_targets_per_episode = _non_negative_int(
            payload.get("max_targets_per_episode", 0), "max_targets_per_episode"
        )
        min_targets_per_episode = _positive_int(
            payload.get("min_targets_per_episode", 1), "min_targets_per_episode"
        )
        if max_targets_per_episode > 0 and max_targets_per_episode < min_targets_per_episode:
            raise ConfigurationError(
                "max_targets_per_episode must be 0 (disabled) or >= min_targets_per_episode"
            )
    else:
        placement = PlacementPolicy(str(payload["placement"]))
        order = OrderPolicy(str(payload["order"]))
        target_count = _positive_int(payload["target_count"], "target_count")
        retain = bool(payload.get("retain_targets_after_contact", False))
        max_planning_raw = payload.get("max_planning_failure_per_target")
        max_planning_failure_per_target = (
            3
            if max_planning_raw is None
            else _positive_int(max_planning_raw, "max_planning_failure_per_target")
        )
        max_target_raw = payload.get("max_target_failures")
        max_target_failures = (
            default_max_target_failures(target_count)
            if max_target_raw is None
            else _non_negative_int(max_target_raw, "max_target_failures")
        )
        max_consecutive_raw = payload.get("max_consecutive_unplanned_targets")
        max_consecutive_unplanned_targets = (
            default_max_consecutive_unplanned_targets(target_count)
            if max_consecutive_raw is None
            else _max_consecutive_unplanned_targets(max_consecutive_raw)
        )
        reconsider_raw = payload.get("max_reconsider_passes")
        max_reconsider_passes = (
            target_count
            if reconsider_raw is None
            else _positive_int(reconsider_raw, "max_reconsider_passes")
        )
        max_consecutive_target_failures = 5
        max_total_target_failures = 0
        max_targets_per_episode = 0
        min_targets_per_episode = 1
    episode_count = _positive_int(payload["episode_count"], "episode_count")
    max_failed_episodes = _non_negative_int(
        payload.get("max_failed_episodes", 0), "max_failed_episodes"
    )
    tip_links = tuple(str(item) for item in payload["tip_allow_link_names"])
    if not tip_links or any(not item.strip() for item in tip_links):
        raise ConfigurationError("tip_allow_link_names must be a non-empty list of link names")
    field = payload["field_aabb"]
    minimum_m = _tuple3(field["minimum_m"], "field_aabb.minimum_m")
    maximum_m = _tuple3(field["maximum_m"], "field_aabb.maximum_m")
    if any(lo >= hi for lo, hi in zip(minimum_m, maximum_m)):
        raise ConfigurationError("field_aabb maximum_m must be strictly greater than minimum_m")
    arm_z_raw = payload.get("arm_z_motion_range_m")
    if arm_z_raw is None:
        raise ConfigurationError(
            "arm_z_motion_range_m must be declared explicitly "
            "(meters; typically vendor working_radius_m)"
        )
    arm_z_motion_range_m = float(arm_z_raw)
    if not math.isfinite(arm_z_motion_range_m) or arm_z_motion_range_m <= 0.0:
        raise ConfigurationError("arm_z_motion_range_m must be positive and finite")
    edge = float(payload["target_edge_m"])
    if not math.isfinite(edge) or edge <= 0.0:
        raise ConfigurationError("target_edge_m must be positive and finite")
    from mycobot_curobo.robot_model import load_robot_model_spec

    robot_edge = load_robot_model_spec().min_detectable_obstacle_edge_m
    if edge + 1.0e-12 < robot_edge:
        raise ConfigurationError(
            "target_edge_m must be >= robot min_detectable_obstacle_edge_m "
            f"(target_edge_m={edge}, min_detectable_obstacle_edge_m={robot_edge})"
        )
    normal = _tuple3(payload["outward_normal_base"], "outward_normal_base")
    start = tuple(float(item) for item in payload["start_joint_position_rad"])
    if len(start) != len(JOINT_NAMES) or not all(math.isfinite(item) for item in start):
        raise ConfigurationError("start_joint_position_rad must contain six finite values")
    rolls_deg = payload.get("roll_candidates_deg")
    if "fixed_roll_rad" in payload:
        fixed_roll = payload.get("fixed_roll_rad")
    else:
        fixed_roll = None if rolls_deg is not None else 0.0
    if rolls_deg is not None and fixed_roll is not None:
        raise ConfigurationError("fixed_roll_rad and roll_candidates_deg are mutually exclusive")
    if rolls_deg is None:
        roll_candidates: tuple[float, ...] = ()
        fixed = float(fixed_roll) if fixed_roll is not None else 0.0
    else:
        fixed = None
        roll_candidates = tuple(math.radians(float(item)) for item in rolls_deg)
        if not roll_candidates:
            raise ConfigurationError("roll_candidates_deg must be non-empty when provided")
    pre_approach = float(payload["pre_approach_distance_m"])
    manual: list[NumberedTarget] = []
    if placement is PlacementPolicy.MANUAL:
        raw_targets = payload.get("targets")
        if not isinstance(raw_targets, list) or len(raw_targets) != target_count:
            raise ConfigurationError(
                "manual placement requires targets list matching target_count"
            )
        for item in raw_targets:
            manual.append(
                NumberedTarget(
                    target_id=str(item["target_id"]),
                    center_m=_tuple3(item["center_m"], "manual center_m"),
                    edge_m=float(item.get("edge_m", edge)),
                    outward_normal_base=_tuple3(
                        item.get("outward_normal_base", normal), "manual outward_normal_base"
                    ),
                    fixed_roll_rad=(
                        None
                        if item.get("fixed_roll_rad", fixed) is None
                        else float(item.get("fixed_roll_rad", fixed))
                    ),
                    roll_candidates_rad=tuple(
                        float(angle) for angle in item.get("roll_candidates_rad", roll_candidates)
                    ),
                    pre_approach_distance_m=float(
                        item.get("pre_approach_distance_m", pre_approach)
                    ),
                )
            )
    elif payload.get("targets"):
        raise ConfigurationError("targets list is only valid when placement is manual")
    layout = parse_layout_spec(payload.get("layout"))
    if placement is PlacementPolicy.LAYOUT:
        if layout is None:
            raise ConfigurationError("layout placement requires a layout mapping")
    elif layout is not None:
        raise ConfigurationError("layout is only valid when placement is layout")
    if placement is PlacementPolicy.RANDOM and payload.get("layout") is not None:
        raise ConfigurationError("layout is only valid when placement is layout")
    keep_outs = parse_keep_outs(payload.get("keep_outs"))
    flange_diameter_assumption_m = float(payload["flange_diameter_assumption_m"])
    if not math.isfinite(flange_diameter_assumption_m) or flange_diameter_assumption_m <= 0.0:
        raise ConfigurationError("flange_diameter_assumption_m must be positive finite")
    clearance_raw = payload.get("ee_approach_clearance_m")
    ee_approach_clearance_m = (
        flange_diameter_assumption_m if clearance_raw is None else float(clearance_raw)
    )
    if not math.isfinite(ee_approach_clearance_m) or ee_approach_clearance_m < 0.0:
        raise ConfigurationError("ee_approach_clearance_m must be finite and >= 0")
    radial_raw = payload.get("max_target_radial_m")
    max_target_radial_m = None if radial_raw is None else float(radial_raw)
    if max_target_radial_m is not None and (
        not math.isfinite(max_target_radial_m) or max_target_radial_m <= 0.0
    ):
        raise ConfigurationError("max_target_radial_m must be positive finite when set")
    ee_floor = ee_clearance_min_center_separation_m(
        edge,
        flange_diameter_assumption_m,
        ee_approach_clearance_m=ee_approach_clearance_m,
    )
    sep_raw = payload.get("min_center_separation_m")
    if sep_raw is None:
        min_center_separation_m = ee_floor
    else:
        min_center_separation_m = float(sep_raw)
        if not math.isfinite(min_center_separation_m) or min_center_separation_m <= 0.0:
            raise ConfigurationError("min_center_separation_m must be positive finite")
        if min_center_separation_m + 1.0e-12 < ee_floor:
            raise ConfigurationError(
                "min_center_separation_m is below EE clearance floor "
                f"(min_center_separation_m={min_center_separation_m}, "
                f"floor={ee_floor}=edge+flange+ee_approach_clearance_m)"
            )
    attempts_raw = payload.get("max_placement_attempts", 1000)
    max_placement_attempts = _positive_int(attempts_raw, "max_placement_attempts")
    overhang_tol_raw = payload.get("flange_face_overhang_tolerance_m")
    flange_face_overhang_tolerance_m = (
        0.005 if overhang_tol_raw is None else float(overhang_tol_raw)
    )
    if (
        not math.isfinite(flange_face_overhang_tolerance_m)
        or flange_face_overhang_tolerance_m < 0.0
    ):
        raise ConfigurationError("flange_face_overhang_tolerance_m must be finite and >= 0")
    fraction_raw = payload.get("z_band_fraction", DEFAULT_Z_BAND_FRACTION)
    z_band_fraction = float(fraction_raw)
    if not math.isfinite(z_band_fraction) or z_band_fraction <= 0.0:
        raise ConfigurationError("z_band_fraction must be positive finite")
    delta_raw = payload.get("delta_z_m")
    delta_z_m = None if delta_raw is None else float(delta_raw)
    if delta_z_m is not None and (not math.isfinite(delta_z_m) or delta_z_m <= 0.0):
        raise ConfigurationError("delta_z_m must be positive finite when set")
    # Resolve once so invalid combinations fail at config load.
    resolve_z_band_half_m(
        arm_z_motion_range_m=arm_z_motion_range_m,
        z_band_fraction=z_band_fraction,
        delta_z_m=delta_z_m,
    )
    gain_raw = payload.get("z_separation_gain", 1.0)
    z_separation_gain = float(gain_raw)
    if not math.isfinite(z_separation_gain) or z_separation_gain < 1.0:
        raise ConfigurationError("z_separation_gain must be finite and >= 1.0")
    dexterous_reach = parse_dexterous_reach(payload.get("dexterous_reach"))
    if not incremental:
        max_reach_raw = payload.get("max_reach_rejections")
        max_reach_rejections = (
            None
            if max_reach_raw is None
            else _non_negative_int(max_reach_raw, "max_reach_rejections")
        )
        max_ik_raw = payload.get("max_ik_rejections")
        max_ik_rejections = (
            None if max_ik_raw is None else _non_negative_int(max_ik_raw, "max_ik_rejections")
        )
        require_tip_ik = bool(payload.get("require_tip_ik", False))
        max_field_regenerations = _non_negative_int(
            payload.get("max_field_regenerations", 3), "max_field_regenerations"
        )
    warn = payload.get("warn_planning_duration_s")
    warn_s = None if warn is None else float(warn)
    if warn_s is not None and (not math.isfinite(warn_s) or warn_s <= 0.0):
        raise ConfigurationError("warn_planning_duration_s must be positive and finite when set")
    lighting = dict(payload["lighting"])
    return MultiTargetSuiteConfig(
        episode_count=episode_count,
        target_count=target_count,
        root_seed=int(payload["root_seed"]),
        frame="g_base",
        placement=placement,
        order=order,
        retain_targets_after_contact=retain,
        max_planning_failure_per_target=max_planning_failure_per_target,
        max_target_failures=max_target_failures,
        max_reconsider_passes=max_reconsider_passes,
        max_failed_episodes=max_failed_episodes,
        max_consecutive_unplanned_targets=max_consecutive_unplanned_targets,
        tip_allow_link_names=tip_links,
        field_minimum_m=minimum_m,
        field_maximum_m=maximum_m,
        arm_z_motion_range_m=arm_z_motion_range_m,
        target_edge_m=edge,
        outward_normal_base=normal,
        manual_targets=tuple(manual),
        start_joint_position_rad=start,
        planner_profile=str(payload["planner_profile"]),
        validation_profile=str(payload["validation_profile"]),
        scene_revision_prefix=str(payload["scene_revision_prefix"]),
        artifact_path=str(payload["artifact_path"]),
        minimum_self_collision_clearance_m=float(payload["minimum_self_collision_clearance_m"]),
        minimum_world_collision_clearance_m=float(payload["minimum_world_collision_clearance_m"]),
        lighting=lighting,
        pre_approach_distance_m=pre_approach,
        roll_candidates_rad=roll_candidates,
        fixed_roll_rad=fixed,
        warn_planning_duration_s=warn_s,
        flange_diameter_assumption_m=flange_diameter_assumption_m,
        require_flange_face_containment=bool(
            payload.get("require_flange_face_containment", False)
        ),
        flange_face_overhang_tolerance_m=flange_face_overhang_tolerance_m,
        min_center_separation_m=min_center_separation_m,
        ee_approach_clearance_m=ee_approach_clearance_m,
        max_target_radial_m=max_target_radial_m,
        keep_outs=keep_outs,
        max_placement_attempts=max_placement_attempts,
        layout=layout,
        z_band_fraction=z_band_fraction,
        delta_z_m=delta_z_m,
        z_separation_gain=z_separation_gain,
        dexterous_reach=dexterous_reach,
        max_reach_rejections=max_reach_rejections,
        max_ik_rejections=max_ik_rejections,
        require_tip_ik=require_tip_ik,
        max_field_regenerations=max_field_regenerations,
        target_population=target_population,
        max_consecutive_target_failures=max_consecutive_target_failures,
        max_total_target_failures=max_total_target_failures,
        max_targets_per_episode=max_targets_per_episode,
        min_targets_per_episode=min_targets_per_episode,
    )


def _grid_layout_shape(count: int) -> tuple[int, int]:
    columns = int(math.ceil(math.sqrt(count)))
    rows = int(math.ceil(count / columns))
    return rows, columns


def default_max_target_failures(target_count: int | None = None) -> int:
    """Return the default episode target-failure budget (fixed at 3)."""

    if target_count is not None:
        _positive_int(target_count, "target_count")
    return 3


def override_suite_target_count(
    config: MultiTargetSuiteConfig, target_count: int
) -> MultiTargetSuiteConfig:
    """Return a config with ``target_count`` overridden for smoke/CLI use.

    Manual lists shorter than ``target_count`` switch to grid placement inside
    the declared field AABB so ``--targets N`` stays usable without a new YAML.
    When the listed manual set is long enough, it is truncated to the first N
    targets in list order. ``max_reconsider_passes`` defaults to the new
    ``target_count`` unless the caller already set an explicit value that
    differs from the previous ``target_count``.
    """

    if config.target_population is TargetPopulation.INCREMENTAL:
        raise ConfigurationError(
            "override_suite_target_count is invalid for target_population=incremental "
            "(achieved count is an outcome, not an input)"
        )
    count = _positive_int(target_count, "target_count")
    reconsider = (
        count
        if config.max_reconsider_passes == config.target_count
        else config.max_reconsider_passes
    )
    consecutive = (
        default_max_consecutive_unplanned_targets(count)
        if config.max_consecutive_unplanned_targets
        == default_max_consecutive_unplanned_targets(config.target_count)
        else config.max_consecutive_unplanned_targets
    )
    if config.placement in {
        PlacementPolicy.GRID,
        PlacementPolicy.RANDOM,
        PlacementPolicy.LAYOUT,
    }:
        return replace(
            config,
            target_count=count,
            manual_targets=(),
            max_reconsider_passes=reconsider,
            max_consecutive_unplanned_targets=consecutive,
        )
    if len(config.manual_targets) >= count:
        return replace(
            config,
            target_count=count,
            manual_targets=config.manual_targets[:count],
            max_reconsider_passes=reconsider,
            max_consecutive_unplanned_targets=consecutive,
        )
    # Not enough explicit poses: fall back to a deterministic grid.
    return replace(
        config,
        target_count=count,
        placement=PlacementPolicy.GRID,
        manual_targets=(),
        layout=None,
        max_reconsider_passes=reconsider,
        max_consecutive_unplanned_targets=consecutive,
    )


def build_grid_centers(
    count: int,
    minimum_m: Sequence[float],
    maximum_m: Sequence[float],
    *,
    arm_z_motion_range_m: float,
    placement_seed: int | None = None,
    z_band_fraction: float = DEFAULT_Z_BAND_FRACTION,
    delta_z_m: float | None = None,
) -> tuple[tuple[float, float, float], ...]:
    """Place ``count`` centres on an XY lattice with mid-Z band variability.

    X/Y follow an evenly spaced grid inside the declared AABB. Z is centered on
    the AABB mid-height and spaced evenly across a band of width
    ``delta_z_m`` when set, else ``z_band_fraction * arm_z_motion_range_m``
    (default fraction 0.5). The band is **not** clipped to ``field_aabb`` Z;
    arm-reach validation and substitute retries reject unreachable centres.

    When ``placement_seed`` is set, apply a deterministic toroidal phase shift in
    X/Y/Z so multi-episode suites get distinct obstacle fields while remaining
    inside the same AABB / Z band.
    """

    if count < 1:
        raise ConfigurationError("grid count must be positive")
    lo = _tuple3(minimum_m, "minimum_m")
    hi = _tuple3(maximum_m, "maximum_m")
    rows, columns = _grid_layout_shape(count)
    mid_z, z_lo, z_hi = z_band_bounds(
        lo,
        hi,
        arm_z_motion_range_m=arm_z_motion_range_m,
        z_band_fraction=z_band_fraction,
        delta_z_m=delta_z_m,
    )
    half_band = 0.5 * (z_hi - z_lo)
    phase_x = 0.0
    phase_y = 0.0
    phase_z = 0.0
    if placement_seed is not None:
        rng = np.random.default_rng(int(placement_seed))
        phase_x = float(rng.uniform(0.0, 1.0))
        phase_y = float(rng.uniform(0.0, 1.0))
        phase_z = float(rng.uniform(0.0, 1.0))
    centers: list[tuple[float, float, float]] = []
    for index in range(count):
        row = index // columns
        col = index % columns
        if placement_seed is None:
            x = lo[0] if columns == 1 else lo[0] + (col + 0.5) * (hi[0] - lo[0]) / columns
            y = lo[1] if rows == 1 else lo[1] + (row + 0.5) * (hi[1] - lo[1]) / rows
            if count == 1:
                z = mid_z
            else:
                z = (mid_z - half_band) + (index + 0.5) * (2.0 * half_band) / count
        else:
            x_frac = ((col + 0.5) / columns + phase_x) % 1.0
            y_frac = ((row + 0.5) / rows + phase_y) % 1.0
            x = lo[0] + x_frac * (hi[0] - lo[0])
            y = lo[1] + y_frac * (hi[1] - lo[1])
            if count == 1:
                z = mid_z + (2.0 * phase_z - 1.0) * half_band
            else:
                z_frac = ((index + 0.5) / count + phase_z) % 1.0
                z = (mid_z - half_band) + z_frac * (2.0 * half_band)
        centers.append((float(x), float(y), float(z)))
    return tuple(centers)


def _targets_from_centers(
    config: MultiTargetSuiteConfig, centers: Sequence[tuple[float, float, float]]
) -> tuple[NumberedTarget, ...]:
    return tuple(
        NumberedTarget(
            target_id=str(index + 1),
            center_m=center,
            edge_m=config.target_edge_m,
            outward_normal_base=config.outward_normal_base,
            fixed_roll_rad=config.fixed_roll_rad,
            roll_candidates_rad=config.roll_candidates_rad,
            pre_approach_distance_m=config.pre_approach_distance_m,
        )
        for index, center in enumerate(centers)
    )


def build_target_field(
    config: MultiTargetSuiteConfig,
    *,
    order_seed: int,
    placement_seed: int | None = None,
    reach_budget: ReachRejectionBudget | None = None,
    tip_ik_fn: TipIkFn | None = None,
    ik_budget: IkRejectionBudget | None = None,
    log: LogFn | None = None,
) -> TargetField:
    """Build a numbered field and apply shuffle or listed contact order."""

    if config.require_tip_ik and tip_ik_fn is None:
        raise ConfigurationError(
            "require_tip_ik is true but no tip_ik_fn was provided for placement"
        )

    if config.placement is PlacementPolicy.MANUAL:
        targets = config.manual_targets
        if len(targets) != config.target_count:
            raise ConfigurationError("manual target count must match target_count")
        validate_centers_separation(
            [target.center_m for target in targets],
            min_center_separation_m=config.min_center_separation_m,
            edge_m=config.target_edge_m,
            keep_outs=config.keep_outs,
            outward_normal_base=config.outward_normal_base,
            max_target_radial_m=config.max_target_radial_m,
            z_separation_gain=config.z_separation_gain,
            pre_approach_distance_m=config.pre_approach_distance_m,
            field_minimum_m=config.field_minimum_m,
            field_maximum_m=config.field_maximum_m,
            arm_z_motion_range_m=config.arm_z_motion_range_m,
            require_arm_reach=False,
            require_dexterous_reach=True,
            dexterous_reach=config.dexterous_reach,
        )
        if tip_ik_fn is not None:
            for index, target in enumerate(targets):
                accepted = tuple(other.center_m for j, other in enumerate(targets) if j != index)
                if not tip_ik_fn(target.center_m, accepted):
                    raise ConfigurationError(
                        f"manual target {target.target_id} failed tip IK screen "
                        f"at centre={target.center_m}"
                    )
    elif config.placement is PlacementPolicy.RANDOM:
        if placement_seed is None:
            raise ConfigurationError("random placement requires placement_seed")
        centers = build_random_centers(
            config.target_count,
            config.field_minimum_m,
            config.field_maximum_m,
            arm_z_motion_range_m=config.arm_z_motion_range_m,
            edge_m=config.target_edge_m,
            min_center_separation_m=config.min_center_separation_m,
            keep_outs=config.keep_outs,
            placement_seed=placement_seed,
            max_placement_attempts=config.max_placement_attempts,
            outward_normal_base=config.outward_normal_base,
            max_target_radial_m=config.max_target_radial_m,
            z_band_fraction=config.z_band_fraction,
            delta_z_m=config.delta_z_m,
            z_separation_gain=config.z_separation_gain,
            pre_approach_distance_m=config.pre_approach_distance_m,
            dexterous_reach=config.dexterous_reach,
            reach_budget=reach_budget,
            tip_ik_fn=tip_ik_fn,
            ik_budget=ik_budget,
            log=log,
        )
        targets = _targets_from_centers(config, centers)
    elif config.placement is PlacementPolicy.LAYOUT:
        if config.layout is None:
            raise ConfigurationError("layout placement requires layout spec")
        centers = build_layout_centers(
            config.target_count,
            config.field_minimum_m,
            config.field_maximum_m,
            layout=config.layout,
            arm_z_motion_range_m=config.arm_z_motion_range_m,
            edge_m=config.target_edge_m,
            min_center_separation_m=config.min_center_separation_m,
            keep_outs=config.keep_outs,
            placement_seed=placement_seed,
            outward_normal_base=config.outward_normal_base,
            max_target_radial_m=config.max_target_radial_m,
            z_band_fraction=config.z_band_fraction,
            delta_z_m=config.delta_z_m,
            z_separation_gain=config.z_separation_gain,
            pre_approach_distance_m=config.pre_approach_distance_m,
            dexterous_reach=config.dexterous_reach,
            reach_budget=reach_budget,
            tip_ik_fn=tip_ik_fn,
            ik_budget=ik_budget,
            log=log,
        )
        targets = _targets_from_centers(config, centers)
    else:
        # Phase-shifted grids can land a lattice point in a base keep-out; search
        # deterministic seed offsets so surround-capable fields stay valid.
        # Keep-outs / phase-shifted Z-aware packs: search deterministic seed offsets.
        max_grid_seed_offsets = 64
        last_error: ConfigurationError | None = None
        centers: tuple[tuple[float, float, float], ...] | None = None
        _, z_lo_band, z_hi_band = z_band_bounds(
            config.field_minimum_m,
            config.field_maximum_m,
            arm_z_motion_range_m=config.arm_z_motion_range_m,
            z_band_fraction=config.z_band_fraction,
            delta_z_m=config.delta_z_m,
        )
        for offset in range(max_grid_seed_offsets):
            seed = None if placement_seed is None else int(placement_seed) + offset
            # Snapshot budget so a failed offset does not permanently consume
            # suite-wide rejections from an abandoned lattice phase.
            budget_snapshot = None if reach_budget is None else len(reach_budget.rejections)
            ik_snapshot = None if ik_budget is None else len(ik_budget.rejections)
            candidate = build_grid_centers(
                config.target_count,
                config.field_minimum_m,
                config.field_maximum_m,
                arm_z_motion_range_m=config.arm_z_motion_range_m,
                placement_seed=seed,
                z_band_fraction=config.z_band_fraction,
                delta_z_m=config.delta_z_m,
            )
            try:
                candidate = replace_out_of_reach_centers(
                    candidate,
                    field_minimum_m=config.field_minimum_m,
                    field_maximum_m=config.field_maximum_m,
                    arm_z_motion_range_m=config.arm_z_motion_range_m,
                    edge_m=config.target_edge_m,
                    min_center_separation_m=config.min_center_separation_m,
                    keep_outs=config.keep_outs,
                    outward_normal_base=config.outward_normal_base,
                    max_target_radial_m=config.max_target_radial_m,
                    z_lo_band=z_lo_band,
                    z_hi_band=z_hi_band,
                    z_separation_gain=config.z_separation_gain,
                    pre_approach_distance_m=config.pre_approach_distance_m,
                    placement_seed=seed,
                    dexterous_reach=config.dexterous_reach,
                    reach_budget=reach_budget,
                    tip_ik_fn=tip_ik_fn,
                    ik_budget=ik_budget,
                    log=log,
                )
                validate_centers_separation(
                    candidate,
                    min_center_separation_m=config.min_center_separation_m,
                    edge_m=config.target_edge_m,
                    keep_outs=config.keep_outs,
                    outward_normal_base=config.outward_normal_base,
                    max_target_radial_m=config.max_target_radial_m,
                    z_separation_gain=config.z_separation_gain,
                    pre_approach_distance_m=config.pre_approach_distance_m,
                    field_minimum_m=config.field_minimum_m,
                    field_maximum_m=config.field_maximum_m,
                    arm_z_motion_range_m=config.arm_z_motion_range_m,
                    require_arm_reach=False,
                    require_dexterous_reach=True,
                    dexterous_reach=config.dexterous_reach,
                )
            except ConfigurationError as exc:
                if "max_reach_rejections" in str(exc) or "max_ik_rejections" in str(exc):
                    raise
                last_error = exc
                if reach_budget is not None and budget_snapshot is not None:
                    del reach_budget.rejections[budget_snapshot:]
                if ik_budget is not None and ik_snapshot is not None:
                    del ik_budget.rejections[ik_snapshot:]
                continue
            centers = candidate
            break
        if centers is None:
            # Lattice + dexterous screen can leave no feasible phase for dense
            # --targets fallbacks; regenerate with random placement instead.
            if log is not None:
                log(
                    "phase7_4_placement: grid lattice infeasible under dexterous "
                    "reach / packing; falling back to random regeneration"
                    + ("" if last_error is None else f" (last: {last_error})")
                )
            fallback_seed = 0 if placement_seed is None else int(placement_seed)
            centers = build_random_centers(
                config.target_count,
                config.field_minimum_m,
                config.field_maximum_m,
                arm_z_motion_range_m=config.arm_z_motion_range_m,
                edge_m=config.target_edge_m,
                min_center_separation_m=config.min_center_separation_m,
                keep_outs=config.keep_outs,
                placement_seed=fallback_seed,
                max_placement_attempts=max(config.max_placement_attempts, 20000),
                outward_normal_base=config.outward_normal_base,
                max_target_radial_m=config.max_target_radial_m,
                z_band_fraction=config.z_band_fraction,
                delta_z_m=config.delta_z_m,
                z_separation_gain=config.z_separation_gain,
                pre_approach_distance_m=config.pre_approach_distance_m,
                dexterous_reach=config.dexterous_reach,
                reach_budget=reach_budget,
                tip_ik_fn=tip_ik_fn,
                ik_budget=ik_budget,
                log=log,
            )
        targets = _targets_from_centers(config, centers)
    listed_ids = tuple(target.target_id for target in targets)
    if config.order is OrderPolicy.LISTED:
        order_ids = listed_ids
    elif config.order is OrderPolicy.Z_DESC:
        normal = np.asarray(config.outward_normal_base, dtype=float)
        magnitude = float(np.linalg.norm(normal))
        if magnitude <= 1.0e-12:
            raise ConfigurationError("outward_normal_base must have non-zero magnitude")
        unit = normal / magnitude

        def _top_face_along_normal(target: NumberedTarget) -> float:
            center = np.asarray(target.center_m, dtype=float)
            return float(np.dot(center, unit) + 0.5 * float(target.edge_m))

        ordered = sorted(
            targets,
            key=lambda target: (-_top_face_along_normal(target), int(target.target_id)),
        )
        order_ids = tuple(target.target_id for target in ordered)
        _ = order_seed  # recorded on the episode; unused by z_desc
    else:
        rng = np.random.default_rng(order_seed)
        permutation = rng.permutation(len(listed_ids))
        order_ids = tuple(listed_ids[int(index)] for index in permutation)
    return TargetField(
        targets=targets,
        placement=config.placement,
        order=config.order,
        retain_targets_after_contact=config.retain_targets_after_contact,
        contact_order_ids=order_ids,
    )


@dataclass(frozen=True)
class MultiTargetEpisode:
    episode_index: int
    root_seed: int
    episode_seed: int
    order_seed: int
    field: TargetField
    start_position_rad: tuple[float, ...]
    planner_profile: str
    tip_allow_link_names: tuple[str, ...]
    max_planning_failure_per_target: int
    max_target_failures: int
    max_reconsider_passes: int
    max_failed_episodes: int
    max_consecutive_unplanned_targets: int
    scene_revision_prefix: str
    retain_targets_after_contact: bool
    reach_rejections: tuple[ReachRejectionRecord, ...] = ()


@dataclass(frozen=True)
class SuitePlacementStats:
    """Timing and reach/IK-rejection evidence for one suite placement pass."""

    generation_duration_s: float
    reach_rejections: tuple[ReachRejectionRecord, ...]
    max_reach_rejections: int
    ik_rejections: tuple[IkRejectionRecord, ...] = ()
    max_ik_rejections_per_episode: int = 0
    field_regenerations: int = 0


def resolve_invocation_root_seed(cli_seed: int | None = None) -> int:
    """Resolve a CLI-provided root seed (non-negative integer).

    Callers that omit ``--root-seed`` should use
    ``sample_multi_target_episodes(..., independent_random_episode_seeds=True)``
    instead of this helper so each episode draws its own seed.
    """

    if cli_seed is None:
        raise ConfigurationError("--root-seed must be a non-negative integer")
    if isinstance(cli_seed, bool) or not isinstance(cli_seed, int):
        raise ConfigurationError("--root-seed must be a non-negative integer")
    if cli_seed < 0:
        raise ConfigurationError("--root-seed must be a non-negative integer")
    return int(cli_seed)


def _draw_distinct_episode_seeds(count: int) -> tuple[tuple[int, int], ...]:
    """Draw a distinct (episode_seed, order_seed) pair for each episode."""

    pairs: list[tuple[int, int]] = []
    used: set[int] = set()
    for _ in range(count):
        episode_seed = int(secrets.randbelow(2**31))
        while episode_seed in used:
            episode_seed = int(secrets.randbelow(2**31))
        used.add(episode_seed)
        order_seed = int(secrets.randbelow(2**31))
        while order_seed in used:
            order_seed = int(secrets.randbelow(2**31))
        used.add(order_seed)
        pairs.append((episode_seed, order_seed))
    return tuple(pairs)


def sample_multi_target_episodes(
    config: MultiTargetSuiteConfig,
    *,
    root_seed: int | None = None,
    episode_count: int | None = None,
    independent_random_episode_seeds: bool = False,
    tip_ik_fn: TipIkFn | None = None,
    log: LogFn | None = None,
    placement_stats_out: list[SuitePlacementStats] | None = None,
) -> tuple[MultiTargetEpisode, ...]:
    """Sample multi-target episodes from the suite configuration.

    When ``independent_random_episode_seeds`` is true, each episode draws its
    own fresh ``episode_seed`` / ``order_seed`` (and stores that episode seed as
    ``root_seed``). When false, episodes derive deterministic seeds from
    ``root_seed`` (or YAML ``config.root_seed`` when ``root_seed`` is None).

    Dexterous-reach rejections accumulate suite-wide across episode fields.
    Tip-IK rejections use a **per-episode** budget defaulting to
    ``target_count``. When ``log`` is set, placement accept/reject lines and
    the suite generation duration are emitted. When ``placement_stats_out`` is
    provided, a single :class:`SuitePlacementStats` is appended.
    """

    if config.target_population is TargetPopulation.INCREMENTAL:
        raise ConfigurationError(
            "sample_multi_target_episodes does not support target_population=incremental; "
            "use IncrementalPopulationRunner"
        )
    count = (
        config.episode_count
        if episode_count is None
        else _positive_int(episode_count, "episode_count")
    )
    if independent_random_episode_seeds:
        seed_pairs = _draw_distinct_episode_seeds(count)
        shared_root: int | None = None
    else:
        seed = config.root_seed if root_seed is None else int(root_seed)
        shared_root = seed
        seed_pairs = tuple(
            (seed + 1009 * (index + 1), seed + 9176 * (index + 1)) for index in range(count)
        )
    max_reach = (
        int(config.max_reach_rejections)
        if config.max_reach_rejections is not None
        else int(config.target_count) * int(count)
    )
    max_ik_per_episode = (
        int(config.max_ik_rejections)
        if config.max_ik_rejections is not None
        else int(config.target_count)
    )
    reach_budget = ReachRejectionBudget(max_rejections=max_reach, log=log)
    ik_rejections: list[IkRejectionRecord] = []
    if log is not None:
        log(
            "phase7_4_placement: suite generation start "
            f"episodes={count} targets_per_episode={config.target_count} "
            f"max_reach_rejections={max_reach} "
            f"max_ik_rejections_per_episode={max_ik_per_episode} "
            f"require_tip_ik={config.require_tip_ik} "
            f"delta_z_m={config.delta_z_m} z_band_fraction={config.z_band_fraction}"
        )
    started = time.perf_counter()
    episodes: list[MultiTargetEpisode] = []
    for index, (episode_seed, order_seed) in enumerate(seed_pairs):
        reach_budget.episode_index = index
        ik_budget = IkRejectionBudget(
            max_rejections=max_ik_per_episode, episode_index=index, log=log
        )
        if log is not None:
            log(
                "phase7_4_placement: "
                f"episode={index} begin episode_seed={episode_seed} order_seed={order_seed}"
            )
        # Grid / random / layout suites use episode_seed so each episode gets a
        # distinct field; manual suites ignore placement_seed (fixed centres).
        field = build_target_field(
            config,
            order_seed=order_seed,
            placement_seed=episode_seed,
            reach_budget=reach_budget,
            tip_ik_fn=tip_ik_fn,
            ik_budget=ik_budget,
            log=log,
        )
        ik_rejections.extend(ik_budget.rejections)
        episodes.append(
            MultiTargetEpisode(
                episode_index=index,
                root_seed=episode_seed if shared_root is None else shared_root,
                episode_seed=episode_seed,
                order_seed=order_seed,
                field=field,
                start_position_rad=config.start_joint_position_rad,
                planner_profile=config.planner_profile,
                tip_allow_link_names=config.tip_allow_link_names,
                max_planning_failure_per_target=config.max_planning_failure_per_target,
                max_target_failures=config.max_target_failures,
                max_reconsider_passes=config.max_reconsider_passes,
                max_failed_episodes=config.max_failed_episodes,
                max_consecutive_unplanned_targets=config.max_consecutive_unplanned_targets,
                scene_revision_prefix=config.scene_revision_prefix,
                retain_targets_after_contact=config.retain_targets_after_contact,
                reach_rejections=tuple(reach_budget.rejections),
            )
        )
    duration_s = float(time.perf_counter() - started)
    stats = SuitePlacementStats(
        generation_duration_s=duration_s,
        reach_rejections=tuple(reach_budget.rejections),
        max_reach_rejections=max_reach,
        ik_rejections=tuple(ik_rejections),
        max_ik_rejections_per_episode=max_ik_per_episode,
        field_regenerations=0,
    )
    if placement_stats_out is not None:
        placement_stats_out.append(stats)
    if log is not None:
        log(
            "phase7_4_placement: suite target generation completed "
            f"duration_s={duration_s:.3f} "
            f"reach_rejections={len(stats.reach_rejections)}/{max_reach} "
            f"ik_rejections={len(stats.ik_rejections)} "
            f"(max_per_episode={max_ik_per_episode}) "
            f"episodes={count} targets_per_episode={config.target_count}"
        )
    return tuple(episodes)


def regenerate_episode_field(
    config: MultiTargetSuiteConfig,
    episode: MultiTargetEpisode,
    *,
    tip_ik_fn: TipIkFn | None = None,
    log: LogFn | None = None,
    regen_index: int = 0,
) -> MultiTargetEpisode:
    """Rebuild one episode's target field with fresh seeds after planning abort.

    Used when consecutive unplanned targets exhaust the per-episode defer
    budget: discard the failed field and resample under the same suite
    geometry / tip-IK constraints. Suite-wide ``max_field_regenerations``
    is enforced by the episode runner, not this helper.
    """

    episode_seed = int(secrets.randbelow(2**31))
    order_seed = int(secrets.randbelow(2**31))
    max_reach = (
        int(config.max_reach_rejections)
        if config.max_reach_rejections is not None
        else int(config.target_count)
    )
    max_ik = (
        int(config.max_ik_rejections)
        if config.max_ik_rejections is not None
        else int(config.target_count)
    )
    if log is not None:
        log(
            "phase7_4_placement: field regeneration begin "
            f"episode={episode.episode_index} regen_index={regen_index} "
            f"episode_seed={episode_seed} order_seed={order_seed} "
            f"max_reach_rejections={max_reach} max_ik_rejections={max_ik}"
        )
    reach_budget = ReachRejectionBudget(max_rejections=max_reach, log=log)
    reach_budget.episode_index = int(episode.episode_index)
    ik_budget = IkRejectionBudget(
        max_rejections=max_ik, episode_index=int(episode.episode_index), log=log
    )
    field = build_target_field(
        config,
        order_seed=order_seed,
        placement_seed=episode_seed,
        reach_budget=reach_budget,
        tip_ik_fn=tip_ik_fn,
        ik_budget=ik_budget,
        log=log,
    )
    if log is not None:
        log(
            "phase7_4_placement: field regeneration completed "
            f"episode={episode.episode_index} regen_index={regen_index} "
            f"reach_rejections={reach_budget.count}/{max_reach} "
            f"ik_rejections={ik_budget.count}/{max_ik}"
        )
    return replace(
        episode,
        episode_seed=episode_seed,
        order_seed=order_seed,
        field=field,
        reach_rejections=tuple(reach_budget.rejections),
    )


def serialize_episode(episode: MultiTargetEpisode) -> dict[str, Any]:
    """Serialize every frozen episode field for exact replay."""

    return asdict(episode)


def deserialize_episode(payload: dict[str, Any]) -> MultiTargetEpisode:
    """Rebuild a frozen episode from its JSON-compatible representation."""

    data = dict(payload)
    targets = tuple(NumberedTarget(**item) for item in data["field"]["targets"])
    field_payload = data["field"]
    target_field = TargetField(
        targets=targets,
        placement=PlacementPolicy(field_payload["placement"]),
        order=OrderPolicy(field_payload["order"]),
        retain_targets_after_contact=bool(field_payload["retain_targets_after_contact"]),
        contact_order_ids=tuple(field_payload["contact_order_ids"]),
    )
    rejection_payload = data.get("reach_rejections", ())
    reach_rejections = tuple(
        ReachRejectionRecord(
            episode_index=int(item["episode_index"]),
            center_m=tuple(float(v) for v in item["center_m"]),
            reason=str(item.get("reason", "outside_dexterous_reach")),
        )
        for item in rejection_payload
    )
    return MultiTargetEpisode(
        episode_index=int(data["episode_index"]),
        root_seed=int(data["root_seed"]),
        episode_seed=int(data["episode_seed"]),
        order_seed=int(data["order_seed"]),
        field=target_field,
        start_position_rad=tuple(data["start_position_rad"]),
        planner_profile=str(data["planner_profile"]),
        tip_allow_link_names=tuple(data["tip_allow_link_names"]),
        max_planning_failure_per_target=int(data.get("max_planning_failure_per_target", 3)),
        max_target_failures=int(data.get("max_target_failures", default_max_target_failures())),
        max_reconsider_passes=int(
            data.get("max_reconsider_passes", len(target_field.contact_order_ids))
        ),
        max_failed_episodes=int(data.get("max_failed_episodes", 0)),
        max_consecutive_unplanned_targets=_max_consecutive_unplanned_targets(
            data["max_consecutive_unplanned_targets"]
            if "max_consecutive_unplanned_targets" in data
            else (
                0
                if not target_field.contact_order_ids
                else default_max_consecutive_unplanned_targets(len(target_field.contact_order_ids))
            )
        ),
        scene_revision_prefix=str(data["scene_revision_prefix"]),
        retain_targets_after_contact=bool(data["retain_targets_after_contact"]),
        reach_rejections=reach_rejections,
    )


@dataclass(frozen=True)
class MultiTargetLegResult:
    from_id: str
    to_id: str
    planning_succeeded: bool
    validation_passed: bool
    contact_kind: ContactKind | None
    failure_category: MultiTargetFailureCategory | None
    failure_reason: str | None
    planner_status: str
    planning_duration_s: float | None = None
    motion_duration_s: float | None = None
    time_to_contact_s: float | None = None
    request_id: str | None = None
    scene_revision: str | None = None
    validation_metrics: ValidationMetrics | None = None
    final_joint_position_rad: tuple[float, ...] | None = None
    attempt_index: int = 0


@dataclass(frozen=True)
class MultiTargetEpisodeResult:
    episode: MultiTargetEpisode
    succeeded: bool
    failure_category: MultiTargetFailureCategory | None
    failure_reason: str | None
    planning_failure_count: int
    target_failure_count: int
    failed_target_ids: tuple[str, ...]
    legs: tuple[MultiTargetLegResult, ...]
    contacted_ids: tuple[str, ...]
    removed_ids: tuple[str, ...]
    episode_duration_s: float | None = None
    deferred_target_ids: tuple[str, ...] = ()
    planned_target_ids: tuple[str, ...] = ()

    @property
    def tip_contact_count(self) -> int:
        return len(self.contacted_ids)


@dataclass(frozen=True)
class MultiTargetSuiteSummary:
    root_seed: int
    total_episodes: int
    successes: int
    success_rate: float
    failure_category_counts: dict[str, int]
    total_tip_contacts: int
    total_body_contacts: int
    failed_episodes: int
    total_planning_failures: int
    total_target_failures: int
    planning_duration_s: dict[str, float | None]
    episode_duration_s: dict[str, float | None]


def _distribution(values: list[float]) -> dict[str, float | None]:
    if not values:
        return {"p50": None, "p95": None}
    return {"p50": float(np.percentile(values, 50)), "p95": float(np.percentile(values, 95))}


def aggregate_multi_target_results(
    results: Sequence[MultiTargetEpisodeResult], *, root_seed: int
) -> MultiTargetSuiteSummary:
    if not results:
        raise ConfigurationError("cannot aggregate an empty multi-target suite")
    failures = Counter(
        result.failure_category.value for result in results if result.failure_category is not None
    )
    planning_durations = [
        float(leg.planning_duration_s)
        for result in results
        for leg in result.legs
        if leg.planning_duration_s is not None
    ]
    episode_durations = [
        float(result.episode_duration_s)
        for result in results
        if result.episode_duration_s is not None
    ]
    body_contacts = sum(
        1
        for result in results
        for leg in result.legs
        if leg.contact_kind is ContactKind.PROHIBITED_BODY_CONTACT
    )
    failed_episodes = sum(1 for result in results if not result.succeeded)
    return MultiTargetSuiteSummary(
        root_seed=root_seed,
        total_episodes=len(results),
        successes=sum(result.succeeded for result in results),
        success_rate=sum(result.succeeded for result in results) / len(results),
        failure_category_counts=dict(sorted(failures.items())),
        total_tip_contacts=sum(result.tip_contact_count for result in results),
        total_body_contacts=body_contacts,
        failed_episodes=failed_episodes,
        total_planning_failures=sum(result.planning_failure_count for result in results),
        total_target_failures=sum(result.target_failure_count for result in results),
        planning_duration_s=_distribution(planning_durations),
        episode_duration_s=_distribution(episode_durations),
    )


def suite_acceptance_passed(summary: MultiTargetSuiteSummary, *, max_failed_episodes: int) -> bool:
    """Return whether suite acceptance passes under ``max_failed_episodes``."""

    return summary.failed_episodes <= max_failed_episodes


def format_leg_console_row(
    leg: MultiTargetLegResult, *, episode_index: int, episode_count: int
) -> str:
    failure = None if leg.failure_category is None else leg.failure_category.value
    plan_s = "n/a" if leg.planning_duration_s is None else f"{leg.planning_duration_s:.3f}"
    motion_s = "n/a" if leg.motion_duration_s is None else f"{leg.motion_duration_s:.3f}"
    contact_s = "n/a" if leg.time_to_contact_s is None else f"{leg.time_to_contact_s:.3f}"
    contact = None if leg.contact_kind is None else leg.contact_kind.value
    return (
        f"[ep {episode_index + 1}/{episode_count}] {leg.from_id}->{leg.to_id} "
        f"plan={leg.planning_succeeded} valid={leg.validation_passed} "
        f"contact={contact} plan_s={plan_s} motion_s={motion_s} "
        f"ttc_s={contact_s} attempt={leg.attempt_index} failure={failure}"
    )


def format_episode_console_row(
    result: MultiTargetEpisodeResult, *, count: int | None = None
) -> str:
    prefix = (
        f"{result.episode.episode_index + 1}/{count}"
        if count
        else str(result.episode.episode_index + 1)
    )
    failure = None if result.failure_category is None else result.failure_category.value
    duration = "n/a" if result.episode_duration_s is None else f"{result.episode_duration_s:.3f}"
    return (
        f"[{prefix}] targets={len(result.episode.field.targets)} "
        f"contacted={len(result.contacted_ids)} removed={len(result.removed_ids)} "
        f"plan_fails={result.planning_failure_count} "
        f"target_fails={result.target_failure_count} succeeded={result.succeeded} "
        f"episode_s={duration} failure={failure}"
    )


def format_suite_summary(summary: MultiTargetSuiteSummary) -> str:
    return (
        f"Phase 7.2: {summary.successes}/{summary.total_episodes} "
        f"({summary.success_rate:.1%}) failures={summary.failure_category_counts} "
        f"tip={summary.total_tip_contacts} body={summary.total_body_contacts} "
        f"failed_episodes={summary.failed_episodes} "
        f"plan_fails={summary.total_planning_failures} "
        f"target_fails={summary.total_target_failures} "
        f"plan_p50/p95={summary.planning_duration_s['p50']}/"
        f"{summary.planning_duration_s['p95']}"
    )


def write_multi_target_suite_report(
    summary: MultiTargetSuiteSummary,
    results: Sequence[MultiTargetEpisodeResult],
    output_dir: Path | str,
) -> Path:
    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    path = destination / f"phase7_2_multi_target_seed_{summary.root_seed}.json"
    path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "summary": asdict(summary),
                "results": [asdict(result) for result in results],
            },
            indent=2,
            default=lambda value: value.value if isinstance(value, Enum) else value,
        ),
        encoding="utf-8",
    )
    return path


@dataclass
class _EpisodeState:
    removed_ids: list[str] = field(default_factory=list)
    contacted_ids: list[str] = field(default_factory=list)
    failed_target_ids: list[str] = field(default_factory=list)
    deferred_target_ids: list[str] = field(default_factory=list)
    planned_target_ids: list[str] = field(default_factory=list)
    current_count_planning_failure_per_target: int = 0
    planning_failure_count: int = 0
    target_failure_count: int = 0
    consecutive_unplanned_targets: int = 0
    reconsider_pass: int = 0
    current_joints: tuple[float, ...] = ()
    from_id: str = "start"


class OptimisticTipContactDetector:
    """Planning-process detector that assumes validated legs produce tip contact."""

    def __init__(self, target_id: str) -> None:
        self._target_id = target_id

    def classify(self) -> ContactEvent:
        return ContactEvent(ContactKind.ALLOWED_TIP_CONTACT, target_id=self._target_id)


class MultiTargetEpisodeRunner:
    """Orchestrate per-target plan retries and episode clearance loops.

    Args:
        planner_factory: Builds a planner for ``(seed, scene_model, disable_links)``.
        validator: Independently validates a plan; third arg is non-contact
            clearance cubes; fourth arg is the active contact cube.
        contact_detector_factory: Builds a ``ContactDetector`` for ``(episode, to_id)``.
        motion_executor: Optional playback hook returning motion duration seconds.
        warn_planning_duration_s: Advisory planning-latency threshold (log only).
    """

    def __init__(
        self,
        *,
        planner_factory: Callable[[int, dict[str, Any], tuple[str, ...]], Any],
        validator: Callable[
            [NominalPlan, PlanningRequest, tuple[CubeGeometry, ...], CubeGeometry],
            ValidatedPlan,
        ],
        contact_detector_factory: Callable[[MultiTargetEpisode, str], ContactDetector],
        motion_executor: Callable[[MultiTargetEpisode, MultiTargetLegResult], float] | None = None,
        plan_sink: Callable[[NominalPlan], None] | None = None,
        warn_planning_duration_s: float | None = None,
        console_log: Callable[[str], None] | None = None,
    ) -> None:
        self._planner_factory = planner_factory
        self._validator = validator
        self._contact_detector_factory = contact_detector_factory
        self._motion_executor = motion_executor
        self._plan_sink = plan_sink
        self._warn_planning_duration_s = warn_planning_duration_s
        self._console_log = print if console_log is None else console_log

    def run(
        self,
        episodes: Sequence[MultiTargetEpisode],
        *,
        regenerate_field: (
            Callable[[MultiTargetEpisode, int], MultiTargetEpisode | None] | None
        ) = None,
        max_field_regenerations: int = 0,
    ) -> tuple[MultiTargetEpisodeResult, ...]:
        """Run episodes in order.

        On ``max_consecutive_unplanned_targets`` abort or ``targets_unplanned``,
        optionally regenerate that episode's field (suite-wide budget
        ``max_field_regenerations``) and retry. Exhausting the regen budget
        after consecutive-unplanned aborts remaining suite planning; after
        ``targets_unplanned`` the failed episode is recorded and the suite
        continues.
        """

        if max_field_regenerations < 0:
            raise ConfigurationError("max_field_regenerations must be non-negative")
        regen_categories = (
            MultiTargetFailureCategory.MAX_CONSECUTIVE_UNPLANNED_TARGETS_EXCEEDED,
            MultiTargetFailureCategory.TARGETS_UNPLANNED,
        )
        working = list(episodes)
        results: list[MultiTargetEpisodeResult] = []
        abort_reason: str | None = None
        field_regens_used = 0
        index = 0
        while index < len(working):
            episode = working[index]
            if abort_reason is not None:
                results.append(
                    MultiTargetEpisodeResult(
                        episode=episode,
                        succeeded=False,
                        failure_category=(
                            MultiTargetFailureCategory.MAX_CONSECUTIVE_UNPLANNED_TARGETS_EXCEEDED
                        ),
                        failure_reason=abort_reason,
                        planning_failure_count=0,
                        target_failure_count=0,
                        failed_target_ids=(),
                        legs=(),
                        contacted_ids=(),
                        removed_ids=(),
                        episode_duration_s=0.0,
                        deferred_target_ids=(),
                        planned_target_ids=(),
                    )
                )
                index += 1
                continue
            result = self._run_episode(episode, len(working))
            if (
                result.failure_category in regen_categories
                and regenerate_field is not None
                and field_regens_used < max_field_regenerations
            ):
                replacement = regenerate_field(episode, field_regens_used)
                field_regens_used += 1
                if replacement is not None:
                    self._console_log(
                        "phase7_2_plan: regenerating target field after "
                        f"{result.failure_category.value} "
                        f"(episode={episode.episode_index} "
                        f"regen={field_regens_used}/{max_field_regenerations})"
                    )
                    working[index] = replacement
                    continue
            results.append(result)
            if (
                result.failure_category
                is MultiTargetFailureCategory.MAX_CONSECUTIVE_UNPLANNED_TARGETS_EXCEEDED
            ):
                abort_reason = (
                    "suite planning aborted: "
                    f"{result.failure_reason or 'max_consecutive_unplanned_targets exceeded'}"
                )
                self._console_log(f"phase7_2_plan: {abort_reason}")
            index += 1
        return tuple(results)

    def _run_episode(
        self, episode: MultiTargetEpisode, episode_count: int
    ) -> MultiTargetEpisodeResult:
        started = time.perf_counter()
        state = _EpisodeState(current_joints=episode.start_position_rad)
        legs: list[MultiTargetLegResult] = []
        order = list(episode.field.contact_order_ids)
        to_do = list(order)
        deferred: list[str] = []
        progress_in_pass = False

        while to_do or deferred:
            if not to_do:
                # Reconsider only helps after tip-contact progress shrinks the
                # obstacle set. A pass that deferred everyone with no tip
                # contact (including the first pass) must fail closed — do not
                # replan the same full field for max_reconsider_passes rounds.
                if deferred and not progress_in_pass:
                    state.failed_target_ids = list(deferred)
                    state.deferred_target_ids = list(deferred)
                    reason = (
                        "unplanned targets after pass with no tip-contact progress: "
                        f"{sorted(deferred)}"
                    )
                    return self._fail_episode(
                        episode,
                        legs,
                        state,
                        started,
                        MultiTargetFailureCategory.TARGETS_UNPLANNED,
                        reason,
                    )
                if state.reconsider_pass >= episode.max_reconsider_passes:
                    state.failed_target_ids = list(deferred)
                    state.deferred_target_ids = list(deferred)
                    reason = (
                        f"reconsider passes {state.reconsider_pass} reached "
                        f"max_reconsider_passes={episode.max_reconsider_passes}; "
                        f"unplanned={sorted(deferred)}"
                    )
                    return self._fail_episode(
                        episode,
                        legs,
                        state,
                        started,
                        MultiTargetFailureCategory.MAX_RECONSIDER_PASSES_EXCEEDED,
                        reason,
                    )
                deferred_set = set(deferred)
                to_do = [target_id for target_id in order if target_id in deferred_set]
                deferred = []
                state.reconsider_pass += 1
                progress_in_pass = False
                continue

            to_id = to_do[0]
            leg = self._run_leg(episode, state, to_id)
            legs.append(leg)
            self._console_log(
                format_leg_console_row(
                    leg, episode_index=episode.episode_index, episode_count=episode_count
                )
            )
            if leg.failure_category is MultiTargetFailureCategory.BODY_CONTACT:
                return self._fail_episode(
                    episode, legs, state, started, leg.failure_category, leg.failure_reason
                )
            if (
                leg.failure_category
                is MultiTargetFailureCategory.MAX_PLANNING_FAILURE_PER_TARGET_EXCEEDED
            ):
                # Defer for a later pass with a reduced obstacle set.
                deferred.append(to_id)
                state.deferred_target_ids = list(deferred)
                state.target_failure_count = len(deferred)
                to_do.pop(0)
                state.current_count_planning_failure_per_target = 0
                # 0 = no maximum: skip consecutive-unplanned tracking/abort.
                if episode.max_consecutive_unplanned_targets > 0:
                    state.consecutive_unplanned_targets += 1
                    if (
                        state.consecutive_unplanned_targets
                        >= episode.max_consecutive_unplanned_targets
                    ):
                        state.failed_target_ids = list(deferred)
                        reason = (
                            "consecutive unplanned targets "
                            f"{state.consecutive_unplanned_targets} reached "
                            f"max_consecutive_unplanned_targets="
                            f"{episode.max_consecutive_unplanned_targets}; "
                            f"deferred={sorted(deferred)}"
                        )
                        return self._fail_episode(
                            episode,
                            legs,
                            state,
                            started,
                            MultiTargetFailureCategory.MAX_CONSECUTIVE_UNPLANNED_TARGETS_EXCEEDED,
                            reason,
                        )
                continue
            if not leg.planning_succeeded or not leg.validation_passed:
                # Same-target retry: leave to_do[0] unchanged.
                continue
            if leg.contact_kind is ContactKind.ALLOWED_TIP_CONTACT:
                if to_id not in state.planned_target_ids:
                    state.planned_target_ids.append(to_id)
                state.contacted_ids.append(to_id)
                if not episode.retain_targets_after_contact:
                    state.removed_ids.append(to_id)
                if leg.final_joint_position_rad is not None:
                    state.current_joints = leg.final_joint_position_rad
                state.from_id = to_id
                to_do.pop(0)
                state.current_count_planning_failure_per_target = 0
                state.consecutive_unplanned_targets = 0
                progress_in_pass = True
                continue
            reason = f"tip contact missed on {state.from_id}->{to_id}"
            legs[-1] = replace(
                leg,
                failure_category=MultiTargetFailureCategory.TIP_CONTACT_MISSED,
                failure_reason=reason,
                attempt_index=state.current_count_planning_failure_per_target,
            )
            return self._fail_episode(
                episode,
                legs,
                state,
                started,
                MultiTargetFailureCategory.TIP_CONTACT_MISSED,
                reason,
            )

        required_ids = set(episode.field.contact_order_ids)
        planned = set(state.planned_target_ids)
        contacted = set(state.contacted_ids)
        if planned != required_ids or contacted != required_ids:
            missing = sorted(required_ids - planned)
            state.failed_target_ids = missing
            reason = f"unplanned targets remain: {missing}"
            return self._fail_episode(
                episode,
                legs,
                state,
                started,
                MultiTargetFailureCategory.TARGETS_UNPLANNED,
                reason,
            )
        return MultiTargetEpisodeResult(
            episode=episode,
            succeeded=True,
            failure_category=None,
            failure_reason=None,
            planning_failure_count=state.planning_failure_count,
            target_failure_count=0,
            failed_target_ids=(),
            legs=tuple(legs),
            contacted_ids=tuple(state.contacted_ids),
            removed_ids=tuple(state.removed_ids),
            episode_duration_s=time.perf_counter() - started,
            deferred_target_ids=(),
            planned_target_ids=tuple(state.planned_target_ids),
        )

    def _record_planning_failure(self, state: _EpisodeState) -> None:
        state.current_count_planning_failure_per_target += 1
        state.planning_failure_count += 1

    def _fail_episode(
        self,
        episode: MultiTargetEpisode,
        legs: Sequence[MultiTargetLegResult],
        state: _EpisodeState,
        started: float,
        category: MultiTargetFailureCategory,
        reason: str | None,
    ) -> MultiTargetEpisodeResult:
        return MultiTargetEpisodeResult(
            episode=episode,
            succeeded=False,
            failure_category=category,
            failure_reason=reason,
            planning_failure_count=state.planning_failure_count,
            target_failure_count=state.target_failure_count,
            failed_target_ids=tuple(state.failed_target_ids),
            legs=tuple(legs),
            contacted_ids=tuple(state.contacted_ids),
            removed_ids=tuple(state.removed_ids),
            episode_duration_s=time.perf_counter() - started,
            deferred_target_ids=tuple(state.deferred_target_ids),
            planned_target_ids=tuple(state.planned_target_ids),
        )

    def _planning_failure_leg(
        self,
        *,
        episode: MultiTargetEpisode,
        state: _EpisodeState,
        from_id: str,
        to_id: str,
        category: MultiTargetFailureCategory,
        failure_reason: str | None,
        planner_status: str,
        planning_duration_s: float,
        request_id: str,
        scene_revision: str,
        planning_succeeded: bool,
        validation_metrics: ValidationMetrics | None = None,
    ) -> MultiTargetLegResult:
        self._record_planning_failure(state)
        final_category = category
        # Defer when the failure count *reaches* the budget (not budget+1).
        if (
            state.current_count_planning_failure_per_target
            >= episode.max_planning_failure_per_target
        ):
            final_category = MultiTargetFailureCategory.MAX_PLANNING_FAILURE_PER_TARGET_EXCEEDED
        return MultiTargetLegResult(
            from_id=from_id,
            to_id=to_id,
            planning_succeeded=planning_succeeded,
            validation_passed=False,
            contact_kind=None,
            failure_category=final_category,
            failure_reason=failure_reason,
            planner_status=planner_status,
            planning_duration_s=planning_duration_s,
            request_id=request_id,
            scene_revision=scene_revision,
            validation_metrics=validation_metrics,
            attempt_index=state.current_count_planning_failure_per_target,
        )

    def _run_leg(
        self, episode: MultiTargetEpisode, state: _EpisodeState, to_id: str
    ) -> MultiTargetLegResult:
        from_id = state.from_id
        target = episode.field.target_by_id(to_id)
        geometries = episode.field.active_geometries(removed_ids=state.removed_ids)
        # Shared Option B / Phase 7.2 invariant: omit the active contact cube so
        # the tip can occupy the face; other remaining targets stay as obstacles.
        planning_geometries = leg_world_geometries(
            geometries, active_contact_name=target.cube_geometry.name
        )
        scene_model = cubes_to_curobo_scene_dict(planning_geometries)
        scene_revision = (
            f"{episode.scene_revision_prefix}-"
            f"{multi_cube_scene_revision(geometries) if geometries else 'empty'}"
        )
        attempt = state.current_count_planning_failure_per_target
        request_id = f"ep{episode.episode_index:03d}_{from_id}_to_{to_id}_attempt{attempt}"
        request = PlanningRequest(
            current_joint_state=NamedJointState.create(JOINT_NAMES, state.current_joints),
            surface_target=target.to_surface_target(),
            scene_revision=scene_revision,
            planner_profile=episode.planner_profile,
            random_seed=episode.episode_seed + attempt,
            request_id=request_id,
            disable_collision_links=(),
        )
        # Independent world clearance uses the same omit-active invariant.
        clearance_geometries = planning_geometries
        plan_started = time.perf_counter()
        try:
            planner = self._planner_factory(
                request.random_seed, scene_model, episode.tip_allow_link_names
            )
            outcome: PlanningOutcome = planner.plan(request)
        except (RuntimeError, ValueError, ConfigurationError) as exc:
            return self._planning_failure_leg(
                episode=episode,
                state=state,
                from_id=from_id,
                to_id=to_id,
                category=MultiTargetFailureCategory.PLAN_FAILED,
                failure_reason=str(exc),
                planner_status="exception",
                planning_duration_s=time.perf_counter() - plan_started,
                request_id=request_id,
                scene_revision=scene_revision,
                planning_succeeded=False,
            )
        planning_duration_s = time.perf_counter() - plan_started
        if (
            self._warn_planning_duration_s is not None
            and planning_duration_s > self._warn_planning_duration_s
        ):
            self._console_log(
                f"WARN planning_duration_s={planning_duration_s:.3f} exceeded advisory "
                f"{self._warn_planning_duration_s:.3f} on {from_id}->{to_id} "
                "(sim host evidence only; not an Orin AGX budget)"
            )
        if not outcome.succeeded or outcome.plan is None:
            failure = outcome.failure
            return self._planning_failure_leg(
                episode=episode,
                state=state,
                from_id=from_id,
                to_id=to_id,
                category=MultiTargetFailureCategory.PLAN_FAILED,
                failure_reason=None if failure is None else failure.reason,
                planner_status="" if failure is None else failure.planner_status,
                planning_duration_s=planning_duration_s,
                request_id=request_id,
                scene_revision=scene_revision,
                planning_succeeded=False,
            )
        validated = self._validator(
            outcome.plan, request, clearance_geometries, target.cube_geometry
        )
        if not validated.report.valid:
            return self._planning_failure_leg(
                episode=episode,
                state=state,
                from_id=from_id,
                to_id=to_id,
                category=MultiTargetFailureCategory.VALIDATION_FAILED,
                failure_reason="; ".join(
                    violation.reason for violation in validated.report.violations
                ),
                planner_status=outcome.plan.planner_status,
                planning_duration_s=planning_duration_s,
                request_id=request_id,
                scene_revision=scene_revision,
                planning_succeeded=True,
                validation_metrics=validated.report.metrics,
            )
        if self._plan_sink is not None:
            self._plan_sink(outcome.plan)
        final = tuple(float(item) for item in outcome.plan.combined_trajectory.position_rad[-1])
        leg = MultiTargetLegResult(
            from_id=from_id,
            to_id=to_id,
            planning_succeeded=True,
            validation_passed=True,
            contact_kind=None,
            failure_category=None,
            failure_reason=None,
            planner_status=outcome.plan.planner_status,
            planning_duration_s=planning_duration_s,
            request_id=request_id,
            scene_revision=scene_revision,
            validation_metrics=validated.report.metrics,
            final_joint_position_rad=final,
            attempt_index=state.current_count_planning_failure_per_target,
        )
        motion_duration_s = 0.0
        if self._motion_executor is not None:
            motion_duration_s = float(self._motion_executor(episode, leg))
        detector = self._contact_detector_factory(episode, to_id)
        contact = detector.classify()
        time_to_contact_s = None
        if contact.kind is ContactKind.ALLOWED_TIP_CONTACT:
            time_to_contact_s = planning_duration_s + motion_duration_s
        failure_category = None
        failure_reason = None
        if contact.kind is ContactKind.PROHIBITED_BODY_CONTACT:
            failure_category = MultiTargetFailureCategory.BODY_CONTACT
            failure_reason = f"body contact on {from_id}->{to_id}"
        return replace(
            leg,
            contact_kind=contact.kind,
            motion_duration_s=motion_duration_s,
            time_to_contact_s=time_to_contact_s,
            failure_category=failure_category,
            failure_reason=failure_reason,
        )
