"""Phase 7.3/7.4 target-block placement: random, layouts, Z band, ROM retries."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Any, Sequence

import numpy as np

from mycobot_curobo.errors import ConfigurationError

# Default generated-centre Z band width = this fraction of arm_z_motion_range_m.
DEFAULT_Z_BAND_FRACTION = 0.5
# Back-compat alias used by Phase 7.2 tests and docs.
GRID_Z_VARIABILITY_FRACTION = DEFAULT_Z_BAND_FRACTION
# Per-target substitute attempts when a centre falls outside arm reach.
MAX_ROM_SUBSTITUTE_RETRIES = 3


def ee_clearance_min_center_separation_m(
    edge_m: float,
    flange_diameter_assumption_m: float,
    ee_approach_clearance_m: float | None = None,
) -> float:
    """Approach-plane centre floor: edge + flange + approach clearance."""

    edge = float(edge_m)
    flange = float(flange_diameter_assumption_m)
    clearance = flange if ee_approach_clearance_m is None else float(ee_approach_clearance_m)
    if not math.isfinite(edge) or edge <= 0.0:
        raise ConfigurationError("target_edge_m must be positive finite")
    if not math.isfinite(flange) or flange <= 0.0:
        raise ConfigurationError("flange_diameter_assumption_m must be positive finite")
    if not math.isfinite(clearance) or clearance < 0.0:
        raise ConfigurationError("ee_approach_clearance_m must be finite and >= 0")
    return edge + flange + clearance


def approach_plane_separation_m(
    first_m: Sequence[float],
    second_m: Sequence[float],
    outward_normal_base: Sequence[float],
) -> float:
    """Pairwise centre distance in the plane perpendicular to ``outward_normal``."""

    normal = np.asarray(outward_normal_base, dtype=float).reshape(3)
    norm = float(np.linalg.norm(normal))
    if not math.isfinite(norm) or norm <= 1.0e-12:
        raise ConfigurationError("outward_normal_base must be a non-zero finite vector")
    unit = normal / norm
    first = np.asarray(first_m, dtype=float).reshape(3)
    second = np.asarray(second_m, dtype=float).reshape(3)
    delta = first - second
    planar = delta - float(np.dot(delta, unit)) * unit
    return float(np.linalg.norm(planar))


def top_face_delta_m(
    first_m: Sequence[float],
    second_m: Sequence[float],
    *,
    edge_m: float,
    outward_normal_base: Sequence[float],
) -> float:
    """Absolute top-face separation along ``outward_normal_base`` (metres)."""

    if not math.isfinite(edge_m) or edge_m <= 0.0:
        raise ConfigurationError("edge_m must be positive finite")
    normal = np.asarray(outward_normal_base, dtype=float).reshape(3)
    norm = float(np.linalg.norm(normal))
    if not math.isfinite(norm) or norm <= 1.0e-12:
        raise ConfigurationError("outward_normal_base must be a non-zero finite vector")
    unit = normal / norm
    first = np.asarray(first_m, dtype=float).reshape(3)
    second = np.asarray(second_m, dtype=float).reshape(3)
    half = 0.5 * float(edge_m)
    first_top = first + half * unit
    second_top = second + half * unit
    return abs(float(np.dot(first_top - second_top, unit)))


def z_aware_required_separation_m(
    first_m: Sequence[float],
    second_m: Sequence[float],
    *,
    min_center_separation_m: float,
    edge_m: float,
    outward_normal_base: Sequence[float],
    z_separation_gain: float,
    pre_approach_distance_m: float,
) -> float:
    """Pairwise approach-plane floor including the Phase 7.4 Z-aware term."""

    if not math.isfinite(min_center_separation_m) or min_center_separation_m <= 0.0:
        raise ConfigurationError("min_center_separation_m must be positive finite")
    if not math.isfinite(z_separation_gain) or z_separation_gain < 1.0:
        raise ConfigurationError("z_separation_gain must be finite and >= 1.0")
    if not math.isfinite(pre_approach_distance_m) or pre_approach_distance_m < 0.0:
        raise ConfigurationError("pre_approach_distance_m must be finite and >= 0")
    delta_z = top_face_delta_m(
        first_m,
        second_m,
        edge_m=edge_m,
        outward_normal_base=outward_normal_base,
    )
    return float(
        min_center_separation_m + z_separation_gain * min(delta_z, float(pre_approach_distance_m))
    )


def center_violates_rim(
    center_m: Sequence[float],
    *,
    edge_m: float,
    max_target_radial_m: float | None,
) -> bool:
    """True when cube extent exceeds the optional radial working envelope."""

    if max_target_radial_m is None:
        return False
    radial_limit = float(max_target_radial_m)
    if not math.isfinite(radial_limit) or radial_limit <= 0.0:
        raise ConfigurationError("max_target_radial_m must be positive finite when set")
    point = np.asarray(center_m, dtype=float).reshape(3)
    extent = math.hypot(float(point[0]), float(point[1])) + 0.5 * float(edge_m)
    return extent > radial_limit + 1.0e-12


class LayoutName(str, Enum):
    ROWS = "rows"
    ARC = "arc"


@dataclass(frozen=True)
class KeepOutAabb:
    """Axis-aligned keep-out box in ``g_base`` (metres)."""

    minimum_m: tuple[float, float, float]
    maximum_m: tuple[float, float, float]

    def __post_init__(self) -> None:
        lo = np.asarray(self.minimum_m, dtype=float)
        hi = np.asarray(self.maximum_m, dtype=float)
        if (
            lo.shape != (3,)
            or hi.shape != (3,)
            or not np.all(np.isfinite(lo))
            or not np.all(np.isfinite(hi))
        ):
            raise ConfigurationError("keep_out bounds must be finite length-3")
        if np.any(lo >= hi):
            raise ConfigurationError("keep_out maximum_m must exceed minimum_m")
        object.__setattr__(self, "minimum_m", tuple(float(x) for x in lo))
        object.__setattr__(self, "maximum_m", tuple(float(x) for x in hi))


@dataclass(frozen=True)
class LayoutSpec:
    """Named parameterized layout (rows or arc)."""

    name: LayoutName
    rows: int | None = None
    columns: int | None = None
    radius_m: float | None = None
    span_rad: float | None = None
    center_xy_m: tuple[float, float] | None = None
    z_m: float | None = None
    start_angle_rad: float | None = None


def _tuple3(value: Any, label: str) -> tuple[float, float, float]:
    array = np.asarray(value, dtype=float)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ConfigurationError(f"{label} must contain three finite values")
    return tuple(float(item) for item in array)


def parse_keep_outs(raw: Any) -> tuple[KeepOutAabb, ...]:
    """Parse optional ``keep_outs`` YAML list."""

    if raw is None:
        return ()
    if not isinstance(raw, list):
        raise ConfigurationError("keep_outs must be a list when provided")
    keep_outs: list[KeepOutAabb] = []
    for index, item in enumerate(raw):
        if not isinstance(item, dict):
            raise ConfigurationError(f"keep_outs[{index}] must be a mapping")
        keep_outs.append(
            KeepOutAabb(
                minimum_m=_tuple3(item["minimum_m"], f"keep_outs[{index}].minimum_m"),
                maximum_m=_tuple3(item["maximum_m"], f"keep_outs[{index}].maximum_m"),
            )
        )
    return tuple(keep_outs)


def parse_layout_spec(raw: Any) -> LayoutSpec | None:
    """Parse optional ``layout`` YAML mapping."""

    if raw is None:
        return None
    if not isinstance(raw, dict):
        raise ConfigurationError("layout must be a mapping when provided")
    try:
        name = LayoutName(str(raw["name"]))
    except (KeyError, ValueError) as exc:
        raise ConfigurationError("layout.name must be 'rows' or 'arc'") from exc
    if name is LayoutName.ROWS:
        rows = int(raw["rows"])
        columns = int(raw["columns"])
        if rows < 1 or columns < 1:
            raise ConfigurationError("layout rows/columns must be positive")
        return LayoutSpec(name=name, rows=rows, columns=columns)
    radius = float(raw["radius_m"])
    span = float(raw["span_rad"])
    if not math.isfinite(radius) or radius <= 0.0:
        raise ConfigurationError("layout.radius_m must be positive finite")
    if not math.isfinite(span) or span <= 0.0:
        raise ConfigurationError("layout.span_rad must be positive finite")
    center_xy = np.asarray(raw["center_xy_m"], dtype=float)
    if center_xy.shape != (2,) or not np.all(np.isfinite(center_xy)):
        raise ConfigurationError("layout.center_xy_m must be two finite values")
    z_raw = raw.get("z_m")
    z_m = None if z_raw is None else float(z_raw)
    if z_m is not None and not math.isfinite(z_m):
        raise ConfigurationError("layout.z_m must be finite when set")
    start_raw = raw.get("start_angle_rad")
    start = None if start_raw is None else float(start_raw)
    if start is not None and not math.isfinite(start):
        raise ConfigurationError("layout.start_angle_rad must be finite when set")
    return LayoutSpec(
        name=name,
        radius_m=radius,
        span_rad=span,
        center_xy_m=(float(center_xy[0]), float(center_xy[1])),
        z_m=z_m,
        start_angle_rad=start,
    )


def resolve_z_band_half_m(
    *,
    arm_z_motion_range_m: float,
    z_band_fraction: float = DEFAULT_Z_BAND_FRACTION,
    delta_z_m: float | None = None,
) -> float:
    """Half-width of the generated Z band about field mid-Z (metres).

    ``delta_z_m``, when set, is the **full** band width and is **not** clamped.
    Otherwise band width is ``z_band_fraction * arm_z_motion_range_m``; the
    fraction must be positive finite and is also **not** upper-clamped.
    """

    if not math.isfinite(arm_z_motion_range_m) or arm_z_motion_range_m <= 0.0:
        raise ConfigurationError("arm_z_motion_range_m must be positive and finite")
    if delta_z_m is not None:
        width = float(delta_z_m)
        if not math.isfinite(width) or width <= 0.0:
            raise ConfigurationError("delta_z_m must be positive finite when set")
        return 0.5 * width
    fraction = float(z_band_fraction)
    if not math.isfinite(fraction) or fraction <= 0.0:
        raise ConfigurationError("z_band_fraction must be positive finite")
    return 0.5 * fraction * arm_z_motion_range_m


def z_band_bounds(
    field_minimum_m: Sequence[float],
    field_maximum_m: Sequence[float],
    *,
    arm_z_motion_range_m: float,
    z_band_fraction: float = DEFAULT_Z_BAND_FRACTION,
    delta_z_m: float | None = None,
) -> tuple[float, float, float]:
    """Return ``(mid_z, z_lo, z_hi)`` for the generated vertical band.

    The band is **not** clipped to ``field_aabb`` Z; out-of-reach centres are
    rejected by arm-reach validation and substitute retries instead.
    """

    lo = _tuple3(field_minimum_m, "field_minimum_m")
    hi = _tuple3(field_maximum_m, "field_maximum_m")
    mid_z = 0.5 * (lo[2] + hi[2])
    half_band = resolve_z_band_half_m(
        arm_z_motion_range_m=arm_z_motion_range_m,
        z_band_fraction=z_band_fraction,
        delta_z_m=delta_z_m,
    )
    return mid_z, mid_z - half_band, mid_z + half_band


def arm_reach_z_bounds(
    field_minimum_m: Sequence[float],
    field_maximum_m: Sequence[float],
    *,
    arm_z_motion_range_m: float,
) -> tuple[float, float, float]:
    """Return ``(mid_z, z_lo, z_hi)`` for the full addressable arm Z envelope."""

    if not math.isfinite(arm_z_motion_range_m) or arm_z_motion_range_m <= 0.0:
        raise ConfigurationError("arm_z_motion_range_m must be positive and finite")
    lo = _tuple3(field_minimum_m, "field_minimum_m")
    hi = _tuple3(field_maximum_m, "field_maximum_m")
    mid_z = 0.5 * (lo[2] + hi[2])
    half = 0.5 * float(arm_z_motion_range_m)
    return mid_z, mid_z - half, mid_z + half


def center_outside_arm_reach(
    center_m: Sequence[float],
    *,
    edge_m: float,
    field_minimum_m: Sequence[float],
    field_maximum_m: Sequence[float],
    arm_z_motion_range_m: float,
    max_target_radial_m: float | None = None,
) -> bool:
    """True when the centre is outside the declared tip-reach envelope."""

    point = _tuple3(center_m, "center_m")
    _, z_lo, z_hi = arm_reach_z_bounds(
        field_minimum_m,
        field_maximum_m,
        arm_z_motion_range_m=arm_z_motion_range_m,
    )
    if point[2] < z_lo - 1.0e-12 or point[2] > z_hi + 1.0e-12:
        return True
    return center_violates_rim(point, edge_m=edge_m, max_target_radial_m=max_target_radial_m)


def cube_intersects_keep_out(
    center_m: Sequence[float],
    edge_m: float,
    keep_out: KeepOutAabb,
) -> bool:
    """True when an axis-aligned cube at ``center_m`` intersects ``keep_out``."""

    if not math.isfinite(edge_m) or edge_m <= 0.0:
        raise ConfigurationError("edge_m must be positive finite")
    center = _tuple3(center_m, "center_m")
    half = 0.5 * edge_m
    for axis in range(3):
        cube_lo = center[axis] - half
        cube_hi = center[axis] + half
        if cube_hi <= keep_out.minimum_m[axis] or cube_lo >= keep_out.maximum_m[axis]:
            return False
    return True


def center_violates_keep_outs(
    center_m: Sequence[float],
    edge_m: float,
    keep_outs: Sequence[KeepOutAabb],
) -> bool:
    return any(cube_intersects_keep_out(center_m, edge_m, keep_out) for keep_out in keep_outs)


def validate_centers_separation(
    centers: Sequence[Sequence[float]],
    *,
    min_center_separation_m: float,
    edge_m: float,
    keep_outs: Sequence[KeepOutAabb] = (),
    outward_normal_base: Sequence[float] = (0.0, 0.0, 1.0),
    max_target_radial_m: float | None = None,
    z_separation_gain: float = 1.0,
    pre_approach_distance_m: float | None = None,
    field_minimum_m: Sequence[float] | None = None,
    field_maximum_m: Sequence[float] | None = None,
    arm_z_motion_range_m: float | None = None,
    require_arm_reach: bool = False,
) -> None:
    """Fail closed on approach-plane spacing, keep-outs, rim, or arm reach."""

    if not math.isfinite(min_center_separation_m) or min_center_separation_m <= 0.0:
        raise ConfigurationError("min_center_separation_m must be positive finite")
    points = [np.asarray(center, dtype=float).reshape(3) for center in centers]
    for point in points:
        if not np.all(np.isfinite(point)):
            raise ConfigurationError("target centres must be finite")
        if center_violates_keep_outs(point, edge_m, keep_outs):
            raise ConfigurationError("target centre intersects a keep_out AABB")
        if center_violates_rim(point, edge_m=edge_m, max_target_radial_m=max_target_radial_m):
            raise ConfigurationError(
                "target centre violates max_target_radial_m rim guard "
                f"(limit={max_target_radial_m})"
            )
        if require_arm_reach:
            if field_minimum_m is None or field_maximum_m is None or arm_z_motion_range_m is None:
                raise ConfigurationError(
                    "arm-reach validation requires field bounds and arm_z_motion_range_m"
                )
            if center_outside_arm_reach(
                point,
                edge_m=edge_m,
                field_minimum_m=field_minimum_m,
                field_maximum_m=field_maximum_m,
                arm_z_motion_range_m=arm_z_motion_range_m,
                max_target_radial_m=max_target_radial_m,
            ):
                raise ConfigurationError(
                    "target centre is outside the arm reach envelope "
                    f"(arm_z_motion_range_m={arm_z_motion_range_m})"
                )
    for index, first in enumerate(points):
        for second in points[index + 1 :]:
            if pre_approach_distance_m is None:
                required = min_center_separation_m
            else:
                required = z_aware_required_separation_m(
                    first,
                    second,
                    min_center_separation_m=min_center_separation_m,
                    edge_m=edge_m,
                    outward_normal_base=outward_normal_base,
                    z_separation_gain=z_separation_gain,
                    pre_approach_distance_m=pre_approach_distance_m,
                )
            separation = approach_plane_separation_m(first, second, outward_normal_base)
            if separation + 1.0e-12 < required:
                raise ConfigurationError(
                    "target centres violate approach-plane min_center_separation_m "
                    f"(required>={required}, observed={separation})"
                )


def _pair_ok(
    candidate: Sequence[float],
    existing: Sequence[Sequence[float]],
    *,
    min_center_separation_m: float,
    edge_m: float,
    outward_normal_base: Sequence[float],
    z_separation_gain: float,
    pre_approach_distance_m: float,
) -> bool:
    for other in existing:
        required = z_aware_required_separation_m(
            candidate,
            other,
            min_center_separation_m=min_center_separation_m,
            edge_m=edge_m,
            outward_normal_base=outward_normal_base,
            z_separation_gain=z_separation_gain,
            pre_approach_distance_m=pre_approach_distance_m,
        )
        if approach_plane_separation_m(candidate, other, outward_normal_base) + 1.0e-12 < required:
            return False
    return True


def _sample_z_in_reach_band(
    rng: np.random.Generator,
    *,
    z_lo_band: float,
    z_hi_band: float,
    z_lo_reach: float,
    z_hi_reach: float,
) -> float | None:
    lo = max(z_lo_band, z_lo_reach)
    hi = min(z_hi_band, z_hi_reach)
    if hi + 1.0e-12 < lo:
        return None
    if abs(hi - lo) <= 1.0e-15:
        return float(lo)
    return float(rng.uniform(lo, hi))


def build_random_centers(
    count: int,
    field_minimum_m: Sequence[float],
    field_maximum_m: Sequence[float],
    *,
    arm_z_motion_range_m: float,
    edge_m: float,
    min_center_separation_m: float,
    keep_outs: Sequence[KeepOutAabb] = (),
    placement_seed: int,
    max_placement_attempts: int = 1000,
    outward_normal_base: Sequence[float] = (0.0, 0.0, 1.0),
    max_target_radial_m: float | None = None,
    z_band_fraction: float = DEFAULT_Z_BAND_FRACTION,
    delta_z_m: float | None = None,
    z_separation_gain: float = 1.0,
    pre_approach_distance_m: float = 0.05,
) -> tuple[tuple[float, float, float], ...]:
    """Sample ``count`` centres with separation, keep-out, and ROM constraints."""

    if count < 1:
        raise ConfigurationError("random placement count must be positive")
    if max_placement_attempts < count:
        raise ConfigurationError("max_placement_attempts must be >= target_count")
    lo = _tuple3(field_minimum_m, "field_minimum_m")
    hi = _tuple3(field_maximum_m, "field_maximum_m")
    _, z_lo, z_hi = z_band_bounds(
        lo,
        hi,
        arm_z_motion_range_m=arm_z_motion_range_m,
        z_band_fraction=z_band_fraction,
        delta_z_m=delta_z_m,
    )
    _, z_lo_reach, z_hi_reach = arm_reach_z_bounds(
        lo, hi, arm_z_motion_range_m=arm_z_motion_range_m
    )
    rng = np.random.default_rng(int(placement_seed))
    chosen: list[tuple[float, float, float]] = []
    attempts = 0
    rom_retries_for_slot = 0
    while len(chosen) < count and attempts < max_placement_attempts:
        attempts += 1
        candidate = (
            float(rng.uniform(lo[0], hi[0])),
            float(rng.uniform(lo[1], hi[1])),
            float(rng.uniform(z_lo, z_hi)),
        )
        if center_outside_arm_reach(
            candidate,
            edge_m=edge_m,
            field_minimum_m=lo,
            field_maximum_m=hi,
            arm_z_motion_range_m=arm_z_motion_range_m,
            max_target_radial_m=max_target_radial_m,
        ):
            rom_retries_for_slot += 1
            if rom_retries_for_slot > MAX_ROM_SUBSTITUTE_RETRIES:
                raise ConfigurationError(
                    f"random placement failed after {MAX_ROM_SUBSTITUTE_RETRIES} "
                    "arm-reach substitute retries for one target; reduce delta_z_m "
                    "/ z_band_fraction or widen the reach envelope"
                )
            continue
        if center_violates_keep_outs(candidate, edge_m, keep_outs):
            continue
        if center_violates_rim(candidate, edge_m=edge_m, max_target_radial_m=max_target_radial_m):
            continue
        if not _pair_ok(
            candidate,
            chosen,
            min_center_separation_m=min_center_separation_m,
            edge_m=edge_m,
            outward_normal_base=outward_normal_base,
            z_separation_gain=z_separation_gain,
            pre_approach_distance_m=pre_approach_distance_m,
        ):
            continue
        chosen.append(candidate)
        rom_retries_for_slot = 0
    if len(chosen) < count:
        raise ConfigurationError(
            f"random placement failed after {max_placement_attempts} attempts "
            f"(placed {len(chosen)}/{count}); relax keep_outs, separation, or AABB"
        )
    validate_centers_separation(
        chosen,
        min_center_separation_m=min_center_separation_m,
        edge_m=edge_m,
        keep_outs=keep_outs,
        outward_normal_base=outward_normal_base,
        max_target_radial_m=max_target_radial_m,
        z_separation_gain=z_separation_gain,
        pre_approach_distance_m=pre_approach_distance_m,
        field_minimum_m=lo,
        field_maximum_m=hi,
        arm_z_motion_range_m=arm_z_motion_range_m,
        require_arm_reach=True,
    )
    return tuple(chosen)


def replace_out_of_reach_centers(
    centers: Sequence[tuple[float, float, float]],
    *,
    field_minimum_m: Sequence[float],
    field_maximum_m: Sequence[float],
    arm_z_motion_range_m: float,
    edge_m: float,
    min_center_separation_m: float,
    keep_outs: Sequence[KeepOutAabb],
    outward_normal_base: Sequence[float],
    max_target_radial_m: float | None,
    z_lo_band: float,
    z_hi_band: float,
    z_separation_gain: float,
    pre_approach_distance_m: float,
    placement_seed: int | None,
) -> tuple[tuple[float, float, float], ...]:
    """Discard invalid centres and sample in-reach Z substitutes (≤3 each).

    A centre is discarded when it is outside arm reach, intersects a keep-out,
    or fails the Z-aware approach-plane floor against already-accepted centres.
    Substitutes keep XY and resample Z inside the intersection of the requested
    band and the arm-reach envelope. Pairwise checks use only finalized
    centres; callers re-validate the full set afterward.
    """

    lo = _tuple3(field_minimum_m, "field_minimum_m")
    hi = _tuple3(field_maximum_m, "field_maximum_m")
    _, z_lo_reach, z_hi_reach = arm_reach_z_bounds(
        lo, hi, arm_z_motion_range_m=arm_z_motion_range_m
    )
    rng = np.random.default_rng(0 if placement_seed is None else int(placement_seed) + 17)
    result: list[tuple[float, float, float]] = []
    for center in centers:
        if (
            not center_outside_arm_reach(
                center,
                edge_m=edge_m,
                field_minimum_m=lo,
                field_maximum_m=hi,
                arm_z_motion_range_m=arm_z_motion_range_m,
                max_target_radial_m=max_target_radial_m,
            )
            and not center_violates_keep_outs(center, edge_m, keep_outs)
            and _pair_ok(
                center,
                result,
                min_center_separation_m=min_center_separation_m,
                edge_m=edge_m,
                outward_normal_base=outward_normal_base,
                z_separation_gain=z_separation_gain,
                pre_approach_distance_m=pre_approach_distance_m,
            )
        ):
            result.append(center)
            continue
        # Prefer Z of the nearest accepted neighbour (collapses Δz for close XY
        # pairs), then mid-band, then random samples within reach∩band.
        trial_zs: list[float] = []
        if result:
            nearest = min(
                result,
                key=lambda point: approach_plane_separation_m(center, point, outward_normal_base),
            )
            trial_zs.append(float(nearest[2]))
        trial_zs.append(0.5 * (z_lo_band + z_hi_band))
        while len(trial_zs) < MAX_ROM_SUBSTITUTE_RETRIES:
            sampled = _sample_z_in_reach_band(
                rng,
                z_lo_band=z_lo_band,
                z_hi_band=z_hi_band,
                z_lo_reach=z_lo_reach,
                z_hi_reach=z_hi_reach,
            )
            if sampled is None:
                break
            trial_zs.append(sampled)
        replaced = False
        for z_sub in trial_zs[:MAX_ROM_SUBSTITUTE_RETRIES]:
            lo_z = max(z_lo_band, z_lo_reach)
            hi_z = min(z_hi_band, z_hi_reach)
            if z_sub < lo_z - 1.0e-12 or z_sub > hi_z + 1.0e-12:
                continue
            candidate = (float(center[0]), float(center[1]), float(z_sub))
            if center_outside_arm_reach(
                candidate,
                edge_m=edge_m,
                field_minimum_m=lo,
                field_maximum_m=hi,
                arm_z_motion_range_m=arm_z_motion_range_m,
                max_target_radial_m=max_target_radial_m,
            ):
                continue
            if center_violates_keep_outs(candidate, edge_m, keep_outs):
                continue
            if not _pair_ok(
                candidate,
                result,
                min_center_separation_m=min_center_separation_m,
                edge_m=edge_m,
                outward_normal_base=outward_normal_base,
                z_separation_gain=z_separation_gain,
                pre_approach_distance_m=pre_approach_distance_m,
            ):
                continue
            result.append(candidate)
            replaced = True
            break
        if not replaced:
            raise ConfigurationError(
                f"target generation failed after {MAX_ROM_SUBSTITUTE_RETRIES} "
                "substitute retries for one target (arm-reach / keep-out / "
                "Z-aware separation); reduce delta_z_m / z_band_fraction or "
                "relax packing constraints"
            )
    return tuple(result)


def build_layout_centers(
    count: int,
    field_minimum_m: Sequence[float],
    field_maximum_m: Sequence[float],
    *,
    layout: LayoutSpec,
    arm_z_motion_range_m: float,
    edge_m: float,
    min_center_separation_m: float,
    keep_outs: Sequence[KeepOutAabb] = (),
    placement_seed: int | None = None,
    outward_normal_base: Sequence[float] = (0.0, 0.0, 1.0),
    max_target_radial_m: float | None = None,
    z_band_fraction: float = DEFAULT_Z_BAND_FRACTION,
    delta_z_m: float | None = None,
    z_separation_gain: float = 1.0,
    pre_approach_distance_m: float = 0.05,
) -> tuple[tuple[float, float, float], ...]:
    """Build centres for a named layout; optional seed rotates/phases the set."""

    if count < 1:
        raise ConfigurationError("layout count must be positive")
    lo = _tuple3(field_minimum_m, "field_minimum_m")
    hi = _tuple3(field_maximum_m, "field_maximum_m")
    mid_z, z_lo, z_hi = z_band_bounds(
        lo,
        hi,
        arm_z_motion_range_m=arm_z_motion_range_m,
        z_band_fraction=z_band_fraction,
        delta_z_m=delta_z_m,
    )
    if layout.name is LayoutName.ROWS:
        assert layout.rows is not None and layout.columns is not None
        capacity = layout.rows * layout.columns
        if capacity < count:
            raise ConfigurationError(
                f"layout rows*columns ({capacity}) must be >= target_count ({count})"
            )
        phase = 0.0
        if placement_seed is not None:
            phase = float(np.random.default_rng(int(placement_seed)).uniform(0.0, 1.0))
        centers: list[tuple[float, float, float]] = []
        for index in range(count):
            shifted = (index + int(phase * capacity)) % capacity
            row = shifted // layout.columns
            col = shifted % layout.columns
            x = (
                lo[0]
                if layout.columns == 1
                else lo[0] + (col + 0.5) * (hi[0] - lo[0]) / layout.columns
            )
            y = lo[1] if layout.rows == 1 else lo[1] + (row + 0.5) * (hi[1] - lo[1]) / layout.rows
            if count == 1:
                z = mid_z
            else:
                z = z_lo + (index + 0.5) * (z_hi - z_lo) / count
            centers.append((float(x), float(y), float(z)))
    else:
        assert layout.radius_m is not None
        assert layout.span_rad is not None
        assert layout.center_xy_m is not None
        start = (
            -0.5 * layout.span_rad
            if layout.start_angle_rad is None
            else float(layout.start_angle_rad)
        )
        if placement_seed is not None:
            start += float(np.random.default_rng(int(placement_seed)).uniform(0.0, 0.25))
        z = mid_z if layout.z_m is None else float(layout.z_m)
        if layout.z_m is not None and (z < lo[2] - 1.0e-9 or z > hi[2] + 1.0e-9):
            raise ConfigurationError("layout.z_m must lie within field_aabb Z")
        centers = []
        for index in range(count):
            if count == 1:
                angle = start + 0.5 * layout.span_rad
            else:
                angle = start + index * layout.span_rad / (count - 1)
            x = layout.center_xy_m[0] + layout.radius_m * math.cos(angle)
            y = layout.center_xy_m[1] + layout.radius_m * math.sin(angle)
            if x < lo[0] - 1.0e-9 or x > hi[0] + 1.0e-9:
                raise ConfigurationError("arc layout centre X outside field_aabb")
            if y < lo[1] - 1.0e-9 or y > hi[1] + 1.0e-9:
                raise ConfigurationError("arc layout centre Y outside field_aabb")
            centers.append((float(x), float(y), float(z)))
    centers = list(
        replace_out_of_reach_centers(
            centers,
            field_minimum_m=lo,
            field_maximum_m=hi,
            arm_z_motion_range_m=arm_z_motion_range_m,
            edge_m=edge_m,
            min_center_separation_m=min_center_separation_m,
            keep_outs=keep_outs,
            outward_normal_base=outward_normal_base,
            max_target_radial_m=max_target_radial_m,
            z_lo_band=z_lo,
            z_hi_band=z_hi,
            z_separation_gain=z_separation_gain,
            pre_approach_distance_m=pre_approach_distance_m,
            placement_seed=placement_seed,
        )
    )
    validate_centers_separation(
        centers,
        min_center_separation_m=min_center_separation_m,
        edge_m=edge_m,
        keep_outs=keep_outs,
        outward_normal_base=outward_normal_base,
        max_target_radial_m=max_target_radial_m,
        z_separation_gain=z_separation_gain,
        pre_approach_distance_m=pre_approach_distance_m,
        field_minimum_m=lo,
        field_maximum_m=hi,
        arm_z_motion_range_m=arm_z_motion_range_m,
        require_arm_reach=True,
    )
    return tuple(centers)
