"""Phase 7.3 target-block placement: random sampling, named layouts, keep-outs."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Any, Sequence

import numpy as np

from mycobot_curobo.errors import ConfigurationError

GRID_Z_VARIABILITY_FRACTION = 0.5


def ee_clearance_min_center_separation_m(
    edge_m: float, flange_diameter_assumption_m: float
) -> float:
    """Minimum centre distance for tip/EE clearance (edge + flange diameter)."""

    edge = float(edge_m)
    flange = float(flange_diameter_assumption_m)
    if not math.isfinite(edge) or edge <= 0.0:
        raise ConfigurationError("target_edge_m must be positive finite")
    if not math.isfinite(flange) or flange <= 0.0:
        raise ConfigurationError("flange_diameter_assumption_m must be positive finite")
    return edge + flange


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


def z_band_bounds(
    field_minimum_m: Sequence[float],
    field_maximum_m: Sequence[float],
    *,
    arm_z_motion_range_m: float,
) -> tuple[float, float, float]:
    """Return ``(mid_z, z_lo, z_hi)`` for the grid/random vertical band."""

    if not math.isfinite(arm_z_motion_range_m) or arm_z_motion_range_m <= 0.0:
        raise ConfigurationError("arm_z_motion_range_m must be positive and finite")
    lo = _tuple3(field_minimum_m, "field_minimum_m")
    hi = _tuple3(field_maximum_m, "field_maximum_m")
    mid_z = 0.5 * (lo[2] + hi[2])
    half_band = 0.5 * GRID_Z_VARIABILITY_FRACTION * arm_z_motion_range_m
    return mid_z, mid_z - half_band, mid_z + half_band


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
) -> None:
    """Fail closed when centres are too close or intersect keep-outs."""

    if not math.isfinite(min_center_separation_m) or min_center_separation_m <= 0.0:
        raise ConfigurationError("min_center_separation_m must be positive finite")
    points = [np.asarray(center, dtype=float).reshape(3) for center in centers]
    for point in points:
        if not np.all(np.isfinite(point)):
            raise ConfigurationError("target centres must be finite")
        if center_violates_keep_outs(point, edge_m, keep_outs):
            raise ConfigurationError("target centre intersects a keep_out AABB")
    for index, first in enumerate(points):
        for second in points[index + 1 :]:
            if float(np.linalg.norm(first - second)) + 1.0e-12 < min_center_separation_m:
                raise ConfigurationError(
                    "target centres violate min_center_separation_m "
                    f"(required>={min_center_separation_m})"
                )


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
) -> tuple[tuple[float, float, float], ...]:
    """Sample ``count`` centres with separation and keep-out constraints."""

    if count < 1:
        raise ConfigurationError("random placement count must be positive")
    if max_placement_attempts < count:
        raise ConfigurationError("max_placement_attempts must be >= target_count")
    lo = _tuple3(field_minimum_m, "field_minimum_m")
    hi = _tuple3(field_maximum_m, "field_maximum_m")
    _, z_lo, z_hi = z_band_bounds(lo, hi, arm_z_motion_range_m=arm_z_motion_range_m)
    rng = np.random.default_rng(int(placement_seed))
    chosen: list[tuple[float, float, float]] = []
    attempts = 0
    while len(chosen) < count and attempts < max_placement_attempts:
        attempts += 1
        candidate = (
            float(rng.uniform(lo[0], hi[0])),
            float(rng.uniform(lo[1], hi[1])),
            float(rng.uniform(z_lo, z_hi)),
        )
        if center_violates_keep_outs(candidate, edge_m, keep_outs):
            continue
        point = np.asarray(candidate, dtype=float)
        if any(
            float(np.linalg.norm(point - np.asarray(existing, dtype=float))) + 1.0e-12
            < min_center_separation_m
            for existing in chosen
        ):
            continue
        chosen.append(candidate)
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
    )
    return tuple(chosen)


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
) -> tuple[tuple[float, float, float], ...]:
    """Build centres for a named layout; optional seed rotates/phases the set."""

    if count < 1:
        raise ConfigurationError("layout count must be positive")
    lo = _tuple3(field_minimum_m, "field_minimum_m")
    hi = _tuple3(field_maximum_m, "field_maximum_m")
    mid_z, z_lo, z_hi = z_band_bounds(lo, hi, arm_z_motion_range_m=arm_z_motion_range_m)
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
        if z < z_lo - 1.0e-9 or z > z_hi + 1.0e-9:
            # Allow explicit z_m inside field AABB even if outside the mid-band.
            if z < lo[2] - 1.0e-9 or z > hi[2] + 1.0e-9:
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
    validate_centers_separation(
        centers,
        min_center_separation_m=min_center_separation_m,
        edge_m=edge_m,
        keep_outs=keep_outs,
    )
    return tuple(centers)
