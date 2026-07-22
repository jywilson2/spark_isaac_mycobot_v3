"""Offline sparse collision-sphere covers for target-scale obstacles (Phase 1.1).

Option A (thickness-capped): centres stay on mesh-derived sample points. Each
sphere radius is capped by a local medial/thickness estimate (and optionally by
``E``) so the cover cannot balloon past thin link cross-sections. Remaining
samples densify with smaller spheres until the edge-``E`` detectability
guarantee holds under project sphere–AABB clearance math.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np

from mycobot_curobo.errors import ConfigurationError


@dataclass(frozen=True)
class CollisionSphere:
    """One static link-frame collision sphere."""

    center_m: tuple[float, float, float]
    radius_m: float

    def as_mapping(self) -> dict[str, object]:
        return {
            "center": [float(self.center_m[0]), float(self.center_m[1]), float(self.center_m[2])],
            "radius": float(self.radius_m),
        }


def _local(tag: str) -> str:
    return tag.split("}")[-1] if "}" in tag else tag


def _dae_unit_meters(root: ET.Element) -> float:
    """Return COLLADA ``<unit meter="...">`` scale (defaults to 1.0)."""

    for element in root.iter():
        if _local(element.tag) != "unit":
            continue
        meter = element.get("meter")
        if meter is None:
            continue
        scale = float(meter)
        if not math.isfinite(scale) or scale <= 0.0:
            raise ConfigurationError("COLLADA unit meter must be positive finite")
        return scale
    return 1.0


def load_dae_vertex_positions(path: Path | str) -> np.ndarray:
    """Return (N,3) vertex positions in meters from a COLLADA mesh.

    Prefers float_arrays whose id contains ``position`` so UV/normal arrays are
    not mistaken for geometry. Applies ``<unit meter>`` scaling.
    """

    source = Path(path)
    if not source.is_file():
        raise ConfigurationError(f"COLLADA mesh not found: {source}")
    root = ET.fromstring(source.read_text(encoding="utf-8", errors="ignore"))
    unit_m = _dae_unit_meters(root)
    positioned: list[np.ndarray] = []
    fallback: list[np.ndarray] = []
    for element in root.iter():
        if _local(element.tag) != "float_array" or not element.text:
            continue
        values = np.fromstring(element.text, sep=" ", dtype=float)
        if values.size < 9 or values.size % 3 != 0:
            continue
        points = values.reshape(-1, 3)
        array_id = (element.get("id") or "").lower()
        if "position" in array_id:
            positioned.append(points)
        else:
            fallback.append(points)
    candidates = positioned if positioned else fallback
    if not candidates:
        raise ConfigurationError(f"no xyz float_array vertices in {source}")
    best = max(candidates, key=lambda item: item.shape[0])
    if not np.all(np.isfinite(best)):
        raise ConfigurationError(f"non-finite vertices in {source}")
    return best * unit_m


def transform_points(
    points_m: np.ndarray, *, origin_xyz: Sequence[float], origin_rpy: Sequence[float]
) -> np.ndarray:
    """Apply URDF visual/collision origin (xyz + rpy) to mesh points."""

    xyz = np.asarray(origin_xyz, dtype=float).reshape(3)
    rpy = np.asarray(origin_rpy, dtype=float).reshape(3)
    if xyz.shape != (3,) or rpy.shape != (3,) or not np.all(np.isfinite(xyz)):
        raise ConfigurationError("origin_xyz/origin_rpy must be finite length-3")
    if not np.all(np.isfinite(rpy)):
        raise ConfigurationError("origin_rpy must be finite")
    roll, pitch, yaw = (float(rpy[0]), float(rpy[1]), float(rpy[2]))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )
    return points_m @ rotation.T + xyz


def voxel_downsample_points(points_m: np.ndarray, pitch_m: float) -> np.ndarray:
    """Keep one representative point per pitch-sized voxel (deterministic)."""

    pitch = float(pitch_m)
    if not math.isfinite(pitch) or pitch <= 0.0:
        raise ConfigurationError("voxel pitch_m must be positive finite")
    if points_m.ndim != 2 or points_m.shape[1] != 3 or points_m.shape[0] == 0:
        raise ConfigurationError("points_m must be a non-empty (N,3) array")
    keys = np.floor(points_m / pitch).astype(np.int64)
    order = np.lexsort((keys[:, 2], keys[:, 1], keys[:, 0]))
    sorted_keys = keys[order]
    sorted_points = points_m[order]
    unique_mask = np.ones(sorted_keys.shape[0], dtype=bool)
    unique_mask[1:] = np.any(sorted_keys[1:] != sorted_keys[:-1], axis=1)
    return sorted_points[unique_mask]


def estimate_local_medial_radius_m(
    points_m: np.ndarray,
    query_m: np.ndarray,
    *,
    neighbor_radius_m: float,
    min_neighbors: int = 8,
) -> float:
    """Estimate local inradius from the thinnest local AABB half-extent.

    Uses neighbours within ``neighbor_radius_m`` (falling back to the nearest
    ``min_neighbors`` points). Returns a positive finite medial proxy.
    """

    points = np.asarray(points_m, dtype=float)
    query = np.asarray(query_m, dtype=float).reshape(3)
    radius = float(neighbor_radius_m)
    if points.ndim != 2 or points.shape[1] != 3 or points.shape[0] == 0:
        raise ConfigurationError("points_m must be a non-empty (N,3) array")
    if query.shape != (3,) or not np.all(np.isfinite(query)):
        raise ConfigurationError("query_m must be a finite 3-vector")
    if not math.isfinite(radius) or radius <= 0.0:
        raise ConfigurationError("neighbor_radius_m must be positive finite")
    dist = np.linalg.norm(points - query, axis=1)
    mask = dist <= radius
    if int(np.count_nonzero(mask)) < min_neighbors:
        order = np.argsort(dist)
        neighbors = points[order[: max(min_neighbors, 1)]]
    else:
        neighbors = points[mask]
    extents = neighbors.max(axis=0) - neighbors.min(axis=0)
    half = 0.5 * float(np.min(extents))
    if not math.isfinite(half) or half <= 0.0:
        # Degenerate neighbourhood (planar/collinear): use mean nearest spacing.
        positive = dist[dist > 1.0e-9]
        if positive.size == 0:
            raise ConfigurationError("cannot estimate medial radius from coincident points")
        half = 0.5 * float(np.min(positive))
    return half


def sparse_cover_points_for_obstacle_edge(
    points_m: np.ndarray,
    obstacle_edge_m: float,
    *,
    max_radius_scale: float = 1.0,
    min_radius_scale: float = 0.25,
    thickness_cap: bool = True,
    also_cap_by_e: bool = True,
    thickness_factor: float = 0.85,
    max_spheres: int = 512,
) -> tuple[CollisionSphere, ...]:
    """Greedy thickness-capped ball cover of mesh sample points for edge ``E``.

    Option A: each sphere centre is a sample point. Radius is at most
    ``thickness_factor * local_medial`` and, when ``also_cap_by_e``, also at most
    ``max_radius_scale * E``. Floor radius is
    ``min(min_radius_scale * E, thickness_cap)``. Remaining uncovered samples
    densify until covered or ``max_spheres`` is exceeded (fail closed).
    """

    edge = float(obstacle_edge_m)
    if not math.isfinite(edge) or edge <= 0.0:
        raise ConfigurationError("obstacle_edge_m must be positive finite")
    if max_radius_scale < min_radius_scale or min_radius_scale <= 0.0:
        raise ConfigurationError("radius scales must satisfy max >= min > 0")
    if not math.isfinite(thickness_factor) or thickness_factor <= 0.0 or thickness_factor > 1.0:
        raise ConfigurationError("thickness_factor must be in (0, 1]")
    if max_spheres < 1:
        raise ConfigurationError("max_spheres must be positive")
    working = np.asarray(points_m, dtype=float)
    if working.shape[0] > 50_000:
        stride = int(math.ceil(working.shape[0] / 50_000))
        working = working[::stride]
    # Slightly finer pitch under thickness caps so thin links still densify
    # without exploding past the host overlay sphere budget (~1k–2k).
    pitch = 0.4 * edge if thickness_cap else 0.5 * edge
    samples = voxel_downsample_points(working, pitch_m=pitch)
    if samples.shape[0] == 0:
        raise ConfigurationError("no samples after voxel downsample")
    global_max = max_radius_scale * edge
    global_min = min_radius_scale * edge
    remaining = np.ones(samples.shape[0], dtype=bool)
    spheres: list[CollisionSphere] = []
    neighbor_r = max(2.0 * edge, 3.0 * pitch)
    while np.any(remaining):
        if len(spheres) >= max_spheres:
            raise ConfigurationError(
                f"sphere budget {max_spheres} exceeded before covering samples"
            )
        seed_index = int(np.flatnonzero(remaining)[0])
        center = samples[seed_index]
        if thickness_cap:
            medial = estimate_local_medial_radius_m(working, center, neighbor_radius_m=neighbor_r)
            thickness_max = thickness_factor * medial
        else:
            thickness_max = global_max
        max_radius = min(thickness_max, global_max) if also_cap_by_e else thickness_max
        max_radius = max(max_radius, 1.0e-4)
        min_radius = min(global_min, max_radius)
        delta = samples - center
        dist = np.linalg.norm(delta, axis=1)
        covered = remaining & (dist <= max_radius)
        covered_dist = dist[covered]
        radius = float(
            max(min_radius, float(np.max(covered_dist)) if covered_dist.size else min_radius)
        )
        radius = min(radius, max_radius)
        spheres.append(
            CollisionSphere(
                center_m=(float(center[0]), float(center[1]), float(center[2])),
                radius_m=radius,
            )
        )
        remaining[covered] = False
        # If the seed itself was not marked covered (numerical), clear it.
        remaining[seed_index] = False

    # Densify for raw working vertices still outside all spheres.
    centers = np.asarray([sphere.center_m for sphere in spheres], dtype=float)
    radii = np.asarray([sphere.radius_m for sphere in spheres], dtype=float)
    for point in working:
        dist = np.linalg.norm(centers - point, axis=1)
        nearest = int(np.argmin(dist))
        needed = float(dist[nearest])
        if needed <= radii[nearest] + 1.0e-9:
            continue
        if thickness_cap:
            medial = estimate_local_medial_radius_m(working, point, neighbor_radius_m=neighbor_r)
            thickness_max = thickness_factor * medial
        else:
            thickness_max = global_max
        max_radius = min(thickness_max, global_max) if also_cap_by_e else thickness_max
        max_radius = max(max_radius, 1.0e-4)
        if needed <= max_radius + 1.0e-9:
            radii[nearest] = max(radii[nearest], min(needed, max_radius))
            continue
        if centers.shape[0] >= max_spheres:
            raise ConfigurationError(f"sphere budget {max_spheres} exceeded during densify pass")
        centers = np.vstack([centers, point.reshape(1, 3)])
        radii = np.append(radii, min(global_min, max_radius))
    return tuple(
        CollisionSphere(
            center_m=(float(centers[i, 0]), float(centers[i, 1]), float(centers[i, 2])),
            radius_m=float(radii[i]),
        )
        for i in range(centers.shape[0])
    )


def sample_points_on_sphere_union(
    spheres: Sequence[CollisionSphere],
    obstacle_edge_m: float,
    *,
    seed: int = 0,
) -> np.ndarray:
    """Sample mesh-envelope points on the union of seed spheres (link frame, m)."""

    edge = float(obstacle_edge_m)
    if not math.isfinite(edge) or edge <= 0.0:
        raise ConfigurationError("obstacle_edge_m must be positive finite")
    if not spheres:
        raise ConfigurationError("seed spheres must be non-empty")
    rng = np.random.default_rng(seed)
    points: list[np.ndarray] = []
    for sphere in spheres:
        center = np.asarray(sphere.center_m, dtype=float)
        radius = float(sphere.radius_m)
        if radius <= 0.0 or not np.all(np.isfinite(center)):
            raise ConfigurationError("seed spheres must have finite positive radii")
        points.append(center.reshape(1, 3))
        area = 4.0 * math.pi * radius * radius
        cell = max(0.25 * edge * edge, 1.0e-8)
        count = max(8, int(math.ceil(area / cell)))
        count = min(count, 256)
        indices = np.arange(count, dtype=float) + 0.5
        phi = np.arccos(1.0 - 2.0 * indices / count)
        theta = math.pi * (1.0 + 5**0.5) * indices
        directions = np.column_stack(
            (np.sin(phi) * np.cos(theta), np.sin(phi) * np.sin(theta), np.cos(phi))
        )
        jitter = 0.02 * edge * rng.standard_normal(directions.shape)
        surface = center + radius * directions + jitter
        points.append(surface)
    stacked = np.vstack(points)
    return voxel_downsample_points(stacked, pitch_m=0.5 * edge)


def points_covered_by_spheres(
    points_m: np.ndarray, spheres: Sequence[CollisionSphere], *, atol_m: float = 1.0e-9
) -> bool:
    """True when every point lies inside at least one sphere (inclusive)."""

    if not spheres:
        return points_m.shape[0] == 0
    centers = np.asarray([sphere.center_m for sphere in spheres], dtype=float)
    radii = np.asarray([sphere.radius_m for sphere in spheres], dtype=float)
    delta = points_m[:, None, :] - centers[None, :, :]
    dist = np.linalg.norm(delta, axis=2)
    return bool(np.all(np.min(dist - radii[None, :], axis=1) <= atol_m))


def parse_urdf_collision_mesh_origins(
    urdf_path: Path | str, link_names: Sequence[str]
) -> dict[str, tuple[Path, tuple[float, float, float], tuple[float, float, float]]]:
    """Map link name -> (mesh_path, origin_xyz, origin_rpy) from first collision mesh."""

    path = Path(urdf_path)
    root = ET.fromstring(path.read_text(encoding="utf-8"))
    wanted = set(link_names)
    found: dict[str, tuple[Path, tuple[float, float, float], tuple[float, float, float]]] = {}
    for link in root.iter("link"):
        name = link.get("name")
        if name not in wanted:
            continue
        collision = link.find("collision")
        if collision is None:
            continue
        mesh = collision.find("geometry/mesh")
        if mesh is None or not mesh.get("filename"):
            continue
        filename = str(mesh.get("filename"))
        mesh_name = Path(filename).name
        origin = collision.find("origin")
        xyz = (0.0, 0.0, 0.0)
        rpy = (0.0, 0.0, 0.0)
        if origin is not None:
            if origin.get("xyz"):
                parts = [float(item) for item in origin.get("xyz", "").split()]
                if len(parts) == 3:
                    xyz = (parts[0], parts[1], parts[2])
            if origin.get("rpy"):
                parts = [float(item) for item in origin.get("rpy", "").split()]
                if len(parts) == 3:
                    rpy = (parts[0], parts[1], parts[2])
        found[name] = (Path(mesh_name), xyz, rpy)
    missing = [name for name in link_names if name not in found]
    if missing:
        raise ConfigurationError(f"URDF missing collision meshes for links: {missing}")
    return found
