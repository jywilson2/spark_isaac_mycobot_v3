"""Offline sparse collision-sphere covers for target-scale obstacles (Phase 1.1).

Centres stay on mesh-derived sample points. Radii grow up to a bound set by the
minimum detectable obstacle edge ``E`` so an axis-aligned cube of edge ``E``
that meets the sampled envelope also meets a sphere under project clearance
math (every sample point is contained in at least one sphere).
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
    # Prefer the largest positions array when several exist.
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
    # Lexicographic unique for stable order.
    order = np.lexsort((keys[:, 2], keys[:, 1], keys[:, 0]))
    sorted_keys = keys[order]
    sorted_points = points_m[order]
    unique_mask = np.ones(sorted_keys.shape[0], dtype=bool)
    unique_mask[1:] = np.any(sorted_keys[1:] != sorted_keys[:-1], axis=1)
    return sorted_points[unique_mask]


def sparse_cover_points_for_obstacle_edge(
    points_m: np.ndarray,
    obstacle_edge_m: float,
    *,
    max_radius_scale: float = 2.0,
    min_radius_scale: float = 0.5,
) -> tuple[CollisionSphere, ...]:
    """Greedy sparse ball cover of mesh sample points for obstacle edge ``E``.

    Every returned sphere centre is one of the (downsampled) sample points.
    Radius is at least ``min_radius_scale * E`` and at most
    ``max_radius_scale * E``, grown to cover as many remaining samples as
    possible within that cap (sparsity).
    """

    edge = float(obstacle_edge_m)
    if not math.isfinite(edge) or edge <= 0.0:
        raise ConfigurationError("obstacle_edge_m must be positive finite")
    if max_radius_scale < min_radius_scale or min_radius_scale <= 0.0:
        raise ConfigurationError("radius scales must satisfy max >= min > 0")
    # Cap raw mesh size before voxel filter (COLLADA dumps can be huge).
    working = np.asarray(points_m, dtype=float)
    if working.shape[0] > 50_000:
        stride = int(math.ceil(working.shape[0] / 50_000))
        working = working[::stride]
    samples = voxel_downsample_points(working, pitch_m=0.5 * edge)
    if samples.shape[0] == 0:
        raise ConfigurationError("no samples after voxel downsample")
    min_radius = min_radius_scale * edge
    max_radius = max_radius_scale * edge
    remaining = np.ones(samples.shape[0], dtype=bool)
    spheres: list[CollisionSphere] = []
    # O(n) greedy: seed at the first remaining sample, cover neighbors within
    # max_radius (no full pairwise distance matrix).
    while np.any(remaining):
        seed_index = int(np.flatnonzero(remaining)[0])
        center = samples[seed_index]
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
    # Grow radii (capped) so remaining working vertices near a sphere are covered
    # without exploding sphere count.
    centers = np.asarray([sphere.center_m for sphere in spheres], dtype=float)
    radii = np.asarray([sphere.radius_m for sphere in spheres], dtype=float)
    for point in working:
        dist = np.linalg.norm(centers - point, axis=1)
        nearest = int(np.argmin(dist))
        needed = float(dist[nearest])
        if needed <= radii[nearest] + 1.0e-9:
            continue
        if needed <= max_radius + 1.0e-9:
            radii[nearest] = max(radii[nearest], needed)
            continue
        centers = np.vstack([centers, point.reshape(1, 3)])
        radii = np.append(radii, min_radius)
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
    """Sample mesh-envelope points on the union of seed spheres (link frame, m).

    Phase 1 scaffolding spheres are treated as a mesh-derived envelope when
    COLLADA node scales are unavailable. Surface spacing tracks ``E``.
    """

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
        # Fibonacci-ish count: surface area / (E/2)^2
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
        # Small deterministic jitter keeps samples off exact lattice seams.
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
        # package://mycobot_description/urdf/mycobot_280_m5/foo.dae
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
