"""Typed, Isaac-independent cube geometry and clearance helpers."""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np

from mycobot_curobo.errors import ConfigurationError


def _vector3(value: Sequence[float], label: str) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ConfigurationError(f"{label} must be three finite values")
    return array


def _normal(value: Sequence[float]) -> np.ndarray:
    vector = _vector3(value, "outward_normal")
    magnitude = float(np.linalg.norm(vector))
    if magnitude <= 1.0e-12:
        raise ConfigurationError("outward_normal must have non-zero magnitude")
    return vector / magnitude


@dataclass(frozen=True)
class CubeGeometry:
    """Axis-aligned cube expressed in the explicit ``g_base`` frame."""

    center_m: tuple[float, float, float]
    edge_m: float
    orientation_wxyz: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    name: str = "phase7_1_cube"

    def __post_init__(self) -> None:
        center = _vector3(self.center_m, "cube center_m")
        edge = float(self.edge_m)
        orientation = np.asarray(self.orientation_wxyz, dtype=float)
        if not math.isfinite(edge) or edge <= 0.0:
            raise ConfigurationError("cube edge_m must be positive and finite")
        if orientation.shape != (4,) or not np.all(np.isfinite(orientation)):
            raise ConfigurationError("cube orientation_wxyz must contain four finite values")
        if not np.allclose(orientation, (1.0, 0.0, 0.0, 0.0), atol=1.0e-12):
            raise ConfigurationError("Phase 7.1 cubes must be identity-oriented AABBs")
        if not self.name:
            raise ConfigurationError("cube name must be non-empty")
        object.__setattr__(self, "center_m", tuple(float(item) for item in center))
        object.__setattr__(self, "edge_m", edge)


def cube_face_center(
    center_m: Sequence[float], edge_m: float, outward_normal: Sequence[float]
) -> np.ndarray:
    """Return the cube face centre selected by a unit outward direction."""

    edge = float(edge_m)
    if not math.isfinite(edge) or edge <= 0.0:
        raise ConfigurationError("cube edge_m must be positive and finite")
    return _vector3(center_m, "cube center_m") + (edge * 0.5) * _normal(outward_normal)


def cube_approach_target_position(
    center_m: Sequence[float],
    edge_m: float,
    outward_normal: Sequence[float],
    standoff_m: float,
) -> np.ndarray:
    """Return a positive-standoff target outside the selected cube face."""

    standoff = float(standoff_m)
    if not math.isfinite(standoff) or standoff <= 0.0:
        raise ConfigurationError("terminal standoff_m must be positive and finite")
    return cube_face_center(center_m, edge_m, outward_normal) + standoff * _normal(outward_normal)


def cube_to_curobo_scene_dict(
    geometry: CubeGeometry,
) -> dict[str, dict[str, dict[str, list[float]]]]:
    """Convert an immutable cube to cuRobo's cuboid scene-model mapping."""

    return cubes_to_curobo_scene_dict((geometry,))


def cubes_to_curobo_scene_dict(
    geometries: Sequence[CubeGeometry],
) -> dict[str, dict[str, dict[str, list[float]]]]:
    """Convert zero or more immutable cubes to cuRobo's cuboid scene mapping."""

    if not geometries:
        return {"cuboid": {}}
    names = [geometry.name for geometry in geometries]
    if len(set(names)) != len(names):
        raise ConfigurationError("cube geometry names must be unique within a scene")
    return {
        "cuboid": {
            geometry.name: {
                "dims": [geometry.edge_m] * 3,
                "pose": [*geometry.center_m, *geometry.orientation_wxyz],
            }
            for geometry in geometries
        }
    }


def cube_scene_revision(geometry: CubeGeometry) -> str:
    """Create a stable scene revision that changes with collision geometry."""

    canonical = json.dumps(
        cubes_to_curobo_scene_dict((geometry,)), sort_keys=True, separators=(",", ":")
    )
    return f"cube-{hashlib.sha256(canonical.encode('utf-8')).hexdigest()[:16]}"


def multi_cube_scene_revision(geometries: Sequence[CubeGeometry]) -> str:
    """Create a stable scene revision for a multi-cuboid obstacle field."""

    canonical = json.dumps(
        cubes_to_curobo_scene_dict(geometries), sort_keys=True, separators=(",", ":")
    )
    return f"cubes-{hashlib.sha256(canonical.encode('utf-8')).hexdigest()[:16]}"


def sphere_aabb_clearance_m(
    sphere_centers_m: np.ndarray,
    sphere_radii_m: np.ndarray,
    aabb_center_m: Sequence[float],
    aabb_half_extents_m: Sequence[float],
) -> float:
    """Return the minimum signed sphere-to-AABB clearance in metres."""

    centers = np.asarray(sphere_centers_m, dtype=float)
    radii = np.asarray(sphere_radii_m, dtype=float)
    center = _vector3(aabb_center_m, "aabb_center_m")
    half = _vector3(aabb_half_extents_m, "aabb_half_extents_m")
    if centers.ndim != 2 or centers.shape[1] != 3 or radii.shape != (centers.shape[0],):
        raise ConfigurationError("sphere centers/radii must have shapes [N,3] and [N]")
    if centers.shape[0] == 0 or not np.all(np.isfinite(centers)) or not np.all(np.isfinite(radii)):
        raise ConfigurationError("spheres must be non-empty and finite")
    if np.any(radii < 0.0) or np.any(half <= 0.0):
        raise ConfigurationError(
            "sphere radii and AABB half extents must be non-negative/positive"
        )
    delta = np.abs(centers - center)
    signed_axis = half - delta
    inside = np.all(signed_axis >= 0.0, axis=1)
    closest_delta = np.maximum(delta - half, 0.0)
    exterior = np.linalg.norm(closest_delta, axis=1) - radii
    interior = np.max(signed_axis, axis=1) * -1.0 - radii
    clearance = np.where(inside, interior, exterior)
    return float(np.min(clearance))


def batch_sphere_cube_clearance_m(
    spheres_xyzw_radius: np.ndarray, cube_center_m: Sequence[float], cube_edge_m: float
) -> np.ndarray:
    """Return each waypoint's minimum signed robot-sphere/cube clearance."""

    spheres = np.asarray(spheres_xyzw_radius, dtype=float)
    edge = float(cube_edge_m)
    if spheres.ndim != 3 or spheres.shape[2] != 4:
        raise ConfigurationError("spheres must have shape [waypoint, sphere, 4]")
    if spheres.shape[0] == 0 or spheres.shape[1] == 0:
        raise ConfigurationError("spheres must contain at least one waypoint and sphere")
    if not math.isfinite(edge) or edge <= 0.0:
        raise ConfigurationError("cube edge_m must be positive and finite")
    return np.asarray(
        [
            sphere_aabb_clearance_m(row[:, :3], row[:, 3], cube_center_m, (edge * 0.5,) * 3)
            for row in spheres
        ],
        dtype=float,
    )


def flange_disk_face_overhang_m(
    tcp_position_m: Sequence[float],
    face_outward_normal_base: Sequence[float],
    flange_diameter_m: float,
    cube: CubeGeometry,
) -> float:
    """Return how far a flange disk extends past the selected cube face (metres).

    The flange is modeled as a circle of diameter ``flange_diameter_m`` in the
    plane through ``tcp_position_m`` perpendicular to ``face_outward_normal_base``.
    For an identity-oriented AABB, the selected face is the face whose outward
    normal matches ``face_outward_normal_base``. Zero means the disk is contained
    in the face square; positive means flange/face collision (overhang) at tip
    contact.
    """

    diameter = float(flange_diameter_m)
    if not math.isfinite(diameter) or diameter <= 0.0:
        raise ConfigurationError("flange_diameter_m must be positive finite")
    radius = 0.5 * diameter
    tcp = _vector3(tcp_position_m, "tcp_position_m")
    outward = _normal(face_outward_normal_base)
    axis = int(np.argmax(np.abs(outward)))
    sign = 1.0 if outward[axis] >= 0.0 else -1.0
    half = 0.5 * float(cube.edge_m)
    face_center = np.asarray(cube.center_m, dtype=float).copy()
    face_center[axis] = face_center[axis] + sign * half
    planar = tcp - face_center
    planar = planar - float(np.dot(planar, outward)) * outward
    other_axes = tuple(i for i in range(3) if i != axis)
    overhang = 0.0
    for face_axis in other_axes:
        overhang = max(overhang, abs(float(planar[face_axis])) + radius - half)
    return float(max(0.0, overhang))


def flange_disk_collides_contact_face(
    tcp_position_m: Sequence[float],
    face_outward_normal_base: Sequence[float],
    flange_diameter_m: float,
    cube: CubeGeometry,
    *,
    overhang_tolerance_m: float = 1.0e-4,
) -> bool:
    """True when the flange disk overhangs the contact face beyond tolerance."""

    tol = float(overhang_tolerance_m)
    if not math.isfinite(tol) or tol < 0.0:
        raise ConfigurationError("overhang_tolerance_m must be finite and >= 0")
    return (
        flange_disk_face_overhang_m(
            tcp_position_m, face_outward_normal_base, flange_diameter_m, cube
        )
        > tol
    )
