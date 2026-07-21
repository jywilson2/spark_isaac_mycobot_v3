"""Unit tests for Phase 1.1 sparse target-scale collision sphere covers."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import yaml

from mycobot_curobo.collision_sphere_cover import (
    CollisionSphere,
    load_dae_vertex_positions,
    points_covered_by_spheres,
    sparse_cover_points_for_obstacle_edge,
    voxel_downsample_points,
)
from mycobot_curobo.cube_scene import sphere_aabb_clearance_m
from mycobot_curobo.errors import ConfigurationError
from mycobot_curobo.multi_target import load_multi_target_suite_config
from mycobot_curobo.robot_model import load_robot_model_spec

ROOT = Path(__file__).parents[2]


def test_voxel_downsample_is_deterministic() -> None:
    points = np.array(
        [[0.0, 0.0, 0.0], [0.001, 0.0, 0.0], [0.02, 0.0, 0.0]],
        dtype=float,
    )
    first = voxel_downsample_points(points, 0.01)
    second = voxel_downsample_points(points, 0.01)
    assert first.shape[0] == 2
    assert np.array_equal(first, second)


def test_sparse_cover_contains_all_samples_for_edge() -> None:
    edge = 0.014
    # Synthetic capsule-like cloud in link frame (meters).
    zs = np.linspace(0.0, 0.08, 40)
    ring = np.linspace(0.0, 2.0 * np.pi, 24, endpoint=False)
    points = []
    for z in zs:
        for angle in ring:
            points.append([0.02 * np.cos(angle), 0.02 * np.sin(angle), z])
    cloud = np.asarray(points, dtype=float)
    spheres = sparse_cover_points_for_obstacle_edge(cloud, edge)
    samples = voxel_downsample_points(cloud, pitch_m=0.5 * edge)
    assert len(spheres) < cloud.shape[0]
    assert points_covered_by_spheres(samples, spheres)
    assert points_covered_by_spheres(cloud, spheres)
    assert all(sphere.radius_m >= 0.5 * edge - 1.0e-12 for sphere in spheres)
    assert all(sphere.radius_m <= 2.0 * edge + 1.0e-12 for sphere in spheres)


def test_covered_envelope_detects_edge_cube_at_sample() -> None:
    edge = 0.014
    cloud = np.array([[0.0, 0.0, 0.0], [0.01, 0.0, 0.0], [0.0, 0.01, 0.0]], dtype=float)
    spheres = sparse_cover_points_for_obstacle_edge(cloud, edge)
    centers = np.asarray([sphere.center_m for sphere in spheres], dtype=float)
    radii = np.asarray([sphere.radius_m for sphere in spheres], dtype=float)
    # Place an edge-E cube that contains the first sample point.
    sample = cloud[0]
    cube_center = sample
    clearance = sphere_aabb_clearance_m(centers, radii, cube_center, (0.5 * edge,) * 3)
    assert clearance <= 0.0


def test_robot_overlay_loads_phase1_1_edge_and_more_spheres() -> None:
    spec = load_robot_model_spec()
    assert spec.min_detectable_obstacle_edge_m == pytest.approx(0.014)
    total = sum(spec.collision_sphere_count_by_link.values())
    assert total == 128
    assert total > 32  # denser than Phase 1 scaffolding


def test_suite_rejects_target_edge_smaller_than_robot_e(tmp_path: Path) -> None:
    source = ROOT / "config/phase7_2_multi_target_manual.yml"
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    payload["target_edge_m"] = 0.010
    path = tmp_path / "too_small.yml"
    path.write_text(yaml.safe_dump(payload), encoding="utf-8")
    with pytest.raises(ConfigurationError, match="min_detectable_obstacle_edge_m"):
        load_multi_target_suite_config(path)


def test_dae_loader_prefers_positions_not_uv_map() -> None:
    mesh = (
        ROOT
        / "third_party/mycobot_ros2/mycobot_description/urdf/mycobot_280_m5/joint1.dae"
    )
    if not mesh.is_file():
        pytest.skip("vendor meshes not downloaded")
    points = load_dae_vertex_positions(mesh)
    extent = points.max(axis=0) - points.min(axis=0)
    assert float(extent.max()) < 0.2
    assert points.shape[0] > 100


def test_collision_sphere_mapping_roundtrip() -> None:
    sphere = CollisionSphere(center_m=(0.1, 0.0, 0.0), radius_m=0.02)
    assert sphere.as_mapping() == {"center": [0.1, 0.0, 0.0], "radius": 0.02}
