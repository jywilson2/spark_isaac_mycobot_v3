#!/usr/bin/env python3
"""Regenerate Phase 1.1 target-scale collision spheres into a robot YAML overlay.

Requires vendor COLLADA meshes from ``./scripts/download_mycobot_ros2.sh``.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "src"))

from mycobot_curobo.collision_sphere_cover import (  # noqa: E402
    load_dae_vertex_positions,
    parse_urdf_collision_mesh_origins,
    sparse_cover_points_for_obstacle_edge,
    transform_points,
)
from mycobot_curobo.errors import ConfigurationError  # noqa: E402


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--robot-yaml",
        type=Path,
        default=REPO_ROOT / "config/robots/mycobot_280_m5.yml",
    )
    parser.add_argument(
        "--overlay-yaml",
        type=Path,
        default=REPO_ROOT / "config/robots/mycobot_280_m5_phase1_1_spheres.yml",
    )
    parser.add_argument(
        "--mesh-dir",
        type=Path,
        default=REPO_ROOT
        / "third_party/mycobot_ros2/mycobot_description/urdf/mycobot_280_m5",
    )
    parser.add_argument(
        "--obstacle-edge-m",
        type=float,
        default=0.014,
        help="min_detectable_obstacle_edge_m (default Phase 7.2 target_edge_m)",
    )
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    robot_path = args.robot_yaml.resolve()
    overlay_path = args.overlay_yaml.resolve()
    mesh_dir = args.mesh_dir.resolve()
    if not robot_path.is_file():
        raise SystemExit(f"robot yaml not found: {robot_path}")
    if not mesh_dir.is_dir():
        raise SystemExit(
            f"mesh dir not found: {mesh_dir} (run ./scripts/download_mycobot_ros2.sh)"
        )
    payload = yaml.safe_load(robot_path.read_text(encoding="utf-8"))
    kinematics = payload["robot_cfg"]["kinematics"]
    urdf_rel = Path(kinematics["urdf_path"])
    urdf_path = (REPO_ROOT / urdf_rel).resolve() if not urdf_rel.is_absolute() else urdf_rel
    link_names = tuple(kinematics["collision_link_names"])
    origins = parse_urdf_collision_mesh_origins(urdf_path, link_names)
    edge = float(args.obstacle_edge_m)
    if edge <= 0.0:
        raise SystemExit("obstacle-edge-m must be positive")

    new_spheres: dict[str, list[dict[str, object]]] = {}
    counts: dict[str, int] = {}
    for link in link_names:
        mesh_name, xyz, rpy = origins[link]
        mesh_path = mesh_dir / mesh_name
        try:
            vertices = load_dae_vertex_positions(mesh_path)
            points = transform_points(vertices, origin_xyz=xyz, origin_rpy=rpy)
            extent = points.max(axis=0) - points.min(axis=0)
            max_extent = float(extent.max())
            if max_extent > 0.5:
                raise ConfigurationError(
                    f"mesh extent {max_extent:.3f} m exceeds 0.5 m; check units"
                )
            cover = sparse_cover_points_for_obstacle_edge(points, edge)
        except ConfigurationError as exc:
            raise SystemExit(f"{link}: {exc}") from exc
        new_spheres[link] = [sphere.as_mapping() for sphere in cover]
        counts[link] = len(cover)
        print(
            f"{link}: {counts[link]} spheres max_extent_m={max_extent:.4f}",
            flush=True,
        )

    overlay = {
        "min_detectable_obstacle_edge_m": edge,
        "collision_spheres": new_spheres,
        "generator": {
            "script": "scripts/host/regenerate_target_scale_collision_spheres.py",
            "obstacle_edge_m": edge,
            "max_radius_scale": 2.0,
            "min_radius_scale": 0.5,
            "voxel_pitch_scale": 0.5,
            "source": "urdf_collision_dae_positions",
        },
    }
    total = sum(counts.values())
    print(f"total_spheres={total} E={edge}", flush=True)
    if args.dry_run:
        return 0
    header = (
        "# Phase 1.1 target-scale collision spheres (generated; do not hand-edit).\n"
        "# Regenerate: python3 scripts/host/regenerate_target_scale_collision_spheres.py\n"
    )
    overlay_path.write_text(
        header + yaml.safe_dump(overlay, default_flow_style=False, sort_keys=False),
        encoding="utf-8",
    )
    print(f"wrote {overlay_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
