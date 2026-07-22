"""Measured +Z tip-contact workspace sampling (candidate region, not a claim)."""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Sequence

import numpy as np

from mycobot_curobo.errors import ConfigurationError
from mycobot_curobo.target_placement import KeepOutAabb, center_violates_rim, parse_keep_outs
from mycobot_curobo.targets import SurfaceTarget


@dataclass(frozen=True)
class TipContactWorkspaceConfig:
    """CPU-side sampling contract for tip-contact reachability probes."""

    frame: str
    max_target_radial_m: float
    z_minimum_m: float
    z_maximum_m: float
    grid_step_m: float
    z_layers: int
    keep_outs: tuple[KeepOutAabb, ...]
    pre_approach_distance_m: float
    target_edge_m: float
    outward_normal_base: tuple[float, float, float]
    start_joint_bank_rad: tuple[tuple[float, ...], ...]
    planner_profile: str
    random_seed: int
    declaration: str = "measured_tip_contact_candidate_region_v1"


@dataclass(frozen=True)
class TipContactSample:
    """One sampled tip-face centre in ``g_base``."""

    sample_id: str
    center_m: tuple[float, float, float]


@dataclass(frozen=True)
class TipContactSampleResult:
    sample_id: str
    center_m: tuple[float, float, float]
    start_label: str
    succeeded: bool
    planner_status: str
    failure_reason: str | None
    planning_duration_s: float | None


def default_tip_contact_workspace_config() -> TipContactWorkspaceConfig:
    """Conservative sampler defaults under the Phase 7.2 radial rim."""

    return TipContactWorkspaceConfig(
        frame="g_base",
        max_target_radial_m=0.28,
        z_minimum_m=0.10,
        z_maximum_m=0.22,
        grid_step_m=0.04,
        z_layers=3,
        keep_outs=(KeepOutAabb(minimum_m=(-0.08, -0.08, 0.08), maximum_m=(0.08, 0.08, 0.24)),),
        pre_approach_distance_m=0.01,
        target_edge_m=0.014,
        outward_normal_base=(0.0, 0.0, 1.0),
        start_joint_bank_rad=((0.0, 0.0, 0.0, 0.0, 0.0, 0.0),),
        planner_profile="benchmark_reproducible",
        random_seed=123,
    )


def build_tip_contact_sample_centers(
    config: TipContactWorkspaceConfig,
) -> tuple[TipContactSample, ...]:
    """Even XY lattice clipped to rim + keep-outs, with evenly spaced Z layers."""

    if config.frame != "g_base":
        raise ConfigurationError("tip-contact workspace sampler requires frame g_base")
    step = float(config.grid_step_m)
    if not math.isfinite(step) or step <= 0.0:
        raise ConfigurationError("grid_step_m must be positive finite")
    if config.z_layers < 1:
        raise ConfigurationError("z_layers must be >= 1")
    if config.z_maximum_m <= config.z_minimum_m:
        raise ConfigurationError("z_maximum_m must exceed z_minimum_m")
    rim = float(config.max_target_radial_m)
    if not math.isfinite(rim) or rim <= 0.0:
        raise ConfigurationError("max_target_radial_m must be positive finite")
    half = rim
    xs = np.arange(-half, half + 0.5 * step, step, dtype=float)
    ys = np.arange(-half, half + 0.5 * step, step, dtype=float)
    if config.z_layers == 1:
        zs = (0.5 * (config.z_minimum_m + config.z_maximum_m),)
    else:
        zs = tuple(
            float(z) for z in np.linspace(config.z_minimum_m, config.z_maximum_m, config.z_layers)
        )
    samples: list[TipContactSample] = []
    index = 0
    for z in zs:
        for y in ys:
            for x in xs:
                center = (float(x), float(y), float(z))
                if center_violates_rim(
                    center, edge_m=config.target_edge_m, max_target_radial_m=rim
                ):
                    continue
                if any(
                    keep.minimum_m[0] <= center[0] <= keep.maximum_m[0]
                    and keep.minimum_m[1] <= center[1] <= keep.maximum_m[1]
                    and keep.minimum_m[2] <= center[2] <= keep.maximum_m[2]
                    for keep in config.keep_outs
                ):
                    continue
                samples.append(TipContactSample(sample_id=f"s{index:04d}", center_m=center))
                index += 1
    if not samples:
        raise ConfigurationError("tip-contact workspace sampler produced zero samples")
    return tuple(samples)


def surface_target_for_sample(
    sample: TipContactSample,
    config: TipContactWorkspaceConfig,
) -> SurfaceTarget:
    """Build a +Z (or configured) flange-normal face centre target."""

    # Face centre: treat sample centre as cube centre; contact face is offset
    # along outward normal by half-edge (matches multi-target face centre).
    normal = np.asarray(config.outward_normal_base, dtype=float)
    norm = float(np.linalg.norm(normal))
    if norm <= 1.0e-12:
        raise ConfigurationError("outward_normal_base must be non-zero")
    unit = normal / norm
    face = np.asarray(sample.center_m, dtype=float) + 0.5 * float(config.target_edge_m) * unit
    return SurfaceTarget.create(
        position_base_m=tuple(float(v) for v in face),
        surface_normal_base=tuple(float(v) for v in unit),
        fixed_roll_rad=0.0,
        pre_approach_distance_m=config.pre_approach_distance_m,
        target_id=sample.sample_id,
    )


def summarize_tip_contact_results(
    results: Sequence[TipContactSampleResult],
) -> dict[str, Any]:
    """Aggregate success rate and axis-aligned bounds of successful centres."""

    total = len(results)
    successes = [item for item in results if item.succeeded]
    summary: dict[str, Any] = {
        "total_samples": total,
        "successes": len(successes),
        "success_rate": (0.0 if total == 0 else len(successes) / total),
        "declaration": "measured_tip_contact_candidate_region_v1",
        "success_aabb_m": None,
    }
    if successes:
        pts = np.asarray([item.center_m for item in successes], dtype=float)
        summary["success_aabb_m"] = {
            "minimum_m": [float(v) for v in pts.min(axis=0)],
            "maximum_m": [float(v) for v in pts.max(axis=0)],
        }
    return summary


def serialize_tip_contact_workspace(
    *,
    config: TipContactWorkspaceConfig,
    results: Sequence[TipContactSampleResult],
    schema_version: int = 1,
) -> dict[str, Any]:
    keep_outs = [
        {"minimum_m": list(keep.minimum_m), "maximum_m": list(keep.maximum_m)}
        for keep in config.keep_outs
    ]
    return {
        "schema_version": schema_version,
        "declaration": config.declaration,
        "config": {
            "frame": config.frame,
            "max_target_radial_m": config.max_target_radial_m,
            "z_minimum_m": config.z_minimum_m,
            "z_maximum_m": config.z_maximum_m,
            "grid_step_m": config.grid_step_m,
            "z_layers": config.z_layers,
            "keep_outs": keep_outs,
            "pre_approach_distance_m": config.pre_approach_distance_m,
            "target_edge_m": config.target_edge_m,
            "outward_normal_base": list(config.outward_normal_base),
            "start_joint_bank_rad": [list(row) for row in config.start_joint_bank_rad],
            "planner_profile": config.planner_profile,
            "random_seed": config.random_seed,
        },
        "summary": summarize_tip_contact_results(results),
        "results": [asdict(item) for item in results],
    }


def write_tip_contact_workspace_artifact(path: Path | str, payload: dict[str, Any]) -> Path:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return destination


def load_tip_contact_workspace_config_from_mapping(
    payload: dict[str, Any],
) -> TipContactWorkspaceConfig:
    """Load sampler config from a YAML/JSON mapping (host CLI)."""

    base = default_tip_contact_workspace_config()
    keep_outs = parse_keep_outs(
        payload.get(
            "keep_outs",
            [
                {"minimum_m": list(keep.minimum_m), "maximum_m": list(keep.maximum_m)}
                for keep in base.keep_outs
            ],
        )
    )
    bank_raw = payload.get("start_joint_bank_rad", [list(base.start_joint_bank_rad[0])])
    bank = tuple(tuple(float(v) for v in row) for row in bank_raw)
    if not bank or any(len(row) != 6 for row in bank):
        raise ConfigurationError("start_joint_bank_rad must be non-empty 6-DoF rows")
    normal = payload.get("outward_normal_base", list(base.outward_normal_base))
    return TipContactWorkspaceConfig(
        frame=str(payload.get("frame", base.frame)),
        max_target_radial_m=float(payload.get("max_target_radial_m", base.max_target_radial_m)),
        z_minimum_m=float(payload.get("z_minimum_m", base.z_minimum_m)),
        z_maximum_m=float(payload.get("z_maximum_m", base.z_maximum_m)),
        grid_step_m=float(payload.get("grid_step_m", base.grid_step_m)),
        z_layers=int(payload.get("z_layers", base.z_layers)),
        keep_outs=keep_outs,
        pre_approach_distance_m=float(
            payload.get("pre_approach_distance_m", base.pre_approach_distance_m)
        ),
        target_edge_m=float(payload.get("target_edge_m", base.target_edge_m)),
        outward_normal_base=(float(normal[0]), float(normal[1]), float(normal[2])),
        start_joint_bank_rad=bank,
        planner_profile=str(payload.get("planner_profile", base.planner_profile)),
        random_seed=int(payload.get("random_seed", base.random_seed)),
        declaration=str(payload.get("declaration", base.declaration)),
    )
