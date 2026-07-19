"""Isaac stage helpers for Phase 7.1 lighting and static cube geometry."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping, Sequence


def _finite_vector(value: Sequence[float], label: str, length: int) -> tuple[float, ...]:
    result = tuple(float(item) for item in value)
    if len(result) != length or not all(math.isfinite(item) for item in result):
        raise ValueError(f"{label} must contain {length} finite values")
    return result


@dataclass(frozen=True)
class IsaacLightingConfig:
    """Validated light parameters, with color expressed as linear RGB."""

    dome_intensity: float
    distant_intensity: float
    distant_angle_deg: tuple[float, float, float]
    color: tuple[float, float, float]

    @classmethod
    def from_mapping(cls, payload: Mapping[str, Any]) -> "IsaacLightingConfig":
        try:
            dome = float(payload["dome_intensity"])
            distant = float(payload["distant_intensity"])
            angles = _finite_vector(payload["distant_angle_deg"], "distant_angle_deg", 3)
            color = _finite_vector(payload["color"], "color", 3)
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"invalid Isaac lighting configuration: {exc}") from exc
        if not math.isfinite(dome) or not math.isfinite(distant) or dome < 0.0 or distant < 0.0:
            raise ValueError("light intensities must be finite and non-negative")
        if any(component < 0.0 or component > 1.0 for component in color):
            raise ValueError("light color components must be in [0, 1]")
        return cls(dome, distant, angles, color)


DEFAULT_LIGHTING = IsaacLightingConfig(
    dome_intensity=1000.0,
    distant_intensity=3000.0,
    distant_angle_deg=(45.0, -30.0, 0.0),
    color=(1.0, 1.0, 1.0),
)


def _prim_paths(root_path: str) -> tuple[str, str]:
    root = root_path.rstrip("/")
    if not root.startswith("/") or not root:
        raise ValueError("root_path must be an absolute USD path")
    return f"{root}/DomeLight", f"{root}/DistantLight"


def add_scene_lighting(
    stage: Any,
    config: IsaacLightingConfig | Mapping[str, Any],
    *,
    root_path: str = "/World/Lights",
) -> tuple[str, str]:
    """Define a dome and distant light and return their prim paths."""

    from pxr import Gf, UsdGeom, UsdLux

    light_config = config
    if not isinstance(light_config, IsaacLightingConfig):
        light_config = IsaacLightingConfig.from_mapping(config)
    dome_path, distant_path = _prim_paths(root_path)
    UsdGeom.Xform.Define(stage, root_path)
    dome = UsdLux.DomeLight.Define(stage, dome_path)
    dome.CreateIntensityAttr(light_config.dome_intensity)
    dome.CreateColorAttr(Gf.Vec3f(*light_config.color))
    distant = UsdLux.DistantLight.Define(stage, distant_path)
    distant.CreateIntensityAttr(light_config.distant_intensity)
    distant.CreateColorAttr(Gf.Vec3f(*light_config.color))
    xformable = UsdGeom.Xformable(distant.GetPrim())
    rotate = xformable.AddRotateXYZOp()
    rotate.Set(Gf.Vec3f(*light_config.distant_angle_deg))
    return dome_path, distant_path


def add_cube_prim(
    stage: Any,
    *,
    prim_path: str,
    center_m: Sequence[float],
    edge_m: float,
    color_rgba: Sequence[float] = (0.2, 0.6, 0.9, 1.0),
) -> str:
    """Define a static colliding cube suitable for PhysX contact reporting."""

    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    center = _finite_vector(center_m, "center_m", 3)
    color = _finite_vector(color_rgba, "color_rgba", 4)
    edge = float(edge_m)
    if not prim_path.startswith("/") or not math.isfinite(edge) or edge <= 0.0:
        raise ValueError("prim_path must be absolute and edge_m must be positive finite")
    if any(component < 0.0 or component > 1.0 for component in color):
        raise ValueError("color_rgba components must be in [0, 1]")
    cube = UsdGeom.Cube.Define(stage, prim_path)
    cube.CreateSizeAttr(edge)
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color[:3])])
    # UsdGeom displayOpacity is a float array primvar, not a scalar double.
    cube.CreateDisplayOpacityAttr([float(color[3])])
    xformable = UsdGeom.Xformable(cube.GetPrim())
    translate = xformable.AddTranslateOp()
    translate.Set(Gf.Vec3d(*center))
    prim = cube.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    contact = PhysxSchema.PhysxContactReportAPI.Apply(prim)
    contact.CreateThresholdAttr(0.0)
    return prim_path


def lighting_ready(stage: Any, expected_paths: Sequence[str]) -> bool:
    """Return true only when every expected lighting prim exists and is valid."""

    return bool(expected_paths) and all(
        stage.GetPrimAtPath(path).IsValid() for path in expected_paths
    )
