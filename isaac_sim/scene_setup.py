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

# Kit viewport lighting menubar (omni.kit.viewport.menubar.lighting).
_AUTO_LIGHT_RIG_ENABLED = (
    "/persistent/exts/omni.kit.viewport.menubar.lighting/autoLightRig/enabled"
)
_LIGHTING_NOTIFICATION_DURATION = "/exts/omni.kit.viewport.menubar.lighting/notificationDuration"
_RTX_VIEW_LIGHTING_MODE = "/rtx/useViewLightingMode"
_LIGHTING_MODE_PREFIX = "/exts/omni.kit.viewport.menubar.lighting/lightingMode/"


def _prim_paths(root_path: str) -> tuple[str, str]:
    root = root_path.rstrip("/")
    if not root.startswith("/") or not root:
        raise ValueError("root_path must be an absolute USD path")
    return f"{root}/DomeLight", f"{root}/DistantLight"


def configure_kit_for_stage_lighting() -> bool:
    """Disable Kit auto light-rig before opening a USD that has no lights yet.

    Isaac Sim's viewport lighting menubar defaults ``autoLightRig.enabled`` to
    true. Opening the prepared robot USD (no ``LightAPI`` prims) then posts
    ``No lights found in stage, applying lighting: 'Default'`` and applies a
    light rig that **hides** later UsdLux prims. Call this after
    ``SimulationApp`` construction and **before** ``open_stage``.
    """

    try:
        import carb
    except ImportError:
        return False
    settings = carb.settings.get_settings()
    settings.set(_AUTO_LIGHT_RIG_ENABLED, False)
    settings.set(_LIGHTING_NOTIFICATION_DURATION, 0)
    settings.set(_RTX_VIEW_LIGHTING_MODE, False)
    return True


def add_scene_lighting(
    stage: Any,
    config: IsaacLightingConfig | Mapping[str, Any],
    *,
    root_path: str = "/World/Lights",
) -> tuple[str, str]:
    """Define a dome and distant light and return their prim paths.

    Safe to call more than once on the same stage: existing light prims are
    updated in place without stacking duplicate ``xformOp:rotateXYZ`` entries.
    """

    from pxr import Gf, UsdGeom, UsdLux

    light_config = config
    if not isinstance(light_config, IsaacLightingConfig):
        light_config = IsaacLightingConfig.from_mapping(config)
    dome_path, distant_path = _prim_paths(root_path)
    UsdGeom.Xform.Define(stage, root_path)
    dome = UsdLux.DomeLight.Define(stage, dome_path)
    intensity = dome.GetIntensityAttr()
    if intensity:
        intensity.Set(light_config.dome_intensity)
    else:
        dome.CreateIntensityAttr(light_config.dome_intensity)
    color_attr = dome.GetColorAttr()
    if color_attr:
        color_attr.Set(Gf.Vec3f(*light_config.color))
    else:
        dome.CreateColorAttr(Gf.Vec3f(*light_config.color))
    # Align dome with the stage up-axis so RTX does not treat it as inverted.
    if hasattr(dome, "OrientToStageUpAxis"):
        dome.OrientToStageUpAxis()
    distant = UsdLux.DistantLight.Define(stage, distant_path)
    intensity = distant.GetIntensityAttr()
    if intensity:
        intensity.Set(light_config.distant_intensity)
    else:
        distant.CreateIntensityAttr(light_config.distant_intensity)
    color_attr = distant.GetColorAttr()
    if color_attr:
        color_attr.Set(Gf.Vec3f(*light_config.color))
    else:
        distant.CreateColorAttr(Gf.Vec3f(*light_config.color))
    xformable = UsdGeom.Xformable(distant.GetPrim())
    rotate_attr = distant.GetPrim().GetAttribute("xformOp:rotateXYZ")
    if rotate_attr and rotate_attr.IsValid():
        rotate_attr.Set(Gf.Vec3f(*light_config.distant_angle_deg))
    else:
        rotate = xformable.AddRotateXYZOp()
        rotate.Set(Gf.Vec3f(*light_config.distant_angle_deg))
    return dome_path, distant_path


def enable_viewport_stage_lighting() -> bool:
    """Switch Kit/RTX from camera/rig lighting to stage UsdLux lights.

    Isaac Sim's viewport lighting menu defaults can leave stage lights hidden
    (camera light or a light rig). Creating UsdLux prims alone is not enough:
    ``SetLightingMenuModeCommand(lighting_mode="stage")`` makes LightAPI prims
    visible and clears ``/rtx/useViewLightingMode``. Prefer the Kit command
    (explicit UsdContext) over the menubar action, which needs an active
    viewport and can no-op early during stage settle.
    """

    try:
        import carb
    except ImportError:
        return False

    settings = carb.settings.get_settings()
    settings.set(_RTX_VIEW_LIGHTING_MODE, False)
    try:
        import omni.kit.commands

        success, _result = omni.kit.commands.execute(
            "SetLightingMenuModeCommand",
            lighting_mode="stage",
            usd_context_name="",
        )
        if success:
            return True
    except (ImportError, AttributeError, RuntimeError, TypeError):
        pass
    try:
        import omni.kit.actions.core

        action = omni.kit.actions.core.get_action_registry().get_action(
            "omni.kit.viewport.menubar.lighting",
            "set_lighting_mode_stage",
        )
        if action is not None:
            action.execute()
        # RTX flag above is enough when the menubar action is absent (headless).
        return True
    except (ImportError, AttributeError, RuntimeError):
        # Headless Kit without the menubar extension still benefits from the
        # RTX setting above; treat action failure as soft when that was set.
        return True


def prepare_illuminated_stage(
    stage: Any,
    config: IsaacLightingConfig | Mapping[str, Any],
    *,
    root_path: str = "/World/Lights",
) -> tuple[tuple[str, str], bool]:
    """Create lights, force stage-lighting mode, and report prim readiness."""

    paths = add_scene_lighting(stage, config, root_path=root_path)
    enable_viewport_stage_lighting()
    return paths, lighting_ready(stage, paths)


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


def stage_lighting_mode_active() -> bool:
    """True when RTX view lighting is off and the menubar mode is stage.

    Menubar ``lightingMode`` stores ``""`` for stage lighting, ``camera`` /
    ``off`` for those modes, or a light-rig name. ``useViewLightingMode`` alone
    is insufficient: a Default light rig can leave that flag false while still
    hiding stage ``LightAPI`` prims.
    """

    try:
        import carb
    except ImportError:
        return False
    settings = carb.settings.get_settings()
    if settings.get(_RTX_VIEW_LIGHTING_MODE) is not False:
        return False
    try:
        import omni.usd
        from pxr import UsdUtils
    except ImportError:
        return True
    context = omni.usd.get_context()
    stage = context.get_stage() if context is not None else None
    if stage is None:
        return True
    stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
    mode = settings.get(f"{_LIGHTING_MODE_PREFIX}{stage_id}") or ""
    return mode in ("", "stage")
