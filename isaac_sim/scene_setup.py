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
    dome_intensity=400.0,
    distant_intensity=1000.0,
    distant_angle_deg=(45.0, -30.0, 0.0),
    color=(1.0, 1.0, 1.0),
)

# Closer than Kit's default far perspective: look from +Y at the arm / field.
DEFAULT_VIEWPORT_EYE_M = (0.28, 0.55, 0.32)
DEFAULT_VIEWPORT_TARGET_M = (0.14, -0.08, 0.14)
# Conservative MyCobot reach envelope in g_base for content-aware framing.
DEFAULT_ARM_ENVELOPE_MIN_M = (-0.05, -0.12, 0.0)
DEFAULT_ARM_ENVELOPE_MAX_M = (0.32, 0.12, 0.36)
DEFAULT_VIEWPORT_VERTICAL_FOV_DEG = 35.0
DEFAULT_VIEWPORT_ASPECT = 16.0 / 9.0
DEFAULT_VIEWPORT_MARGIN = 0.12

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


def union_aabb_m(
    first_min_m: Sequence[float],
    first_max_m: Sequence[float],
    second_min_m: Sequence[float],
    second_max_m: Sequence[float],
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Return the axis-aligned union of two AABBs in metres."""

    a_min = _finite_vector(first_min_m, "first_min_m", 3)
    a_max = _finite_vector(first_max_m, "first_max_m", 3)
    b_min = _finite_vector(second_min_m, "second_min_m", 3)
    b_max = _finite_vector(second_max_m, "second_max_m", 3)
    if any(a_max[i] < a_min[i] or b_max[i] < b_min[i] for i in range(3)):
        raise ValueError("AABB maximum_m must be >= minimum_m on each axis")
    return (
        (min(a_min[0], b_min[0]), min(a_min[1], b_min[1]), min(a_min[2], b_min[2])),
        (max(a_max[0], b_max[0]), max(a_max[1], b_max[1]), max(a_max[2], b_max[2])),
    )


def content_aabb_from_field(
    field_minimum_m: Sequence[float],
    field_maximum_m: Sequence[float],
    *,
    target_edge_m: float,
    arm_minimum_m: Sequence[float] = DEFAULT_ARM_ENVELOPE_MIN_M,
    arm_maximum_m: Sequence[float] = DEFAULT_ARM_ENVELOPE_MAX_M,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Union of field cubes (expanded by half-edge) with a fixed arm envelope."""

    field_min = _finite_vector(field_minimum_m, "field_minimum_m", 3)
    field_max = _finite_vector(field_maximum_m, "field_maximum_m", 3)
    edge = float(target_edge_m)
    if not math.isfinite(edge) or edge <= 0.0:
        raise ValueError("target_edge_m must be positive finite")
    half = 0.5 * edge
    expanded_min = (field_min[0] - half, field_min[1] - half, field_min[2] - half)
    expanded_max = (field_max[0] + half, field_max[1] + half, field_max[2] + half)
    return union_aabb_m(expanded_min, expanded_max, arm_minimum_m, arm_maximum_m)


def compute_viewport_framing(
    content_minimum_m: Sequence[float],
    content_maximum_m: Sequence[float],
    *,
    vertical_fov_deg: float = DEFAULT_VIEWPORT_VERTICAL_FOV_DEG,
    aspect: float = DEFAULT_VIEWPORT_ASPECT,
    margin: float = DEFAULT_VIEWPORT_MARGIN,
    fallback_eye_m: Sequence[float] = DEFAULT_VIEWPORT_EYE_M,
    fallback_target_m: Sequence[float] = DEFAULT_VIEWPORT_TARGET_M,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Closest eye/target on the default view ray that keeps content in frame.

    Primary metric: maximize closeness subject to arm + all targets remaining
    in view (approx. perspective FOV with ``margin`` padding). Degenerate or
    non-finite bounds fall back to ``fallback_*``.
    """

    fallback_eye = _finite_vector(fallback_eye_m, "fallback_eye_m", 3)
    fallback_target = _finite_vector(fallback_target_m, "fallback_target_m", 3)
    try:
        c_min = _finite_vector(content_minimum_m, "content_minimum_m", 3)
        c_max = _finite_vector(content_maximum_m, "content_maximum_m", 3)
    except ValueError:
        return fallback_eye, fallback_target
    if any(c_max[i] < c_min[i] for i in range(3)):
        return fallback_eye, fallback_target
    span = tuple(c_max[i] - c_min[i] for i in range(3))
    if max(span) <= 1.0e-9:
        return fallback_eye, fallback_target
    if (
        not math.isfinite(vertical_fov_deg)
        or vertical_fov_deg <= 0.0
        or vertical_fov_deg >= 179.0
        or not math.isfinite(aspect)
        or aspect <= 0.0
        or not math.isfinite(margin)
        or margin < 0.0
    ):
        return fallback_eye, fallback_target

    target = (
        0.5 * (c_min[0] + c_max[0]),
        0.5 * (c_min[1] + c_max[1]),
        0.5 * (c_min[2] + c_max[2]),
    )
    view = tuple(fallback_eye[i] - fallback_target[i] for i in range(3))
    view_norm = math.sqrt(sum(component * component for component in view))
    if view_norm <= 1.0e-12:
        return fallback_eye, fallback_target
    forward = tuple(component / view_norm for component in view)
    # Camera "up" preference; fall back if nearly parallel to the view ray.
    world_up = (0.0, 0.0, 1.0)
    up_dot = sum(forward[i] * world_up[i] for i in range(3))
    up_proj = tuple(world_up[i] - up_dot * forward[i] for i in range(3))
    up_norm = math.sqrt(sum(component * component for component in up_proj))
    if up_norm <= 1.0e-9:
        world_up = (0.0, 1.0, 0.0)
        up_dot = sum(forward[i] * world_up[i] for i in range(3))
        up_proj = tuple(world_up[i] - up_dot * forward[i] for i in range(3))
        up_norm = math.sqrt(sum(component * component for component in up_proj))
        if up_norm <= 1.0e-9:
            return fallback_eye, fallback_target
    up = tuple(component / up_norm for component in up_proj)
    right = (
        forward[1] * up[2] - forward[2] * up[1],
        forward[2] * up[0] - forward[0] * up[2],
        forward[0] * up[1] - forward[1] * up[0],
    )
    right_norm = math.sqrt(sum(component * component for component in right))
    if right_norm <= 1.0e-12:
        return fallback_eye, fallback_target
    right = tuple(component / right_norm for component in right)

    corners: list[tuple[float, float, float]] = []
    for ix in (0, 1):
        for iy in (0, 1):
            for iz in (0, 1):
                corners.append(
                    (
                        c_min[0] if ix == 0 else c_max[0],
                        c_min[1] if iy == 0 else c_max[1],
                        c_min[2] if iz == 0 else c_max[2],
                    )
                )
    max_right = 0.0
    max_up = 0.0
    for corner in corners:
        delta = tuple(corner[i] - target[i] for i in range(3))
        max_right = max(max_right, abs(sum(delta[i] * right[i] for i in range(3))))
        max_up = max(max_up, abs(sum(delta[i] * up[i] for i in range(3))))
    half_vfov = math.radians(vertical_fov_deg) * 0.5
    half_hfov = math.atan(math.tan(half_vfov) * aspect)
    # Distance so the padded half-extents fit in both FOV axes.
    pad = 1.0 + margin
    dist_v = (max_up * pad) / math.tan(half_vfov) if max_up > 0.0 else 0.0
    dist_h = (max_right * pad) / math.tan(half_hfov) if max_right > 0.0 else 0.0
    distance = max(dist_v, dist_h, 0.25)
    eye = tuple(target[i] + forward[i] * distance for i in range(3))
    return (
        (float(eye[0]), float(eye[1]), float(eye[2])),
        (float(target[0]), float(target[1]), float(target[2])),
    )


def frame_viewport_on_arm(
    *,
    eye_m: Sequence[float] = DEFAULT_VIEWPORT_EYE_M,
    target_m: Sequence[float] = DEFAULT_VIEWPORT_TARGET_M,
) -> bool:
    """Zoom the active perspective camera onto the arm / target field.

    Kit's default perspective is often too far for MyCobot + 14 mm cubes.
    Fail closed (return False) when Kit viewport helpers are unavailable
    (unit tests / headless without viewport utility).
    """

    eye = _finite_vector(eye_m, "eye_m", 3)
    target = _finite_vector(target_m, "target_m", 3)
    try:
        from isaacsim.core.utils.viewports import set_camera_view
    except ImportError:
        try:
            from omni.isaac.core.utils.viewports import set_camera_view  # type: ignore
        except ImportError:
            return False
    try:
        set_camera_view(eye=eye, target=target)
        return True
    except (RuntimeError, AttributeError, TypeError, ValueError):
        return False


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


TIP_CONTACT_COLOR_RGBA = (0.15, 0.85, 0.25, 1.0)
BODY_CONTACT_COLOR_RGBA = (0.9, 0.15, 0.1, 1.0)
TIP_CONTACT_FAILED_COLOR_RGBA = BODY_CONTACT_COLOR_RGBA
PENDING_CONTACT_COLOR_RGBA = (1.0, 0.85, 0.0, 1.0)
DEFAULT_TARGET_COLOR_RGBA = (0.2, 0.6, 0.9, 1.0)
# Bright red digits for viewport contrast on blue / yellow / green cubes.
LABEL_COLOR_RGBA = (1.0, 0.12, 0.08, 1.0)

# Seven-segment masks for digits 0-9 (a,b,c,d,e,f,g).
_DIGIT_SEGMENTS: dict[str, tuple[str, ...]] = {
    "0": ("a", "b", "c", "d", "e", "f"),
    "1": ("b", "c"),
    "2": ("a", "b", "g", "e", "d"),
    "3": ("a", "b", "g", "c", "d"),
    "4": ("f", "g", "b", "c"),
    "5": ("a", "f", "g", "c", "d"),
    "6": ("a", "f", "g", "e", "c", "d"),
    "7": ("a", "b", "c"),
    "8": ("a", "b", "c", "d", "e", "f", "g"),
    "9": ("a", "b", "c", "d", "f", "g"),
}


def label_digit_segment_boxes(
    target_id: str,
    *,
    digit_height_m: float = 0.022,
    digit_width_m: float = 0.014,
    stroke_m: float = 0.003,
    digit_gap_m: float = 0.004,
    depth_m: float = 0.003,
) -> tuple[tuple[tuple[float, float, float], tuple[float, float, float]], ...]:
    """Return local (center_xyz_m, size_xyz_m) boxes for viewport digit labels.

    Layout is a classic 7-segment glyph per digit in the local XZ plane
    (Z up, X across digits, Y thin depth) so labels stand upright above
    cubes. Multi-digit ids are laid out left to right. Pure geometry helper
    — no USD/Kit dependency.
    """

    if not target_id.strip():
        raise ValueError("target_id must be non-empty")
    if any(
        not math.isfinite(value) or value <= 0.0
        for value in (digit_height_m, digit_width_m, stroke_m, digit_gap_m, depth_m)
    ):
        raise ValueError("digit geometry sizes must be positive finite")
    digits = tuple(character for character in target_id.strip() if character.isdigit())
    if not digits:
        raise ValueError("target_id must contain at least one digit for a visible label")
    pitch = digit_width_m + digit_gap_m
    origin_x = -0.5 * (len(digits) - 1) * pitch
    half_w = 0.5 * digit_width_m
    half_h = 0.5 * digit_height_m
    boxes: list[tuple[tuple[float, float, float], tuple[float, float, float]]] = []
    # Per-digit face in XZ: (local_x, local_z), (size_x, size_z); Y is depth.
    segment_local = {
        "a": ((0.0, half_h), (digit_width_m, stroke_m)),
        "b": ((half_w, 0.5 * half_h), (stroke_m, half_h)),
        "c": ((half_w, -0.5 * half_h), (stroke_m, half_h)),
        "d": ((0.0, -half_h), (digit_width_m, stroke_m)),
        "e": ((-half_w, -0.5 * half_h), (stroke_m, half_h)),
        "f": ((-half_w, 0.5 * half_h), (stroke_m, half_h)),
        "g": ((0.0, 0.0), (digit_width_m, stroke_m)),
    }
    for index, digit in enumerate(digits):
        segments = _DIGIT_SEGMENTS.get(digit)
        if segments is None:
            raise ValueError(f"unsupported digit {digit!r} in target_id")
        digit_x = origin_x + index * pitch
        for name in segments:
            (local_x, local_z), (size_x, size_z) = segment_local[name]
            boxes.append(
                (
                    (float(digit_x + local_x), 0.0, float(local_z)),
                    (float(size_x), float(depth_m), float(size_z)),
                )
            )
    return tuple(boxes)


def set_cube_color(stage: Any, prim_path: str, color_rgba: Sequence[float]) -> None:
    """Recolor an existing cube prim for tip/body contact feedback."""

    from pxr import Gf, UsdGeom

    color = _finite_vector(color_rgba, "color_rgba", 4)
    if any(component < 0.0 or component > 1.0 for component in color):
        raise ValueError("color_rgba components must be in [0, 1]")
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise ValueError(f"cube prim not found: {prim_path}")
    cube = UsdGeom.Cube(prim)
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color[:3])])
    cube.CreateDisplayOpacityAttr([float(color[3])])


def remove_prim(stage: Any, prim_path: str) -> None:
    """Remove a prim from the stage if it exists."""

    prim = stage.GetPrimAtPath(prim_path)
    if prim.IsValid():
        stage.RemovePrim(prim_path)


def _add_visual_box(
    stage: Any,
    *,
    prim_path: str,
    center_m: Sequence[float],
    size_m: Sequence[float],
    color_rgba: Sequence[float],
) -> str:
    """Define a non-colliding display cube (label stroke geometry)."""

    from pxr import Gf, UsdGeom

    center = _finite_vector(center_m, "center_m", 3)
    size = _finite_vector(size_m, "size_m", 3)
    color = _finite_vector(color_rgba, "color_rgba", 4)
    if any(component <= 0.0 for component in size):
        raise ValueError("size_m components must be positive")
    if any(component < 0.0 or component > 1.0 for component in color):
        raise ValueError("color_rgba components must be in [0, 1]")
    # UsdGeom.Cube size is a single edge; scale non-uniformly via xform scale.
    edge = max(size)
    cube = UsdGeom.Cube.Define(stage, prim_path)
    cube.CreateSizeAttr(edge)
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color[:3])])
    cube.CreateDisplayOpacityAttr([float(color[3])])
    xformable = UsdGeom.Xformable(cube.GetPrim())
    translate = xformable.AddTranslateOp()
    translate.Set(Gf.Vec3d(*center))
    scale = xformable.AddScaleOp()
    scale.Set(Gf.Vec3f(size[0] / edge, size[1] / edge, size[2] / edge))
    return prim_path


def label_parent_local_offset_m(height_offset_m: float = 0.03) -> tuple[float, float, float]:
    """Return the label Xform translate in the parent cube's local frame.

    Labels are parented under the translated cube prim, so only a local Z lift
    is applied. Using world ``center_m`` here would double-count the parent
    translate and place digits far from the block.
    """

    offset = float(height_offset_m)
    if not math.isfinite(offset):
        raise ValueError("height_offset_m must be finite")
    return (0.0, 0.0, offset)


def add_target_label(
    stage: Any,
    *,
    prim_path: str,
    target_id: str,
    center_m: Sequence[float],
    height_offset_m: float = 0.03,
    color_rgba: Sequence[float] = LABEL_COLOR_RGBA,
) -> str:
    """Add a viewport-visible numbered label above a target.

    Creates high-contrast 7-segment digit geometry (non-colliding cubes) parented
    under the target prim, plus ``target_id`` custom data for log cross-checks.
    The label Xform uses a parent-local Z offset only; ``center_m`` is validated
    for call-site consistency but must not be applied as a second world translate.
    """

    from pxr import Gf, UsdGeom

    _finite_vector(center_m, "center_m", 3)
    if not prim_path.startswith("/") or not target_id.strip():
        raise ValueError("prim_path must be absolute and target_id non-empty")
    label_path = f"{prim_path.rstrip('/')}/label_{target_id}"
    xform = UsdGeom.Xform.Define(stage, label_path)
    xformable = UsdGeom.Xformable(xform.GetPrim())
    translate = xformable.AddTranslateOp()
    local_offset = label_parent_local_offset_m(height_offset_m)
    translate.Set(Gf.Vec3d(*local_offset))
    # Digits are authored in local XZ with +Y as the glyph face. Rotate 180°
    # about Z so they are right-reading from the default +Y viewport camera.
    rotate_z = xformable.AddRotateZOp()
    rotate_z.Set(180.0)
    xform.GetPrim().SetCustomDataByKey("target_id", str(target_id))
    for index, (local_center, size) in enumerate(label_digit_segment_boxes(target_id)):
        _add_visual_box(
            stage,
            prim_path=f"{label_path}/seg_{index}",
            center_m=local_center,
            size_m=size,
            color_rgba=color_rgba,
        )
    return label_path
