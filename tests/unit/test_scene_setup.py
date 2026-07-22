import pytest

from isaac_sim.scene_setup import (
    DEFAULT_LIGHTING,
    DEFAULT_VIEWPORT_EYE_M,
    DEFAULT_VIEWPORT_TARGET_M,
    IsaacLightingConfig,
    _prim_paths,
    compute_viewport_framing,
    configure_kit_for_stage_lighting,
    content_aabb_from_field,
    enable_viewport_stage_lighting,
    frame_viewport_on_arm,
)


def test_lighting_config_validates_cube_suite_defaults() -> None:
    config = IsaacLightingConfig.from_mapping(
        {
            "dome_intensity": 400.0,
            "distant_intensity": 1000.0,
            "distant_angle_deg": [45.0, -30.0, 0.0],
            "color": [1.0, 1.0, 1.0],
        }
    )
    assert config == DEFAULT_LIGHTING
    assert _prim_paths("/World/Lights") == (
        "/World/Lights/DomeLight",
        "/World/Lights/DistantLight",
    )


def test_frame_viewport_on_arm_without_kit_returns_false() -> None:
    # Pure unit environment has no Kit viewport helpers; fail closed, not raise.
    assert frame_viewport_on_arm() is False
    assert len(DEFAULT_VIEWPORT_EYE_M) == 3
    assert len(DEFAULT_VIEWPORT_TARGET_M) == 3
    with pytest.raises(ValueError, match="eye_m"):
        frame_viewport_on_arm(eye_m=(1.0, 2.0))


def test_compute_viewport_framing_fits_content_closer_than_far_fallback() -> None:
    content_min, content_max = content_aabb_from_field(
        (-0.18, -0.18, 0.12),
        (0.18, 0.18, 0.20),
        target_edge_m=0.014,
    )
    eye, target = compute_viewport_framing(content_min, content_max)
    assert len(eye) == 3 and len(target) == 3
    # Look-at is content centre; eye stays on the default +Y/elevated view ray.
    assert target[0] == pytest.approx(0.5 * (content_min[0] + content_max[0]))
    assert target[1] == pytest.approx(0.5 * (content_min[1] + content_max[1]))
    assert target[2] == pytest.approx(0.5 * (content_min[2] + content_max[2]))
    fallback_dir = [DEFAULT_VIEWPORT_EYE_M[i] - DEFAULT_VIEWPORT_TARGET_M[i] for i in range(3)]
    eye_dir = [eye[i] - target[i] for i in range(3)]
    # Parallel (same direction) within numerical tolerance.
    cross = (
        eye_dir[1] * fallback_dir[2] - eye_dir[2] * fallback_dir[1],
        eye_dir[2] * fallback_dir[0] - eye_dir[0] * fallback_dir[2],
        eye_dir[0] * fallback_dir[1] - eye_dir[1] * fallback_dir[0],
    )
    assert max(abs(component) for component in cross) < 1.0e-6
    # Degenerate bounds fall back to defaults.
    eye_fb, target_fb = compute_viewport_framing((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert eye_fb == DEFAULT_VIEWPORT_EYE_M
    assert target_fb == DEFAULT_VIEWPORT_TARGET_M


def test_configure_kit_for_stage_lighting_without_kit_returns_false() -> None:
    # Pure unit environment has no carb/omni; helper must fail closed, not raise.
    assert configure_kit_for_stage_lighting() is False


def test_enable_viewport_stage_lighting_without_kit_returns_false() -> None:
    # Pure unit environment has no carb/omni; helper must fail closed, not raise.
    assert enable_viewport_stage_lighting() is False


def test_enable_viewport_stage_lighting_prefers_kit_command() -> None:
    import inspect

    from isaac_sim import scene_setup

    body = inspect.getsource(scene_setup.enable_viewport_stage_lighting)
    assert "SetLightingMenuModeCommand" in body
    assert "configure_kit_for_stage_lighting" in inspect.getsource(scene_setup)


def test_add_scene_lighting_doc_requires_idempotent_rotate() -> None:
    from isaac_sim import scene_setup

    source = scene_setup.add_scene_lighting.__doc__ or ""
    assert "Safe to call more than once" in source
    # Implementation must re-use existing rotateXYZ rather than AddRotateXYZOp
    # unconditionally (World.reset re-apply path).
    import inspect

    body = inspect.getsource(scene_setup.add_scene_lighting)
    assert 'GetAttribute("xformOp:rotateXYZ")' in body
    assert "AddRotateXYZOp" in body


@pytest.mark.parametrize(
    "payload",
    [
        {
            "dome_intensity": -1.0,
            "distant_intensity": 1.0,
            "distant_angle_deg": [0.0, 0.0, 0.0],
            "color": [1.0, 1.0, 1.0],
        },
        {
            "dome_intensity": 1.0,
            "distant_intensity": 1.0,
            "distant_angle_deg": [0.0, 0.0],
            "color": [1.0, 1.0, 1.0],
        },
        {
            "dome_intensity": 1.0,
            "distant_intensity": 1.0,
            "distant_angle_deg": [0.0, 0.0, 0.0],
            "color": [1.1, 1.0, 1.0],
        },
    ],
)
def test_lighting_config_rejects_invalid_values(payload: dict[str, object]) -> None:
    with pytest.raises(ValueError):
        IsaacLightingConfig.from_mapping(payload)


def test_phase7_2_color_constants_are_valid() -> None:
    from isaac_sim.scene_setup import (
        BODY_CONTACT_COLOR_RGBA,
        DEFAULT_TARGET_COLOR_RGBA,
        LABEL_COLOR_RGBA,
        PENDING_CONTACT_COLOR_RGBA,
        TIP_CONTACT_COLOR_RGBA,
        TIP_CONTACT_FAILED_COLOR_RGBA,
    )

    for color in (
        TIP_CONTACT_COLOR_RGBA,
        BODY_CONTACT_COLOR_RGBA,
        TIP_CONTACT_FAILED_COLOR_RGBA,
        PENDING_CONTACT_COLOR_RGBA,
        DEFAULT_TARGET_COLOR_RGBA,
        LABEL_COLOR_RGBA,
    ):
        assert len(color) == 4
        assert all(0.0 <= component <= 1.0 for component in color)
    assert TIP_CONTACT_FAILED_COLOR_RGBA == BODY_CONTACT_COLOR_RGBA
    assert PENDING_CONTACT_COLOR_RGBA[0] > PENDING_CONTACT_COLOR_RGBA[2]  # yellow-ish
    assert LABEL_COLOR_RGBA[0] > 0.9
    assert LABEL_COLOR_RGBA[1] < 0.3
    assert LABEL_COLOR_RGBA[2] < 0.3
