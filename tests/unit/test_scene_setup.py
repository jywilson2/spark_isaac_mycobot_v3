import pytest

from isaac_sim.scene_setup import (
    DEFAULT_LIGHTING,
    IsaacLightingConfig,
    _prim_paths,
    configure_kit_for_stage_lighting,
    enable_viewport_stage_lighting,
)


def test_lighting_config_validates_cube_suite_defaults() -> None:
    config = IsaacLightingConfig.from_mapping(
        {
            "dome_intensity": 1000.0,
            "distant_intensity": 3000.0,
            "distant_angle_deg": [45.0, -30.0, 0.0],
            "color": [1.0, 1.0, 1.0],
        }
    )
    assert config == DEFAULT_LIGHTING
    assert _prim_paths("/World/Lights") == (
        "/World/Lights/DomeLight",
        "/World/Lights/DistantLight",
    )


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
        TIP_CONTACT_COLOR_RGBA,
    )

    for color in (TIP_CONTACT_COLOR_RGBA, BODY_CONTACT_COLOR_RGBA, DEFAULT_TARGET_COLOR_RGBA):
        assert len(color) == 4
        assert all(0.0 <= component <= 1.0 for component in color)
