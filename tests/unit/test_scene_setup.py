import pytest

from isaac_sim.scene_setup import DEFAULT_LIGHTING, IsaacLightingConfig, _prim_paths


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
