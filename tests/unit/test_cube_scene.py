import numpy as np
import pytest

from mycobot_curobo.cube_scene import (
    CubeGeometry,
    batch_sphere_cube_clearance_m,
    cube_approach_target_position,
    cube_scene_revision,
    cube_to_curobo_scene_dict,
    sphere_aabb_clearance_m,
)
from mycobot_curobo.errors import ConfigurationError


def test_cube_scene_and_approach_are_typed_and_stable() -> None:
    cube = CubeGeometry((0.1, 0.2, 0.3), 0.014, name="cube")
    assert cube_approach_target_position(
        cube.center_m, cube.edge_m, (1, 0, 0), 0.01
    ) == pytest.approx((0.117, 0.2, 0.3))
    scene = cube_to_curobo_scene_dict(cube)
    assert scene["cuboid"]["cube"]["pose"] == [0.1, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0]
    assert cube_scene_revision(cube) == cube_scene_revision(cube)


def test_sphere_aabb_clearance_handles_exterior_and_penetration() -> None:
    assert sphere_aabb_clearance_m(
        np.array([[2.0, 0.0, 0.0]]), np.array([0.25]), (0, 0, 0), (1, 1, 1)
    ) == pytest.approx(0.75)
    assert sphere_aabb_clearance_m(
        np.array([[0.0, 0.0, 0.0]]), np.array([0.1]), (0, 0, 0), (1, 1, 1)
    ) == pytest.approx(-1.1)
    clearance = batch_sphere_cube_clearance_m(
        np.array([[[2.0, 0.0, 0.0, 0.25]], [[0.0, 0.0, 0.0, 0.1]]]), (0, 0, 0), 2.0
    )
    assert clearance == pytest.approx([0.75, -1.1])


def test_cube_rejects_non_identity_orientation() -> None:
    with pytest.raises(ConfigurationError):
        CubeGeometry((0, 0, 0), 0.01, orientation_wxyz=(0, 1, 0, 0))
