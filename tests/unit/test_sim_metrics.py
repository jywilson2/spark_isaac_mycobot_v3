import math

import pytest

from isaac_sim.sim_metrics import orientation_error_rad, tip_position_error_m


def test_tip_position_error_uses_meters() -> None:
    assert tip_position_error_m([0.0, 0.0, 0.0], [0.003, 0.004, 0.0]) == pytest.approx(0.005)


def test_orientation_error_accepts_quaternion_sign_equivalence() -> None:
    assert orientation_error_rad([1.0, 0.0, 0.0, 0.0], [-1.0, 0.0, 0.0, 0.0]) == 0.0
    assert orientation_error_rad(
        [math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4)],
        [1.0, 0.0, 0.0, 0.0],
    ) == pytest.approx(math.pi / 2)


@pytest.mark.parametrize(
    ("function", "actual", "goal"),
    [
        (tip_position_error_m, [0.0, 0.0], [0.0, 0.0, 0.0]),
        (orientation_error_rad, [0.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]),
    ],
)
def test_pose_metrics_reject_invalid_input(function, actual, goal) -> None:
    with pytest.raises(ValueError):
        function(actual, goal)
