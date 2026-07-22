"""Unit tests for Phase 7.2 playback tip-face proximity helper."""

from __future__ import annotations

import math

import numpy as np
import pytest

from isaac_sim.play_multi_target_suite import tip_reaches_surface_m


def test_tip_reaches_surface_within_tolerance() -> None:
    tip = np.array([0.20, -0.05, 0.12])
    face = tip + np.array([0.0, 0.0, 0.014])
    assert tip_reaches_surface_m(tip, face, tolerance_m=0.015)
    assert not tip_reaches_surface_m(tip, face, tolerance_m=0.010)


def test_tip_reaches_surface_rejects_non_finite() -> None:
    tip = np.array([0.0, 0.0, 0.0])
    face = np.array([math.nan, 0.0, 0.0])
    assert not tip_reaches_surface_m(tip, face)


def test_tip_reaches_surface_rejects_negative_tolerance() -> None:
    with pytest.raises(ValueError, match="tolerance_m"):
        tip_reaches_surface_m(np.zeros(3), np.zeros(3), tolerance_m=-0.001)
