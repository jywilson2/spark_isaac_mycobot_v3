"""Unit tests for viewport digit-label geometry (Kit-free)."""

from __future__ import annotations

from pathlib import Path

import pytest

from isaac_sim.scene_setup import (
    label_digit_segment_boxes,
    label_parent_local_offset_m,
)

ROOT = Path(__file__).parents[2]
SCENE_SETUP = ROOT / "isaac_sim" / "scene_setup.py"
WORKFLOW = ROOT / ".github" / "workflows" / "pytest.yml"


def test_label_parent_local_offset_is_z_only() -> None:
    """Labels parent under the cube: only local Z lift, never world center."""

    assert label_parent_local_offset_m(0.03) == (0.0, 0.0, 0.03)
    assert label_parent_local_offset_m(0.05) == (0.0, 0.0, 0.05)
    with pytest.raises(ValueError, match="finite"):
        label_parent_local_offset_m(float("nan"))


def test_digit_one_uses_two_vertical_segments() -> None:
    boxes = label_digit_segment_boxes("1")
    assert len(boxes) == 2
    for center, size in boxes:
        assert len(center) == 3
        assert len(size) == 3
        assert all(component > 0.0 for component in size)


def test_digit_eight_has_seven_segments() -> None:
    assert len(label_digit_segment_boxes("8")) == 7


def test_multi_digit_layout_spans_x() -> None:
    single = label_digit_segment_boxes("1")
    double = label_digit_segment_boxes("12")
    assert len(double) == len(single) + len(label_digit_segment_boxes("2"))
    xs = [center[0] for center, _ in double]
    assert min(xs) < 0.0 < max(xs)


def test_label_rejects_non_digit_id() -> None:
    with pytest.raises(ValueError, match="digit"):
        label_digit_segment_boxes("abc")


def test_add_target_label_builds_visible_segment_geometry() -> None:
    source = SCENE_SETUP.read_text(encoding="utf-8")
    assert "label_digit_segment_boxes" in source
    assert "label_parent_local_offset_m" in source
    assert "seg_" in source
    assert "_add_visual_box" in source
    assert "LABEL_COLOR_RGBA" in source
    assert "SetCustomDataByKey" in source
    # Must not re-apply world center on the child label (double-counts parent).
    assert "center[0], center[1], center[2] + height_offset_m" not in source
    assert "label_parent_local_offset_m(height_offset_m)" in source
    # Right-reading from the default +Y viewport camera.
    assert "AddRotateZOp" in source
    assert "180.0" in source


def test_github_actions_ci_uses_setup_python_for_pytest() -> None:
    workflow = WORKFLOW.read_text(encoding="utf-8")
    assert "SPARK_PYTEST_PYTHON" in workflow
    assert "pythonLocation" in workflow
    assert "--no-deps" in workflow
    assert "nvidia-curobo" in workflow  # commented rationale
    assert 'pip install -e ".[dev]"' not in workflow
