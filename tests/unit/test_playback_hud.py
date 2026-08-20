"""Unit tests for playback HUD helpers (Kit-free)."""

from __future__ import annotations

from isaac_sim.play_multi_target_suite import parse_args
from isaac_sim.playback_hud import (
    PLAYBACK_HUD_FONT_SIZE,
    format_phase8_ab_hud_text,
    format_playback_hud_text,
    show_playback_hud,
)


def test_format_playback_hud_text_normalizes_whitespace() -> None:
    assert format_playback_hud_text("  PASS A   residual OFF  ") == "PASS A residual OFF"
    assert format_playback_hud_text("") == ""
    assert format_playback_hud_text("   ") == ""


def test_playback_hud_font_is_about_ten_point() -> None:
    assert PLAYBACK_HUD_FONT_SIZE == 10


def test_phase8_ab_hud_text_names_pass_and_residual_state() -> None:
    off = format_phase8_ab_hud_text(pass_id="Pass A", residual_on=False, mode="demo")
    on = format_phase8_ab_hud_text(pass_id="Pass B", residual_on=True, mode="subtle")
    assert off.startswith("TEST Pass A of 2:")
    assert "Residual OFF" in off
    assert "biased tip" in off
    assert "[DEMO]" in off
    assert on.startswith("TEST Pass B of 2:")
    assert "Residual ON" in on
    assert "corrected" in on
    assert "[SUBTLE]" in on


def test_show_playback_hud_without_omni_ui_returns_none() -> None:
    assert show_playback_hud("") is None
    # Without Kit / omni.ui this must fail soft.
    assert show_playback_hud("PASS A — residual OFF") is None


def test_parse_args_accepts_run_label(tmp_path) -> None:
    bundle = tmp_path / "bundle.json"
    bundle.write_text("{}", encoding="utf-8")
    report = tmp_path / "report.json"
    args = parse_args(
        [
            "--bundle",
            str(bundle),
            "--output-report",
            str(report),
            "--run-label",
            "PASS B — residual ON",
        ]
    )
    assert args.run_label == "PASS B — residual ON"
