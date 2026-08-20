"""Kit GUI banner for identifying which playback pass is running.

Simulation-only UI helper. Safe to call when omni.ui is unavailable (returns
None). Does not affect planning, PhysX gating, or acceptance metrics.
"""

from __future__ import annotations

from typing import Any

# omni.ui font_size is approximately point size; keep the pass banner compact.
PLAYBACK_HUD_FONT_SIZE = 10
PLAYBACK_HUD_WIDTH = 560
PLAYBACK_HUD_HEIGHT = 40


def format_playback_hud_text(run_label: str) -> str:
    """Normalize operator-facing HUD text; empty label means no banner."""
    return " ".join(str(run_label).split())


def format_phase8_ab_hud_text(
    *,
    pass_id: str,
    residual_on: bool,
    mode: str,
) -> str:
    """Build an unambiguous Pass A/B banner string for Phase 8 residual smoke."""

    pass_key = format_playback_hud_text(pass_id)
    mode_key = str(mode).strip().upper() or "DEMO"
    if residual_on:
        detail = "Residual ON (tip bias corrected)"
    else:
        detail = "Residual OFF (biased tip, no correction)"
    return format_playback_hud_text(f"TEST {pass_key} of 2: {detail} [{mode_key}]")


def show_playback_hud(run_label: str) -> Any | None:
    """Show a floating Kit UI banner with ``run_label``.

    The window title and label both carry the full test name so the pass is
    readable even when the body text is small.

    Returns the ``omni.ui.Window`` handle when created, else ``None``.
    """
    text = format_playback_hud_text(run_label)
    if not text:
        return None
    try:
        import omni.ui as ui
    except ImportError:
        return None

    flags = ui.WINDOW_FLAGS_NO_SCROLLBAR | ui.WINDOW_FLAGS_NO_COLLAPSE
    no_close = getattr(ui, "WINDOW_FLAGS_NO_CLOSE", 0)
    if no_close:
        flags |= no_close
    no_resize = getattr(ui, "WINDOW_FLAGS_NO_RESIZE", 0)
    if no_resize:
        flags |= no_resize

    # Title bar repeats the test id (always visible in Kit chrome).
    window = ui.Window(
        text,
        width=PLAYBACK_HUD_WIDTH,
        height=PLAYBACK_HUD_HEIGHT,
        position_x=24,
        position_y=36,
        flags=flags,
    )
    with window.frame:
        with ui.VStack(spacing=0):
            ui.Spacer(height=2)
            ui.Label(
                text,
                height=PLAYBACK_HUD_HEIGHT - 8,
                alignment=ui.Alignment.LEFT_CENTER,
                # High-contrast white at ~10 pt for readability on dark Kit UI.
                style={"font_size": PLAYBACK_HUD_FONT_SIZE, "color": 0xFFFFFFFF},
            )
            ui.Spacer(height=2)
    window.visible = True
    return window
