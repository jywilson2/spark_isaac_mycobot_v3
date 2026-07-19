# Copyright 2026 spark_isaac_mycobot_v3 contributors
"""Detect Isaac Lab / Omniverse imports when run via ``isaaclab.sh -p``.

Phase 8 prerequisite check. Not used by Phases 0–6 core planning.
"""

from __future__ import annotations


def main() -> int:
    try:
        import isaaclab  # noqa: F401

        print("isaaclab: OK")
    except ImportError as exc:
        print(f"isaaclab: MISSING ({exc})")
        return 1
    try:
        import omni  # noqa: F401

        print("omni: OK")
    except ImportError as exc:
        print(f"omni: optional/missing ({exc})")
    print("detect_isaac_lab: success")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
