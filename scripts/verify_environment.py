#!/usr/bin/env python3
"""Verify the Phase 0 cuRobo v0.8.0 CUDA runtime and write JSON evidence."""

from __future__ import annotations

import sys

from mycobot_curobo.version_guard import main


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
