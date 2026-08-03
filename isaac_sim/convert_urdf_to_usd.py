#!/usr/bin/env python3
# Copyright 2026 spark_isaac_mycobot_v3 contributors
"""Convert MyCobot URDF → USD using host Isaac Sim (no IK animation).

${ISAACSIM_PATH}/python.sh isaac_sim/convert_urdf_to_usd.py
./scripts/convert_urdf_to_usd.sh
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from isaac_sim.urdf_import import URDF_IMPORTER_EXTENSION, import_urdf_to_usd  # noqa: E402
from isaac_sim.urdf_utils import default_prepared_urdf, prepare_robot_assets  # noqa: E402


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="MyCobot URDF → USD via Isaac Sim")
    p.add_argument("--repo-root", type=Path, default=REPO_ROOT)
    p.add_argument("--keep-prepared", action="store_true")
    p.add_argument("--headless", action="store_true", default=True)
    p.add_argument("--no-headless", action="store_true", help="Show GUI during import")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    repo = args.repo_root.resolve()
    prepared = default_prepared_urdf(repo)
    if not (args.keep_prepared and prepared.is_file()):
        prepared = prepare_robot_assets(repo)
    out = prepared.with_suffix(".usd")

    from isaacsim import SimulationApp  # noqa: WPS433

    headless = not args.no_headless
    app = SimulationApp(
        {"headless": headless, "extra_args": ["--enable", URDF_IMPORTER_EXTENSION]}
    )
    try:
        usd = import_urdf_to_usd(prepared, out, app)
        from isaac_sim.prepared_usd_self_collision import enhance_prepared_mycobot_usd

        # Prepared tree is gitignored; re-author self-collision + contact reports
        # after every import so arm–arm PhysX contacts are monitorable.
        enhance_root = usd.parent if usd.name.endswith(".usda") else out.parent
        patches = enhance_prepared_mycobot_usd(enhance_root)
        print(f"Wrote {usd}")
        print(f"phase7_usd: self-collision enhance {patches}")
        return 0
    finally:
        app.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ImportError as exc:
        print(
            f"Run with Isaac Sim python.sh on the host.\nImportError: {exc}",
            file=sys.stderr,
        )
        raise SystemExit(1) from exc
