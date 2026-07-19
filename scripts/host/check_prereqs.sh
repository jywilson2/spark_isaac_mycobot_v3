#!/usr/bin/env bash
# Quick host prerequisite check for Isaac Sim scene build iteration.
# Run on the DGX Spark HOST (native terminal):
#   ./scripts/host/check_prereqs.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.isaac_host.sh
source "${SCRIPT_DIR}/env.isaac_host.sh"

spark_host_require_native_shell || true
spark_host_check_prereqs

echo
echo "Optional next steps (Phase 7+):"
echo "  ./scripts/download_mycobot_ros2.sh     # vendor URDF + meshes"
echo "  ./scripts/convert_urdf_to_usd.sh       # host URDF → USD"
echo "  ./scripts/host/launch_isaac_sim.sh     # empty-stage GUI"
