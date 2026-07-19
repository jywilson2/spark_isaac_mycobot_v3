#!/usr/bin/env bash
# Phase 7 Isaac viz smoke placeholder.
#
# V3 deliberately did not copy the prior project's IK/recovery player. This
# script exists so host helpers and docs can reference a stable path. It exits
# nonzero until a NominalPlan / ConstraintReport player is implemented.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

cat >&2 <<EOF
PHASE7_NOT_IMPLEMENTED
${ROOT}/scripts/host/smoke_isaac_viz.sh is a placeholder.

Implement a V3-native player that consumes validated NominalPlan trajectories
(see docs/implementation_phases.md Phase 7) before enabling the GUI push gate.
EOF
exit 3
