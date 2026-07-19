#!/usr/bin/env bash
# Ensure lightweight container CI lint tools (Ruff) without installing
# cuRobo, CUDA PyTorch, Isaac Kit, or the full project dependency set.
#
# Prefer the project-local .venv. If the workspace is not writable (common when
# the bind-mount is owned by another UID), fall back to:
#   ${XDG_CACHE_HOME:-$HOME/.cache}/spark_isaac_mycobot_v3/dev-venv
# then /tmp/spark_isaac_mycobot_v3-dev-venv-$UID.
#
# This venv is for Ruff only. Unit tests continue to use the container/system
# Python that already provides NumPy/PyYAML. Safe to re-run.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

# Keep versions aligned with pyproject.toml [project.optional-dependencies].dev
RUFF_SPEC="${SPARK_RUFF_SPEC:-ruff>=0.4}"

can_use_dir() {
  local path="$1"
  mkdir -p "${path}" 2>/dev/null || return 1
  [[ -w "${path}" ]]
}

default_venv_dir() {
  if [[ -n "${SPARK_DEV_VENV:-}" ]]; then
    printf '%s\n' "${SPARK_DEV_VENV}"
    return 0
  fi

  local candidate
  for candidate in \
    "${ROOT}/.venv" \
    "${XDG_CACHE_HOME:-${HOME}/.cache}/spark_isaac_mycobot_v3/dev-venv" \
    "/tmp/spark_isaac_mycobot_v3-dev-venv-${UID:-0}"
  do
    if can_use_dir "${candidate}"; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done

  echo "ERROR: no writable location for a CI tooling venv." >&2
  echo "Set SPARK_DEV_VENV to a writable path and retry." >&2
  return 1
}

resolve_ruff_python() {
  local candidate
  for candidate in \
    "${SPARK_DEV_VENV:-}/bin/python" \
    "${ROOT}/.venv/bin/python" \
    "${XDG_CACHE_HOME:-${HOME}/.cache}/spark_isaac_mycobot_v3/dev-venv/bin/python" \
    "/tmp/spark_isaac_mycobot_v3-dev-venv-${UID:-0}/bin/python" \
    "python3"
  do
    if [[ -z "${candidate}" || "${candidate}" == "/bin/python" ]]; then
      continue
    fi
    if [[ "${candidate}" == "python3" ]] || [[ -x "${candidate}" ]]; then
      if "${candidate}" -m ruff --version >/dev/null 2>&1; then
        printf '%s\n' "${candidate}"
        return 0
      fi
    fi
  done
  return 1
}

VENV_DIR="$(default_venv_dir)"

if RUFF_PYTHON="$(resolve_ruff_python)"; then
  echo "=== Ruff already available via ${RUFF_PYTHON}: $(${RUFF_PYTHON} -m ruff --version) ==="
  exit 0
fi

if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
  echo "=== Creating Ruff-only CI tooling venv at ${VENV_DIR} ==="
  python3 -m venv "${VENV_DIR}"
fi

# shellcheck disable=SC1091
source "${VENV_DIR}/bin/activate"
python -m pip install --upgrade pip
echo "=== Installing container CI lint tool (${RUFF_SPEC}) ==="
python -m pip install "${RUFF_SPEC}"

if ! python -m ruff --version >/dev/null 2>&1; then
  echo "ERROR: Ruff is still unavailable after bootstrap." >&2
  exit 1
fi

echo "=== Container Ruff ready in ${VENV_DIR}: $(python -m ruff --version) ==="
echo "=== Unit tests should still use the system/container Python (NumPy/PyYAML). ==="
