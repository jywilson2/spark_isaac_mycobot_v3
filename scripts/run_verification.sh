#!/usr/bin/env bash
# Unified verification entry point — CI (remote PR) vs Spark host.
#
#   ./scripts/run_verification.sh ci       # remote GitHub PR / agent CI gate
#   ./scripts/run_verification.sh spark    # DGX Spark host gates
#   ./scripts/run_verification.sh help
#
# Until Phase 7 GUI smoke exists, spark mode runs unit tests plus optional GPU
# integration tests. GUI smoke becomes a hard push gate only after Phase 7.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

MODE="${1:-}"
shift || true

usage() {
  cat <<'EOF'
Usage: ./scripts/run_verification.sh <ci|spark|help>

  ci      Remote GitHub PR / CI verification (headless only)
            1) ensure Ruff when missing (container-safe, Ruff-only venv)
            2) pytest tests/unit -q  (system/container Python)
            3) ruff check . && ruff format --check .

  spark   DGX Spark host development
            1) ensure Ruff when missing
            2) pytest tests/unit -q
            3) ruff check . && ruff format --check .
            4) Optional: pytest -m gpu tests/integration when CUDA/cuRobo exist
            5) Phase 7 GUI smoke: required only after that player lands

Options (after mode):
  --skip-pytest    Skip the unit suite (debug only)
  --skip-ruff      Skip lint/format checks (debug only)
  --with-gpu       For spark/ci: also run GPU-marked integration tests

Examples:
  ./scripts/run_verification.sh ci
  ./scripts/run_verification.sh spark --with-gpu
EOF
}

SKIP_PYTEST=0
SKIP_RUFF=0
WITH_GPU=0
EXTRA=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --skip-pytest) SKIP_PYTEST=1; shift ;;
    --skip-ruff) SKIP_RUFF=1; shift ;;
    --with-gpu) WITH_GPU=1; shift ;;
    -h|--help|help)
      usage
      exit 0
      ;;
    *)
      EXTRA+=("$1")
      shift
      ;;
  esac
done

if [[ -z "${MODE}" || "${MODE}" == "help" || "${MODE}" == "-h" || "${MODE}" == "--help" ]]; then
  usage
  exit 0
fi

export SPARK_REPO_ROOT="${SPARK_REPO_ROOT:-${ROOT}}"
export PYTHONPATH="${ROOT}/src:${ROOT}${PYTHONPATH:+:${PYTHONPATH}}"

# Prefer the system/container interpreter for pytest so a Ruff-only bootstrap
# venv does not hide NumPy/PyYAML or pull in broken ROS pytest plugins.
PYTEST_PYTHON="${SPARK_PYTEST_PYTHON:-/usr/bin/python3}"
if [[ ! -x "${PYTEST_PYTHON}" ]]; then
  PYTEST_PYTHON="python3"
fi

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

RUFF_PYTHON=""

ensure_ci_tools() {
  if [[ "${SKIP_RUFF}" -eq 1 ]]; then
    return 0
  fi
  if RUFF_PYTHON="$(resolve_ruff_python)"; then
    return 0
  fi
  echo "=== Bootstrapping container CI lint tool (Ruff) ==="
  bash "${ROOT}/scripts/ensure_container_dev_tools.sh"
  if ! RUFF_PYTHON="$(resolve_ruff_python)"; then
    echo "ERROR: Ruff unavailable after bootstrap." >&2
    exit 1
  fi
}

run_pytest_unit() {
  if [[ "${SKIP_PYTEST}" -eq 1 ]]; then
    echo "=== Skipping pytest (--skip-pytest) ==="
    return 0
  fi
  local cache_dir="${SPARK_PYTEST_CACHE_DIR:-/tmp/spark_isaac_mycobot_v3-pytest-${UID:-0}}"
  mkdir -p "${cache_dir}"
  echo "=== Unit tests (pytest tests/unit via ${PYTEST_PYTHON}) ==="
  "${PYTEST_PYTHON}" -m pytest tests/unit -q \
    -o "cache_dir=${cache_dir}" \
    "${EXTRA[@]+"${EXTRA[@]}"}"
}

run_ruff() {
  if [[ "${SKIP_RUFF}" -eq 1 ]]; then
    echo "=== Skipping ruff (--skip-ruff) ==="
    return 0
  fi
  export RUFF_CACHE_DIR="${SPARK_RUFF_CACHE_DIR:-/tmp/spark_isaac_mycobot_v3-ruff-${UID:-0}}"
  mkdir -p "${RUFF_CACHE_DIR}"
  if [[ -z "${RUFF_PYTHON}" ]]; then
    RUFF_PYTHON="$(resolve_ruff_python)"
  fi
  echo "=== Ruff lint/format (via ${RUFF_PYTHON}) ==="
  "${RUFF_PYTHON}" -m ruff check .
  "${RUFF_PYTHON}" -m ruff format --check .
}

run_gpu_integration() {
  echo "=== GPU integration tests ==="
  if [[ -f /.dockerenv ]]; then
    bash "${ROOT}/scripts/host/spark_host_exec.sh" \
      ./scripts/run_verification.sh ci --skip-pytest --skip-ruff --with-gpu
  else
    # GPU tests use the same Isaac Sim Python environment that carries the
    # pinned cuRobo/CUDA stack; host system Python may legitimately lack it.
    # shellcheck source=host/env.isaac_host.sh
    source "${ROOT}/scripts/host/env.isaac_host.sh"
    spark_host_apply_env
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
      "${ISAACSIM_PYTHON_EXE}" -m pytest -m gpu tests/integration -q
  fi
}

case "${MODE}" in
  ci)
    echo "############################################"
    echo "# Verification mode: CI / remote GitHub PR #"
    echo "############################################"
    ensure_ci_tools
    run_pytest_unit
    run_ruff
    if [[ "${WITH_GPU}" -eq 1 ]]; then
      run_gpu_integration
    else
      echo "NOTE: GPU integration not run (pass --with-gpu when CUDA/cuRobo available)."
    fi
    echo "=== CI verification PASSED ==="
    ;;
  spark)
    echo "############################################"
    echo "# Verification mode: DGX Spark host        #"
    echo "############################################"
    ensure_ci_tools
    run_pytest_unit
    run_ruff
    if [[ "${WITH_GPU}" -eq 1 || "${SPARK_RUN_GPU_TESTS:-0}" == "1" ]]; then
      run_gpu_integration
    else
      echo "NOTE: GPU integration not run (pass --with-gpu or SPARK_RUN_GPU_TESTS=1)."
    fi
    if [[ -x "${ROOT}/scripts/host/smoke_isaac_viz.sh" ]]; then
      if grep -q 'PHASE7_NOT_IMPLEMENTED' "${ROOT}/scripts/host/smoke_isaac_viz.sh"; then
        echo "NOTE: Phase 7 GUI smoke stub present; GUI push gate not active yet."
      fi
    fi
    echo "=== Spark verification PASSED ==="
    ;;
  *)
    echo "ERROR: unknown mode '${MODE}'" >&2
    usage >&2
    exit 2
    ;;
esac
