"""Tests for container CI tool bootstrap helpers."""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
ENSURE_SCRIPT = ROOT / "scripts" / "ensure_container_dev_tools.sh"
VERIFY_SCRIPT = ROOT / "scripts" / "run_verification.sh"


def test_ensure_container_dev_tools_script_exists_and_is_executable() -> None:
    assert ENSURE_SCRIPT.is_file()
    assert ENSURE_SCRIPT.stat().st_mode & 0o111


def test_ensure_script_installs_ruff_without_curobo_or_full_deps() -> None:
    text = ENSURE_SCRIPT.read_text(encoding="utf-8")
    assert "ruff>=" in text
    assert "nvidia-curobo" not in text
    assert "numpy" not in text
    assert "Isaac Kit" in text
    assert "Ruff only" in text or "Ruff-only" in text or "for Ruff only" in text
    assert ".venv" in text
    assert "dev-venv" in text
    assert "/tmp/spark_isaac_mycobot_v3-dev-venv-" in text
    assert "pip install" in text


def test_run_verification_auto_bootstraps_ruff_separately_from_pytest() -> None:
    text = VERIFY_SCRIPT.read_text(encoding="utf-8")
    assert "ensure_container_dev_tools.sh" in text
    assert "ensure_ci_tools" in text
    assert "resolve_ruff_python" in text
    assert "PYTEST_PYTHON" in text
    assert "SPARK_PYTEST_CACHE_DIR" in text
    assert "cache_dir=" in text
    assert "SPARK_RUFF_CACHE_DIR" in text
    assert "RUFF_CACHE_DIR" in text
    assert "./scripts/run_verification.sh ci --skip-pytest --skip-ruff --with-gpu" in text
    assert 'spark_host_exec.sh" \\\n      python3 -m pytest' not in text
    assert 'source "${ROOT}/scripts/host/env.isaac_host.sh"' in text
    assert '"${ISAACSIM_PYTHON_EXE}" -m pytest -m gpu' in text
    assert "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1" in text
    assert "SPARK_PYTEST_BASETEMP" in text
    assert "--basetemp=" in text
    assert "python3 -m ruff check ." not in text
    assert "-m ruff check ." in text
    assert "-m ruff format --check ." in text
