"""Phase 0 tests for deterministic runtime compatibility checks."""

from __future__ import annotations

import json

import pytest

from mycobot_curobo.errors import EnvironmentVerificationError
from mycobot_curobo.version_guard import (
    EnvironmentReport,
    RuntimeSnapshot,
    numeric_version_prefix,
    require_valid_environment,
    validate_snapshot,
    write_report,
)


def valid_snapshot(**overrides: object) -> RuntimeSnapshot:
    """Return a valid synthetic snapshot with selected fields replaced."""

    values: dict[str, object] = {
        "python_version": "3.12.3",
        "curobo_version": "0.8.0",
        "curobo_source_revision": "v0.8.0",
        "torch_version": "2.5.1+cu124",
        "cuda_runtime_version": "12.4",
        "cuda_available": True,
        "gpu_name": "Synthetic NVIDIA GPU",
        "selected_device": "cuda:0",
        "selected_dtype": "float32",
        "gpu_tensor_allocation_ok": True,
        "public_api_imports_ok": True,
        "legacy_public_symbols": (),
    }
    values.update(overrides)
    return RuntimeSnapshot(**values)  # type: ignore[arg-type]


@pytest.mark.parametrize(
    ("version", "expected"),
    [
        ("0.8.0", (0, 8, 0)),
        ("2.5.1+cu124", (2, 5, 1)),
        ("3.12.0-rc1", (3, 12, 0)),
    ],
)
def test_numeric_version_prefix(version: str, expected: tuple[int, ...]) -> None:
    assert numeric_version_prefix(version) == expected


@pytest.mark.parametrize("version", ["0.8", "main", "0.x.0"])
def test_numeric_version_prefix_rejects_ambiguous_versions(version: str) -> None:
    with pytest.raises(ValueError):
        numeric_version_prefix(version)


def test_valid_snapshot_passes() -> None:
    assert validate_snapshot(valid_snapshot()) == ()


@pytest.mark.parametrize(
    ("overrides", "message"),
    [
        ({"curobo_version": None}, "not installed"),
        ({"curobo_version": "0.7.7"}, "0.8.0 required"),
        ({"torch_version": "2.4.1"}, "PyTorch >=2.5"),
        ({"cuda_available": False}, "CPU planning fallback is prohibited"),
        ({"gpu_tensor_allocation_ok": False}, "tensor allocation failed"),
        ({"public_api_imports_ok": False}, "public APIs"),
        ({"legacy_public_symbols": ("MotionGen",)}, "legacy cuRobo APIs"),
    ],
)
def test_invalid_snapshot_reports_specific_reason(
    overrides: dict[str, object], message: str
) -> None:
    errors = validate_snapshot(valid_snapshot(**overrides))
    assert any(message in error for error in errors)


def test_require_valid_environment_fails_closed() -> None:
    snapshot = valid_snapshot(cuda_available=False)
    report = EnvironmentReport(
        valid=False,
        errors=validate_snapshot(snapshot),
        snapshot=snapshot,
    )
    with pytest.raises(EnvironmentVerificationError, match="CUDA is unavailable"):
        require_valid_environment(report)


def test_write_report_is_machine_readable(tmp_path) -> None:
    snapshot = valid_snapshot()
    report = EnvironmentReport(valid=True, errors=(), snapshot=snapshot)
    output = tmp_path / "reports" / "environment.json"

    write_report(report, output)

    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["valid"] is True
    assert payload["snapshot"]["curobo_version"] == "0.8.0"
    assert payload["snapshot"]["selected_device"] == "cuda:0"
