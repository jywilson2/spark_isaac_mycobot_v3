"""Verify the pinned cuRoboV2 GPU runtime before planning code is imported.

Phase 0 intentionally does not construct a planner. The guard validates package
versions, public cuRobo v0.8.0 imports, CUDA visibility, and a real GPU tensor
allocation. It emits a machine-readable report even when verification fails so
environment defects are actionable and reproducible.
"""

from __future__ import annotations

import argparse
import importlib.metadata
import importlib.util
import json
import platform
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Sequence

from mycobot_curobo.errors import EnvironmentVerificationError

REQUIRED_CUROBO_VERSION = "0.8.0"
MINIMUM_TORCH_VERSION = (2, 5)
LEGACY_PUBLIC_SYMBOLS = (
    "MotionGen",
    "MotionGenConfig",
    "MotionGenPlanConfig",
    "PoseCostMetric",
)


def numeric_version_prefix(version: str, *, components: int = 3) -> tuple[int, ...]:
    """Return the leading numeric version components.

    The function is deliberately dependency-free. It accepts local build
    suffixes such as ``2.5.1+cu128`` but rejects missing or non-numeric leading
    components rather than guessing compatibility.
    """

    core = str(version).split("+", maxsplit=1)[0].split("-", maxsplit=1)[0]
    pieces = core.split(".")
    if len(pieces) < components:
        raise ValueError(f"version has fewer than {components} components: {version!r}")
    try:
        return tuple(int(piece) for piece in pieces[:components])
    except ValueError as exc:
        raise ValueError(f"version has a non-numeric component: {version!r}") from exc


def curobo_source_revision() -> str:
    """Return the installed distribution's requested Git revision or commit."""

    try:
        distribution = importlib.metadata.distribution("nvidia-curobo")
        direct_url_text = distribution.read_text("direct_url.json")
        if not direct_url_text:
            return "not_available"
        direct_url = json.loads(direct_url_text)
        vcs_info = direct_url.get("vcs_info", {})
        return str(
            vcs_info.get("commit_id") or vcs_info.get("requested_revision") or "not_available"
        )
    except (importlib.metadata.PackageNotFoundError, json.JSONDecodeError):
        return "not_available"


@dataclass(frozen=True)
class RuntimeSnapshot:
    """Observed Phase 0 runtime facts; versions are strings as reported upstream."""

    python_version: str
    curobo_version: str | None
    curobo_source_revision: str
    torch_version: str | None
    cuda_runtime_version: str | None
    cuda_available: bool
    gpu_name: str | None
    selected_device: str
    selected_dtype: str
    gpu_tensor_allocation_ok: bool
    public_api_imports_ok: bool
    legacy_public_symbols: tuple[str, ...]


@dataclass(frozen=True)
class EnvironmentReport:
    """Machine-readable verification result."""

    valid: bool
    errors: tuple[str, ...]
    snapshot: RuntimeSnapshot

    def to_dict(self) -> dict[str, Any]:
        """Convert the immutable report to JSON-compatible values."""

        return asdict(self)


def validate_snapshot(snapshot: RuntimeSnapshot) -> tuple[str, ...]:
    """Return all Phase 0 incompatibilities without hiding later failures."""

    errors: list[str] = []
    try:
        python_version = numeric_version_prefix(snapshot.python_version)
        if python_version < (3, 10, 0):
            errors.append(f"Python >=3.10 required; found {snapshot.python_version}")
    except ValueError as exc:
        errors.append(str(exc))

    if snapshot.curobo_version is None:
        errors.append("nvidia-curobo is not installed; install the exact NVlabs/curobo v0.8.0 tag")
    else:
        try:
            if numeric_version_prefix(snapshot.curobo_version) != (0, 8, 0):
                errors.append(
                    f"cuRobo {REQUIRED_CUROBO_VERSION} required; found {snapshot.curobo_version}"
                )
        except ValueError as exc:
            errors.append(str(exc))

    if snapshot.torch_version is None:
        errors.append("PyTorch >=2.5 with CUDA support is not installed")
    else:
        try:
            if numeric_version_prefix(snapshot.torch_version)[:2] < MINIMUM_TORCH_VERSION:
                errors.append(f"PyTorch >=2.5 required; found {snapshot.torch_version}")
        except ValueError as exc:
            errors.append(str(exc))

    if not snapshot.public_api_imports_ok:
        errors.append("required cuRobo v0.8.0 public APIs could not be imported")
    if snapshot.legacy_public_symbols:
        errors.append(
            "legacy cuRobo APIs exposed through required public modules: "
            + ", ".join(snapshot.legacy_public_symbols)
        )
    if not snapshot.cuda_available:
        errors.append("CUDA is unavailable to PyTorch; CPU planning fallback is prohibited")
    if not snapshot.gpu_tensor_allocation_ok:
        errors.append("CUDA tensor allocation failed")
    if not snapshot.gpu_name:
        errors.append("GPU name is unavailable")
    return tuple(errors)


def collect_runtime_snapshot() -> RuntimeSnapshot:
    """Inspect installed packages and allocate a small CUDA tensor."""

    curobo_version: str | None = None
    torch_version: str | None = None
    cuda_runtime_version: str | None = None
    cuda_available = False
    gpu_name: str | None = None
    gpu_tensor_allocation_ok = False
    public_api_imports_ok = False
    legacy_symbols: list[str] = []

    try:
        curobo_version = importlib.metadata.version("nvidia-curobo")
    except importlib.metadata.PackageNotFoundError:
        pass

    if importlib.util.find_spec("curobo") is not None:
        try:
            import curobo.motion_planner as motion_planner
            import curobo.types as curobo_types

            required_motion = ("MotionPlanner", "MotionPlannerCfg")
            required_types = ("GoalToolPose", "JointState", "Pose")
            public_api_imports_ok = all(
                hasattr(motion_planner, name) for name in required_motion
            ) and all(hasattr(curobo_types, name) for name in required_types)
            for name in LEGACY_PUBLIC_SYMBOLS:
                if hasattr(motion_planner, name) or hasattr(curobo_types, name):
                    legacy_symbols.append(name)
        except (ImportError, RuntimeError):
            public_api_imports_ok = False

    if importlib.util.find_spec("torch") is not None:
        try:
            import torch

            torch_version = str(torch.__version__)
            cuda_runtime_version = None if torch.version.cuda is None else str(torch.version.cuda)
            cuda_available = bool(torch.cuda.is_available())
            if cuda_available:
                gpu_name = str(torch.cuda.get_device_name(0))
                tensor = torch.zeros(1, device="cuda:0", dtype=torch.float32)
                gpu_tensor_allocation_ok = bool(tensor.is_cuda and tensor.numel() == 1)
        except (ImportError, RuntimeError):
            gpu_tensor_allocation_ok = False

    return RuntimeSnapshot(
        python_version=platform.python_version(),
        curobo_version=curobo_version,
        curobo_source_revision=curobo_source_revision(),
        torch_version=torch_version,
        cuda_runtime_version=cuda_runtime_version,
        cuda_available=cuda_available,
        gpu_name=gpu_name,
        selected_device="cuda:0",
        selected_dtype="float32",
        gpu_tensor_allocation_ok=gpu_tensor_allocation_ok,
        public_api_imports_ok=public_api_imports_ok,
        legacy_public_symbols=tuple(sorted(set(legacy_symbols))),
    )


def verify_environment() -> EnvironmentReport:
    """Collect and validate the runtime without constructing a planner."""

    snapshot = collect_runtime_snapshot()
    errors = validate_snapshot(snapshot)
    return EnvironmentReport(valid=not errors, errors=errors, snapshot=snapshot)


def require_valid_environment(report: EnvironmentReport) -> None:
    """Raise an actionable error when a collected runtime report is invalid."""

    if not report.valid:
        raise EnvironmentVerificationError("; ".join(report.errors))


def write_report(report: EnvironmentReport, output_path: Path) -> None:
    """Write a stable JSON report, creating only the requested parent directory."""

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(report.to_dict(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def build_argument_parser() -> argparse.ArgumentParser:
    """Build the environment-verification command-line parser."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("artifacts/reports/environment.json"),
        help="machine-readable JSON report path",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Verify the runtime and return nonzero when Phase 0 requirements fail."""

    args = build_argument_parser().parse_args(argv)
    report = verify_environment()
    write_report(report, args.output)
    print(json.dumps(report.to_dict(), indent=2, sort_keys=True))
    try:
        require_valid_environment(report)
    except EnvironmentVerificationError as exc:
        print(f"environment verification failed: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
