"""GPU acceptance for the deterministic Phase 6 smoke benchmark."""

from __future__ import annotations

import importlib.util
import os
import tempfile
from pathlib import Path

import pytest

from mycobot_curobo.benchmark import (
    aggregate_results,
    load_case_fixture,
    write_benchmark_reports,
)
from mycobot_curobo.cli import create_benchmark_runtime

pytestmark = pytest.mark.gpu
ROOT = Path(__file__).resolve().parents[2]


def _runtime_available() -> bool:
    if importlib.util.find_spec("curobo") is None or importlib.util.find_spec("torch") is None:
        return False
    import torch

    return bool(torch.cuda.is_available())


@pytest.mark.skipif(not _runtime_available(), reason="cuRobo v0.8.0 CUDA runtime required")
def test_phase6_smoke_writes_reports_and_repeats_deterministically() -> None:
    config, runner = create_benchmark_runtime(
        ROOT / "config/benchmark_workspace.yml",
        execute_zero_residual=False,
    )
    # Full 20-case smoke remains available via scripts/benchmark_random_targets.py.
    # The GPU gate exercises a short dual-run subset to keep host verification
    # practical under the fresh-backend + mandatory-warmup lifecycle.
    cases = load_case_fixture(ROOT / "tests/data/benchmark_smoke_cases.json")[:2]
    assert config.stage_sizes["smoke"] >= 20
    assert config.repeat_count >= 2

    results = runner.run(cases)
    summary = aggregate_results(results, root_seed=6006, stage="smoke")
    # Avoid pytest's shared /tmp/pytest-of-<user> ownership trap when host GPU
    # tests run through runuser after earlier root-owned temp directories.
    report_root = Path(
        tempfile.mkdtemp(
            prefix="phase6_smoke_",
            dir=os.environ.get("SPARK_PYTEST_BASETEMP", tempfile.gettempdir()),
        )
    )
    json_path, markdown_path = write_benchmark_reports(summary, results, report_root)

    assert len(results) == 2
    assert summary.total_cases == 2
    assert summary.repeat_disagreement_count == 0
    assert json_path.is_file()
    assert markdown_path.is_file()
    assert all(result.replay_request for result in results if not result.succeeded)
