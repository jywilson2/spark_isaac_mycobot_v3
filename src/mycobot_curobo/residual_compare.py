"""Residual-on vs residual-off comparison reports (sim-only labeling)."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Sequence

from mycobot_curobo.benchmark import BenchmarkResult, BenchmarkSummary, aggregate_results
from mycobot_curobo.errors import ConfigurationError


@dataclass(frozen=True)
class ResidualComparisonReport:
    """Paired Phase 6-style summaries with explicit residual and sim labels."""

    schema_version: int
    sim_only: bool
    residual_mode_off: str
    residual_mode_on: str
    root_seed: int
    stage: str
    off_summary: BenchmarkSummary
    on_summary: BenchmarkSummary
    off_execution_failure_count: int
    on_execution_failure_count: int


def build_residual_comparison_report(
    *,
    off_results: Sequence[BenchmarkResult],
    on_results: Sequence[BenchmarkResult],
    root_seed: int,
    stage: str,
) -> ResidualComparisonReport:
    """Aggregate residual-off and residual-on results into one labeled report."""

    if len(off_results) != len(on_results):
        raise ConfigurationError("residual comparison requires equal on/off result counts")
    off_summary = aggregate_results(off_results, root_seed=root_seed, stage=stage)
    on_summary = aggregate_results(on_results, root_seed=root_seed, stage=stage)
    return ResidualComparisonReport(
        schema_version=1,
        sim_only=True,
        residual_mode_off="off",
        residual_mode_on="on",
        root_seed=root_seed,
        stage=stage,
        off_summary=off_summary,
        on_summary=on_summary,
        off_execution_failure_count=off_summary.execution_failure_count,
        on_execution_failure_count=on_summary.execution_failure_count,
    )


def write_residual_comparison_report(
    report: ResidualComparisonReport,
    output_dir: Path | str,
) -> tuple[Path, Path]:
    """Write JSON and Markdown residual comparison reports."""

    if not report.sim_only:
        raise ConfigurationError("residual comparison reports must be labeled sim_only")
    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    stem = f"phase8_residual_compare_{report.stage}_seed_{report.root_seed}"
    json_path = destination / f"{stem}.json"
    md_path = destination / f"{stem}.md"
    payload = {
        "schema_version": report.schema_version,
        "sim_only": True,
        "residual_mode_off": report.residual_mode_off,
        "residual_mode_on": report.residual_mode_on,
        "root_seed": report.root_seed,
        "stage": report.stage,
        "off_summary": asdict(report.off_summary),
        "on_summary": asdict(report.on_summary),
        "off_execution_failure_count": report.off_execution_failure_count,
        "on_execution_failure_count": report.on_execution_failure_count,
        "label": "simulation_only_residual_comparison",
    }
    json_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    md = "\n".join(
        [
            "# Phase 8 residual-on vs residual-off (simulation only)",
            "",
            f"- sim_only: `{report.sim_only}`",
            f"- stage: `{report.stage}`",
            f"- root_seed: `{report.root_seed}`",
            f"- residual_mode_off success rate: `{report.off_summary.success_rate:.3f}`",
            f"- residual_mode_on success rate: `{report.on_summary.success_rate:.3f}`",
            f"- residual_mode_off execution failures: `{report.off_execution_failure_count}`",
            f"- residual_mode_on execution failures: `{report.on_execution_failure_count}`",
            "",
            "These metrics are simulation-only. They are not hardware accuracy claims.",
            "",
        ]
    )
    md_path.write_text(md, encoding="utf-8")
    return json_path, md_path
