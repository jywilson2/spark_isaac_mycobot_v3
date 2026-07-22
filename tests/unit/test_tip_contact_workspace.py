"""CPU tests for tip-contact workspace sampling (no cuRobo)."""

from pathlib import Path

import pytest

from mycobot_curobo.errors import ConfigurationError
from mycobot_curobo.tip_contact_workspace import (
    TipContactSampleResult,
    build_tip_contact_sample_centers,
    default_tip_contact_workspace_config,
    serialize_tip_contact_workspace,
    summarize_tip_contact_results,
    surface_target_for_sample,
    write_tip_contact_workspace_artifact,
)


def test_default_samples_respect_rim_and_keep_out() -> None:
    config = default_tip_contact_workspace_config()
    samples = build_tip_contact_sample_centers(config)
    assert len(samples) >= 10
    keep = config.keep_outs[0]
    for sample in samples:
        x, y, z = sample.center_m
        assert (x * x + y * y) ** 0.5 + 0.5 * config.target_edge_m <= (
            config.max_target_radial_m + 1.0e-9
        )
        in_keep = (
            keep.minimum_m[0] <= x <= keep.maximum_m[0]
            and keep.minimum_m[1] <= y <= keep.maximum_m[1]
            and keep.minimum_m[2] <= z <= keep.maximum_m[2]
        )
        assert not in_keep


def test_surface_target_is_plus_z_face() -> None:
    config = default_tip_contact_workspace_config()
    samples = build_tip_contact_sample_centers(config)
    target = surface_target_for_sample(samples[0], config)
    assert target.fixed_roll_rad == pytest.approx(0.0)
    assert target.surface_normal_base[2] == pytest.approx(1.0)
    assert target.position_base_m[2] == pytest.approx(
        samples[0].center_m[2] + 0.5 * config.target_edge_m
    )


def test_serialize_and_summary(tmp_path: Path) -> None:
    config = default_tip_contact_workspace_config()
    results = (
        TipContactSampleResult(
            sample_id="s0000",
            center_m=(0.12, 0.0, 0.16),
            start_label="zeros",
            succeeded=True,
            planner_status="ok",
            failure_reason=None,
            planning_duration_s=1.0,
        ),
        TipContactSampleResult(
            sample_id="s0001",
            center_m=(0.20, 0.05, 0.16),
            start_label="zeros",
            succeeded=False,
            planner_status="fail",
            failure_reason="infeasible",
            planning_duration_s=0.5,
        ),
    )
    summary = summarize_tip_contact_results(results)
    assert summary["successes"] == 1
    assert summary["success_aabb_m"]["minimum_m"][0] == pytest.approx(0.12)
    payload = serialize_tip_contact_workspace(config=config, results=results)
    path = write_tip_contact_workspace_artifact(tmp_path / "ws.json", payload)
    assert path.is_file()
    text = path.read_text(encoding="utf-8")
    assert "measured_tip_contact_candidate_region_v1" in text


def test_empty_sampler_fails_closed() -> None:
    from dataclasses import replace

    config = replace(
        default_tip_contact_workspace_config(),
        max_target_radial_m=0.01,
        keep_outs=default_tip_contact_workspace_config().keep_outs,
        grid_step_m=0.05,
    )
    with pytest.raises(ConfigurationError, match="zero samples"):
        build_tip_contact_sample_centers(config)
