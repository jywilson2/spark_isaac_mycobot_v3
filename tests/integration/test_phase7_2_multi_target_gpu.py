"""GPU acceptance coverage for Phase 7.2 multi-target planning."""

from __future__ import annotations

from pathlib import Path

import pytest

pytest.importorskip("curobo")

from mycobot_curobo.multi_target import (  # noqa: E402
    load_multi_target_suite_config,
    sample_multi_target_episodes,
)

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.gpu
def test_phase7_2_default_suite_plans_full_episode() -> None:
    from isaac_sim.plan_multi_target_suite import plan_and_validate

    config = load_multi_target_suite_config(ROOT / "config/phase7_2_multi_target.yml")
    episodes = sample_multi_target_episodes(config, root_seed=7, episode_count=1)
    results, trajectories = plan_and_validate(
        episodes,
        validation_profile_name=config.validation_profile,
        warn_planning_duration_s=config.warn_planning_duration_s,
    )
    assert len(results) == 1
    assert results[0].succeeded, results[0].failure_reason
    assert results[0].contacted_ids == results[0].episode.field.contact_order_ids
    assert trajectories
    successful_ids = {
        leg.request_id
        for leg in results[0].legs
        if leg.request_id is not None and leg.planning_succeeded and leg.validation_passed
    }
    assert set(trajectories) == successful_ids
    assert results[0].planning_failure_count >= 0
    assert results[0].episode.max_planning_failure_per_target == 5
    assert results[0].episode.max_failed_episodes == 0
