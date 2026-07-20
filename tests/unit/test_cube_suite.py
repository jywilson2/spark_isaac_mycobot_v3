from pathlib import Path

from mycobot_curobo.cube_suite import (
    StartMode,
    deserialize_episode,
    load_cube_suite_config,
    sample_cube_episodes,
    serialize_episode,
)

ROOT = Path(__file__).parents[2]


def test_default_suite_is_deterministic_and_uses_mode_a() -> None:
    config = load_cube_suite_config(ROOT / "config/phase7_1_cube_suite.yml")
    episodes = sample_cube_episodes(config, root_seed=123)
    assert len(episodes) == 5
    assert {episode.start_mode for episode in episodes} == {StartMode.A}
    assert episodes == sample_cube_episodes(config, root_seed=123)
    assert deserialize_episode(serialize_episode(episodes[0])) == episodes[0]
    request = episodes[0].to_planning_request()
    assert request.surface_target.position_base_m.shape == (3,)
    assert config.terminal_standoff_m == 0.08
    assert all(episode.goal_label for episode in episodes)
    assert all(episode.fixed_roll_rad == 0.0 for episode in episodes)


def test_goal_region_sample_budget_is_named_constant() -> None:
    from mycobot_curobo import cube_suite as module

    assert module.GOAL_REGION_SAMPLE_ATTEMPTS >= 1
    assert module.GOAL_REGION_SAMPLE_ATTEMPTS == 64


def test_acceptance_schedule_exercises_all_start_modes() -> None:
    config = load_cube_suite_config(ROOT / "config/phase7_1_cube_suite.yml")
    episodes = sample_cube_episodes(
        config, root_seed=7, episode_count=6, force_modes=("A", "B", "C", "D")
    )
    assert {episode.start_mode for episode in episodes} == set(StartMode)
    assert all(episode.cube_edge_m == config.cube_edge_m for episode in episodes)
    assert len(config.goal_joint_bank) >= 3
