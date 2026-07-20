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


def test_mode_d_places_cube_on_tip_face_side() -> None:
    """Bare-flange tip (+Z) must look at the cube; wrist/back must not lead."""

    import numpy as np

    from mycobot_curobo.cube_suite import _quaternion_to_rotation
    from mycobot_curobo.frames import TaskFrameConfig, build_task_frame_candidates
    from mycobot_curobo.robot_model import forward_kinematics, load_robot_model_spec

    config = load_cube_suite_config(ROOT / "config/phase7_1_cube_suite.yml")
    episodes = sample_cube_episodes(config, root_seed=123, episode_count=5)
    spec = load_robot_model_spec()
    task_frame = TaskFrameConfig()
    assert task_frame.tool_approach_sign == 1
    for episode in episodes:
        goal = next(item for item in config.goal_joint_bank if item.label == episode.goal_label)
        pose = forward_kinematics(np.asarray(goal.position_rad), spec=spec)
        rotation = _quaternion_to_rotation(pose.quaternion_wxyz)
        tip_plus_z = rotation[:, 2] / float(np.linalg.norm(rotation[:, 2]))
        to_cube = np.asarray(episode.cube_center_m, dtype=float) - pose.position_m
        to_cube /= float(np.linalg.norm(to_cube))
        assert float(np.dot(tip_plus_z, to_cube)) > 0.99
        candidate = build_task_frame_candidates(
            episode.to_planning_request().surface_target,
            task_frame,
        )[0]
        assert float(np.dot(candidate.rotation_base_from_tool[:, 2], to_cube)) > 0.99
        assert float(np.dot(candidate.approach_direction_base, to_cube)) > 0.99


def test_chained_force_modes_sample_mode_b_only() -> None:
    config = load_cube_suite_config(ROOT / "config/phase7_1_cube_suite.yml")
    episodes = sample_cube_episodes(config, root_seed=123, episode_count=4, force_modes=("B", "D"))
    assert len(episodes) == 4
    assert all(episode.start_mode is StartMode.B for episode in episodes)


def test_acceptance_schedule_exercises_all_start_modes() -> None:
    config = load_cube_suite_config(ROOT / "config/phase7_1_cube_suite.yml")
    episodes = sample_cube_episodes(
        config, root_seed=7, episode_count=6, force_modes=("A", "B", "C", "D")
    )
    assert {episode.start_mode for episode in episodes} == set(StartMode)
    assert all(episode.cube_edge_m == config.cube_edge_m for episode in episodes)
    assert len(config.goal_joint_bank) >= 3
