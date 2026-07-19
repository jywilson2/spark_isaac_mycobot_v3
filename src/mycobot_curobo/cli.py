"""Command-line assembly for Phase 6 benchmarks and failed-case replay."""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, replace
from pathlib import Path
from typing import Sequence

from mycobot_curobo.benchmark import (
    BenchmarkConfig,
    BenchmarkRunner,
    aggregate_results,
    deserialize_request,
    load_benchmark_config,
    sample_benchmark_cases,
    write_benchmark_reports,
)
from mycobot_curobo.config import load_app_config
from mycobot_curobo.execution import (
    CpuTcpPoseEvaluator,
    InMemoryCommandAdapter,
    ReplayRobotStateProvider,
    TrajectoryExecutor,
)
from mycobot_curobo.planner import (
    NominalPlanner,
    create_curobo_planner,
    load_planner_profile,
)
from mycobot_curobo.residual import ZeroResidualCorrector
from mycobot_curobo.robot_model import load_robot_model_spec
from mycobot_curobo.safety import SafetyProjector, load_residual_safety_profile
from mycobot_curobo.validation import (
    CuroboTrajectoryEvaluator,
    load_validation_profile,
    validate_nominal_plan,
)


def create_benchmark_runtime(
    benchmark_config_path: Path,
    *,
    execute_zero_residual: bool,
) -> tuple[BenchmarkConfig, BenchmarkRunner]:
    app = load_app_config()
    benchmark = load_benchmark_config(benchmark_config_path)
    base_profile = load_planner_profile(benchmark.planner_profile)
    validation_profile = load_validation_profile(benchmark.validation_profile)
    robot_spec = load_robot_model_spec(app.robot_config_path)

    def profile_for(seed: int):
        # NominalPlanner requires request.random_seed == profile.random_seed.
        # A fresh copied profile and planner preserve that invariant per sweep seed.
        return replace(base_profile, random_seed=seed)

    def planner_factory(seed: int) -> NominalPlanner:
        profile = profile_for(seed)
        return NominalPlanner(
            lambda: create_curobo_planner(
                profile,
                robot_config_path=app.robot_config_path,
                scene_config_path=app.scene_config_path,
                warmup=False,
            ),
            profile,
            task_frame_config=app.task_frame,
        )

    def validator(plan, request):
        profile = profile_for(request.random_seed)
        backend = create_curobo_planner(
            profile,
            robot_config_path=app.robot_config_path,
            scene_config_path=app.scene_config_path,
            warmup=False,
        )
        return validate_nominal_plan(
            plan,
            request,
            profile=validation_profile,
            evaluator=CuroboTrajectoryEvaluator(backend, scene_is_empty=True),
            robot_spec=robot_spec,
            task_frame_config=app.task_frame,
        )

    executor = None
    if execute_zero_residual:
        residual_profile = load_residual_safety_profile(benchmark.residual_safety_profile)

        def execute(plan, request):
            adapter = InMemoryCommandAdapter()
            trajectory_executor = TrajectoryExecutor(
                corrector=ZeroResidualCorrector(),
                projector=SafetyProjector(residual_profile, robot_spec.limits),
                state_provider=ReplayRobotStateProvider(),
                pose_evaluator=CpuTcpPoseEvaluator(robot_spec),
                adapter=adapter,
                task_frame_config=app.task_frame,
            )
            return trajectory_executor.execute(plan, request)

        executor = execute

    return benchmark, BenchmarkRunner(
        planner_factory=planner_factory,
        validator=validator,
        repeat_count=benchmark.repeat_count,
        executor=executor,
    )


def benchmark_main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run the Phase 6 randomized benchmark")
    parser.add_argument("--config", type=Path, default=Path("config/benchmark_workspace.yml"))
    parser.add_argument("--stage", choices=("smoke", "regression", "exploratory"), default="smoke")
    parser.add_argument("--root-seed", type=int, default=123)
    parser.add_argument("--count", type=int)
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/benchmarks"))
    parser.add_argument("--execute-zero-residual", action="store_true")
    args = parser.parse_args(argv)

    config, runner = create_benchmark_runtime(
        args.config, execute_zero_residual=args.execute_zero_residual
    )
    cases = sample_benchmark_cases(
        config,
        root_seed=args.root_seed,
        stage=args.stage,
        count=args.count,
    )
    results = runner.run(cases)
    summary = aggregate_results(results, root_seed=args.root_seed, stage=args.stage)
    json_path, markdown_path = write_benchmark_reports(summary, results, args.output_dir)
    print(
        json.dumps(
            {
                "json_report": str(json_path),
                "markdown_report": str(markdown_path),
                "summary": asdict(summary),
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


def replay_main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Replay one serialized Phase 6 request")
    parser.add_argument("request_json", type=Path)
    parser.add_argument("--failed-index", type=int, default=0)
    parser.add_argument(
        "--benchmark-config",
        type=Path,
        default=Path("config/benchmark_workspace.yml"),
    )
    args = parser.parse_args(argv)
    payload = json.loads(args.request_json.read_text(encoding="utf-8"))
    if "failed_replay_requests" in payload:
        payload = payload["failed_replay_requests"][args.failed_index]
    request = deserialize_request(payload)

    _, runner = create_benchmark_runtime(args.benchmark_config, execute_zero_residual=False)
    # Preserve the serialized request exactly; labels only describe the replay.
    from mycobot_curobo.benchmark import BenchmarkCase

    target = request.surface_target
    case = BenchmarkCase(
        case_id=request.request_id,
        root_seed=request.random_seed,
        sample_index=0,
        region_label="serialized_replay",
        normal_bin_label="serialized_replay",
        start_joint_label="serialized_replay",
        position_base_m=tuple(target.position_base_m),
        surface_normal_base=tuple(target.surface_normal_base),
        tangent_hint_base=(
            None if target.tangent_hint_base is None else tuple(target.tangent_hint_base)
        ),
        start_joint_position_rad=tuple(request.current_joint_state.position_rad),
        pre_approach_distance_m=target.pre_approach_distance_m,
        planner_seed=request.random_seed,
        fixed_roll_rad=target.fixed_roll_rad,
        roll_candidates_rad=target.roll_candidates_rad,
        scene_revision=request.scene_revision,
        planner_profile=request.planner_profile,
    )
    result = runner.run((case,))[0]
    print(json.dumps(asdict(result), default=str, indent=2, sort_keys=True))
    return 0 if result.succeeded else 2
