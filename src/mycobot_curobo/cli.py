"""Command-line assembly for Phase 6 benchmarks, Phase 8 residual, and replay."""

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
from mycobot_curobo.residual import (
    PolicyResidualCorrector,
    ZeroResidualCorrector,
    load_residual_policy_checkpoint,
)
from mycobot_curobo.residual_compare import (
    build_residual_comparison_report,
    write_residual_comparison_report,
)
from mycobot_curobo.residual_mapping import FiniteDifferenceResidualMapper
from mycobot_curobo.residual_train import train_offline_residual_policy
from mycobot_curobo.robot_model import load_robot_model_spec
from mycobot_curobo.safety import SafetyProjector, load_residual_safety_profile
from mycobot_curobo.validation import (
    CuroboTrajectoryEvaluator,
    load_validation_profile,
    validate_nominal_plan,
)


def _build_executor(
    *,
    residual_profile_name: str,
    robot_spec,
    task_frame_config,
    corrector,
    enable_mapping: bool,
):
    residual_profile = load_residual_safety_profile(residual_profile_name)
    pose_evaluator = CpuTcpPoseEvaluator(robot_spec)
    mapper = None
    if enable_mapping:
        mapper = FiniteDifferenceResidualMapper(
            robot_spec=robot_spec,
            max_joint_delta_rad=residual_profile.max_joint_delta_rad,
            pose_evaluator=pose_evaluator,
        )

    def execute(plan, request):
        adapter = InMemoryCommandAdapter()
        trajectory_executor = TrajectoryExecutor(
            corrector=corrector,
            projector=SafetyProjector(residual_profile, robot_spec.limits),
            state_provider=ReplayRobotStateProvider(),
            pose_evaluator=pose_evaluator,
            adapter=adapter,
            task_frame_config=task_frame_config,
            joint_mapper=mapper,
        )
        return trajectory_executor.execute(plan, request)

    return execute


def create_benchmark_runtime(
    benchmark_config_path: Path,
    *,
    execute_zero_residual: bool = False,
    execute_residual_checkpoint: Path | None = None,
    residual_safety_profile: str | None = None,
) -> tuple[BenchmarkConfig, BenchmarkRunner]:
    app = load_app_config()
    benchmark = load_benchmark_config(benchmark_config_path)
    base_profile = load_planner_profile(benchmark.planner_profile)
    validation_profile = load_validation_profile(benchmark.validation_profile)
    robot_spec = load_robot_model_spec(app.robot_config_path)
    profile_name = residual_safety_profile or benchmark.residual_safety_profile

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
    if execute_residual_checkpoint is not None:
        checkpoint = load_residual_policy_checkpoint(execute_residual_checkpoint)
        executor = _build_executor(
            residual_profile_name=profile_name,
            robot_spec=robot_spec,
            task_frame_config=app.task_frame,
            corrector=PolicyResidualCorrector(checkpoint),
            enable_mapping=True,
        )
    elif execute_zero_residual:
        executor = _build_executor(
            residual_profile_name=profile_name,
            robot_spec=robot_spec,
            task_frame_config=app.task_frame,
            corrector=ZeroResidualCorrector(),
            enable_mapping=False,
        )

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
    parser.add_argument(
        "--execute-residual-checkpoint",
        type=Path,
        help="Advisory sim-only residual checkpoint for residual-on execution",
    )
    parser.add_argument(
        "--residual-safety-profile",
        type=str,
        default=None,
        help="Override residual safety profile name from the benchmark config",
    )
    args = parser.parse_args(argv)

    config, runner = create_benchmark_runtime(
        args.config,
        execute_zero_residual=args.execute_zero_residual,
        execute_residual_checkpoint=args.execute_residual_checkpoint,
        residual_safety_profile=args.residual_safety_profile,
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
                "sim_only": True,
                "residual_mode": (
                    "on"
                    if args.execute_residual_checkpoint is not None
                    else ("off" if args.execute_zero_residual else "none")
                ),
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


def residual_train_main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Train a sim-only advisory residual policy (offline synthetic backend)"
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("artifacts/residuals/phase8_offline_policy.json"),
    )
    parser.add_argument("--sample-count", type=int, default=128)
    parser.add_argument("--seed", type=int, default=8008)
    parser.add_argument("--tip-bias-m", type=float, nargs=3, default=(0.001, 0.0, 0.0))
    args = parser.parse_args(argv)
    summary = train_offline_residual_policy(
        output_path=str(args.output),
        sample_count=args.sample_count,
        seed=args.seed,
        tip_bias_m=tuple(args.tip_bias_m),
    )
    print(json.dumps(asdict(summary), indent=2, sort_keys=True))
    return 0


def residual_compare_main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Compare residual-off vs residual-on on Phase 6 scenes (sim only)"
    )
    parser.add_argument("--config", type=Path, default=Path("config/benchmark_workspace.yml"))
    parser.add_argument("--stage", choices=("smoke", "regression", "exploratory"), default="smoke")
    parser.add_argument("--root-seed", type=int, default=6006)
    parser.add_argument("--count", type=int)
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/benchmarks"))
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument(
        "--residual-safety-profile",
        type=str,
        default="simulation_bounded_residual",
    )
    args = parser.parse_args(argv)

    config, off_runner = create_benchmark_runtime(
        args.config,
        execute_zero_residual=True,
        residual_safety_profile=args.residual_safety_profile,
    )
    _, on_runner = create_benchmark_runtime(
        args.config,
        execute_residual_checkpoint=args.checkpoint,
        residual_safety_profile=args.residual_safety_profile,
    )
    cases = sample_benchmark_cases(
        config,
        root_seed=args.root_seed,
        stage=args.stage,
        count=args.count,
    )
    off_results = off_runner.run(cases)
    on_results = on_runner.run(cases)
    report = build_residual_comparison_report(
        off_results=off_results,
        on_results=on_results,
        root_seed=args.root_seed,
        stage=args.stage,
    )
    json_path, md_path = write_residual_comparison_report(report, args.output_dir)
    print(
        json.dumps(
            {
                "json_report": str(json_path),
                "markdown_report": str(md_path),
                "sim_only": True,
                "residual_mode_off": "off",
                "residual_mode_on": "on",
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
