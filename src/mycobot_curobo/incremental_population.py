"""Phase 7.5 incremental target-population runner and console helpers."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field, replace
from enum import Enum
from typing import Any, Callable, Sequence

import numpy as np

from mycobot_curobo.cube_scene import (
    CubeGeometry,
    batch_sphere_cube_clearance_m,
    candidate_clears_recorded_corridors,
    cubes_to_curobo_scene_dict,
    multi_cube_scene_revision,
)
from mycobot_curobo.errors import ConfigurationError
from mycobot_curobo.multi_target import (
    ContactDetector,
    ContactKind,
    MultiTargetEpisode,
    MultiTargetEpisodeResult,
    MultiTargetFailureCategory,
    MultiTargetLegResult,
    MultiTargetSuiteConfig,
    MultiTargetSuiteSummary,
    NumberedTarget,
    OrderPolicy,
    PlacementPolicy,
    TargetField,
    TargetPopulation,
    _draw_distinct_episode_seeds,
    suite_acceptance_passed,
)
from mycobot_curobo.planner import (
    JOINT_NAMES,
    NamedJointState,
    NominalPlan,
    PlanningOutcome,
    PlanningRequest,
)
from mycobot_curobo.planning_world import leg_world_geometries
from mycobot_curobo.target_placement import (
    GeometricRejectCounts,
    draw_incremental_candidate,
    resolve_z_band_half_m,
    z_band_bounds,
)
from mycobot_curobo.validation import ValidatedPlan

# Optional FK sphere callback: joint positions [N,6] -> spheres [N,S,4].
WaypointSpheresFn = Callable[[np.ndarray], np.ndarray]


class PopulationStopReason(str, Enum):
    CONSECUTIVE_FAILURES = "consecutive_failures"
    TOTAL_FAILURES = "total_failures"
    GEOMETRIC_FULL = "geometric_full"
    MAX_TARGETS = "max_targets"


@dataclass(frozen=True)
class CandidateFailureRecord:
    """One planner-verified candidate failure persisted in suite extras."""

    candidate_serial: int
    center_m: tuple[float, float, float]
    failure_category: str
    failure_reason: str
    planner_status: str
    plan_duration_s: float
    streak_after: int


@dataclass
class IncrementalEpisodeExtras:
    """Phase 7.5 population metadata attached alongside a standard episode result."""

    stop_reason: PopulationStopReason
    consecutive_failures_at_stop: int
    total_target_failures: int
    accepted_plan_durations_s: tuple[float, ...]
    failed_plan_durations_s: tuple[float, ...]
    geometric_rejects: GeometricRejectCounts
    draws: int
    planned_candidates: int
    z_band_lo_m: float
    z_band_hi_m: float
    delta_z_m: float
    wall_duration_s: float
    candidate_failures: tuple[CandidateFailureRecord, ...] = ()


@dataclass
class IncrementalSuiteRun:
    """Full incremental suite outcome for host packaging and console output."""

    results: tuple[MultiTargetEpisodeResult, ...]
    extras: tuple[IncrementalEpisodeExtras, ...]
    trajectories: dict[str, Any]
    root_seed: int | None
    episode_seeds: tuple[int, ...]
    artifact_base_name: str
    summary: MultiTargetSuiteSummary
    suite_accepted: bool


def z_density_width_m(config: MultiTargetSuiteConfig) -> float:
    """Full vertical band width (metres) that labels the suite Z-density."""

    half = resolve_z_band_half_m(
        arm_z_motion_range_m=config.arm_z_motion_range_m,
        z_band_fraction=config.z_band_fraction,
        delta_z_m=config.delta_z_m,
    )
    return 2.0 * float(half)


def format_dz_label(width_m: float) -> str:
    """Format ``0.30`` → ``dz0_30`` (decimal point → underscore)."""

    if not math.isfinite(width_m) or width_m <= 0.0:
        raise ConfigurationError("Z-density width must be positive finite")
    return f"dz{width_m:.2f}".replace(".", "_")


def build_incremental_artifact_base_name(
    *,
    scene_revision_prefix: str,
    delta_z_m: float,
    accepted_counts: Sequence[int],
    root_seed: int,
) -> str:
    """Build ``{prefix}_dz{w}_n{N1-N2-...}_seed{root_seed}``."""

    counts = "-".join(str(int(count)) for count in accepted_counts)
    if not counts:
        counts = "0"
    return f"{scene_revision_prefix}_{format_dz_label(delta_z_m)}_n{counts}_seed{int(root_seed)}"


def sample_mean_std(values: Sequence[float]) -> tuple[float | None, float | None]:
    """Mean and sample standard deviation (n−1); σ is None when n < 2."""

    if not values:
        return None, None
    mean = float(sum(values) / len(values))
    if len(values) < 2:
        return mean, None
    variance = sum((float(value) - mean) ** 2 for value in values) / (len(values) - 1)
    return mean, float(math.sqrt(variance))


def _format_mu_sigma(mean: float | None, std: float | None) -> str:
    if mean is None:
        return "plan µ=n/a σ=n/a"
    mean_text = f"{mean:.1f}s"
    std_text = "n/a" if std is None else f"{std:.1f}s"
    return f"plan µ={mean_text} σ={std_text}"


def format_z_dist_header(config: MultiTargetSuiteConfig) -> tuple[str, float, float, float]:
    """Return ``(header_fragment, width, z_lo, z_hi)`` for console lines."""

    width = z_density_width_m(config)
    _, z_lo, z_hi = z_band_bounds(
        config.field_minimum_m,
        config.field_maximum_m,
        arm_z_motion_range_m=config.arm_z_motion_range_m,
        z_band_fraction=config.z_band_fraction,
        delta_z_m=config.delta_z_m,
    )
    fragment = f"z-dist uniform dz={width:.2f} band {z_lo:.2f}–{z_hi:.2f} m"
    return fragment, width, float(z_lo), float(z_hi)


def format_populate_begin(
    *,
    episode_index: int,
    episode_count: int,
    config: MultiTargetSuiteConfig,
) -> str:
    fragment, _, _, _ = format_z_dist_header(config)
    consecutive = config.max_consecutive_target_failures
    consecutive_text = "off" if consecutive <= 0 else str(consecutive)
    total = config.max_total_target_failures
    total_text = "off" if total <= 0 else str(total)
    return (
        f"phase7_5_populate: ep {episode_index + 1}/{episode_count} BEGIN | "
        f"{fragment} | primary_stop geometric_full | "
        f"secondary consecutive={consecutive_text} total_fails={total_text}"
    )


def format_failure_annotation(
    *,
    failure_category: str | None,
    failure_reason: str | None,
) -> str:
    """Render ``(category)`` or ``(category: first_reason)`` for populate FAIL lines."""

    category = (failure_category or "plan_failed").strip() or "plan_failed"
    reason = (failure_reason or "").strip()
    if reason and reason != category:
        # Prefer the first semicolon-delimited reason fragment when present.
        first_reason = reason.split(";", 1)[0].strip()
        return f"{category}: {first_reason}" if first_reason else category
    return category


def format_populate_candidate(
    *,
    episode_index: int,
    episode_count: int,
    accepted: int,
    candidate_index: int,
    center_m: Sequence[float],
    plan_ok: bool,
    plan_duration_s: float,
    failure_reason: str | None,
    streak: int,
    threshold: int,
    failure_category: str | None = None,
) -> str:
    radial = math.hypot(float(center_m[0]), float(center_m[1]))
    if plan_ok:
        status = f"plan OK {plan_duration_s:.1f}s"
    else:
        annotation = format_failure_annotation(
            failure_category=failure_category,
            failure_reason=failure_reason,
        )
        status = f"plan FAIL {plan_duration_s:.1f}s ({annotation})"
    if threshold <= 0:
        streak_text = f"streak {streak} (consecutive stop off)"
    else:
        streak_text = f"streak {streak}/{threshold}"
        if streak > 0:
            remaining = max(0, threshold - streak)
            streak_text += f" — {remaining} more consecutive failures end episode"
    return (
        f"phase7_5_populate: ep {episode_index + 1}/{episode_count} | "
        f"accepted {accepted} | cand {candidate_index} "
        f"z={float(center_m[2]):.3f} r={radial:.3f} | {status} | {streak_text}"
    )


def format_sampling_line(
    *,
    episode_index: int,
    episode_count: int,
    draws: int,
    rejects: GeometricRejectCounts,
    planned: int,
) -> str:
    return (
        f"phase7_5_sampling: ep {episode_index + 1}/{episode_count} | "
        f"draws {draws} | geometric rejects {rejects.total} "
        f"(separation {rejects.separation}, keep_out {rejects.keep_out}, "
        f"rim {rejects.rim}"
        + (f", reach {rejects.reach}" if rejects.reach else "")
        + (f", aabb {rejects.aabb}" if rejects.aabb else "")
        + (f", corridor {rejects.corridor}" if rejects.corridor else "")
        + f") | planned {planned}"
    )


def format_episode_done(
    *,
    episode_index: int,
    episode_count: int,
    accepted: int,
    extras: IncrementalEpisodeExtras,
    min_targets: int,
    succeeded: bool,
    consecutive_threshold: int = 0,
) -> str:
    mean, std = sample_mean_std(extras.accepted_plan_durations_s)
    stop = extras.stop_reason.value
    if extras.stop_reason is PopulationStopReason.CONSECUTIVE_FAILURES:
        threshold = max(int(consecutive_threshold), int(extras.consecutive_failures_at_stop))
        stop = f"consecutive_failures {extras.consecutive_failures_at_stop}/{threshold}"
    elif extras.stop_reason is PopulationStopReason.TOTAL_FAILURES:
        stop = f"total_failures {extras.total_target_failures}"
    z_lo = extras.z_band_lo_m
    z_hi = extras.z_band_hi_m
    verdict = "PASS" if succeeded else "FAIL"
    return (
        f"phase7_5_episode: ep {episode_index + 1}/{episode_count} DONE | "
        f"tip_contacts {accepted} | populate_s {extras.wall_duration_s:.1f} | "
        f"stop {stop} | fails {extras.total_target_failures} total | "
        f"z {z_lo:.3f}–{z_hi:.3f} | {_format_mu_sigma(mean, std)} | "
        f"min {min_targets}: {verdict}"
    )


def format_suite_table(
    *,
    results: Sequence[MultiTargetEpisodeResult],
    extras: Sequence[IncrementalEpisodeExtras],
    artifact_base_name: str,
) -> str:
    rows: list[tuple[str, ...]] = [
        (
            "ep",
            "tip_contacts",
            "populate_s",
            "stop",
            "fails",
            "plan_mu",
            "plan_sigma",
        )
    ]
    all_plan: list[float] = []
    total_accepted = 0
    for result, extra in zip(results, extras):
        mean, std = sample_mean_std(extra.accepted_plan_durations_s)
        all_plan.extend(extra.accepted_plan_durations_s)
        total_accepted += len(result.contacted_ids)
        rows.append(
            (
                str(result.episode.episode_index + 1),
                str(len(result.contacted_ids)),
                f"{extra.wall_duration_s:.1f}",
                extra.stop_reason.value,
                str(extra.total_target_failures),
                "n/a" if mean is None else f"{mean:.1f}",
                "n/a" if std is None else f"{std:.1f}",
            )
        )
    suite_mean, suite_std = sample_mean_std(all_plan)
    rows.append(
        (
            "total",
            str(total_accepted),
            f"{sum(extra.wall_duration_s for extra in extras):.1f}",
            "",
            str(sum(extra.total_target_failures for extra in extras)),
            "n/a" if suite_mean is None else f"{suite_mean:.1f}",
            "n/a" if suite_std is None else f"{suite_std:.1f}",
        )
    )
    widths = [max(len(row[col]) for row in rows) for col in range(len(rows[0]))]
    lines = ["phase7_5_suite:"]
    for row in rows:
        lines.append(
            "phase7_5_suite:  "
            + "  ".join(cell.ljust(widths[index]) for index, cell in enumerate(row))
        )
    lines.append(f"phase7_5_suite: artifact {artifact_base_name}")
    return "\n".join(lines)


def format_replay_begin(
    *,
    episode_index: int,
    episode_count: int,
    z_fragment: str,
    target_count: int,
) -> str:
    return (
        f"phase7_5_replay: ep {episode_index + 1}/{episode_count} BEGIN | "
        f"{z_fragment} | targets {target_count}"
    )


def format_replay_leg(
    *,
    episode_index: int,
    episode_count: int,
    leg_index: int,
    target_count: int,
    target_id: str,
    plan_duration_s: float | None,
    contact_kind: str | None,
) -> str:
    plan_text = "n/a" if plan_duration_s is None else f"{plan_duration_s:.1f}s"
    contact = contact_kind or "none"
    return (
        f"phase7_5_replay: ep {episode_index + 1}/{episode_count} | "
        f"leg {leg_index + 1}/{target_count} target {target_id} | "
        f"recorded plan {plan_text} | contact {contact}"
    )


def format_replay_done(
    *,
    episode_index: int,
    episode_count: int,
    target_count: int,
    contacted: int,
    plan_durations_s: Sequence[float],
    populate_duration_s: float | None = None,
) -> str:
    mean, std = sample_mean_std(plan_durations_s)
    populate_text = "n/a" if populate_duration_s is None else f"{float(populate_duration_s):.1f}"
    return (
        f"phase7_5_replay: ep {episode_index + 1}/{episode_count} DONE | "
        f"tip_contacts {contacted} | populate_s {populate_text} | "
        f"targets {target_count} | {_format_mu_sigma(mean, std)} (recorded)"
    )


@dataclass
class _PopulationState:
    accepted: list[NumberedTarget] = field(default_factory=list)
    contacted_ids: list[str] = field(default_factory=list)
    planned_target_ids: list[str] = field(default_factory=list)
    legs: list[MultiTargetLegResult] = field(default_factory=list)
    accepted_plan_durations_s: list[float] = field(default_factory=list)
    failed_plan_durations_s: list[float] = field(default_factory=list)
    geometric_rejects: GeometricRejectCounts = field(default_factory=GeometricRejectCounts)
    draws: int = 0
    planned_candidates: int = 0
    consecutive_failures: int = 0
    total_target_failures: int = 0
    current_joints: tuple[float, ...] = ()
    from_id: str = "start"
    candidate_serial: int = 0
    corridor_spheres: list[np.ndarray] = field(default_factory=list)
    candidate_failures: list[CandidateFailureRecord] = field(default_factory=list)


class IncrementalPopulationRunner:
    """Grow a retained multi-target field one planner-verified candidate at a time."""

    def __init__(
        self,
        *,
        planner_factory: Callable[[int, dict[str, Any], tuple[str, ...]], Any],
        validator: Callable[
            [NominalPlan, PlanningRequest, tuple[CubeGeometry, ...], CubeGeometry],
            ValidatedPlan,
        ],
        contact_detector_factory: Callable[[MultiTargetEpisode, str], ContactDetector],
        plan_sink: Callable[[NominalPlan], None] | None = None,
        warn_planning_duration_s: float | None = None,
        console_log: Callable[[str], None] | None = None,
        apply_reach_prefilter: bool = True,
        waypoint_spheres_fn: WaypointSpheresFn | None = None,
    ) -> None:
        self._planner_factory = planner_factory
        self._validator = validator
        self._contact_detector_factory = contact_detector_factory
        self._plan_sink = plan_sink
        self._warn_planning_duration_s = warn_planning_duration_s
        self._console_log = print if console_log is None else console_log
        self._apply_reach_prefilter = apply_reach_prefilter
        self._waypoint_spheres_fn = waypoint_spheres_fn

    def run_suite(
        self,
        config: MultiTargetSuiteConfig,
        *,
        root_seed: int | None = None,
        episode_count: int | None = None,
        independent_random_episode_seeds: bool = False,
        trajectories_out: dict[str, Any] | None = None,
    ) -> IncrementalSuiteRun:
        if config.target_population is not TargetPopulation.INCREMENTAL:
            raise ConfigurationError(
                "IncrementalPopulationRunner requires target_population=incremental"
            )
        count = config.episode_count if episode_count is None else int(episode_count)
        if count < 1:
            raise ConfigurationError("episode_count must be a positive integer")
        if independent_random_episode_seeds:
            seed_pairs = _draw_distinct_episode_seeds(count)
            shared_root: int | None = None
        else:
            seed = config.root_seed if root_seed is None else int(root_seed)
            shared_root = seed
            seed_pairs = tuple(
                (seed + 1009 * (index + 1), seed + 9176 * (index + 1)) for index in range(count)
            )
        results: list[MultiTargetEpisodeResult] = []
        extras: list[IncrementalEpisodeExtras] = []
        trajectories: dict[str, Any] = {} if trajectories_out is None else trajectories_out
        for index, (episode_seed, _order_seed) in enumerate(seed_pairs):
            result, extra = self.run_episode(
                config,
                episode_index=index,
                episode_count=count,
                root_seed=episode_seed if shared_root is None else shared_root,
                episode_seed=episode_seed,
                trajectories=trajectories,
            )
            results.append(result)
            extras.append(extra)
        summary_seed = (
            shared_root
            if shared_root is not None
            else int(results[0].episode.episode_seed if results else 0)
        )
        from mycobot_curobo.multi_target import aggregate_multi_target_results

        summary = aggregate_multi_target_results(results, root_seed=summary_seed)
        accepted_counts = [len(result.contacted_ids) for result in results]
        artifact = build_incremental_artifact_base_name(
            scene_revision_prefix=config.scene_revision_prefix,
            delta_z_m=z_density_width_m(config),
            accepted_counts=accepted_counts,
            root_seed=summary_seed,
        )
        return IncrementalSuiteRun(
            results=tuple(results),
            extras=tuple(extras),
            trajectories=trajectories,
            root_seed=shared_root,
            episode_seeds=tuple(int(pair[0]) for pair in seed_pairs),
            artifact_base_name=artifact,
            summary=summary,
            suite_accepted=suite_acceptance_passed(
                summary, max_failed_episodes=config.max_failed_episodes
            ),
        )

    def run_episode(
        self,
        config: MultiTargetSuiteConfig,
        *,
        episode_index: int,
        episode_count: int,
        root_seed: int,
        episode_seed: int,
        trajectories: dict[str, Any],
    ) -> tuple[MultiTargetEpisodeResult, IncrementalEpisodeExtras]:
        started = time.perf_counter()
        z_fragment, width, band_lo, band_hi = format_z_dist_header(config)
        del z_fragment
        self._console_log(
            format_populate_begin(
                episode_index=episode_index,
                episode_count=episode_count,
                config=config,
            )
        )
        rng = np.random.default_rng(int(episode_seed))
        state = _PopulationState(current_joints=config.start_joint_position_rad)
        stop_reason: PopulationStopReason | None = None
        placement_attempts = 0
        last_sampling_emit_draws = 0

        while stop_reason is None:
            if (
                config.max_targets_per_episode > 0
                and len(state.accepted) >= config.max_targets_per_episode
            ):
                stop_reason = PopulationStopReason.MAX_TARGETS
                break
            # Secondary timeouts only (0 = disabled). Primary stop is geometric
            # fullness when a legal candidate cannot be drawn.
            if (
                config.max_consecutive_target_failures > 0
                and state.consecutive_failures >= config.max_consecutive_target_failures
            ):
                stop_reason = PopulationStopReason.CONSECUTIVE_FAILURES
                break
            if (
                config.max_total_target_failures > 0
                and state.total_target_failures >= config.max_total_target_failures
            ):
                stop_reason = PopulationStopReason.TOTAL_FAILURES
                break

            center: tuple[float, float, float] | None = None
            while center is None:
                if placement_attempts >= config.max_placement_attempts:
                    stop_reason = PopulationStopReason.GEOMETRIC_FULL
                    break
                state.draws += 1
                placement_attempts += 1
                center = draw_incremental_candidate(
                    rng,
                    field_minimum_m=config.field_minimum_m,
                    field_maximum_m=config.field_maximum_m,
                    arm_z_motion_range_m=config.arm_z_motion_range_m,
                    edge_m=config.target_edge_m,
                    min_center_separation_m=config.min_center_separation_m,
                    accepted_centers_m=[target.center_m for target in state.accepted],
                    keep_outs=config.keep_outs,
                    outward_normal_base=config.outward_normal_base,
                    max_target_radial_m=config.max_target_radial_m,
                    z_band_fraction=config.z_band_fraction,
                    delta_z_m=config.delta_z_m,
                    z_separation_gain=config.z_separation_gain,
                    pre_approach_distance_m=config.pre_approach_distance_m,
                    dexterous_reach=config.dexterous_reach,
                    apply_reach_prefilter=self._apply_reach_prefilter,
                    reject_counts=state.geometric_rejects,
                )
                if center is not None and state.corridor_spheres:
                    if not candidate_clears_recorded_corridors(
                        center,
                        config.target_edge_m,
                        state.corridor_spheres,
                        minimum_clearance_m=config.minimum_world_collision_clearance_m,
                    ):
                        state.geometric_rejects.corridor += 1
                        center = None
                if state.draws - last_sampling_emit_draws >= 25:
                    self._console_log(
                        format_sampling_line(
                            episode_index=episode_index,
                            episode_count=episode_count,
                            draws=state.draws,
                            rejects=state.geometric_rejects,
                            planned=state.planned_candidates,
                        )
                    )
                    last_sampling_emit_draws = state.draws
            if stop_reason is not None:
                break
            assert center is not None
            placement_attempts = 0
            state.candidate_serial += 1
            candidate_id = str(state.candidate_serial)
            candidate = NumberedTarget(
                target_id=candidate_id,
                center_m=center,
                edge_m=config.target_edge_m,
                outward_normal_base=config.outward_normal_base,
                fixed_roll_rad=config.fixed_roll_rad,
                roll_candidates_rad=config.roll_candidates_rad,
                pre_approach_distance_m=config.pre_approach_distance_m,
            )
            accepted_before = len(state.accepted)
            if state.accepted:
                self._verify_retreated_start_clearance(config=config, state=state)
            leg, plan = self._plan_candidate(
                config=config,
                episode_index=episode_index,
                episode_seed=episode_seed,
                root_seed=root_seed,
                state=state,
                candidate=candidate,
            )
            state.planned_candidates += 1
            state.legs.append(leg)
            plan_ok = (
                leg.planning_succeeded
                and leg.validation_passed
                and leg.contact_kind is ContactKind.ALLOWED_TIP_CONTACT
            )
            plan_s = 0.0 if leg.planning_duration_s is None else float(leg.planning_duration_s)
            failure_category_text: str | None = None
            if plan_ok:
                if plan is not None and self._plan_sink is not None:
                    self._plan_sink(plan)
                if plan is not None and leg.request_id is not None:
                    trajectories[leg.request_id] = plan.combined_trajectory
                state.accepted.append(candidate)
                state.contacted_ids.append(candidate_id)
                state.planned_target_ids.append(candidate_id)
                state.accepted_plan_durations_s.append(plan_s)
                state.consecutive_failures = 0
                if leg.final_joint_position_rad is not None:
                    state.current_joints = leg.final_joint_position_rad
                state.from_id = candidate_id
                if plan is not None and self._waypoint_spheres_fn is not None:
                    state.corridor_spheres.append(
                        np.asarray(
                            self._waypoint_spheres_fn(plan.combined_trajectory.position_rad),
                            dtype=float,
                        )
                    )
                failure_reason = None
            else:
                state.consecutive_failures += 1
                state.total_target_failures += 1
                state.failed_plan_durations_s.append(plan_s)
                failure_category_text = (
                    leg.failure_category.value
                    if leg.failure_category is not None
                    else "plan_failed"
                )
                failure_reason = leg.failure_reason
                if not failure_reason:
                    failure_reason = failure_category_text
                state.candidate_failures.append(
                    CandidateFailureRecord(
                        candidate_serial=state.candidate_serial,
                        center_m=(
                            float(center[0]),
                            float(center[1]),
                            float(center[2]),
                        ),
                        failure_category=failure_category_text,
                        failure_reason=str(failure_reason),
                        planner_status=str(leg.planner_status or ""),
                        plan_duration_s=plan_s,
                        streak_after=state.consecutive_failures,
                    )
                )
            self._console_log(
                format_populate_candidate(
                    episode_index=episode_index,
                    episode_count=episode_count,
                    accepted=accepted_before,
                    candidate_index=state.candidate_serial,
                    center_m=center,
                    plan_ok=plan_ok,
                    plan_duration_s=plan_s,
                    failure_reason=failure_reason,
                    failure_category=failure_category_text,
                    streak=state.consecutive_failures,
                    threshold=config.max_consecutive_target_failures,
                )
            )
            # Spec sample keeps "accepted N" as count before this candidate on fail,
            # and after accept the next line shows the new count. On OK, accepted
            # in the line is the count *after* acceptance in the sample:
            # "accepted 12 | cand 15 ... plan OK" then later accepted grows.
            # Sample: accepted 12 on both OK and FAIL lines for cand 15/18 —
            # so accepted is pre-decision count. Good as written.
            if leg.failure_category is MultiTargetFailureCategory.BODY_CONTACT:
                # Body contact during optimistic planning is unexpected; fail episode.
                stop_reason = PopulationStopReason.TOTAL_FAILURES
                break
            if (
                config.max_consecutive_target_failures > 0
                and state.consecutive_failures >= config.max_consecutive_target_failures
            ):
                stop_reason = PopulationStopReason.CONSECUTIVE_FAILURES
                break
            if (
                config.max_total_target_failures > 0
                and state.total_target_failures >= config.max_total_target_failures
            ):
                stop_reason = PopulationStopReason.TOTAL_FAILURES
                break
            if (
                config.max_targets_per_episode > 0
                and len(state.accepted) >= config.max_targets_per_episode
            ):
                stop_reason = PopulationStopReason.MAX_TARGETS
                break

        if stop_reason is None:
            # Prefer geometric_full when the draw loop exhausted placement attempts.
            stop_reason = PopulationStopReason.GEOMETRIC_FULL

        self._console_log(
            format_sampling_line(
                episode_index=episode_index,
                episode_count=episode_count,
                draws=state.draws,
                rejects=state.geometric_rejects,
                planned=state.planned_candidates,
            )
        )

        accepted_z = [float(target.center_m[2]) for target in state.accepted]
        z_lo = min(accepted_z) if accepted_z else band_lo
        z_hi = max(accepted_z) if accepted_z else band_hi
        wall = float(time.perf_counter() - started)
        extras = IncrementalEpisodeExtras(
            stop_reason=stop_reason,
            consecutive_failures_at_stop=state.consecutive_failures,
            total_target_failures=state.total_target_failures,
            accepted_plan_durations_s=tuple(state.accepted_plan_durations_s),
            failed_plan_durations_s=tuple(state.failed_plan_durations_s),
            geometric_rejects=state.geometric_rejects,
            draws=state.draws,
            planned_candidates=state.planned_candidates,
            z_band_lo_m=z_lo,
            z_band_hi_m=z_hi,
            delta_z_m=width,
            wall_duration_s=wall,
            candidate_failures=tuple(state.candidate_failures),
        )
        field = TargetField(
            targets=tuple(state.accepted),
            placement=PlacementPolicy.RANDOM,
            order=OrderPolicy.LISTED,
            retain_targets_after_contact=True,
            contact_order_ids=tuple(target.target_id for target in state.accepted),
        )
        episode = MultiTargetEpisode(
            episode_index=episode_index,
            root_seed=root_seed,
            episode_seed=episode_seed,
            order_seed=episode_seed,
            field=field,
            start_position_rad=config.start_joint_position_rad,
            planner_profile=config.planner_profile,
            tip_allow_link_names=config.tip_allow_link_names,
            max_planning_failure_per_target=1,
            max_target_failures=0,
            max_reconsider_passes=0,
            max_failed_episodes=config.max_failed_episodes,
            max_consecutive_unplanned_targets=0,
            scene_revision_prefix=config.scene_revision_prefix,
            retain_targets_after_contact=True,
        )
        succeeded = len(state.accepted) >= config.min_targets_per_episode
        # Body-contact legs fail the episode regardless of count.
        if any(
            leg.failure_category is MultiTargetFailureCategory.BODY_CONTACT for leg in state.legs
        ):
            succeeded = False
            failure_category = MultiTargetFailureCategory.BODY_CONTACT
            failure_reason = "prohibited body contact during incremental population"
        elif succeeded:
            failure_category = None
            failure_reason = None
        else:
            failure_category = MultiTargetFailureCategory.INSUFFICIENT_TARGETS
            failure_reason = (
                f"accepted {len(state.accepted)} < min_targets_per_episode="
                f"{config.min_targets_per_episode}"
            )
        self._console_log(
            format_episode_done(
                episode_index=episode_index,
                episode_count=episode_count,
                accepted=len(state.accepted),
                extras=extras,
                min_targets=config.min_targets_per_episode,
                succeeded=succeeded,
                consecutive_threshold=config.max_consecutive_target_failures,
            )
        )
        self._console_log(
            f"phase7_5_episode_metrics: ep {episode_index + 1}/{episode_count} | "
            f"tip_contacts {len(state.accepted)} | populate_s {wall:.1f}"
        )
        result = MultiTargetEpisodeResult(
            episode=episode,
            succeeded=succeeded,
            failure_category=failure_category,
            failure_reason=failure_reason,
            planning_failure_count=state.total_target_failures,
            target_failure_count=state.total_target_failures,
            failed_target_ids=(),
            legs=tuple(
                leg
                for leg in state.legs
                if leg.planning_succeeded
                and leg.validation_passed
                and leg.contact_kind is ContactKind.ALLOWED_TIP_CONTACT
            ),
            contacted_ids=tuple(state.contacted_ids),
            removed_ids=(),
            episode_duration_s=wall,
            deferred_target_ids=(),
            planned_target_ids=tuple(state.planned_target_ids),
        )
        return result, extras

    def _verify_retreated_start_clearance(
        self,
        *,
        config: MultiTargetSuiteConfig,
        state: _PopulationState,
    ) -> None:
        """Fail closed when the retreated start pose intersects retained cubes."""

        if self._waypoint_spheres_fn is None:
            return
        spheres = np.asarray(
            self._waypoint_spheres_fn(
                np.asarray(state.current_joints, dtype=float).reshape(1, -1)
            ),
            dtype=float,
        )
        if spheres.ndim != 3 or spheres.shape[0] != 1:
            raise ConfigurationError(
                "waypoint_spheres_fn must return shape [1, sphere, 4] for start clearance"
            )
        for target in state.accepted:
            clearance = float(
                batch_sphere_cube_clearance_m(spheres, target.center_m, target.edge_m)[0]
            )
            if clearance < config.minimum_world_collision_clearance_m:
                raise ConfigurationError(
                    "retreated start state clears retained cube "
                    f"{target.target_id!r} by {clearance:.4f} m < "
                    f"minimum_world_collision_clearance_m="
                    f"{config.minimum_world_collision_clearance_m}; "
                    "increase retreat_distance_m"
                )

    def _plan_candidate(
        self,
        *,
        config: MultiTargetSuiteConfig,
        episode_index: int,
        episode_seed: int,
        root_seed: int,
        state: _PopulationState,
        candidate: NumberedTarget,
    ) -> tuple[MultiTargetLegResult, NominalPlan | None]:
        del root_seed
        targets = tuple(state.accepted) + (candidate,)
        field = TargetField(
            targets=targets,
            placement=PlacementPolicy.RANDOM,
            order=OrderPolicy.LISTED,
            retain_targets_after_contact=True,
            contact_order_ids=tuple(target.target_id for target in targets),
        )
        episode = MultiTargetEpisode(
            episode_index=episode_index,
            root_seed=episode_seed,
            episode_seed=episode_seed,
            order_seed=episode_seed,
            field=field,
            start_position_rad=config.start_joint_position_rad,
            planner_profile=config.planner_profile,
            tip_allow_link_names=config.tip_allow_link_names,
            max_planning_failure_per_target=1,
            max_target_failures=0,
            max_reconsider_passes=0,
            max_failed_episodes=config.max_failed_episodes,
            max_consecutive_unplanned_targets=0,
            scene_revision_prefix=config.scene_revision_prefix,
            retain_targets_after_contact=True,
        )
        from_id = state.from_id
        to_id = candidate.target_id
        geometries = field.active_geometries()
        # Incremental mode forbids exclude_names: retained cubes stay in-world.
        planning_geometries = leg_world_geometries(
            geometries, active_contact_name=candidate.cube_geometry.name
        )
        scene_model = cubes_to_curobo_scene_dict(planning_geometries)
        scene_revision = (
            f"{config.scene_revision_prefix}-"
            f"{multi_cube_scene_revision(geometries) if geometries else 'empty'}"
        )
        request_id = f"ep{episode_index:03d}_{from_id}_to_{to_id}_attempt0"
        request = PlanningRequest(
            current_joint_state=NamedJointState.create(JOINT_NAMES, state.current_joints),
            surface_target=candidate.to_surface_target(),
            scene_revision=scene_revision,
            planner_profile=config.planner_profile,
            random_seed=episode_seed + state.planned_candidates,
            request_id=request_id,
            disable_collision_links=(),
            plan_grasp_to_lift=True,
            retreat_distance_m=float(config.retreat_distance_m),
        )
        plan_started = time.perf_counter()
        try:
            planner = self._planner_factory(
                request.random_seed, scene_model, config.tip_allow_link_names
            )
            outcome: PlanningOutcome = planner.plan(request)
        except (RuntimeError, ValueError, ConfigurationError) as exc:
            duration = time.perf_counter() - plan_started
            return (
                MultiTargetLegResult(
                    from_id=from_id,
                    to_id=to_id,
                    planning_succeeded=False,
                    validation_passed=False,
                    contact_kind=None,
                    failure_category=MultiTargetFailureCategory.PLAN_FAILED,
                    failure_reason=str(exc),
                    planner_status="exception",
                    planning_duration_s=duration,
                    request_id=request_id,
                    scene_revision=scene_revision,
                    attempt_index=0,
                ),
                None,
            )
        duration = time.perf_counter() - plan_started
        if (
            self._warn_planning_duration_s is not None
            and duration > self._warn_planning_duration_s
        ):
            self._console_log(
                f"WARN planning_duration_s={duration:.3f} exceeded advisory "
                f"{self._warn_planning_duration_s:.3f} on {from_id}->{to_id} "
                "(sim host evidence only; not an Orin AGX budget)"
            )
        if not outcome.succeeded or outcome.plan is None:
            failure = outcome.failure
            return (
                MultiTargetLegResult(
                    from_id=from_id,
                    to_id=to_id,
                    planning_succeeded=False,
                    validation_passed=False,
                    contact_kind=None,
                    failure_category=MultiTargetFailureCategory.PLAN_FAILED,
                    failure_reason=None if failure is None else failure.reason,
                    planner_status="" if failure is None else failure.planner_status,
                    planning_duration_s=duration,
                    request_id=request_id,
                    scene_revision=scene_revision,
                    attempt_index=0,
                ),
                None,
            )
        validated = self._validator(
            outcome.plan, request, planning_geometries, candidate.cube_geometry
        )
        if not validated.report.valid:
            return (
                MultiTargetLegResult(
                    from_id=from_id,
                    to_id=to_id,
                    planning_succeeded=True,
                    validation_passed=False,
                    contact_kind=None,
                    failure_category=MultiTargetFailureCategory.VALIDATION_FAILED,
                    failure_reason="; ".join(
                        violation.reason for violation in validated.report.violations
                    ),
                    planner_status=outcome.plan.planner_status,
                    planning_duration_s=duration,
                    request_id=request_id,
                    scene_revision=scene_revision,
                    validation_metrics=validated.report.metrics,
                    attempt_index=0,
                ),
                None,
            )
        final = tuple(float(item) for item in outcome.plan.combined_trajectory.position_rad[-1])
        leg = MultiTargetLegResult(
            from_id=from_id,
            to_id=to_id,
            planning_succeeded=True,
            validation_passed=True,
            contact_kind=None,
            failure_category=None,
            failure_reason=None,
            planner_status=outcome.plan.planner_status,
            planning_duration_s=duration,
            request_id=request_id,
            scene_revision=scene_revision,
            validation_metrics=validated.report.metrics,
            final_joint_position_rad=final,
            attempt_index=0,
        )
        detector = self._contact_detector_factory(episode, to_id)
        contact = detector.classify()
        failure_category = None
        failure_reason = None
        if contact.kind is ContactKind.PROHIBITED_BODY_CONTACT:
            failure_category = MultiTargetFailureCategory.BODY_CONTACT
            failure_reason = f"body contact on {from_id}->{to_id}"
        elif contact.kind is not ContactKind.ALLOWED_TIP_CONTACT:
            failure_category = MultiTargetFailureCategory.TIP_CONTACT_MISSED
            failure_reason = f"tip contact missed on {from_id}->{to_id}"
        return (
            replace(
                leg,
                contact_kind=contact.kind,
                motion_duration_s=0.0,
                time_to_contact_s=duration
                if contact.kind is ContactKind.ALLOWED_TIP_CONTACT
                else None,
                failure_category=failure_category,
                failure_reason=failure_reason,
            ),
            outcome.plan,
        )
