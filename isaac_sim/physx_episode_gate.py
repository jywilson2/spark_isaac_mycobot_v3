"""Host-only PhysX acceptance gate for Phase 7.5 incremental episodes.

Starts a headless Kit playback subprocess per check so cuRobo planning stays
Kit-free. Sphere-cover diagnostics are evaluated in the parent process when
``q_rad`` is present on the failure payload.
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import tempfile
from dataclasses import asdict
from enum import Enum
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]

# Kill a hung gate playback and fail closed instead of wedging the suite.
DEFAULT_GATE_TIMEOUT_S = 900.0

# Kit-injected variables that deadlock a nested SimulationApp bootstrap when
# inherited by the gate child. python.sh rebuilds all of them from scratch.
_KIT_ENV_EXACT = frozenset(
    {
        "LD_PRELOAD",
        "EXP_PATH",
        "ISAAC_PATH",
        "isaac_sim_package_path",
        "PYTHONPATH",
        "PYTHONHOME",
        "LD_LIBRARY_PATH",
    }
)
_KIT_ENV_PREFIXES = ("CARB_", "OMNI_")

from mycobot_curobo.incremental_population import IncrementalEpisodeExtras  # noqa: E402
from mycobot_curobo.multi_target import (  # noqa: E402
    MultiTargetEpisodeResult,
    MultiTargetFailureCategory,
    serialize_episode,
)
from mycobot_curobo.physx_regen import (  # noqa: E402
    PhysxDiscardRecord,
    discard_from_physx_failure_payload,
    min_self_sphere_clearance,
    sphere_link_names_from_counts,
)


def _serialize_trajectory(trajectory: Any) -> dict[str, Any]:
    position = getattr(trajectory, "position_rad", trajectory)
    if hasattr(position, "tolist"):
        position_list = position.tolist()
    else:
        position_list = list(position)
    return {
        "joint_names": list(getattr(trajectory, "joint_names", ())),
        "dt_s": float(getattr(trajectory, "dt_s", 0.0)),
        "position_rad": position_list,
        "velocity_rad_s": (
            None
            if getattr(trajectory, "velocity_rad_s", None) is None
            else trajectory.velocity_rad_s.tolist()
        ),
        "acceleration_rad_s2": (
            None
            if getattr(trajectory, "acceleration_rad_s2", None) is None
            else trajectory.acceleration_rad_s2.tolist()
        ),
        "jerk_rad_s3": (
            None
            if getattr(trajectory, "jerk_rad_s3", None) is None
            else trajectory.jerk_rad_s3.tolist()
        ),
    }


def write_single_episode_gate_bundle(
    path: Path,
    *,
    result: MultiTargetEpisodeResult,
    extra: IncrementalEpisodeExtras,
    trajectories: dict[str, Any],
    tip_allow_link_names: Sequence[str],
    lighting: dict[str, Any],
    z_density: dict[str, Any],
    root_seed: int | None,
    max_failed_episodes: int,
) -> None:
    """Write a one-episode bundle for headless PhysX gate playback."""

    payload = {
        "schema_version": 1,
        "target_population": "incremental",
        "artifact_base_name": "physx_gate_episode",
        "root_seed": root_seed,
        "seed_mode": "physx_gate",
        "episode_seeds": [int(result.episode.episode_seed)],
        "max_failed_episodes": int(max_failed_episodes),
        "suite_accepted": True,
        "tip_allow_link_names": list(tip_allow_link_names),
        "retain_targets_after_contact": True,
        "lighting": lighting,
        "z_density": z_density,
        "incremental_episodes": [
            {
                "stop_reason": extra.stop_reason.value,
                "consecutive_failures_at_stop": extra.consecutive_failures_at_stop,
                "total_target_failures": extra.total_target_failures,
                "accepted_plan_durations_s": list(extra.accepted_plan_durations_s),
                "failed_plan_durations_s": list(extra.failed_plan_durations_s),
                "draws": extra.draws,
                "planned_candidates": extra.planned_candidates,
                "geometric_rejects": asdict(extra.geometric_rejects),
                "z_band_lo_m": extra.z_band_lo_m,
                "z_band_hi_m": extra.z_band_hi_m,
                "wall_duration_s": extra.wall_duration_s,
                "populate_duration_s": extra.wall_duration_s,
                "tip_contacts": len(result.contacted_ids),
                "candidate_failures": [asdict(record) for record in extra.candidate_failures],
            }
        ],
        "results": [asdict(result)],
        "frozen_requests": [serialize_episode(result.episode)],
        "trajectories": {
            request_id: (
                trajectory if isinstance(trajectory, dict) else _serialize_trajectory(trajectory)
            )
            for request_id, trajectory in trajectories.items()
        },
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(
            payload,
            indent=2,
            sort_keys=True,
            default=lambda value: value.value if isinstance(value, Enum) else value,
        )
        + "\n",
        encoding="utf-8",
    )


def sanitized_gate_env(
    base_env: Mapping[str, str],
    *,
    repo_root: Path = REPO_ROOT,
) -> dict[str, str]:
    """Child environment for the gate playback subprocess.

    Strips Kit/carb variables injected into the parent planner process (e.g.
    ``LD_PRELOAD=libcarb.so``, ``CARB_APP_PATH``, ``EXP_PATH``, Kit
    ``PYTHONPATH``/``LD_LIBRARY_PATH``): inheriting them deadlocks the child's
    carb bootstrap in a futex wait before Kit loads. ``python.sh`` rebuilds
    them cleanly. ``PYTHONPATH`` is reset to exactly the repo entries so
    repeated gate launches cannot accumulate duplicates.
    """

    env = {
        key: value
        for key, value in base_env.items()
        if key not in _KIT_ENV_EXACT and not key.startswith(_KIT_ENV_PREFIXES)
    }
    env["PYTHONPATH"] = f"{repo_root / 'src'}:{repo_root}"
    return env


def resolve_gate_python(python_exe: str | None = None) -> str:
    """Executable for the gate child: explicit arg, then Isaac ``python.sh``.

    Never default to ``sys.executable``: inside the planner that is Kit's
    embedded python, and nesting a second Kit under it hangs.
    """

    if python_exe:
        return python_exe
    candidate = os.environ.get("ISAACSIM_PYTHON_EXE")
    if candidate and Path(candidate).is_file():
        return candidate
    isaacsim_path = os.environ.get("ISAACSIM_PATH")
    if isaacsim_path:
        python_sh = Path(isaacsim_path) / "python.sh"
        if python_sh.is_file():
            return str(python_sh)
    return sys.executable


def _hard_physx_failure_from_result(result_payload: dict[str, Any]) -> dict[str, Any] | None:
    """Map a playback episode result to a hard PhysX failure payload, if any."""

    category = result_payload.get("failure_category")
    if category in {
        MultiTargetFailureCategory.SELF_COLLISION.value,
        "self_collision",
        "prohibited_self_collision",
    }:
        return {
            "category": "self_collision",
            "reason": result_payload.get("failure_reason") or "self_collision",
        }
    if category in {
        MultiTargetFailureCategory.BODY_CONTACT.value,
        "body_contact",
        "prohibited_body_contact",
    }:
        return {
            "category": "body_contact",
            "reason": result_payload.get("failure_reason") or "body_contact",
        }
    # Also scan legs when the episode-level category is unset.
    for leg in result_payload.get("legs") or []:
        leg_cat = leg.get("failure_category")
        if leg_cat in {
            MultiTargetFailureCategory.SELF_COLLISION.value,
            "self_collision",
        }:
            return {
                "category": "self_collision",
                "leg_from_id": leg.get("from_id"),
                "leg_to_id": leg.get("to_id"),
                "request_id": leg.get("request_id"),
                "reason": leg.get("failure_reason") or "self_collision",
            }
        if leg_cat in {
            MultiTargetFailureCategory.BODY_CONTACT.value,
            "body_contact",
        }:
            return {
                "category": "body_contact",
                "leg_from_id": leg.get("from_id"),
                "leg_to_id": leg.get("to_id"),
                "request_id": leg.get("request_id"),
                "reason": leg.get("failure_reason") or "body_contact",
            }
    return None


def run_headless_physx_gate(
    *,
    result: MultiTargetEpisodeResult,
    extra: IncrementalEpisodeExtras,
    trajectories: dict[str, Any],
    tip_allow_link_names: Sequence[str],
    lighting: dict[str, Any],
    z_density: dict[str, Any],
    root_seed: int | None,
    max_failed_episodes: int,
    usd: Path,
    repo_root: Path = REPO_ROOT,
    python_exe: str | None = None,
    sphere_clearance_fn: Callable[[np.ndarray], tuple[float, str | None]] | None = None,
    work_dir: Path | None = None,
    timeout_s: float = DEFAULT_GATE_TIMEOUT_S,
) -> tuple[bool, PhysxDiscardRecord | None]:
    """Headless-play one episode; return (passed, discard_or_none)."""

    # Empty / already-failed population episodes: tip-miss alone does not regen.
    if not result.succeeded and result.failure_category in {
        MultiTargetFailureCategory.INSUFFICIENT_TARGETS,
        MultiTargetFailureCategory.PHYSX_REGENERATION_EXHAUSTED,
    }:
        return True, None
    playable = any(leg.planning_succeeded and leg.validation_passed for leg in result.legs)
    if not playable:
        # Nothing to PhysX-validate; treat as pass for the gate (population failure).
        return True, None

    exe = resolve_gate_python(python_exe)
    child_env = sanitized_gate_env(os.environ, repo_root=repo_root)
    with tempfile.TemporaryDirectory(prefix="physx_gate_", dir=work_dir) as tmp:
        tmp_path = Path(tmp)
        bundle_path = tmp_path / "episode.bundle.json"
        report_path = tmp_path / "episode.report.json"
        write_single_episode_gate_bundle(
            bundle_path,
            result=result,
            extra=extra,
            trajectories=trajectories,
            tip_allow_link_names=tip_allow_link_names,
            lighting=lighting,
            z_density=z_density,
            root_seed=root_seed,
            max_failed_episodes=max_failed_episodes,
        )
        cmd = [
            exe,
            str(repo_root / "isaac_sim" / "play_multi_target_suite.py"),
            "--repo-root",
            str(repo_root),
            "--bundle",
            str(bundle_path),
            "--usd",
            str(usd),
            "--headless",
            "--auto-exit",
            "--episode-index",
            "0",
            "--output-report",
            str(report_path),
        ]
        print(
            f"phase7_5_physx_regen: launching headless gate "
            f"episode_seed={result.episode.episode_seed} exe={exe} "
            f"timeout_s={timeout_s:.0f} usd={usd}",
            flush=True,
        )
        # New session avoids Kit sharing the planner's process group (SIGTERM)
        # and lets a timeout kill the whole gate process group, Kit children
        # included. Stream child stdout/stderr so smoke logs stay complete.
        proc = subprocess.Popen(
            cmd,
            env=child_env,
            start_new_session=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        try:
            stdout, _ = proc.communicate(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            stdout, _ = proc.communicate()
            if stdout:
                for line in stdout.splitlines():
                    print(line, flush=True)
            print(
                f"phase7_5_physx_regen: gate timeout after {timeout_s:.0f}s; "
                "discarding episode (fail closed)",
                flush=True,
            )
            discard = PhysxDiscardRecord(
                episode_index=int(result.episode.episode_index),
                regen_attempt=1,
                episode_seed=int(result.episode.episode_seed),
                category="physx_overlap",
                leg_from_id=None,
                leg_to_id=None,
                request_id=None,
                links=None,
                target_id=None,
                waypoint_index=None,
                waypoint_count=None,
                u=None,
                t_s=None,
                q_rad=None,
                sphere_clearance_m=None,
                sphere_pair=None,
                reason=f"gate_timeout after {timeout_s:.0f}s (playback killed)",
            )
            return False, discard
        returncode = int(proc.returncode)
        if stdout:
            for line in stdout.splitlines():
                print(line, flush=True)
        print(
            f"phase7_5_physx_regen: gate play finished exit={returncode} "
            f"report_exists={report_path.is_file()}",
            flush=True,
        )
        if not report_path.is_file():
            discard = PhysxDiscardRecord(
                episode_index=int(result.episode.episode_index),
                regen_attempt=1,
                episode_seed=int(result.episode.episode_seed),
                category="physx_overlap",
                leg_from_id=None,
                leg_to_id=None,
                request_id=None,
                links=None,
                target_id=None,
                waypoint_index=None,
                waypoint_count=None,
                u=None,
                t_s=None,
                q_rad=None,
                sphere_clearance_m=None,
                sphere_pair=None,
                reason=(f"physx gate produced no report (play exit={returncode})"),
            )
            return False, discard
        report = json.loads(report_path.read_text(encoding="utf-8"))
        failures = list(report.get("physx_failures") or [])
        failure_payload = failures[0] if failures else None
        play_results = list(report.get("results") or [])
        play_result = play_results[0] if play_results else {}
        if failure_payload is None:
            failure_payload = _hard_physx_failure_from_result(play_result)
        # Tip-miss alone: do not discard/regenerate (spec).
        if failure_payload is None:
            if returncode == 0 and bool(play_result.get("succeeded", False)):
                return True, None
            tip_miss = play_result.get("failure_category") in {
                MultiTargetFailureCategory.TIP_CONTACT_MISSED.value,
                "tip_contact_missed",
            }
            if tip_miss:
                return True, None
            # Unknown non-zero play failure — treat as hard PhysX/overlap.
            failure_payload = {
                "category": "physx_overlap",
                "reason": (
                    play_result.get("failure_reason") or f"physx gate play exit={returncode}"
                ),
            }
        category = str(failure_payload.get("category") or "")
        if category not in {"self_collision", "body_contact", "physx_overlap"}:
            # Non-hard categories should not trigger regen.
            return True, None

        sphere_clearance_m = None
        sphere_pair = None
        q_raw = failure_payload.get("q_rad")
        if q_raw is not None and sphere_clearance_fn is not None:
            try:
                sphere_clearance_m, sphere_pair = sphere_clearance_fn(
                    np.asarray(q_raw, dtype=float).reshape(-1)
                )
            except Exception as exc:  # noqa: BLE001 — diagnostics must not abort gate
                print(
                    f"phase7_5_physx_regen: sphere clearance eval failed: {exc}",
                    flush=True,
                )
        discard = discard_from_physx_failure_payload(
            episode_index=int(result.episode.episode_index),
            regen_attempt=1,
            episode_seed=int(result.episode.episode_seed),
            failure=failure_payload,
            sphere_clearance_m=sphere_clearance_m,
            sphere_pair=sphere_pair,
        )
        return False, discard


def build_sphere_clearance_fn(
    *,
    waypoint_spheres_fn: Callable[[np.ndarray], np.ndarray],
    collision_pairs: np.ndarray,
    sphere_link_names: Sequence[str] | None = None,
) -> Callable[[np.ndarray], tuple[float, str | None]]:
    """Wrap host FK spheres + active pairs into a q_rad → clearance helper."""

    pairs = np.asarray(collision_pairs, dtype=int)

    def _evaluate(q_rad: np.ndarray) -> tuple[float, str | None]:
        q = np.asarray(q_rad, dtype=float).reshape(1, -1)
        spheres = np.asarray(waypoint_spheres_fn(q), dtype=float)
        if spheres.ndim == 3:
            spheres = spheres[0]
        return min_self_sphere_clearance(spheres, pairs, sphere_link_names=sphere_link_names)

    return _evaluate


def resolve_prepared_usd(repo_root: Path, usd: Path | None = None) -> Path:
    """Locate the prepared MyCobot USD used by suite playback."""

    if usd is not None and usd.is_file():
        return usd.resolve()
    from isaac_sim.urdf_utils import default_prepared_urdf

    default_usd = default_prepared_urdf(repo_root).with_suffix(".usd")
    nested = default_usd.with_suffix("") / default_usd.with_suffix(".usda").name
    for candidate in (default_usd, nested):
        if candidate.is_file():
            return candidate.resolve()
    raise FileNotFoundError(f"prepared robot USD not found under {repo_root}")


def collision_pairs_from_planner(planner: Any) -> np.ndarray:
    """Extract active self-collision sphere pairs from a cuRobo MotionGen-like planner."""

    pairs = (
        planner.kinematics.get_self_collision_config()
        .collision_pairs.detach()
        .cpu()
        .numpy()
        .astype(int)
    )
    return np.asarray(pairs, dtype=int)


__all__ = [
    "DEFAULT_GATE_TIMEOUT_S",
    "build_sphere_clearance_fn",
    "collision_pairs_from_planner",
    "resolve_gate_python",
    "resolve_prepared_usd",
    "run_headless_physx_gate",
    "sanitized_gate_env",
    "sphere_link_names_from_counts",
    "write_single_episode_gate_bundle",
]
