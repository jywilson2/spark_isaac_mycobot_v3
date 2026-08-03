"""Phase 7.5 post-episode PhysX accept/regen helpers (no Isaac / PhysX imports)."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Mapping, Sequence

import numpy as np


@dataclass(frozen=True)
class PhysxDiscardRecord:
    """One PhysX-failed population attempt discarded before suite acceptance."""

    episode_index: int
    regen_attempt: int
    episode_seed: int
    category: str
    leg_from_id: str | None
    leg_to_id: str | None
    request_id: str | None
    links: str | None
    target_id: str | None
    waypoint_index: int | None
    waypoint_count: int | None
    u: float | None
    t_s: float | None
    q_rad: tuple[float, ...] | None
    sphere_clearance_m: float | None
    sphere_pair: str | None
    reason: str

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        if self.q_rad is not None:
            payload["q_rad"] = list(self.q_rad)
        return payload


@dataclass(frozen=True)
class PhysxGateEpisodeOutcome:
    """Result of one host PhysX acceptance check for a populated episode."""

    passed: bool
    discard: PhysxDiscardRecord | None = None


def physx_retry_episode_seed(base_episode_seed: int, regen_attempt: int) -> int:
    """Return the next population seed after a 1-based discard ordinal.

    First population uses ``base_episode_seed``. After discard ``k`` (1-based),
    the next attempt uses ``base_episode_seed + k``.
    """

    attempt = int(regen_attempt)
    if attempt < 1:
        raise ValueError("regen_attempt must be >= 1")
    return int(base_episode_seed) + attempt


def format_physx_regen_discard_line(
    *,
    episode_index: int,
    episode_count: int,
    regen_attempt: int,
    max_physx_regenerations: int,
    discard: PhysxDiscardRecord,
) -> str:
    """Format a normative ``phase7_5_physx_regen: … DISCARD`` console line."""

    leg = "n/a"
    if discard.leg_from_id is not None and discard.leg_to_id is not None:
        leg = f"{discard.leg_from_id}->{discard.leg_to_id}"
        if discard.request_id:
            leg = f"{leg} request={discard.request_id}"
    links = discard.links or "n/a"
    waypoint = "n/a"
    if (
        discard.waypoint_index is not None
        and discard.waypoint_count is not None
        and discard.u is not None
    ):
        waypoint = f"{discard.waypoint_index}/{discard.waypoint_count} u={discard.u:.3f}"
        if discard.t_s is not None:
            waypoint = f"{waypoint} t_s={discard.t_s:.3f}"
    q_text = (
        "n/a" if discard.q_rad is None else "[" + ",".join(f"{q:.4f}" for q in discard.q_rad) + "]"
    )
    sphere = "n/a"
    if discard.sphere_clearance_m is not None:
        sphere = f"sphere_clearance_m={discard.sphere_clearance_m:.4f}"
        if discard.sphere_pair:
            sphere = f"{sphere} sphere_pair={discard.sphere_pair}"
    target = "" if discard.target_id is None else f" | target_id={discard.target_id}"
    return (
        f"phase7_5_physx_regen: ep {episode_index + 1}/{episode_count} "
        f"regen {regen_attempt}/{max_physx_regenerations} DISCARD | "
        f"category {discard.category} | leg {leg} | links {links} | "
        f"waypoint {waypoint} | q_rad={q_text} | {sphere}{target} | "
        f"reason {discard.reason}"
    )


def format_physx_regen_retry_line(
    *,
    episode_index: int,
    episode_count: int,
    regen_attempt: int,
    max_physx_regenerations: int,
    next_episode_seed: int,
) -> str:
    return (
        f"phase7_5_physx_regen: ep {episode_index + 1}/{episode_count} "
        f"regen {regen_attempt}/{max_physx_regenerations} RETRY | "
        f"next_episode_seed={next_episode_seed}"
    )


def format_physx_regen_accept_line(
    *,
    episode_index: int,
    episode_count: int,
    physx_regen_attempts: int,
) -> str:
    return (
        f"phase7_5_physx_regen: ep {episode_index + 1}/{episode_count} ACCEPT | "
        f"physx_regen_attempts={physx_regen_attempts}"
    )


def format_physx_regen_exhausted_line(
    *,
    episode_index: int,
    episode_count: int,
    max_physx_regenerations: int,
) -> str:
    return (
        f"phase7_5_physx_regen: ep {episode_index + 1}/{episode_count} EXHAUSTED | "
        f"physx_regeneration_exhausted after {max_physx_regenerations} discard(s)"
    )


def min_self_sphere_clearance(
    spheres_xyzr: np.ndarray,
    collision_pairs: np.ndarray,
    *,
    sphere_link_names: Sequence[str] | None = None,
) -> tuple[float, str | None]:
    """Return (min signed clearance_m, sphere_pair label) for one joint state.

    ``spheres_xyzr`` shape ``[S,4]``; ``collision_pairs`` shape ``[P,2]`` of
    sphere indices. Negative clearance means the active sphere model already
    sees a fold.
    """

    spheres = np.asarray(spheres_xyzr, dtype=float)
    pairs = np.asarray(collision_pairs, dtype=int)
    if spheres.ndim != 2 or spheres.shape[1] != 4:
        raise ValueError("spheres_xyzr must have shape [S, 4]")
    if pairs.ndim != 2 or pairs.shape[1] != 2:
        raise ValueError("collision_pairs must have shape [P, 2]")
    if pairs.shape[0] == 0:
        return float("inf"), None
    first = spheres[pairs[:, 0], :]
    second = spheres[pairs[:, 1], :]
    clearances = np.linalg.norm(first[:, :3] - second[:, :3], axis=-1) - first[:, 3] - second[:, 3]
    index = int(np.argmin(clearances))
    clearance = float(clearances[index])
    i = int(pairs[index, 0])
    j = int(pairs[index, 1])
    if sphere_link_names is not None and len(sphere_link_names) == spheres.shape[0]:
        label = f"{sphere_link_names[i]}@{i}↔{sphere_link_names[j]}@{j}"
    else:
        label = f"sphere{i}↔sphere{j}"
    return clearance, label


def sphere_link_names_from_counts(
    collision_sphere_count_by_link: Mapping[str, int],
) -> tuple[str, ...]:
    """Expand per-link sphere counts to a flat index→link name table."""

    names: list[str] = []
    for link, count in collision_sphere_count_by_link.items():
        names.extend([str(link)] * int(count))
    return tuple(names)


def discard_from_physx_failure_payload(
    *,
    episode_index: int,
    regen_attempt: int,
    episode_seed: int,
    failure: Mapping[str, Any],
    sphere_clearance_m: float | None = None,
    sphere_pair: str | None = None,
) -> PhysxDiscardRecord:
    """Build a discard record from play-report ``physx_failure`` mapping."""

    q_raw = failure.get("q_rad")
    q_rad = None if q_raw is None else tuple(float(v) for v in q_raw)
    return PhysxDiscardRecord(
        episode_index=int(episode_index),
        regen_attempt=int(regen_attempt),
        episode_seed=int(episode_seed),
        category=str(failure.get("category") or "physx_overlap"),
        leg_from_id=None if failure.get("leg_from_id") is None else str(failure["leg_from_id"]),
        leg_to_id=None if failure.get("leg_to_id") is None else str(failure["leg_to_id"]),
        request_id=None if failure.get("request_id") is None else str(failure["request_id"]),
        links=None if failure.get("links") is None else str(failure["links"]),
        target_id=None if failure.get("target_id") is None else str(failure["target_id"]),
        waypoint_index=(
            None if failure.get("waypoint_index") is None else int(failure["waypoint_index"])
        ),
        waypoint_count=(
            None if failure.get("waypoint_count") is None else int(failure["waypoint_count"])
        ),
        u=None if failure.get("u") is None else float(failure["u"]),
        t_s=None if failure.get("t_s") is None else float(failure["t_s"]),
        q_rad=q_rad,
        sphere_clearance_m=sphere_clearance_m,
        sphere_pair=sphere_pair,
        reason=str(failure.get("reason") or failure.get("category") or "physx hard error"),
    )
