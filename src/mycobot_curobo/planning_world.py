"""Shared planning/validation world construction for tip-contact legs.

Option B (and Phase 7.2) require a single invariant: the planner and
independent world-clearance checks never contain the cube being tip-contacted.
Contact legality remains PhysX playback evidence only.
"""

from __future__ import annotations

from typing import Sequence

from mycobot_curobo.cube_scene import CubeGeometry, cubes_to_curobo_scene_dict
from mycobot_curobo.errors import ConfigurationError


def leg_world_geometries(
    remaining: Sequence[CubeGeometry],
    *,
    active_contact_name: str | None,
    exclude_names: Sequence[str] = (),
) -> tuple[CubeGeometry, ...]:
    """Return cubes that belong in the planner/validation world for one leg.

    Args:
        remaining: Cubes still present after tip-contact removals (or the full
            field when retain-after-contact is enabled).
        active_contact_name: Geometry name of the cube being tip-contacted on
            this leg. Always excluded so the tip may occupy the face centre.
        exclude_names: Additional geometry names to omit (e.g. a just-contacted
            cube that would collide with the start pose when retain-after-contact
            keeps it in the field).

    Returns:
        Ordered subset of ``remaining`` with the active contact and any
        explicit exclusions removed. Fail closed if ``active_contact_name`` is
        set but absent from ``remaining`` (caller bug / stale state).
    """

    names = [geometry.name for geometry in remaining]
    if len(set(names)) != len(names):
        raise ConfigurationError("remaining cube geometry names must be unique")
    excluded = {str(name) for name in exclude_names}
    if active_contact_name is not None:
        active = str(active_contact_name)
        if active not in names:
            raise ConfigurationError(
                f"active contact cube {active!r} is not among remaining geometries"
            )
        excluded.add(active)
    return tuple(geometry for geometry in remaining if geometry.name not in excluded)


def leg_world_scene_dict(
    remaining: Sequence[CubeGeometry],
    *,
    active_contact_name: str | None,
    exclude_names: Sequence[str] = (),
) -> dict[str, dict[str, dict[str, list[float]]]]:
    """Build the cuRobo cuboid scene for one tip-contact leg."""

    return cubes_to_curobo_scene_dict(
        leg_world_geometries(
            remaining,
            active_contact_name=active_contact_name,
            exclude_names=exclude_names,
        )
    )
