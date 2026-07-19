"""PhysX contact counting isolated from Kit-specific event object details."""

from __future__ import annotations

from typing import Any


def _path(value: Any) -> str:
    """Normalize a PhysX actor path / path-like event field."""

    return str(value) if value is not None else ""


def contact_is_prohibited(
    actor0: Any,
    actor1: Any,
    *,
    cube_path: str,
    robot_root_path: str,
) -> bool:
    """Classify robot-to-cube contacts; robot self contacts are deliberately ignored."""

    first, second = _path(actor0), _path(actor1)
    cube_involved = first == cube_path or second == cube_path
    robot_involved = first.startswith(robot_root_path) or second.startswith(robot_root_path)
    return cube_involved and robot_involved


class ProhibitedContactMonitor:
    """Count robot-to-cube contacts from the PhysX contact-report subscription.

    The simulator interface may be injected for unit tests.  If Kit/PhysX is
    unavailable, ``start`` leaves the monitor inactive and records that status
    rather than making the pure-Python unit suite import Isaac.
    """

    def __init__(self, simulation_interface: Any | None = None) -> None:
        self._interface = simulation_interface
        self._subscription: Any | None = None
        self._cube_path = ""
        self._robot_root_path = ""
        self._prohibited_events = 0
        self._available = False

    @property
    def prohibited_events(self) -> int:
        return self._prohibited_events

    def _on_contact(self, headers: Any, _contact_data: Any = None) -> None:
        for header in headers or ():
            actor0 = getattr(header, "actor0", getattr(header, "actor0_path", None))
            actor1 = getattr(header, "actor1", getattr(header, "actor1_path", None))
            if contact_is_prohibited(
                actor0,
                actor1,
                cube_path=self._cube_path,
                robot_root_path=self._robot_root_path,
            ):
                self._prohibited_events += 1

    def start(self, stage: Any, cube_path: str, robot_root_path: str) -> bool:
        """Subscribe once. ``stage`` is retained for the stable public contract."""

        del stage
        self.stop()
        if not cube_path.startswith("/") or not robot_root_path.startswith("/"):
            raise ValueError("cube_path and robot_root_path must be absolute USD paths")
        self._cube_path, self._robot_root_path = cube_path, robot_root_path
        if self._interface is None:
            try:
                from omni.physx import get_physx_simulation_interface

                self._interface = get_physx_simulation_interface()
            except ImportError:
                return False
        subscribe = getattr(self._interface, "subscribe_physics_contact_report_events", None)
        if not callable(subscribe):
            return False
        self._subscription = subscribe(self._on_contact)
        self._available = True
        return True

    def poll(self) -> int:
        """Return the current count without resetting it."""

        return self._prohibited_events

    def reset(self) -> None:
        """Reset the event count for a new episode."""

        self._prohibited_events = 0

    def stop(self) -> None:
        """Release a Kit subscription if the installed API exposes an unsubscribe hook."""

        if self._subscription is not None:
            unsubscribe = getattr(self._subscription, "unsubscribe", None)
            if callable(unsubscribe):
                unsubscribe()
            else:
                unsubscribe = getattr(
                    self._interface, "unsubscribe_physics_contact_report_events", None
                )
                if callable(unsubscribe):
                    unsubscribe(self._subscription)
        self._subscription = None
        self._available = False

    def summary(self) -> dict[str, Any]:
        return {
            "available": self._available,
            "cube_path": self._cube_path,
            "robot_root_path": self._robot_root_path,
            "prohibited_events": self._prohibited_events,
        }
