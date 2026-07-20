"""Tip-vs-body PhysX contact classification for Phase 7.2 multi-target suites."""

from __future__ import annotations

from typing import Any, Sequence

from mycobot_curobo.multi_target import ContactEvent, ContactKind


def _path(value: Any) -> str:
    return str(value) if value is not None else ""


def _link_name_from_actor_path(actor_path: str, robot_root_path: str) -> str | None:
    root = robot_root_path.rstrip("/")
    if not actor_path.startswith(root + "/") and actor_path != root:
        return None
    remainder = actor_path[len(root) :].lstrip("/")
    if not remainder:
        return None
    return remainder.split("/")[0]


def classify_robot_target_contact(
    actor0: Any,
    actor1: Any,
    *,
    target_paths: dict[str, str],
    robot_root_path: str,
    tip_allow_link_names: Sequence[str],
) -> ContactEvent:
    """Classify one PhysX contact header as tip-allowed, body-prohibited, or none.

    Body contact takes priority when both tip and body contacts are present in a
    batch; callers should fold events with ``merge_contact_events``.
    """

    first, second = _path(actor0), _path(actor1)
    path_to_id = {path: target_id for target_id, path in target_paths.items()}
    target_id = None
    other = None
    if first in path_to_id:
        target_id, other = path_to_id[first], second
    elif second in path_to_id:
        target_id, other = path_to_id[second], first
    if target_id is None or other is None:
        return ContactEvent(ContactKind.NONE)
    if not (other == robot_root_path or other.startswith(robot_root_path.rstrip("/") + "/")):
        return ContactEvent(ContactKind.NONE)
    link_name = _link_name_from_actor_path(other, robot_root_path)
    allow = tuple(tip_allow_link_names)
    path_lower = other.lower()
    tip_match = any(name.lower() in path_lower for name in allow)
    if tip_match or (link_name is not None and link_name in allow):
        return ContactEvent(
            ContactKind.ALLOWED_TIP_CONTACT,
            target_id=target_id,
            link_name=link_name,
        )
    return ContactEvent(
        ContactKind.PROHIBITED_BODY_CONTACT,
        target_id=target_id,
        link_name=link_name,
    )


def merge_contact_events(events: Sequence[ContactEvent]) -> ContactEvent:
    """Fold a batch so body contact wins over tip contact over none."""

    body = [event for event in events if event.kind is ContactKind.PROHIBITED_BODY_CONTACT]
    if body:
        return body[0]
    tip = [event for event in events if event.kind is ContactKind.ALLOWED_TIP_CONTACT]
    if tip:
        return tip[0]
    return ContactEvent(ContactKind.NONE)


class TipBodyContactMonitor:
    """Subscribe to PhysX contacts and classify tip vs body against numbered targets."""

    def __init__(self, simulation_interface: Any | None = None) -> None:
        self._interface = simulation_interface
        self._subscription: Any | None = None
        self._target_paths: dict[str, str] = {}
        self._robot_root_path = ""
        self._tip_allow_link_names: tuple[str, ...] = ()
        self._events: list[ContactEvent] = []
        self._available = False

    @property
    def available(self) -> bool:
        return self._available

    def _on_contact(self, headers: Any, _contact_data: Any = None) -> None:
        for header in headers or ():
            actor0 = getattr(header, "actor0", getattr(header, "actor0_path", None))
            actor1 = getattr(header, "actor1", getattr(header, "actor1_path", None))
            event = classify_robot_target_contact(
                actor0,
                actor1,
                target_paths=self._target_paths,
                robot_root_path=self._robot_root_path,
                tip_allow_link_names=self._tip_allow_link_names,
            )
            if event.kind is not ContactKind.NONE:
                self._events.append(event)

    def start(
        self,
        stage: Any,
        *,
        target_paths: dict[str, str],
        robot_root_path: str,
        tip_allow_link_names: Sequence[str],
    ) -> bool:
        """Subscribe once for the active target set."""

        del stage
        self.stop()
        if not robot_root_path.startswith("/"):
            raise ValueError("robot_root_path must be an absolute USD path")
        if not target_paths or any(not path.startswith("/") for path in target_paths.values()):
            raise ValueError("target_paths must map ids to absolute USD paths")
        if not tip_allow_link_names:
            raise ValueError("tip_allow_link_names must be non-empty")
        self._target_paths = dict(target_paths)
        self._robot_root_path = robot_root_path
        self._tip_allow_link_names = tuple(tip_allow_link_names)
        self._events = []
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

    def classify(self) -> ContactEvent:
        """Return the highest-priority contact observed since the last reset."""

        return merge_contact_events(self._events)

    def reset(self) -> None:
        self._events = []

    def stop(self) -> None:
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
            "target_paths": dict(self._target_paths),
            "robot_root_path": self._robot_root_path,
            "tip_allow_link_names": list(self._tip_allow_link_names),
            "event_count": len(self._events),
            "classification": self.classify().kind.value,
        }
