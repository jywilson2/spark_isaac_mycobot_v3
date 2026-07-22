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


def _path_segments(path: str) -> tuple[str, ...]:
    return tuple(segment for segment in path.strip("/").split("/") if segment)


def tip_allow_link_matches(actor_path: str, tip_allow_link_names: Sequence[str]) -> bool:
    """True when any tip-allow name matches a path segment (case-insensitive)."""

    segments = {segment.lower() for segment in _path_segments(actor_path)}
    for name in tip_allow_link_names:
        key = str(name).lower()
        if key in segments:
            return True
        # Also accept substring matches on the full path for nested mesh names.
        if key and key in actor_path.lower():
            return True
    return False


def match_target_id(actor_path: str, target_paths: dict[str, str]) -> str | None:
    """Match a PhysX actor to a target id, including child mesh paths under the cube."""

    if actor_path in target_paths.values():
        for target_id, path in target_paths.items():
            if path == actor_path:
                return target_id
    # Prefer the longest matching target prefix so nested labels/meshes resolve.
    best_id: str | None = None
    best_len = -1
    for target_id, path in target_paths.items():
        root = path.rstrip("/")
        if actor_path == root or actor_path.startswith(root + "/"):
            if len(root) > best_len:
                best_id = target_id
                best_len = len(root)
    return best_id


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
    batch; callers should fold events with ``merge_contact_events``. Tip matching
    uses path segments so flange collision meshes under ``joint6_flange`` count.
    Target matching accepts child prims under the registered cube path (edge /
    face mesh contacts during flange overhang tip contact).
    """

    first, second = _path(actor0), _path(actor1)
    target_id = match_target_id(first, target_paths)
    other = second if target_id is not None else None
    if target_id is None:
        target_id = match_target_id(second, target_paths)
        other = first if target_id is not None else None
    if target_id is None or other is None:
        return ContactEvent(ContactKind.NONE)
    root = robot_root_path.rstrip("/")
    if not (other == root or other.startswith(root + "/")):
        return ContactEvent(ContactKind.NONE)
    link_name = _link_name_from_actor_path(other, robot_root_path)
    if tip_allow_link_matches(other, tip_allow_link_names):
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


def merge_contact_events(
    events: Sequence[ContactEvent],
    *,
    active_target_id: str | None = None,
) -> ContactEvent:
    """Fold a batch so body contact wins over tip contact over none.

    When ``active_target_id`` is set, tip contact on that active target wins over
    body contact on the **same** target. Flange overhang on a face smaller than
    the flange can produce mixed mesh reports; true body hits on *other* targets
    still fail closed.
    """

    body = [event for event in events if event.kind is ContactKind.PROHIBITED_BODY_CONTACT]
    tip = [event for event in events if event.kind is ContactKind.ALLOWED_TIP_CONTACT]
    if active_target_id is not None:
        tip_active = [event for event in tip if event.target_id == active_target_id]
        body_other = [event for event in body if event.target_id != active_target_id]
        body_active = [event for event in body if event.target_id == active_target_id]
        if body_other:
            return body_other[0]
        if tip_active:
            return tip_active[0]
        if body_active:
            return body_active[0]
    if body:
        return body[0]
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
        self._active_target_id: str | None = None
        self._events: list[ContactEvent] = []
        self._raw_log: list[dict[str, str | None]] = []
        self._available = False
        self._log_contacts = False

    @property
    def available(self) -> bool:
        return self._available

    def set_active_target_id(self, target_id: str | None) -> None:
        """Restrict tip-over-body merge priority to the active contact target."""

        self._active_target_id = None if target_id is None else str(target_id)

    def enable_contact_diagnostics(self, enabled: bool = True) -> None:
        self._log_contacts = bool(enabled)

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
            if self._log_contacts:
                self._raw_log.append(
                    {
                        "actor0": _path(actor0),
                        "actor1": _path(actor1),
                        "kind": event.kind.value,
                        "target_id": event.target_id,
                        "link_name": event.link_name,
                        "active_target_id": self._active_target_id,
                    }
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
        active_target_id: str | None = None,
        log_contacts: bool = False,
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
        self._active_target_id = None if active_target_id is None else str(active_target_id)
        self._log_contacts = bool(log_contacts)
        self._events = []
        self._raw_log = []
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

        return merge_contact_events(self._events, active_target_id=self._active_target_id)

    def reset(self) -> None:
        self._events = []
        self._raw_log = []

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
            "active_target_id": self._active_target_id,
            "event_count": len(self._events),
            "classification": self.classify().kind.value,
            "contact_diagnostics": list(self._raw_log),
        }
