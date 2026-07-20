from types import SimpleNamespace

from isaac_sim.tip_body_contact import (
    TipBodyContactMonitor,
    classify_robot_target_contact,
    merge_contact_events,
)
from mycobot_curobo.multi_target import ContactEvent, ContactKind


def test_tip_vs_body_classification() -> None:
    paths = {"1": "/World/Targets/target_1", "2": "/World/Targets/target_2"}
    tip = classify_robot_target_contact(
        "/World/Robot/joint6_flange",
        "/World/Targets/target_1",
        target_paths=paths,
        robot_root_path="/World/Robot",
        tip_allow_link_names=("joint6_flange",),
    )
    assert tip.kind is ContactKind.ALLOWED_TIP_CONTACT
    assert tip.target_id == "1"
    body = classify_robot_target_contact(
        "/World/Robot/joint3",
        "/World/Targets/target_2",
        target_paths=paths,
        robot_root_path="/World/Robot",
        tip_allow_link_names=("joint6_flange",),
    )
    assert body.kind is ContactKind.PROHIBITED_BODY_CONTACT
    assert body.target_id == "2"
    assert (
        classify_robot_target_contact(
            "/World/Robot/joint3",
            "/World/Robot/joint4",
            target_paths=paths,
            robot_root_path="/World/Robot",
            tip_allow_link_names=("joint6_flange",),
        ).kind
        is ContactKind.NONE
    )


def test_body_priority_in_merge() -> None:
    merged = merge_contact_events(
        (
            ContactEvent(ContactKind.ALLOWED_TIP_CONTACT, target_id="1"),
            ContactEvent(ContactKind.PROHIBITED_BODY_CONTACT, target_id="2"),
        )
    )
    assert merged.kind is ContactKind.PROHIBITED_BODY_CONTACT


def test_monitor_records_injected_events() -> None:
    class _Interface:
        def __init__(self) -> None:
            self.callback = None

        def subscribe_physics_contact_report_events(self, callback):
            self.callback = callback
            return self

        def unsubscribe(self) -> None:
            self.callback = None

    interface = _Interface()
    monitor = TipBodyContactMonitor(interface)
    assert monitor.start(
        None,
        target_paths={"1": "/World/Targets/target_1"},
        robot_root_path="/World/Robot",
        tip_allow_link_names=("joint6_flange",),
    )
    interface.callback(
        [
            SimpleNamespace(
                actor0="/World/Robot/joint6_flange",
                actor1="/World/Targets/target_1",
            )
        ]
    )
    assert monitor.classify().kind is ContactKind.ALLOWED_TIP_CONTACT
    monitor.reset()
    interface.callback(
        [
            SimpleNamespace(actor0="/World/Robot/joint2", actor1="/World/Targets/target_1"),
        ]
    )
    assert monitor.classify().kind is ContactKind.PROHIBITED_BODY_CONTACT
    monitor.stop()
