from types import SimpleNamespace

from isaac_sim.tip_body_contact import (
    TipBodyContactMonitor,
    classify_robot_target_contact,
    match_target_id,
    merge_contact_events,
    tip_allow_link_matches,
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


def test_child_mesh_and_flange_collision_prim_count_as_tip() -> None:
    paths = {"1": "/World/Phase7_2/Targets/target_1"}
    tip = classify_robot_target_contact(
        "/World/Robot/joint6_flange/collisions/mesh_0",
        "/World/Phase7_2/Targets/target_1/Cube",
        target_paths=paths,
        robot_root_path="/World/Robot",
        tip_allow_link_names=("joint6_flange",),
    )
    assert tip.kind is ContactKind.ALLOWED_TIP_CONTACT
    assert tip.target_id == "1"
    assert match_target_id("/World/Phase7_2/Targets/target_1/Cube", paths) == "1"
    assert tip_allow_link_matches("/World/Robot/joint6_flange/collisions", ("joint6_flange",))


def test_active_target_tip_wins_over_same_target_body_in_merge() -> None:
    merged = merge_contact_events(
        (
            ContactEvent(ContactKind.PROHIBITED_BODY_CONTACT, target_id="1", link_name="joint5"),
            ContactEvent(
                ContactKind.ALLOWED_TIP_CONTACT,
                target_id="1",
                link_name="joint6_flange",
            ),
        ),
        active_target_id="1",
    )
    assert merged.kind is ContactKind.ALLOWED_TIP_CONTACT
    # Body on a different target still fails closed.
    merged_other = merge_contact_events(
        (
            ContactEvent(ContactKind.ALLOWED_TIP_CONTACT, target_id="1"),
            ContactEvent(ContactKind.PROHIBITED_BODY_CONTACT, target_id="2"),
        ),
        active_target_id="1",
    )
    assert merged_other.kind is ContactKind.PROHIBITED_BODY_CONTACT


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
        active_target_id="1",
        log_contacts=True,
    )
    interface.callback(
        [
            SimpleNamespace(
                actor0="/World/Robot/joint6_flange/collisions",
                actor1="/World/Targets/target_1/Cube",
            )
        ]
    )
    assert monitor.classify().kind is ContactKind.ALLOWED_TIP_CONTACT
    assert monitor.summary()["contact_diagnostics"]
    monitor.reset()
    interface.callback(
        [
            SimpleNamespace(actor0="/World/Robot/joint2", actor1="/World/Targets/target_1"),
        ]
    )
    # Active-target tip priority does not apply when only body is present.
    assert monitor.classify().kind is ContactKind.PROHIBITED_BODY_CONTACT
    monitor.stop()
