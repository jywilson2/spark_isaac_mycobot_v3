from types import SimpleNamespace

from isaac_sim.contact_monitor import ProhibitedContactMonitor, contact_is_prohibited


class _Interface:
    def __init__(self) -> None:
        self.callback = None

    def subscribe_physics_contact_report_events(self, callback):
        self.callback = callback
        return self

    def unsubscribe(self) -> None:
        self.callback = None


def test_contact_classifier_distinguishes_cube_from_robot_self_contact() -> None:
    assert contact_is_prohibited(
        "/World/Robot/link_6",
        "/World/Cubes/cube",
        cube_path="/World/Cubes/cube",
        robot_root_path="/World/Robot",
    )
    assert not contact_is_prohibited(
        "/World/Robot/link_5",
        "/World/Robot/link_6",
        cube_path="/World/Cubes/cube",
        robot_root_path="/World/Robot",
    )
    assert not contact_is_prohibited(
        "/World/Cubes/cube",
        "/World/table",
        cube_path="/World/Cubes/cube",
        robot_root_path="/World/Robot",
    )


def test_monitor_counts_injected_contact_headers() -> None:
    interface = _Interface()
    monitor = ProhibitedContactMonitor(interface)
    assert monitor.start(None, "/World/Cubes/cube", "/World/Robot")
    assert interface.callback is not None
    interface.callback(
        [
            SimpleNamespace(actor0="/World/Robot/link_6", actor1="/World/Cubes/cube"),
            SimpleNamespace(actor0="/World/Robot/link_5", actor1="/World/Robot/link_6"),
        ]
    )
    assert monitor.poll() == 1
    monitor.reset()
    assert monitor.poll() == 0
    monitor.stop()
    assert interface.callback is None
