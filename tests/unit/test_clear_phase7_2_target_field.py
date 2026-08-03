"""Unit tests for inter-episode Phase 7.2/7.5 target-field clearing."""

from __future__ import annotations

from isaac_sim.scene_setup import PHASE7_2_TARGETS_ROOT, clear_phase7_2_target_field


class _FakePath:
    def __init__(self, path: str) -> None:
        self.pathString = path

    def __str__(self) -> str:
        return self.pathString


class _FakePrim:
    def __init__(
        self,
        path: str,
        *,
        valid: bool = True,
        children: list["_FakePrim"] | None = None,
    ):
        self._path = path
        self._valid = valid
        self._children = list(children or [])

    def IsValid(self) -> bool:
        return self._valid

    def GetPath(self) -> _FakePath:
        return _FakePath(self._path)

    def GetChildren(self) -> list["_FakePrim"]:
        return list(self._children)

    def __str__(self) -> str:
        return self._path


class _FakeStage:
    def __init__(self, root: _FakePrim | None) -> None:
        self._root = root
        self.removed: list[str] = []

    def GetPrimAtPath(self, path: str) -> _FakePrim:
        if self._root is not None and path == PHASE7_2_TARGETS_ROOT:
            return self._root
        return _FakePrim(path, valid=False)

    def RemovePrim(self, path: str) -> None:
        self.removed.append(path)
        if self._root is None:
            return
        self._root._children = [child for child in self._root._children if str(child) != path]


def test_clear_phase7_2_target_field_removes_all_children() -> None:
    children = [
        _FakePrim(f"{PHASE7_2_TARGETS_ROOT}/target_1"),
        _FakePrim(f"{PHASE7_2_TARGETS_ROOT}/target_5"),
        _FakePrim(f"{PHASE7_2_TARGETS_ROOT}/target_23"),
    ]
    stage = _FakeStage(_FakePrim(PHASE7_2_TARGETS_ROOT, children=children))
    removed = clear_phase7_2_target_field(stage)
    assert removed == 3
    assert stage.removed == [
        f"{PHASE7_2_TARGETS_ROOT}/target_1",
        f"{PHASE7_2_TARGETS_ROOT}/target_5",
        f"{PHASE7_2_TARGETS_ROOT}/target_23",
    ]


def test_clear_phase7_2_target_field_missing_root_is_noop() -> None:
    stage = _FakeStage(None)
    assert clear_phase7_2_target_field(stage) == 0
    assert stage.removed == []
