"""Tests for the Phase 1 robot-model inspection CLI."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path


def _load_script_module():
    path = Path(__file__).resolve().parents[2] / "scripts" / "inspect_robot_model.py"
    spec = importlib.util.spec_from_file_location("inspect_robot_model", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_inspection_cli_emits_cpu_model_json(capsys) -> None:
    module = _load_script_module()

    assert module.main([]) == 0

    payload = json.loads(capsys.readouterr().out)
    assert payload["base_link"] == "g_base"
    assert payload["tcp_link"] == "tcp_link"
    assert len(payload["joint_names"]) == 6
    assert payload["gpu_checked"] is False
    assert sum(payload["collision_sphere_count_by_link"].values()) == 32


def test_host_wrapper_uses_isaac_python_and_gpu_mode() -> None:
    root = Path(__file__).resolve().parents[2]
    wrapper = (root / "scripts" / "host" / "inspect_robot_model.sh").read_text(encoding="utf-8")

    assert "env.isaac_host.sh" in wrapper
    assert '"${ISAACSIM_PYTHON_EXE}"' in wrapper
    assert "scripts/inspect_robot_model.py" in wrapper
    assert "--gpu" in wrapper
