"""Prepared MyCobot USD self-collision enhancer (gitignored prepared tree)."""

from __future__ import annotations

from pathlib import Path

import pytest

from isaac_sim.prepared_usd_self_collision import enhance_prepared_mycobot_usd

REPO = Path(__file__).resolve().parents[2]
PREPARED = REPO / "assets/mycobot_280_m5/prepared/mycobot_280_m5"


def _write_minimal_prepared(root: Path) -> None:
    physics = root / "payloads/Physics"
    physics.mkdir(parents=True)
    (physics / "physics.usda").write_text(
        'over "Geometry" {\n        bool newton:selfCollisionEnabled = 0\n}\n',
        encoding="utf-8",
    )
    (physics / "physx.usda").write_text(
        'over "firefighter"\n{\n    over "Physics"\n    {\n    }\n}\n',
        encoding="utf-8",
    )
    (root / "payloads/instances.usda").write_text(
        'def Xform "joint1_1" {\n'
        '    over "joint1" (\n'
        '        apiSchemas = ["PhysicsCollisionAPI", "NewtonCollisionAPI", '
        '"PhysicsMeshCollisionAPI", "NewtonMeshCollisionAPI"]\n'
        "    )\n"
        "    {\n"
        '        token physics:approximation = "convexHull"\n'
        '        token purpose = "guide"\n'
        "    }\n"
        "}\n"
        'def Xform "joint2_1" {\n'
        '    over "joint2" (\n'
        '        apiSchemas = ["PhysicsCollisionAPI", "NewtonCollisionAPI", '
        '"PhysicsMeshCollisionAPI", "NewtonMeshCollisionAPI"]\n'
        "    )\n"
        "    {\n"
        '        token physics:approximation = "convexHull"\n'
        '        token purpose = "guide"\n'
        "    }\n"
        "}\n"
        'def Xform "joint3_1" {\n'
        '    over "joint3" (\n'
        '        apiSchemas = ["PhysicsCollisionAPI", "NewtonCollisionAPI", '
        '"PhysicsMeshCollisionAPI", "NewtonMeshCollisionAPI"]\n'
        "    )\n"
        "    {\n"
        '        token physics:approximation = "convexHull"\n'
        '        token purpose = "guide"\n'
        "    }\n"
        "}\n"
        'def Xform "joint4_1" {\n'
        '    over "joint4" (\n'
        '        apiSchemas = ["PhysicsCollisionAPI", "NewtonCollisionAPI", '
        '"PhysicsMeshCollisionAPI", "NewtonMeshCollisionAPI"]\n'
        "    )\n"
        "    {\n"
        '        token physics:approximation = "convexHull"\n'
        '        token purpose = "guide"\n'
        "    }\n"
        "}\n"
        'def Xform "joint5_1" {\n'
        '    over "joint5" (\n'
        '        apiSchemas = ["PhysicsCollisionAPI", "NewtonCollisionAPI", '
        '"PhysicsMeshCollisionAPI", "NewtonMeshCollisionAPI"]\n'
        "    )\n"
        "    {\n"
        '        token physics:approximation = "convexHull"\n'
        '        token purpose = "guide"\n'
        "    }\n"
        "}\n"
        'def Xform "joint6_1" {\n'
        '    over "joint6" (\n'
        '        apiSchemas = ["PhysicsCollisionAPI", "NewtonCollisionAPI", '
        '"PhysicsMeshCollisionAPI", "NewtonMeshCollisionAPI"]\n'
        "    )\n"
        "    {\n"
        '        token physics:approximation = "convexHull"\n'
        '        token purpose = "guide"\n'
        "    }\n"
        "}\n"
        'def Xform "joint7_1" {\n'
        '    over "joint7" (\n'
        '        apiSchemas = ["PhysicsCollisionAPI", "NewtonCollisionAPI", '
        '"PhysicsMeshCollisionAPI", "NewtonMeshCollisionAPI"]\n'
        "    )\n"
        "    {\n"
        '        token physics:approximation = "convexHull"\n'
        '        token purpose = "guide"\n'
        "    }\n"
        "}\n",
        encoding="utf-8",
    )
    (root / "payloads/base.usda").write_text(
        'def Cube "box_1" (\n'
        '                prepend apiSchemas = ["PhysicsCollisionAPI"]\n'
        '                displayName = "box"\n'
        "            )\n"
        "            {\n"
        "                float3[] extent = [(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)]\n"
        '                uniform token purpose = "guide"\n'
        "                double size = 1\n"
        "            }\n",
        encoding="utf-8",
    )


def test_enhance_prepared_mycobot_usd_on_fixture(tmp_path: Path) -> None:
    prepared = tmp_path / "mycobot_280_m5"
    _write_minimal_prepared(prepared)
    result = enhance_prepared_mycobot_usd(prepared)
    assert result["physics_self_collision"]
    assert result["physx_self_collision"]
    assert result["instance_contact_reports"]
    assert result["base_contact_report"]
    instances = (prepared / "payloads/instances.usda").read_text(encoding="utf-8")
    assert instances.count("PhysxContactReportAPI") == 7
    assert "physxArticulation:enabledSelfCollisions = 1" in (
        prepared / "payloads/Physics/physx.usda"
    ).read_text(encoding="utf-8")


_PREPARED_PRESENT = (PREPARED / "payloads/instances.usda").is_file()


@pytest.mark.skipif(not _PREPARED_PRESENT, reason="prepared USD absent")
def test_live_prepared_usd_self_collision_authored() -> None:
    enhance_prepared_mycobot_usd(PREPARED)
    physics = (PREPARED / "payloads/Physics/physics.usda").read_text(encoding="utf-8")
    physx = (PREPARED / "payloads/Physics/physx.usda").read_text(encoding="utf-8")
    instances = (PREPARED / "payloads/instances.usda").read_text(encoding="utf-8")
    base = (PREPARED / "payloads/base.usda").read_text(encoding="utf-8")
    assert "newton:selfCollisionEnabled = 1" in physics
    assert "physxArticulation:enabledSelfCollisions = 1" in physx
    assert instances.count("PhysxContactReportAPI") >= 7
    assert "PhysxContactReportAPI" in base
