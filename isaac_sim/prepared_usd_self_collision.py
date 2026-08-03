"""Author PhysX arm–arm self-collision support into prepared MyCobot USD.

The Isaac URDF importer already emits convexHull collision meshes as
instanceable ``*_1`` prototypes, but historically left articulation
self-collision **off** and omitted ``PhysxContactReportAPI``. Prepared assets
under ``assets/mycobot_280_m5/prepared/`` are gitignored, so this enhancer
must run after every ``convert_urdf_to_usd`` (and may be re-run on an existing
tree).
"""

from __future__ import annotations

from pathlib import Path

_COLLISION_API_SCHEMAS = (
    'apiSchemas = ["PhysicsCollisionAPI", "NewtonCollisionAPI", '
    '"PhysicsMeshCollisionAPI", "NewtonMeshCollisionAPI"]'
)
_COLLISION_API_SCHEMAS_WITH_REPORT = (
    'apiSchemas = ["PhysicsCollisionAPI", "NewtonCollisionAPI", '
    '"PhysicsMeshCollisionAPI", "NewtonMeshCollisionAPI", "PhysxContactReportAPI"]'
)


def enhance_prepared_mycobot_usd(prepared_root: Path) -> dict[str, bool]:
    """Patch prepared USD payloads for arm–arm PhysX contact reporting.

    ``prepared_root`` is the directory containing ``mycobot_280_m5.usda`` and
    ``payloads/`` (nested layout) or the parent of that directory.
    """

    root = _resolve_prepared_root(prepared_root)
    results = {
        "physics_self_collision": _patch_physics_self_collision(
            root / "payloads/Physics/physics.usda"
        ),
        "physx_self_collision": _patch_physx_self_collision(root / "payloads/Physics/physx.usda"),
        "instance_contact_reports": _patch_instance_contact_reports(
            root / "payloads/instances.usda"
        ),
        "base_contact_report": _patch_base_contact_report(root / "payloads/base.usda"),
    }
    if not any(results.values()) and not all(
        (root / rel).is_file()
        for rel in (
            "payloads/Physics/physics.usda",
            "payloads/Physics/physx.usda",
            "payloads/instances.usda",
            "payloads/base.usda",
        )
    ):
        raise FileNotFoundError(f"prepared MyCobot USD payloads missing under {root}")
    return results


def _resolve_prepared_root(prepared_root: Path) -> Path:
    path = prepared_root.resolve()
    if (path / "payloads/instances.usda").is_file():
        return path
    nested = path / "mycobot_280_m5"
    if (nested / "payloads/instances.usda").is_file():
        return nested
    raise FileNotFoundError(f"cannot locate prepared payloads under {prepared_root}")


def _patch_physics_self_collision(path: Path) -> bool:
    if not path.is_file():
        return False
    text = path.read_text(encoding="utf-8")
    updated = text.replace(
        "bool newton:selfCollisionEnabled = 0",
        "bool newton:selfCollisionEnabled = 1",
    )
    if updated == text and "newton:selfCollisionEnabled = 1" not in text:
        return False
    if updated != text:
        path.write_text(updated, encoding="utf-8")
    return "newton:selfCollisionEnabled = 1" in path.read_text(encoding="utf-8")


def _patch_physx_self_collision(path: Path) -> bool:
    if not path.is_file():
        return False
    text = path.read_text(encoding="utf-8")
    if "physxArticulation:enabledSelfCollisions = 1" in text:
        return True
    needle = 'over "firefighter"\n{\n    over "Physics"'
    insert = (
        'over "firefighter"\n'
        "{\n"
        '    over "Geometry" (\n'
        '        prepend apiSchemas = ["PhysxArticulationAPI"]\n'
        "    )\n"
        "    {\n"
        "        bool physxArticulation:enabledSelfCollisions = 1\n"
        "    }\n"
        "\n"
        '    over "Physics"'
    )
    if needle not in text:
        return False
    path.write_text(text.replace(needle, insert, 1), encoding="utf-8")
    return True


def _patch_instance_contact_reports(path: Path) -> bool:
    if not path.is_file():
        return False
    text = path.read_text(encoding="utf-8")
    if _COLLISION_API_SCHEMAS not in text and "PhysxContactReportAPI" in text:
        return text.count("PhysxContactReportAPI") >= 7
    updated = text.replace(_COLLISION_API_SCHEMAS, _COLLISION_API_SCHEMAS_WITH_REPORT)
    # Add threshold attr inside each collision mesh over block when missing.
    if "physxContactReport:threshold = 0" not in updated:
        updated = updated.replace(
            'token physics:approximation = "convexHull"\n            token purpose = "guide"',
            'token physics:approximation = "convexHull"\n'
            '            token purpose = "guide"\n'
            "            float physxContactReport:threshold = 0",
        )
    if updated != text:
        path.write_text(updated, encoding="utf-8")
    return path.read_text(encoding="utf-8").count("PhysxContactReportAPI") >= 7


def _patch_base_contact_report(path: Path) -> bool:
    if not path.is_file():
        return False
    text = path.read_text(encoding="utf-8")
    if "PhysxContactReportAPI" in text and 'def Cube "box_1"' in text:
        return True
    old = (
        'def Cube "box_1" (\n'
        '                prepend apiSchemas = ["PhysicsCollisionAPI"]\n'
        '                displayName = "box"\n'
        "            )\n"
        "            {\n"
        "                float3[] extent = [(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)]\n"
        '                uniform token purpose = "guide"'
    )
    new = (
        'def Cube "box_1" (\n'
        '                prepend apiSchemas = ["PhysicsCollisionAPI", '
        '"PhysxContactReportAPI"]\n'
        '                displayName = "box"\n'
        "            )\n"
        "            {\n"
        "                float3[] extent = [(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)]\n"
        '                uniform token purpose = "guide"\n'
        "                float physxContactReport:threshold = 0"
    )
    if old not in text:
        return False
    path.write_text(text.replace(old, new, 1), encoding="utf-8")
    return True
