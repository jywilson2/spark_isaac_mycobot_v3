# Meshes

Vendor COLLADA/PNG meshes are **not** committed here. Obtain them with:

```bash
./scripts/download_mycobot_ros2.sh
```

Rendered Isaac Sim import expects meshes next to the vendor URDF under
`third_party/mycobot_ros2/...`. Isaac 6.x import workarounds (package:// rewrite,
GUID material fix, G_base box swap) live in `isaac_sim/urdf_utils.py`.
