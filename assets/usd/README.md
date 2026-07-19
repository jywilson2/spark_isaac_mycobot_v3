# USD outputs

Generated USD/USDA/USDC files are gitignored. Convert on the Isaac Sim **host**:

```bash
./scripts/download_mycobot_ros2.sh
./scripts/convert_urdf_to_usd.sh
```

From the Isaac ROS container, prefer:

```bash
./scripts/host/spark_host_exec.sh ./scripts/convert_urdf_to_usd.sh
```
