# XLeRobot research

Shared ROS 2 control for the two-wheel XLeRobot, with selectable Isaac Sim,
physical STS3215, and mock backends.

- [`ros2_ws/`](ros2_ws/README.md): build, launch, controllers, hardware transport,
  diagnostics, and integration tests.
- [`xlerobot_isaac_sim/`](xlerobot_isaac_sim/README.md): the DD robot USD, matching
  source URDF, 33 meshes, upstream license, and model validation.

From `ros2_ws/`:

```bash
./scripts/build.sh
source scripts/env.sh
ros2 launch xlerobot_bringup robot.launch.py backend:=isaac
```

Use `backend:=mock` for testing without a GPU or motors. See the workspace
README for hardware calibration, public interfaces, and known limitations.
