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

Foxglove Bridge starts automatically; connect to `ws://localhost:8765` (or the
robot computer's IP). Add `foxglove:=false` to launch without it.

Manual `cmd_vel` driving now moves the robot in Isaac Sim, confirmed by the user.
See [validation status](ros2_ws/README.md#validation) for automated test coverage.

For clicked-goal navigation with live depth mapping, see the
[Nav2 + nvblox simulation setup](ros2_ws/README.md#clicked-goal-navigation-with-live-nvblox-mapping).

Use `backend:=mock` for testing without a GPU or motors. See the workspace
README for hardware calibration, public interfaces, and known limitations.
