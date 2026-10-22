# XLeRobot research

ROS 2 control and navigation for the two-wheel XLeRobot in Isaac Sim,
with a mock backend for interface tests.

- [`ros2_ws/`](ros2_ws/README.md): build, launch, controllers, navigation,
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

For keyboard driving with the standard ROS 2 teleop node, see
[`TELEOP.md`](TELEOP.md) for the copy-and-paste command and key bindings.
The [speed and turning-radius reference](SPEEDS_AND_TURN_RADIUS.md) records source
links, calculations, and the selected slow preset.

For clicked-goal navigation with live depth mapping, see the
[Nav2 + nvblox simulation setup](ros2_ws/README.md#clicked-goal-navigation-with-live-nvblox-mapping).
Navigation also starts [YOLOE segmentation for Foxglove](ros2_ws/README.md#yoloe-segmentation-in-foxglove);
use `yoloe:=false` to disable it.

Use `backend:=mock` for testing without a GPU or motors. See the workspace
README for public interfaces and known limitations. Physical motor and camera
drivers are deferred until real-robot integration.
