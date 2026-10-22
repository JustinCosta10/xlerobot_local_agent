# Keyboard control

`teleop_twist_keyboard` is already installed on this computer.

With the robot running, open a **second terminal** and paste:

```bash
cd /home/justin/xlerobot-research/ros2_ws
source scripts/env.sh
# Slow preset: 0.10 m/s forward; turn = pi/6 rad/s = 30 degrees per second.
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -r cmd_vel:=/cmd_vel \
  -p stamped:=false \
  -p speed:=0.10 \
  -p turn:=0.5235987755982988
```

Keep this terminal focused while driving. In Isaac Sim, make sure the simulation
is playing. Wait for `Robot controllers ready` in the launch terminal. If Nav2
has an active goal, click **Cancel** in RViz first. Stop any other manual velocity
publisher before driving.

| Key | Movement |
| --- | --- |
| `i` | Forward |
| `,` | Reverse |
| `j` / `l` | Turn left / right in place |
| `u` / `o` | Forward while turning left / right |
| `m` / `.` | Reverse while turning |
| `k` or Space | Stop |
| `q` / `z` | Increase / decrease both speeds by 10% |
| Ctrl+C | Stop and exit teleop |

Hold a movement key for continuous motion; keyboard repeat sends fresh commands.
The controller brakes after **0.5 seconds without a fresh command**, including
after releasing the key. A long keyboard repeat delay can cause a brief pause
when first holding a key. Use `k` or Space to send an explicit stop command.

Starting speeds use XLeRobot's **slow preset: 0.10 m/s forward and 30 degrees/s
(0.523599 rad/s) turning**. The controller caps match these speeds in both
directions. Increasing teleop speed with `q` cannot exceed those caps.
Restart an existing robot or navigation launch, then restart teleop, to load
the updated settings.

For published XLeRobot presets, calculated turning radii, wheel speeds, and
hardware wheel-limit considerations, see [SPEEDS_AND_TURN_RADIUS.md](SPEEDS_AND_TURN_RADIUS.md).

The command publishes plain `Twist` on `/cmd_vel`. The existing launch adapter
adds the correct timestamp and forwards it to the robot's stamped velocity input.
The same command works with Isaac and mock backends, including a custom
robot namespace; no `use_sim_time` flag is needed for teleop.

If the robot is not running yet, start it in the first terminal:

```bash
cd /home/justin/xlerobot-research/ros2_ws
source scripts/env.sh
ros2 launch xlerobot_bringup robot.launch.py backend:=isaac
```

Use the existing navigation launch instead if you want live mapping; launch only
one robot session at a time.

Upstream reference: [ROS 2 teleop_twist_keyboard](https://github.com/ros2/teleop_twist_keyboard).
