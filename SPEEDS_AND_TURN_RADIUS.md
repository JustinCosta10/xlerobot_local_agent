# XLeRobot speeds and turning radius

Reference date: **2026-10-20**. This covers the two-wheel differential-drive
XLeRobot with servo motors. Keyboard commands are in [TELEOP.md](TELEOP.md).

The numbers below distinguish published software presets, local configuration,
and calculated motion. No physical-robot speed, turning-radius, or stopping-distance
measurements were made for this reference. The web search did not find measured
performance data validating the original project's presets under load.

**Original project's published presets**

“Upstream” means the original Vector-Wangel/XLeRobot project. Its two-wheel driver
starts at the slow preset and expresses turning speed in degrees per second.
ROS `Twist.angular.z` and `teleop_twist_keyboard`'s `turn` parameter use radians
per second. The conversions and radii below are calculated from the published
commands. [Original driver](https://github.com/Vector-Wangel/XLeRobot/blob/main/software/src/robots/xlerobot_2wheels/xlerobot_2wheels.py)

| Preset | Forward speed v (m/s) | Turn speed (degrees/s) | ROS turn speed w (rad/s) | Radius when both commands apply (m) |
| --- | ---: | ---: | ---: | ---: |
| Slow, default | 0.10 | 30 | 0.523599 | 0.190986 |
| Medium | 0.20 | 60 | 1.047198 | 0.190986 |
| Fast | 0.30 | 90 | 1.570796 | 0.190986 |

All three have the same nominal radius because forward and turning speeds scale
together. These are requested velocities; the original driver also scales wheel
commands when they exceed its raw wheel-speed cap. A preset alone does not prove
that the robot achieves that speed. [Wheel-command conversion and limiting](https://github.com/Vector-Wangel/XLeRobot/blob/main/software/src/robots/xlerobot_2wheels/xlerobot_2wheels.py#L348-L408)

**Geometry and limits used for this workspace**

| Quantity | Value | Basis |
| --- | ---: | --- |
| Drive-wheel radius r | 0.0635 m | Local model: 127 mm / 5-inch diameter |
| Wheel spacing L | 0.45 m | Local model's distance between drive wheels |
| Current forward/reverse speed cap | +/-0.10 m/s | Controller setting |
| Current turning-speed cap | +/-0.5235987755982988 rad/s (30 degrees/s) | Controller setting: original slow preset |
| Wheel-speed limit in hardware template | 3.0 rad/s per wheel | Provisional template setting |
| Wheel velocity limit in source URDF | 4.72 rad/s | Model value |

Local sources: [controller configuration](ros2_ws/src/xlerobot_control/config/controllers.yaml),
[hardware template](ros2_ws/src/xlerobot_hardware/config/hardware.example.yaml),
and [source URDF](xlerobot_isaac_sim/source/xlerobot_dd.urdf).
The hardware template is incomplete and requires calibration. Its 3.0 rad/s
setting is not a measured motor maximum or an automatically applied Isaac limit.

The original project's software defaults use **r = 0.05 m and L = 0.25 m**,
which differ from this workspace. Its dual-wheel assembly guide specifies
**5-inch wheels**, consistent with this workspace's radius. Retain the geometry
of the actual model when comparing speed commands.
[Original configuration](https://github.com/Vector-Wangel/XLeRobot/blob/main/software/src/robots/xlerobot_2wheels/config_xlerobot_2wheels.py),
[dual-wheel assembly guide](https://xlerobot.readthedocs.io/en/latest/hardware/getting_started/assemble_2wheel.html)

**How the calculations work**

Assume equal wheel radii, direct drive, a flat surface, no slip, and steady motion:

```text
w [rad/s] = turn speed [degrees/s] * pi / 180
R [m]    = abs(v / w)

left wheel speed  [rad/s] = (v - w * L / 2) / r
right wheel speed [rad/s] = (v + w * L / 2) / r
```

`R` is the radius traced by the midpoint of the drive axle (`base_link`). It is
not the clearance needed by the chassis, arms, or rear casters. The equations
use physical left/right wheel directions; this model's joint names and motor
signs require the mappings already in the controller and hardware configuration.
[Original differential-drive equations](https://github.com/Vector-Wangel/XLeRobot/blob/main/software/src/robots/xlerobot_2wheels/README.md)

- Straight driving: `w = 0`, so the radius is infinite.
- Turning in place: `v = 0`, `w != 0`, so the axle midpoint's radius is zero.
- Stopped: `v = w = 0`; no turning radius is defined.
- For positive v and w, a radius smaller than `L / 2 = 0.225 m` requires the
  inside wheel to reverse.

In ROS keyboard teleop, `i` requests straight motion, `j`/`l` request an in-place
turn, and `u`/`o` combine forward and turning commands. The combined-command
radii below apply to the latter. Acceleration limits, saturation, and slip can
change the actual path.
[ROS keyboard controls](https://github.com/ros2/teleop_twist_keyboard)

**Calculated motion with this workspace's geometry**

The table assumes forward motion and a left turn. Wheel speeds are the required
values before limiting. Negative left-wheel speed means that wheel reverses.

| Setting | v (m/s) | w (rad/s) | Axle-center radius (m) | Left wheel (rad/s) | Right wheel (rad/s) |
| --- | ---: | ---: | ---: | ---: | ---: |
| Lower-speed example | 0.05 | 0.15 | 0.3333 | 0.2559 | 1.3189 |
| Lower-turn example | 0.10 | 0.20 | 0.5000 | 0.8661 | 2.2835 |
| Alternative fitting the 3.0 rad/s template wheel limit | 0.10 | 0.40 | 0.2500 | 0.1575 | 2.9921 |
| Current TELEOP.md command and controller caps: original slow preset | 0.10 | 0.523599 | 0.1910 | -0.2805 | 3.4301 |
| Original project's medium preset | 0.20 | 1.047198 | 0.1910 | -0.5609 | 6.8601 |
| Original project's fast preset | 0.30 | 1.570796 | 0.1910 | -0.8414 | 10.2902 |

The slow preset is selected in TELEOP.md and the controller configuration.
The medium and fast rows are hypothetical requests on our geometry; the current
controller caps prevent those combinations from being reached.

For a symmetric wheel limit `q_max`, both wheel commands fit when:

```text
abs(v) + abs(w) * L / 2 <= r * q_max
```

Using the hardware template's `q_max = 3.0 rad/s` gives these calculated bounds:

| Condition | Calculated bound |
| --- | ---: |
| Straight motion, w = 0 | abs(v) <= 0.1905 m/s |
| Turn in place, v = 0 | abs(w) <= 0.8467 rad/s |
| Turning while v = 0.10 m/s | abs(w) <= 0.4022 rad/s |

These are kinematic bounds from a provisional software limit. The body velocity
caps constrain straight motion and in-place turns more tightly, but their
combined maximum exceeds the template's wheel limit. Independently clipping
wheel speeds can change the requested turning radius.

**Selected slow preset**

TELEOP.md now requests **0.10 m/s forward and pi/6 rad/s (30 degrees/s) turning**.
The controller's positive and negative velocity caps match this preset. At steady
state, combined forward and turning commands give a calculated **0.1910 m
axle-center radius** and require up to **3.4301 rad/s** at the faster wheel.
Acceleration limits still apply. Restart the robot or navigation launch and
teleop to load these settings. Autonomous navigation retains its separate
0.10 m/s and 0.20 rad/s command limits.

The hardware template still limits each wheel to **3.0 rad/s**. On physical
hardware using that limit, the faster wheel would be clipped during a combined
slow-preset turn, changing the actual speed and radius. Hardware limits must be
chosen from the calibrated robot; this teleop change does not raise them.

As a lower-turn alternative, **0.10 m/s and 0.40 rad/s** give a **0.25 m radius**
and require at most **2.9921 rad/s** per wheel. This fits the template numerically
with almost no margin for geometry uncertainty. Both combinations are calculated
settings that still require physical validation.

**Motor specification cross-check**

Waveshare lists **45 RPM without load at 12 V** for its ST3215 servo. The following
estimate applies only to a matching motor variant and supply voltage with a
directly driven 5-inch wheel. [Vendor specification](https://docs.waveshare.com/ST3215_Servo)

```text
wheel speed = 45 * 2*pi / 60 = 4.7124 rad/s
linear speed = wheel speed * 0.0635 = 0.2992 m/s
```

This is an unloaded theoretical estimate. Actual payload, supply voltage,
traction, and caster resistance affect achievable speed. It does not establish
0.30 m/s as an appropriate operating speed for the assembled robot.

The current Isaac model approximates rear-caster support with low-friction
spheres attached to the chassis; it has no caster roll or swivel joints. That
limits how closely simulated motion can reproduce real caster resistance.
See [the robot asset](xlerobot_isaac_sim/xlerobot_dd.usd) and
[contact configuration](ros2_ws/src/xlerobot_isaac/scripts/run_isaac.py).
