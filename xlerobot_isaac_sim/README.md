# XLeRobot DD Isaac USD asset

`xlerobot_dd.usd` is the ROS launch default and the selected detailed two-wheel
robot. It contains the 127 mm tires, six-spoke cores, printed hubs, rear-only
casters, saved wheel drive/contact settings, and RealSense mounting setup.

The matching source model is `source/xlerobot_dd.urdf`, with its meshes in
`source/meshes/` and upstream license in `source/LICENSE.XLeRobot`. These were
recovered from the user's saved dual-wheel model (source project revision
`c42d540`); the original files are untouched. The exported asset and URDF are
named `xlerobot_dd`; existing `/xlerobot` prim paths and ROS interfaces are
preserved so references and controllers remain compatible.

The DD asset's `Sensor` variant has `Sensors` (default when opened directly or
through ROS launch) and `None` selections. Set `sensors:=false` during ROS launch
to exclude the external D455 device without changing its saved mounting data.
The runner adds transient ROS control and wheel-contact corrections; see
[`ros2_ws/README.md`](../ros2_ws/README.md) for launch and validation details.
`tests/validate_xlerobot_dd.py` checks the actual launch composition, including
visible wheel meshes, equivalent contact dimensions, and preserved drive tuning.

The DD USD is self-contained apart from the external NVIDIA D455 reference.
The source URDF, all 33 referenced meshes, and their upstream license are kept
here; the ROS stack has no dependency on the MuJoCo browser project.

After editing the source URDF, regenerate the shared ROS description and rebuild:

```bash
cd ../ros2_ws
python3 src/xlerobot_description/scripts/generate_description.py
./scripts/build.sh
```

Keep the USD and URDF kinematics synchronized: launch checks their link transforms
and all 16 joint axes before starting physics. Run `tests/validate_xlerobot_dd.py`
in an Isaac Python environment with the USD bindings and PhysX schema plugins
available to check the composed model without loading the camera or starting a GPU.
