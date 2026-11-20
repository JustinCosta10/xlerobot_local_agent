# Agentic XLeRobot

<table>
  <tr>
    <td width="50%" valign="top">
      <a href="docs/images/simulation-scene.webp"><img src="docs/images/simulation-scene.webp" width="100%" alt="XLeRobot in the simulated room with a table of household objects and blue cubes"></a>
      <br><strong>Isaac Sim scene</strong><br>The robot and tabletop objects before navigation.
    </td>
    <td width="50%" valign="top">
      <a href="docs/images/object-approach.webp"><img src="docs/images/object-approach.webp" width="100%" alt="XLeRobot beside the table after navigating to an approach pose near a detected cracker box"></a>
      <br><strong>Object approach</strong><br>A successful supervised Nav2 approach to the cracker box.
    </td>
  </tr>
  <tr>
    <td width="50%" valign="top">
      <a href="docs/images/rviz-navigation.webp"><img src="docs/images/rviz-navigation.webp" width="100%" alt="RViz showing the colored nvblox reconstruction, navigation costmap, and robot model"></a>
      <br><strong>Navigation in RViz</strong><br>Live nvblox reconstruction, navigation costmap, and robot model.
    </td>
    <td width="50%" valign="top">
      <a href="docs/images/foxglove-cracker-detection.webp"><img src="docs/images/foxglove-cracker-detection.webp" width="100%" alt="Foxglove displaying the robot's live head-camera feed with a YOLOE cracker box detection and confidence label"></a>
      <br><strong>Robot view in Foxglove</strong><br>YOLOE detecting the cracker box through the simulated head camera.
    </td>
  </tr>
</table>

The Foxglove view uses the `cracker box` vocabulary and displays the live
`/xlerobot/yoloe/segmentation/image_raw/compressed` stream.
Click any image for its full-resolution view, or see the
[robot close-up](docs/images/robot-detail.webp) and
[full RViz window](docs/images/rviz-window.webp).

An undergraduate senior thesis exploring **which robot tools and feedback formats
best support a small local language model in carrying out natural-language
mobile-manipulation tasks**. The research platform is a two-wheel XLeRobot in
Isaac Sim, with ROS 2 control, perception, object memory, and navigation.

The intended demonstration is a task such as “find the cube and place it in the
marked area.” The model will choose detection vocabulary and sequence robot
tools. The research compares separate action tools with combined operations,
such as navigation plus grasping, while measuring reliability, latency, and
memory use.

**Status as of 2026-11-19:** the prototype uses a hosted OpenAI model and supports
perception and supervised base navigation. Local inference on Jetson AGX Orin
and tabletop manipulation are planned; the complete pick-and-place demonstration
is not yet implemented.

## Prerequisites

- **Workstation:** Ubuntu 24.04, ROS 2 Jazzy, Python 3.12, and `colcon`. The
  environment script expects ROS at `/opt/ros/jazzy`.
- **Simulation:** Isaac Sim 6.2 and a compatible NVIDIA GPU/driver. Isaac defaults
  to `$HOME/isaacsim`; set `ISAAC_SIM_PATH` for another installation. The mock
  interface and scripted agent demo do not require Isaac or a GPU.
- **ROS and perception dependencies:** follow the
  [workspace dependency setup](ros2_ws/README.md#dependencies). For navigation,
  also install the [Nav2, cuVSLAM, nvblox, YOLOE, and tabletop assets](ros2_ws/README.md#clicked-goal-navigation-with-live-nvblox-mapping)
  before building and launching the full scene.
- **Live language-model calls:** an `OPENAI_API_KEY` in your environment and an
  available model configured in the [agent](ros2_ws/src/xlerobot_agent/README.md#configuration).
  The scripted `--demo` needs no credentials. Jetson hardware is part of the
  planned local-model work, not a prerequisite for the current prototype.

## What is built

| Area | Current implementation |
| --- | --- |
| Robot model and control | Detailed dual-wheel USD and matching URDF; shared `ros2_control` interfaces for the base, two arms, head, and grippers. Isaac and mock backends, wheel odometry, TF, diagnostics, keyboard driving, and command-timeout braking. |
| Simulation scene | Simulated D455 RGB/depth streams, a textured room, obstacles, a table, movable cubes, and textured YCB objects. |
| Localization and navigation | Image-based cuVSLAM localization, live nvblox depth mapping, and Nav2 route planning and obstacle avoidance. RViz clicked goals are available in normal navigation mode. |
| Perception | YOLOE text-prompted detection and segmentation, runtime vocabulary changes through a ROS service, and Foxglove visualization. |
| Object memory | Mask/depth projection into map coordinates, multi-observation confirmation, and SQLite storage of object UUIDs, last-known positions, observation history, and image crops. Records are scoped to map sessions and expose age/staleness. |
| Terminal agent | OpenAI tool calling, explicit plans, vocabulary selection, confirmed-object observation/query, conversation history, and JSONL execution traces. A scripted offline demo runs without API calls or a GPU. |
| Agent navigation | Opt-in tools for absolute map poses, relative destinations, and checked approach positions near confirmed objects. A ROS supervisor and velocity guard enforce readiness, session checks, timeouts, heartbeat expiry, cancellation, and measured-stop confirmation. |

The agent currently exposes `update_plan`, `set_perception_vocabulary`,
`observe_scene`, and `query_objects`. Enabling navigation adds
`get_navigation_state`, `navigate`, `navigate_relative`, `navigate_to_object`,
and `cancel_navigation`. See the [agent guide](ros2_ws/src/xlerobot_agent/README.md)
for arguments, behavior, and limits.

Stored observations can include stale objects and categories outside the active
detector vocabulary. An object query does not establish a fresh sighting, and
arrival near a stored location does not establish grasp readiness. Object
positions describe visible surfaces, not grasp poses. The navigation footprint
excludes articulated arms, and mapping assumes a static scene.

## Planned features and research work

1. **Local-model integration on Jetson AGX Orin.** Add a compact model with
   structured tool use behind the existing provider interface. Isaac Sim will
   run on a separate workstation; perception and motion components may remain
   there too. The current OpenAI adapter can provide a hosted reference for
   selected tasks. A local provider/runtime has not yet been integrated.
2. **Tabletop pick-and-place.** Build grasping, arm/gripper motion, and placement
   tools for a small set of graspable objects, and connect them to base approach
   positioning. Arm and gripper trajectory interfaces already exist; autonomous
   manipulation does not.
3. **Tool and feedback comparisons.** Compare individual action tools with
   combined operations using the same underlying robot capabilities and model
   configuration. Refine descriptions and feedback around object IDs,
   observation freshness, action outcomes, and failure reasons.
4. **Evaluation and thesis results.** Test explicit object names, indirect
   descriptions, multistep instructions, and follow-up requests under comparable
   starting conditions. Measure vocabulary/argument correctness, action order,
   navigation/grasp/placement success, planning latency, total task time,
   model-call count, and peak Jetson memory. Separate model/interface errors from
   perception/motion failures, use fresh observations and simulator recordings
   for scoring, and check a few new instructions or scene variations. Hidden
   scene information will not be supplied to the model.

The work prioritizes a few complete, documented tasks over a broad task catalog.
Physical motor and camera integration remains deferred; no physical robot
backend is included. The mock backend supports interface tests without a GPU
or motors, but does not replace the Isaac perception/navigation scene.

## Getting started

From the repository root, build and start the robot interface:

```bash
cd ros2_ws
./scripts/build.sh
source scripts/env.sh
ros2 launch xlerobot_bringup robot.launch.py backend:=isaac
```

Use `backend:=mock` for interface tests without Isaac. Foxglove Bridge starts
automatically at `ws://localhost:8765` and binds to loopback by default;
use `foxglove:=false` to disable it. See the
[workspace guide](ros2_ws/README.md#build-and-select-a-backend) for remote access.
See [keyboard teleoperation](TELEOP.md) and the
[speed and turning-radius reference](SPEEDS_AND_TURN_RADIUS.md) for manual driving.

The commands below run from `ros2_ws/`. Source `scripts/env.sh` in each terminal,
including after rebuilding. Run only one robot/navigation stack at a time;
each navigation launch already starts the robot and Isaac.

For clicked-goal navigation with mapping, YOLOE, and object memory, stop the
robot-only launch and run:

```bash
ros2 launch xlerobot_navigation sim_navigation.launch.py
```

To try the agent without a running simulator or API credentials:

```bash
ros2 run xlerobot_agent agent --demo --once 'Look for cubes and mugs' \
  --trace log/agent-demo.jsonl
```

This is a fixed script with fake perception, so it demonstrates tool flow rather
than model performance. For live perception, configure `OPENAI_API_KEY` and an
available model as described in the [agent guide](ros2_ws/src/xlerobot_agent/README.md),
keep the normal navigation stack running, and start the agent in another sourced
terminal:

```bash
ros2 run xlerobot_agent agent --trace log/agent-openai-ros.jsonl
```

For agent-controlled motion, replace the normal navigation launch with:

```bash
ros2 launch xlerobot_navigation sim_navigation.launch.py agent_navigation:=true
```

Then start the agent in another sourced terminal with the API key configured:

```bash
ros2 run xlerobot_agent agent --enable-navigation --trace log/agent-navigation.jsonl
```

This mode disables manual velocity inputs and RViz goal control. Use Ctrl+C in
the agent to cancel its current task. See
[supervised simulation navigation](ros2_ws/src/xlerobot_agent/README.md#supervised-simulation-navigation)
for the full launch and cancellation behavior.

## Repository guide

- [ROS 2 workspace](ros2_ws/README.md): packages, dependencies, public interfaces,
  launch configuration, perception/mapping details, and integration tests.
- [Terminal agent](ros2_ws/src/xlerobot_agent/README.md): model configuration,
  tools, conversation behavior, traces, and supervised navigation.
- [Isaac model assets](xlerobot_isaac_sim/README.md): robot USD, source URDF,
  33 meshes, upstream license, and model validation.
