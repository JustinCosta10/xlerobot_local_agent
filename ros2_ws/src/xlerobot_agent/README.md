# Robot terminal agent

An interactive OpenAI agent that makes a short plan, changes YOLOE text prompts,
and reads **confirmed stored objects and their last known locations**. It attaches to the
existing robot stack. Perception is enabled by default; supervised pose navigation
can be enabled explicitly for simulation.

The implementation follows the repository's
[agentic perception plan](../../../AGENTIC_PERCEPTION_PLAN.md).

## Build and try the offline demo

From `ros2_ws/`:

```bash
./scripts/build.sh --packages-select xlerobot_agent
source scripts/env.sh
ros2 run xlerobot_agent agent --demo --once 'Look for cubes and mugs' \
  --trace log/agent-demo.jsonl
```

This uses a **scripted model and fake perception**. It makes no API calls, starts
no ROS node, and requires no credentials or GPU. The script always demonstrates
cubes/mugs; it does not interpret arbitrary instructions. Omit `--once` to try
the interactive terminal. The checked-in [example trace](examples/offline-demo.jsonl)
is explicitly an offline demonstration, not a successful frontier-model test.

The package uses the standard-library HTTPS client for Responses; no OpenAI SDK
or changes to YOLOE's isolated Python environment are needed. ROS dependencies
are declared in `package.xml`; Python runtime dependencies are PyYAML and
jsonschema. Build the existing perception packages first if they are not already
in the workspace overlay.

## Test OpenAI when ready

No API credentials are configured by this implementation. Activate/link your
subscription API-credit benefit in your OpenAI account if needed, then export
an API key locally. Do not put it in YAML, source control, or chat. In Bash:

```bash
read -rsp 'OpenAI API key: ' OPENAI_API_KEY
export OPENAI_API_KEY
printf '\n'
```

First test model-selected functions with fake objects (this **does call OpenAI**):

```bash
ros2 run xlerobot_agent agent --fake-perception --trace log/agent-openai-fake.jsonl
```

For real perception, keep your existing simulation/perception session running,
then in another terminal:

```bash
cd /home/justin/xlerobot-research/ros2_ws
source scripts/env.sh
# Export OPENAI_API_KEY in this terminal too, if needed.
ros2 run xlerobot_agent agent --trace log/agent-openai-ros.jsonl
```

This connects to the default `xlerobot` namespace with simulation time enabled.
The usual `sim_navigation.launch.py` already starts YOLOE and object mapping.
Do not launch another copy if it is running. A standalone perception launch
requires `object_map:=true`, aligned depth, calibration, and map TF.

Example conversation:

```text
Look for cubes and mugs and tell me what you find.
Tell me the IDs and positions of the confirmed cubes.
Now look for bowls instead.
/new
/quit
```

The real LLM chooses tools and arguments; Python does not match demo sentences.
Plan updates, function arguments, returned results, and answers are printed.
Follow-up turns retain conversation memory. `/new` clears it without resetting
YOLOE or shared objects. Ctrl+C interrupts the current task without dispatching
later calls. A vocabulary request already sent to ROS may still finish; it is
not rolled back. The last accepted vocabulary stays active after exit.

## Supervised simulation navigation

From `ros2_ws/`, build the interfaces, supervisor, agent, and updated launch files:

```bash
./scripts/build.sh --packages-select xlerobot_navigation_interfaces xlerobot_navigation xlerobot_agent xlerobot_bringup
source scripts/env.sh
ros2 launch xlerobot_navigation sim_navigation.launch.py agent_navigation:=true
```

Re-source `scripts/env.sh` in **every existing terminal after this build**. The new
`xlerobot_navigation_interfaces` package will not be visible in shells sourced
before it was built. Agent launch checks this before starting Isaac.

This is an **exclusive agent mode**: the controller takes velocity only from the
guard's `cmd_vel_guarded` topic, the manual velocity adapter is absent, and Nav2's
action is remapped to `navigate_to_pose_internal`. RViz remains useful for viewing
the scene, but its goal/cancel buttons do not control agent goals. Use Ctrl+C in
the agent to cancel. Restart the normal launch without `agent_navigation:=true`
to return to manual/RViz operation. Do not run two stacks in the same namespace.
These are ROS routing rules, not an access-control boundary against arbitrary ROS
publishers. Both modes currently use Isaac; physical transport is not implemented.

In another sourced terminal, with the existing API key configured:

```bash
ros2 run xlerobot_agent agent --enable-navigation --trace log/agent-navigation.jsonl
```

Example: `Read the robot pose and navigation readiness. If ready, navigate the base
to map x=0.8 m, y=0 m, yaw=0 radians with a 60-second timeout.` The destination
must have a mapped, reachable route; this example is not a guarantee of clearance.
Navigation cannot be combined with `--demo`, `--fake-perception`, or `--wall-time`.

| Additional tool | Behavior |
| --- | --- |
| `get_navigation_state` | Current map pose, session ID, readiness/reason, active operation, last result, speed and duration limits. Available without a plan. |
| `navigate` | Navigate to a point/base pose: `x`, `y`, `yaw` in `map`, exact current `session_id`, and nullable `timeout_s` (default 120; range 1–300 wall seconds). |
| `navigate_relative` | Resolve `forward_m`, `left_m`, and nullable `yaw_delta_rad` once from the measured starting pose. Positive means forward, left, and counterclockwise. Null yaw change means zero. Translation components are bounded to ±3 m and yaw change to ±π. |
| `navigate_to_object` | Resolve an exact confirmed `object_id` in the current `session_id` to a checked, reachable base pose near the stored location, facing it. Nullable `standoff_m` defaults to 0.8 m; allowed range 0.5–2 m. |
| `cancel_navigation` | Cancels only this client's goal. Available without a plan; inspect `stop_confirmed`. During the serial tool wait use Ctrl+C. |

All three movement tools require a plan, wait for completion, report progress,
and return the resolved destination, final measured pose, and stop confirmation.
Relative and object tools also require `session_id` and nullable `timeout_s` with
the same defaults/limits. Relative movement specifies a destination, so the
differential-drive robot may turn or follow a curved route; it cannot strafe.
Tool results include the starting pose used for a relative move. Never blindly
repeat a relative call after a failure because it would define another offset.
Nav2's 0.12 m XY and 0.15 rad yaw tolerances still apply. Small offsets can already
be satisfied without movement; use the measured final pose when reporting travel.

Examples: `Move forward 0.3 meters, keeping your heading.` / `Move to a safe point
near the nearest known cube and face it.` The model queries stored objects and
uses their actual UUIDs; ambiguous targets need disambiguation.

Object approach selection runs in the supervisor while velocity permission is off.
It reads the confirmed object record (including stale records), the live global
costmap, the configured footprint/padding, and the controller's XY goal tolerance.
A circle enclosing the padded footprint at every heading, plus goal tolerance
and 5 cm of clearance, must fit entirely within known, unoccupied map cells.
The check covers cell interiors and edges, including the cart's rear overhang.
This conservative envelope also leaves room for the final turn toward the object.
The footprint still excludes articulated arms.

The preferred standoff is the horizontal distance from `base_link` to the stored
object surface point, not a distance from the object's full physical boundary.
The search samples 24 headings at the preferred radius and up to four farther
radii in 0.2 m increments, capped at 2 m. It checks at most 24 clear candidates
with Nav2's [ComputePathToPose action](https://api.nav2.org/actions/jazzy/computepathtopose.html),
within a 15-second planning budget and the overall
tool deadline. It re-reads the object and costmap and rechecks clearance before
dispatch. No valid sampled approach produces `NO_SAFE_APPROACH`; a partial search
that runs out of time produces `APPROACH_TIMEOUT`. Neither sends a motion goal.
Map changes, significant target movement during planning, stale costmaps, and
cancellation also stop resolution. Safety remains relative to the configured
footprint and available map; Nav2 handles obstacle avoidance during execution.

Results expose the selected standoff and measured final distance from the object's
last-known location. Reaching that pose does not prove the object remains there or
is visible. Small tracking updates are tolerated; this is not moving-object pursuit.

The independent ROS supervisor allows one goal, requires fresh mapping, visual
tracking, localization, odometry, an active Nav2 navigator, and the current object-map session, and rejects
invalid or old-session requests. Goals use the existing Nav2 behavior tree and
controller limits. Map-session changes and backward clock jumps invalidate motion.
The agent renews a goal-specific heartbeat during the tool wait; loss of that
heartbeat for one second revokes motion. The guard also expires supervisor
permission after 0.3 seconds. Neither watchdog depends on model calls or on the
simulation clock advancing. Normal exit/SIGINT requests cancellation; SIGKILL is
covered by heartbeat expiry. The controller retains its own command timeout.

The supervisor revokes permission and cancels on completion, cancellation, timeout,
lost contact, or stale inputs. It checks measured odometry for a sustained stop
and waits for Nav2 termination. A stop that cannot be verified within five seconds
returns `STOP_UNCONFIRMED` and blocks further navigation until the stack is
restarted. A paused simulator can produce this outcome because fresh odometry is
unavailable. A delayed Nav2 goal acceptance is canceled even after the client has
stopped waiting. Results distinguish rejection, failure, cancellation, timeout,
map change, lost contact, and success; no motion is automatically retried.

The footprint still excludes arms and the mapper assumes a static scene. Unknown
space remains impassable. Arrival establishes only a base pose within Nav2's
configured tolerance, not object presence, visibility, or manipulation readiness.
The supervisor bounds map coordinates to the configured 16 m map and caps duration;
the model cannot supply velocity commands, alternate frames, or behavior-tree paths.

Point, relative, and object destinations share the same supervised executor.
Named waypoints remain deferred. Object surface coordinates must not be used
directly as base goals; perception age does not by itself invalidate a last-known destination.

## Configuration

Defaults are in [config/agent.yaml](config/agent.yaml). Copy that file and use
`--config PATH` for local changes. CLI overrides:

```bash
ros2 run xlerobot_agent agent --model YOUR_AVAILABLE_MODEL_ID --namespace xlerobot
ros2 run xlerobot_agent agent --wall-time   # for a wall-time perception system
```

The default model is `gpt-6-astra`, with low reasoning effort. Set `model` to an
ID available to your API organization; use a compatible `reasoning_effort` or
`null` to omit that option. Account access and credits have not been verified.
The endpoint is a full HTTPS Responses URL. Requests use `store: false`, carry
required opaque continuation items in memory, and do not log private reasoning.

There is one provider interface in `providers/base.py`, one production adapter
in `providers/openai.py`, and a shared serial tool loop. Ollama is deferred; adding
its adapter will not require changing the ROS tools. Unsupported provider names
fail explicitly. There is no automatic fallback or retry.

## Tool behavior and limits

| Tool | Behavior |
| --- | --- |
| `update_plan` | Records/displays up to eight short steps; required before perception calls for each instruction. |
| `set_perception_vocabulary` | Calls the existing service; returns actual categories, IDs, revision, and success. Valid new text categories are allowed. |
| `observe_scene` | Takes no arguments. Immediately reads confirmed stored objects and locations in the current map session, including stale records and categories outside the active vocabulary. No observation window or age cutoff. |
| `query_objects` | Queries confirmed records by label/UUID; current session only, no crops, includes stale objects by default. Set `include_stale: false` to explicitly restrict results to non-stale records. |

Object confirmation is provided by the existing mapper, not the LLM. Once an
object is stored, the LLM can use its label, UUID, and last known map location
without another sighting. Observation ages and stale flags describe that evidence;
they do not hide records or make them unusable. Changing the detector vocabulary
does not hide previously stored categories. Reading the map works without new
detector frames or detector metadata; the object-map query service must be running.
An empty map result does not prove absence, and stored coordinates do not guarantee
current visibility. Only confirmed objects are exposed, not per-image detections.

Query results cap at 200. Filling the requested limit reports unknown completeness;
there is no pagination or exhaustive search claim. Tracked records lack vocabulary
revision, so current vocabulary metadata is not attached to stored observations.
Records from previous map sessions are excluded because their coordinates belong
to a different map origin. Service-call timeouts only bound unavailable or
unresponsive dependencies; they do not expire stored objects.

There is no persistent task state, manipulation, agent-directed automatic recovery,
or concurrent user steering. Use `/new` to bound
conversation growth. JSONL traces contain user text and object locations; keep
them in the ignored `ros2_ws/log/` directory unless intentionally sharing them.

## Verification

```bash
source scripts/env.sh
colcon test --packages-select xlerobot_agent --event-handlers console_direct+
colcon test-result --test-result-base build/xlerobot_agent --verbose
```

Tests use scripted model turns, an injected fake Responses transport, and ROS
service/topic fixtures on domain 87. Navigation fixtures on domain 88 run the real
supervisor, guard, and action client with synthetic sensors and velocity feedback;
they never command a real or Isaac robot or call a hosted model. They cover result
feedback, continuation/call IDs, invalid arguments,
interruption, stored locations regardless of age or active vocabulary, session
isolation, and real DDS service bindings.

To establish actual model quality, run the OpenAI tests above yourself: a request,
a paraphrase, and a vocabulary-changing follow-up. Save the trace and verify
that calls happened and answers match results. Offline tests do not establish
frontier-model performance or live detector accuracy.

Navigation tests also exercise successful/blocked goals, measured stopping, feedback,
Ctrl+C, an agent process killed during motion, lost tracking, session changes,
paused clocks, wall deadlines, concurrent goals, and delayed acceptance. They
establish ROS integration behavior, not live Isaac driving performance.

The separate Isaac acceptance test runs without a model or API key, starts an
isolated stack on domain 89, and writes `log/agent-navigation-test.log`:

```bash
source scripts/env.sh
python3 scripts/test_agent_navigation.py
```

This test now also checks relative navigation and an approach to an actual
confirmed detector record in the simulated scene. Rebuild the navigation action
interface and restart both the navigation stack and agent after updating; an
already-running process still has the previous action definition loaded.

Validated on 2026-11-06: the extended synthetic ROS, geometry, launch, and agent
suites passed 91 tests. Isaac drove 0.407 m toward a 0.5 m map goal, rejected a
destination inside the table, confirmed cancellation, and reached a relative
destination 0.3 m ahead of its measured starting pose. It then selected a clear
approach 0.8 m from a confirmed cube record and finished 0.943 m from that stored
position, facing it. This does not establish model-selected navigation quality;
that requires a separate live-model trial.

References: [OpenAI function calling](https://developers.openai.com/api/docs/guides/function-calling),
[subscription API credits](https://developers.openai.com/api/docs/guides/chatgpt-api-credits),
[model documentation](https://developers.openai.com/api/docs/models/gpt-6-astra).
