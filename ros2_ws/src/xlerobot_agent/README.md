# Perception terminal agent

An interactive OpenAI agent that makes a short plan, changes YOLOE text prompts,
and observes/queries **confirmed tracked objects only**. It attaches to the
existing robot stack. No navigation or other motion tools are registered.

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
| `observe_scene` | Waits 2 seconds by default (0.1–10 allowed). Returns only confirmed current-session objects observed in that window, in active categories, with no stale records. |
| `query_objects` | Queries confirmed records by label/UUID; current session only, no crops, excludes stale objects by default. |

Object confirmation is provided by the existing mapper, not the LLM. The adapter
uses detector frame timestamps/revisions internally to check activity; it never
sends per-image detections or tentative targets to the model. Fresh detector
activity does not establish valid depth/TF mapping. An old confirmed record is
not a fresh observation just because its snapshot was republished.

Query results cap at 200. Filling the requested limit reports unknown completeness;
there is no pagination or exhaustive search claim. Tracked records lack vocabulary
revision, so observation-window revision is explicitly separate from object
provenance. No newly confirmed objects in a short window is not proof of absence.

There is no persistent task state, navigation, manipulation, automatic recovery,
or concurrent user steering. The agent does not command the robot to stop or hold
still; keep it stationary yourself while testing observation. Use `/new` to bound
conversation growth. JSONL traces contain user text and object locations; keep
them in the ignored `ros2_ws/log/` directory unless intentionally sharing them.

## Verification

```bash
source scripts/env.sh
colcon test --packages-select xlerobot_agent --event-handlers console_direct+
colcon test-result --test-result-base build/xlerobot_agent --verbose
```

Tests use scripted model turns, an injected fake Responses transport, and ROS
service/topic fixtures on domain 87. They never call a hosted model or command
movement. They cover result feedback, continuation/call IDs, invalid arguments,
interruption, freshness, vocabulary changes, and real DDS service bindings.

To establish actual model quality, run the OpenAI tests above yourself: a request,
a paraphrase, and a vocabulary-changing follow-up. Save the trace and verify
that calls happened and answers match results. Offline tests do not establish
frontier-model performance or live detector accuracy.

References: [OpenAI function calling](https://developers.openai.com/api/docs/guides/function-calling),
[subscription API credits](https://developers.openai.com/api/docs/guides/chatgpt-api-credits),
[model documentation](https://developers.openai.com/api/docs/models/gpt-6-astra).
