"""Perception allowlist and optional supervised simulation navigation."""

import json

from jsonschema import Draft202012Validator
from xlerobot_perception.vocabulary import validate_classes


class ToolError(RuntimeError):
    def __init__(self, code, message):
        self.code = code
        super().__init__(message)


def schema(name, description, properties):
    return dict(name=name, description=description, parameters=dict(
        type="object", properties=properties, required=list(properties), additionalProperties=False))


SCHEMAS = [
    schema("update_plan", "Record a short visible plan before robot tools, or revise it using results.", {
        "steps": {"type": "array", "minItems": 1, "maxItems": 8,
                  "items": {"type": "string", "minLength": 1, "maxLength": 300}},
    }),
    schema("set_perception_vocabulary", "Replace YOLOE text prompts. Acceptance is not an object observation. "
           "Valid new categories are allowed; existing object history remains. Last vocabulary stays active.", {
        "classes": {"type": "array", "minItems": 1, "maxItems": 128,
                    "items": {"type": "string"}},
    }),
    schema("observe_scene", "Read confirmed stored objects and their last known locations from the current "
           "map session immediately, including stale records and categories outside the active vocabulary. "
           "No observation window or age cutoff. Ages describe the evidence; they do not hide objects.", {}),
    schema("query_objects", "Query confirmed tracked objects in the current map session. Stored records "
           "are not necessarily visible now; inspect observation age. Null fields use documented defaults.", {
        "label": {"type": ["string", "null"], "maxLength": 120,
                  "description": "Exact normalized label; null means all labels."},
        "object_id": {"type": ["string", "null"], "maxLength": 128,
                      "description": "Exact UUID returned earlier; null means any object."},
        "include_stale": {"type": ["boolean", "null"],
                          "description": "Default true: retain stored objects regardless of age. "
                                         "False explicitly restricts results to non-stale records."},
        "limit": {"type": ["integer", "null"], "minimum": 1, "maximum": 200,
                  "description": "Default 100. At the limit, completeness is unknown."},
    }),
]

NAVIGATION_SCHEMAS = [
    schema("get_navigation_state", "Read current base pose in map, session_id, readiness, and last motion result. "
           "Read this before choosing a navigation goal. Coordinates are meters and yaw is radians.", {}),
    schema("navigate", "Navigate the BASE to an explicit grounded pose in the current map session. "
           "Waits for completion and confirmed stopping; Ctrl+C cancels. Use navigate_to_object for "
           "objects: their surface positions cannot be used directly as base destinations. "
           "Do not blindly retry failed or uncertain moves. Simulation only.", {
        "x": {"type": "number", "exclusiveMinimum": -8, "exclusiveMaximum": 8},
        "y": {"type": "number", "exclusiveMinimum": -8, "exclusiveMaximum": 8},
        "yaw": {"type": "number", "minimum": -3.141592653589793, "maximum": 3.141592653589793},
        "session_id": {"type": "string", "minLength": 1, "maxLength": 128,
                       "description": "Exact current session_id from get_navigation_state."},
        "timeout_s": {"type": ["number", "null"], "minimum": 1, "maximum": 300,
                      "description": "Wall-clock motion deadline; null uses 120 seconds."},
    }),
    schema("navigate_relative", "Navigate to an offset resolved ONCE from the measured starting base pose. "
           "Positive forward_m is forward; positive left_m is left; positive yaw_delta_rad turns counterclockwise. "
           "Nav2 may turn and take a route to the destination; this is not a straight-line or sideways drive command. "
           "Returns starting and resolved poses. Nav2 goal tolerances apply; inspect the measured final pose. "
           "Never blindly repeat a relative move after failure.", {
        "forward_m": {"type": "number", "minimum": -3, "maximum": 3},
        "left_m": {"type": "number", "minimum": -3, "maximum": 3},
        "yaw_delta_rad": {"type": ["number", "null"], "minimum": -3.141592653589793,
                          "maximum": 3.141592653589793, "description": "Null keeps the starting heading."},
        "session_id": {"type": "string", "minLength": 1, "maxLength": 128},
        "timeout_s": {"type": ["number", "null"], "minimum": 1, "maximum": 300,
                      "description": "Wall-clock deadline; null uses 120 seconds."},
    }),
    schema("navigate_to_object", "Move to a reachable, clear base pose NEAR a confirmed stored object UUID, "
           "facing its last-known location. The supervisor selects the pose using the costmap, robot footprint, "
           "and Nav2 route planning. Stale objects are allowed in the current map session. Arrival does not "
           "prove the object is still present or visible. Returns the chosen pose and actual distance.", {
        "object_id": {"type": "string", "minLength": 1, "maxLength": 128,
                      "description": "Exact UUID from observe_scene or query_objects; never invent it."},
        "standoff_m": {"type": ["number", "null"], "minimum": .5, "maximum": 2.,
                       "description": "Preferred horizontal base-origin distance from stored object point; "
                                      "null uses 0.8 m. May select up to 0.8 m farther, capped at 2 m, for clearance."},
        "session_id": {"type": "string", "minLength": 1, "maxLength": 128},
        "timeout_s": {"type": ["number", "null"], "minimum": 1, "maximum": 300,
                      "description": "Wall-clock deadline including approach search; null uses 120 seconds."},
    }),
    schema("cancel_navigation", "Cancel this agent's navigation goal. Does not require a plan. "
           "Inspect stop_confirmed; sending cancellation alone does not establish a stop.", {}),
]


class ToolRegistry:
    def __init__(self, robot):
        self.robot = robot
        self.plan = []
        self.schemas = SCHEMAS + (NAVIGATION_SCHEMAS if getattr(robot, "navigation_enabled", False) else [])
        self.validators = {s["name"]: Draft202012Validator(s["parameters"]) for s in self.schemas}

    def dispatch(self, name, arguments):
        if name not in self.validators:
            return {"success": False, "code": "UNKNOWN_TOOL", "message": "Tool is not available."}
        try:
            args = json.loads(arguments, parse_constant=lambda _: (_ for _ in ()).throw(ValueError()))
            errors = list(self.validators[name].iter_errors(args))
            if errors:
                raise ValueError("Arguments must match the tool schema, including required nullable fields.")
            if name == "update_plan":
                if not all(step.strip() for step in args["steps"]):
                    raise ValueError("Plan steps cannot be blank.")
                self.plan = args["steps"]
                return {"success": True, "steps": self.plan}
            if name == "get_navigation_state":
                return self.robot.get_navigation_state()
            if name == "cancel_navigation":
                return self.robot.cancel_navigation()
            if not self.plan:
                raise ToolError("PLAN_REQUIRED", "Call update_plan before robot tools for this instruction.")
            if name in ("navigate", "navigate_relative", "navigate_to_object"):
                args["timeout_s"] = 120. if args["timeout_s"] is None else args["timeout_s"]
                if name == "navigate_relative" and args["yaw_delta_rad"] is None:
                    args["yaw_delta_rad"] = 0.
                if name == "navigate_to_object" and args["standoff_m"] is None:
                    args["standoff_m"] = .8
                return getattr(self.robot, name)(**args)
            if name == "set_perception_vocabulary":
                classes = validate_classes(args["classes"])
                return self.robot.set_vocabulary(classes)
            if name == "observe_scene":
                return self.robot.observe()
            return self.robot.query(label=args["label"] or "", object_id=args["object_id"] or "",
                                    include_stale=True if args["include_stale"] is None else args["include_stale"],
                                    limit=100 if args["limit"] is None else args["limit"])
        except (ValueError, TypeError):
            return {"success": False, "code": "INVALID_ARGUMENT",
                    "message": "Invalid tool arguments; check schema, names, duplicates, and limits."}
        except ToolError as error:
            return {"success": False, "code": error.code, "message": str(error)}
