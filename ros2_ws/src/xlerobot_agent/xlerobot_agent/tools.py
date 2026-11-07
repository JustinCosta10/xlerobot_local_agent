"""The complete allowlist. No motion, shell, arbitrary ROS, or code-execution tools."""

import json
import math

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
    schema("observe_scene", "Wait without commanding motion for freshly observed CONFIRMED tracked objects "
           "in the active categories. Never returns unconfirmed detections. No result is not proof of absence.", {
        "duration_s": {"type": ["number", "null"], "minimum": 0.1, "maximum": 10,
                       "description": "Observation window in seconds; null uses 2 seconds."},
    }),
    schema("query_objects", "Query confirmed tracked objects in the current map session. Stored records "
           "are not necessarily visible now; inspect observation age. Null fields use documented defaults.", {
        "label": {"type": ["string", "null"], "maxLength": 120,
                  "description": "Exact normalized label; null means all labels."},
        "object_id": {"type": ["string", "null"], "maxLength": 128,
                      "description": "Exact UUID returned earlier; null means any object."},
        "include_stale": {"type": ["boolean", "null"], "description": "Default false."},
        "limit": {"type": ["integer", "null"], "minimum": 1, "maximum": 200,
                  "description": "Default 100. At the limit, completeness is unknown."},
    }),
]


class ToolRegistry:
    def __init__(self, robot):
        self.robot = robot
        self.plan = []
        self.validators = {s["name"]: Draft202012Validator(s["parameters"]) for s in SCHEMAS}

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
            if not self.plan:
                raise ToolError("PLAN_REQUIRED", "Call update_plan before perception tools for this instruction.")
            if name == "set_perception_vocabulary":
                classes = validate_classes(args["classes"])
                return self.robot.set_vocabulary(classes)
            if name == "observe_scene":
                duration = 2. if args["duration_s"] is None else args["duration_s"]
                if not math.isfinite(duration):
                    raise ValueError("Observation duration must be finite.")
                return self.robot.observe(duration)
            return self.robot.query(label=args["label"] or "", object_id=args["object_id"] or "",
                                    include_stale=args["include_stale"] or False,
                                    limit=100 if args["limit"] is None else args["limit"])
        except (ValueError, TypeError):
            return {"success": False, "code": "INVALID_ARGUMENT",
                    "message": "Invalid tool arguments; check schema, names, duplicates, and limits."}
        except ToolError as error:
            return {"success": False, "code": error.code, "message": str(error)}
