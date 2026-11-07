"""Bounded object-approach resolution while the supervisor holds motion idle."""

import math
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import GetCostmap
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from xlerobot_perception_interfaces.srv import QueryObjects

from .destinations import ApproachGrid, DestinationError, approach_radius
from .guard import stamp_seconds


def pose_message(node, pose):
    result = PoseStamped()
    result.header.frame_id = "map"
    result.header.stamp = node.get_clock().now().to_msg()
    result.pose.position.x, result.pose.position.y = float(pose["x"]), float(pose["y"])
    result.pose.orientation.z, result.pose.orientation.w = math.sin(pose["yaw"]/2), math.cos(pose["yaw"]/2)
    return result


class ObjectApproach:
    def __init__(self, node):
        self.node = node
        self.objects = node.create_client(QueryObjects, "object_map/query")
        self.costmap = node.create_client(GetCostmap, "global_costmap/get_costmap")
        self.footprint = node.create_client(GetParameters, "global_costmap/global_costmap/get_parameters")
        self.controller = node.create_client(GetParameters, "controller_server/get_parameters")
        self.planner = ActionClient(node, ComputePathToPose, "compute_path_to_pose")

    def call(self, op, client, request):
        if not client.service_is_ready():
            raise DestinationError("APPROACH_UNAVAILABLE", "Object query, costmap or navigation parameters are unavailable.")
        future = client.call_async(request)
        op["services"].append((client, future))
        return future

    def query(self, op):
        request = op["handle"].request
        return self.call(op, self.objects, QueryObjects.Request(
            object_id=request.object_id, session_id=request.session_id,
            include_stale=True, include_images=False, limit=2))

    def begin(self, op, start):
        op.update(phase="querying_object", services=[], approach_deadline=time.monotonic()+15.,
                  approach_start=start, planning_sent=None, planning_handle=None,
                  planning_result=None, planning_canceled=False, candidates=[], attempted=0)
        op["query"] = self.query(op)

    def read_object(self, op, response):
        request = op["handle"].request
        if not response.success:
            raise DestinationError("OBJECT_QUERY_FAILED", response.message)
        if response.current_session_id != request.session_id:
            raise DestinationError("MAP_CHANGED", "Object map session changed while resolving the destination.")
        records = [o for o in response.objects if o.object_id == request.object_id
                   and o.session_id == request.session_id and o.header.frame_id == "map"]
        if len(records) != 1:
            raise DestinationError("OBJECT_NOT_FOUND", "No unique confirmed object with this UUID exists in the current map.")
        obj = records[0]
        if not all(math.isfinite(v) for v in (obj.position.x, obj.position.y, obj.position.z)):
            raise DestinationError("INVALID_OBJECT", "Stored object position is invalid.")
        return obj

    def request_grid(self, op, final=False):
        op["grid_futures"] = [
            self.call(op, self.costmap, GetCostmap.Request()),
            self.call(op, self.footprint, GetParameters.Request(names=["footprint", "footprint_padding"])),
            self.call(op, self.controller, GetParameters.Request(names=["goal_checker.xy_goal_tolerance"])),
        ]
        op["phase"] = "checking_approach" if final else "finding_approach"
        if final:
            op["query"] = self.query(op)

    def read_grid(self, op):
        mapping, footprint, controller = [future.result() for future in op["grid_futures"]]
        now = self.node.get_clock().now().nanoseconds * 1e-9
        age = now - stamp_seconds(mapping.map.metadata.update_time)
        if not -.1 <= age <= 2.:
            raise DestinationError("STALE_COSTMAP", "Approach selection requires a freshly updated costmap.")
        if (len(footprint.values) != 2 or len(controller.values) != 1
                or footprint.values[0].type != ParameterType.PARAMETER_STRING
                or footprint.values[1].type != ParameterType.PARAMETER_DOUBLE
                or controller.values[0].type != ParameterType.PARAMETER_DOUBLE):
            raise DestinationError("INVALID_FOOTPRINT", "Robot footprint or goal tolerance parameters are unavailable.")
        tolerance = controller.values[0].double_value
        radius = approach_radius(footprint.values[0].string_value,
                                 footprint.values[1].double_value, tolerance)
        return ApproachGrid(mapping.map), radius, tolerance

    def advance(self, op):
        if time.monotonic() >= op["approach_deadline"]:
            raise DestinationError("APPROACH_TIMEOUT", "Bounded approach search timed out; no movement was dispatched.")
        phase = op["phase"]
        if phase == "querying_object" and op["query"].done():
            obj = self.read_object(op, op["query"].result())
            op["object"] = obj
            self.request_grid(op)
        elif phase in ("finding_approach", "checking_approach") and all(f.done() for f in op["grid_futures"]):
            if phase == "checking_approach" and not op["query"].done():
                return None
            grid, radius, tolerance = self.read_grid(op)
            op.update(grid=grid, approach_radius=radius, xy_tolerance=tolerance)
            if phase == "checking_approach":
                latest = self.read_object(op, op["query"].result())
                obj = op["object"]
                if math.hypot(latest.position.x-obj.position.x, latest.position.y-obj.position.y) > .15:
                    raise DestinationError("OBJECT_MOVED", "Object position changed while selecting an approach; query it again.")
                goal = op["candidate"]
                if not grid.safe(goal["x"], goal["y"], radius):
                    raise DestinationError("APPROACH_CHANGED", "Selected approach is no longer clear; movement was not dispatched.")
                op["target_details"] = dict(kind="object", object_id=obj.object_id, label=obj.label,
                    object_position_m=[obj.position.x, obj.position.y, obj.position.z],
                    observation_stamp_s=stamp_seconds(obj.header.stamp), stale=obj.stale,
                    requested_standoff_m=op["handle"].request.standoff_m,
                    selected_standoff_m=math.hypot(goal["x"]-obj.position.x, goal["y"]-obj.position.y),
                    clearance_envelope_radius_m=radius, candidates_checked=op["attempted"],
                    evidence="last_known_object_location_and_current_navigation_costmap")
                return pose_message(self.node, goal)
            obj = op["object"]
            op["candidates"] = grid.candidates((obj.position.x, obj.position.y), op["approach_start"],
                                               op["handle"].request.standoff_m, radius)
            self.plan_next(op)
        elif phase == "planning_approach":
            if op["planning_result"] is None or not op["planning_result"].done():
                return None
            response = op["planning_result"].result()
            path = response.result.path
            goal = op["candidate"]
            good = (response.status == GoalStatus.STATUS_SUCCEEDED and response.result.error_code == 0
                    and path.header.frame_id == "map" and len(path.poses) > 0
                    and all(math.isfinite(v) for p in path.poses for v in (p.pose.position.x, p.pose.position.y)))
            if good:
                end = path.poses[-1].pose.position
                good = math.hypot(end.x-goal["x"], end.y-goal["y"]) <= op["xy_tolerance"] + 1e-6
            if good:
                self.request_grid(op, final=True)
            else:
                self.plan_next(op)
        return None

    def plan_next(self, op):
        if not op["candidates"]:
            raise DestinationError("NO_SAFE_APPROACH", "No reachable clear pose found among the bounded approach candidates.")
        if not self.planner.server_is_ready():
            raise DestinationError("APPROACH_UNAVAILABLE", "Nav2 path planning action is unavailable.")
        op["candidate"] = op["candidates"].pop(0)
        op["attempted"] += 1
        request = ComputePathToPose.Goal(goal=pose_message(self.node, op["candidate"]),
                                        use_start=False, planner_id="GridBased")
        op["phase"] = "planning_approach"
        op["planning_handle"] = op["planning_result"] = None
        op["planning_sent"] = self.planner.send_goal_async(request)
        op["planning_sent"].add_done_callback(lambda future: self.plan_accepted(op, future))

    def plan_accepted(self, op, future):
        handle = future.result()
        op["planning_handle"] = handle
        if handle.accepted:
            op["planning_result"] = handle.get_result_async()
            if op["planning_canceled"]:
                handle.cancel_goal_async()
        elif not op["planning_canceled"]:
            # A rejected request is different from an unreachable candidate;
            # do not hammer an inactive planner with every candidate.
            op["planning_rejected"] = True

    def cancel(self, op):
        if "services" not in op:
            return
        op["planning_canceled"] = True
        handle = op["planning_handle"]
        if handle is not None and handle.accepted and not op["planning_result"].done():
            handle.cancel_goal_async()
        for client, future in op["services"]:
            if not future.done():
                client.remove_pending_request(future)
                future.cancel()
