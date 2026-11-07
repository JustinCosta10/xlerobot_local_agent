#!/usr/bin/env python3
"""Exercise point, relative and object navigation in isolated Isaac; no model/API."""

import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
from threading import Thread
import time

from geometry_msgs.msg import PoseStamped
from xlerobot_agent.ros_adapter import RosAdapter


def check(process):
    robot = RosAdapter(namespace="xlerobot_agent_test", enable_navigation=True)
    truth = []
    robot.node.create_subscription(PoseStamped, "sim_ground_truth", truth.append, 10)

    def wait(predicate, timeout=15):
        deadline = time.monotonic() + timeout
        while not predicate():
            assert process.poll() is None, "Isaac launch exited; inspect log/agent-navigation-test.log"
            assert time.monotonic() < deadline, "Acceptance test timed out"
            time.sleep(.05)

    def navigate(x, y, yaw=0., timeout_s=90.):
        wait(lambda: robot.get_navigation_state().get("ready"))
        result = robot.navigate(x=x, y=y, yaw=yaw,
            session_id=robot.get_navigation_state()["session_id"], timeout_s=timeout_s)
        print(result, flush=True)
        return result

    try:
        wait(lambda: robot.get_navigation_state().get("ready") and truth, 300)
        state = robot.get_navigation_state()
        print("PASS: supervisor ready with pose/session and fresh mapping", flush=True)
        start = truth[-1].pose.position
        q = truth[-1].pose.orientation
        world_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        heading = state["pose"]["yaw"] - world_yaw
        c, s = math.cos(heading), math.sin(heading)
        # Ground truth is used only by this test observer to identify the scene's
        # occupied table center and measure travel, never by navigation itself.
        blocked_x = state["pose"]["x"] + c*(-2.4-start.x) - s*(0-start.y)
        blocked_y = state["pose"]["y"] + s*(-2.4-start.x) + c*(0-start.y)
        result = navigate(.5, 0.)
        assert result["success"] and result["stop_confirmed"], result
        finish = truth[-1].pose.position
        travel = math.hypot(finish.x-start.x, finish.y-start.y)
        assert travel > .2, f"Insufficient physical travel: {travel}"
        assert math.hypot(result["final_pose"]["x"]-.5, result["final_pose"]["y"]) < .2
        print(f"PASS: supervised goal drove {travel:.3f} m and stopped", flush=True)
        result = navigate(blocked_x, blocked_y, timeout_s=30.)
        assert not result["success"] and result["code"] == "FAILED" and result["stop_confirmed"], result
        print("PASS: occupied destination failed and stopped", flush=True)
        results = []
        worker = Thread(target=lambda: results.append(navigate(.9, 0.)))
        worker.start()
        wait(lambda: robot.navigation.active is not None and robot.navigation.active.accepted)
        time.sleep(.3)
        stopped = robot.cancel_navigation()
        worker.join(timeout=10.)
        assert stopped["stop_confirmed"] and results and results[0]["code"] == "CANCELED"
        print("PASS: explicit cancellation terminated Nav2 and confirmed a stop", flush=True)
        wait(lambda: robot.get_navigation_state().get("ready"))
        relative = robot.navigate_relative(forward_m=.3, left_m=0., yaw_delta_rad=0.,
            session_id=robot.get_navigation_state()["session_id"], timeout_s=90.)
        print(relative, flush=True)
        assert relative["success"] and relative["stop_confirmed"], relative
        origin, goal = relative["target"]["starting_pose"], relative["resolved_pose"]
        assert abs(math.hypot(goal["x"]-origin["x"], goal["y"]-origin["y"])-.3) < 1e-6
        print("PASS: relative offset resolved from measured starting pose and reached", flush=True)
        objects = []
        def observed():
            objects[:] = robot.query()["objects"]
            return bool(objects)
        wait(observed, 90)
        wait(lambda: robot.get_navigation_state().get("ready"))
        base = robot.get_navigation_state()["pose"]
        target = min(objects, key=lambda o: (o["label"] != "cube",
            math.hypot(o["position_m"][0]-base["x"], o["position_m"][1]-base["y"])))
        print(f"Approaching stored {target['label']} {target['object_id']} at {target['position_m']}", flush=True)
        approach = robot.navigate_to_object(object_id=target["object_id"], standoff_m=.8,
            session_id=target["session_id"], timeout_s=120.)
        print(approach, flush=True)
        assert approach["success"] and approach["stop_confirmed"], approach
        assert .8-1e-6 <= approach["target"]["selected_standoff_m"] <= 1.6+1e-6
        final = approach["final_pose"]
        object_point = approach["target"]["object_position_m"]
        facing = math.atan2(object_point[1]-final["y"], object_point[0]-final["x"])
        assert abs(math.atan2(math.sin(facing-final["yaw"]), math.cos(facing-final["yaw"]))) < .3
        print(f"PASS: clear object approach reached; {approach['target']['final_standoff_m']:.3f} m "
              "from the stored object position, facing it", flush=True)
    finally:
        robot.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain-id", type=int, default=89)
    args = parser.parse_args()
    if not 0 <= args.domain_id <= 101:
        parser.error("domain-id must be in 0..101")
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    workspace = Path(__file__).resolve().parents[1]
    log_path = workspace / "log/agent-navigation-test.log"
    log_path.parent.mkdir(exist_ok=True)
    with log_path.open("w") as log:
        process = subprocess.Popen([
            "ros2", "launch", "xlerobot_navigation", "sim_navigation.launch.py",
            "namespace:=xlerobot_agent_test", "rviz:=false", "foxglove:=false", "headless:=true",
            "agent_navigation:=true", "yoloe_visualize:=false",
            f"object_database:={workspace / 'log/agent-navigation-objects.sqlite3'}",
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            check(process)
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=40)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=10)


if __name__ == "__main__":
    main()
