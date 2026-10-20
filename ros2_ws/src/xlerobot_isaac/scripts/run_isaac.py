#!/usr/bin/env python3
"""Run with Isaac Sim 6.2 python.sh, after sourcing native ROS 2 Jazzy.

Compose a fresh stage around xlerobot_dd.usd. All added scene/graph opinions
live in memory; source USD layers and their sensor calibration are never saved.
"""

import argparse
import json
from pathlib import Path
import signal
import time
import traceback


ROBOT_PATH = "/World/xlerobot"
ROOT_PATH = ROBOT_PATH + "/Geometry/world/chassis"
CONTROL_GRAPH = "/World/ROS2Control"
SENSOR_PATH = ROOT_PATH + "/top_base_link/head_pan_link/head_tilt_link/head_camera_link/realsense_mount/realsense_d455"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--asset", required=True, type=Path)
    parser.add_argument("--description", required=True, type=Path,
                        help="shared kinematic URDF used to verify simulator frame/axis parity")
    parser.add_argument("--controllers", required=True, type=Path,
                        help="shared ros2_control controller YAML")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--sensors", action="store_true",
                        help="load the D455 and enable its camera/TF publishing graph")
    parser.add_argument("--navigation-scene", action="store_true",
                        help="add obstacles and publish ground-truth poses for simulated navigation")
    parser.add_argument("--namespace", default="xlerobot")
    parser.add_argument("--max-steps", type=int, default=0,
                        help="exit after this many updates; 0 runs until interrupted")
    parser.add_argument("--state-log", type=Path,
                        help="optional physics measurements for motion diagnosis (JSON lines)")
    args = parser.parse_args()
    if not args.asset.is_file():
        parser.error(f"Robot asset not found: {args.asset}")
    if not args.description.is_file():
        parser.error(f"Robot description not found: {args.description}")
    if not args.controllers.is_file():
        parser.error(f"Controller configuration not found: {args.controllers}")
    if args.max_steps < 0:
        parser.error("--max-steps cannot be negative")
    return args


def compose_robot(stage, asset, description, sensors=False):
    """Reference the saved DD model, with only ROS integration overrides."""
    import xml.etree.ElementTree as ET
    from pxr import UsdGeom, UsdPhysics

    robot = UsdGeom.Xform.Define(stage, ROBOT_PATH).GetPrim()
    robot.GetVariantSet("Sensor").SetVariantSelection("Sensors" if sensors else "None")
    # The native plugin is the only joint controller in this launch. Preserve
    # the saved Twist graph in the asset, but exclude it before registering nodes.
    stage.OverridePrim(ROBOT_PATH + "/Graphs/cmd_vel").SetActive(False)
    # Sensor selection controls both the device and its saved publishing graph.
    stage.OverridePrim(ROBOT_PATH + "/Graphs/realsense_ros2").SetActive(sensors)
    if not sensors:
        sensor = stage.OverridePrim(SENSOR_PATH)
        sensor.GetReferences().SetReferences([])
        sensor.SetActive(False)
    robot.GetReferences().AddReference(str(asset.resolve()))
    if sensors:
        configure_camera_graph(stage)
    root = stage.GetPrimAtPath(ROOT_PATH)
    if not root or not root.HasAPI(UsdPhysics.ArticulationRootAPI):
        raise RuntimeError(f"Expected xlerobot_dd articulation at {ROOT_PATH}; loaded {asset}")
    configure_wheel_contacts(stage)
    # The URDF import has passive arm/head drives. Add the position servos
    # needed by ros2_control while preserving the saved wheel drives/materials.
    for joint in ET.parse(description).getroot().findall("joint"):
        if joint.get("type") != "revolute":
            continue
        prim = stage.GetPrimAtPath(f"{ROBOT_PATH}/Physics/{joint.get('name')}")
        if not prim:
            raise RuntimeError(f"Missing DD joint: {joint.get('name')}")
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.CreateStiffnessAttr(100.0)
        drive.CreateDampingAttr(10.0)
        drive.CreateMaxForceAttr(float(joint.find("limit").get("effort")))
    validate_model(stage, description)
    print(f"Loaded xlerobot_dd asset: {asset.resolve()}", flush=True)
    return ROOT_PATH


def configure_wheel_contacts(stage):
    """Round tire edges for smooth rolling while preserving overall dimensions."""
    from pxr import PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade

    caster = stage.GetPrimAtPath(ROBOT_PATH + "/Physics/Materials/rear_caster_proxy")
    if caster:
        PhysxSchema.PhysxMaterialAPI.Apply(caster).CreateFrictionCombineModeAttr("min")

    for side in ("left", "right"):
        path = f"{ROOT_PATH}/{side}_wheel"
        cylinder = UsdGeom.Cylinder(stage.GetPrimAtPath(path + "/cylinder"))
        radius, half_width = cylinder.GetRadiusAttr().Get(), cylinder.GetHeightAttr().Get() / 2
        UsdPhysics.CollisionAPI(cylinder.GetPrim()).CreateCollisionEnabledAttr(False)
        margin = 0.005
        mesh = UsdGeom.Cylinder.Define(stage, path + "/rolling_collision")
        mesh.CreateRadiusAttr(radius - margin)
        mesh.CreateHeightAttr(2 * half_width - 2 * margin)
        mesh.CreateAxisAttr(cylinder.GetAxisAttr().Get())
        mesh.GetPrim().CreateAttribute("physxConvexGeometry:margin", Sdf.ValueTypeNames.Float).Set(margin)
        mesh.CreatePurposeAttr("guide")
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        material, _ = UsdShade.MaterialBindingAPI(cylinder.GetPrim()).ComputeBoundMaterial("physics")
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material, materialPurpose="physics")


def configure_camera_graph(stage):
    """Keep camera streams and mount TF; leave clock and robot TF to ROS bringup."""
    graph = ROBOT_PATH + "/Graphs/realsense_ros2"
    for name in ("ClockPublisher", "RobotTfCompute", "RobotTfPublisher",
                 "ChassisToBaseTfPublisher", "HeadTiltToHeadCameraTfPublisher"):
        # Disconnect execution explicitly: deactivating an individual USD node
        # does not stop an already registered OmniGraph node in this Isaac build.
        stage.GetAttributeAtPath(f"{graph}/{name}.inputs:execIn").SetConnections([])
    stage.GetAttributeAtPath(graph + "/Context.inputs:useDomainIDEnvVar").Set(True)
    stage.GetAttributeAtPath(graph + "/ReadSimulationTime.inputs:resetOnStop").Set(False)


def show_control_graph(stage):
    """Open the native ros2_control manager and clock graph in the GUI."""
    import omni.usd
    from omni.graph.window.action import ActionGraphExtension
    from pxr import Gf, Sdf

    for name, position in {
        "Tick": (0, 0), "Time": (0, 170), "Context": (0, 340),
        "Clock": (330, 220), "ControllerManager": (330, 0),
    }.items():
        stage.GetPrimAtPath(f"{CONTROL_GRAPH}/{name}").CreateAttribute(
            "ui:nodegraph:node:pos", Sdf.ValueTypeNames.Float2,
            custom=False, variability=Sdf.VariabilityUniform).Set(Gf.Vec2f(*position))
    omni.usd.get_context().get_selection().set_selected_prim_paths([CONTROL_GRAPH], True)
    ActionGraphExtension.show_graph([stage.GetPrimAtPath(CONTROL_GRAPH)])


def validate_model(stage, description):
    """Reject mismatched link poses/joint axes before physics or commands start."""
    import math
    import xml.etree.ElementTree as ET
    import numpy as np
    from pxr import Gf, UsdGeom, UsdPhysics

    cache = UsdGeom.XformCache()
    count = 0
    paths = {"chassis": ROOT_PATH}
    for joint in ET.parse(description).getroot().findall("joint"):
        parent, child = joint.find("parent").get("link"), joint.find("child").get("link")
        if parent == "base_link":
            continue  # ROS axle frame is not a separate physical USD body.
        paths[child] = paths[parent] + "/" + child
        parent_prim = stage.GetPrimAtPath(paths[parent])
        child_prim = stage.GetPrimAtPath(paths[child])
        if not parent_prim or not child_prim:
            raise RuntimeError(f"Missing simulator body for shared transform {parent} -> {child}")
        # USD uses row vectors; URDF uses column vectors.
        actual = np.array(cache.GetLocalToWorldTransform(child_prim)
                          * cache.GetLocalToWorldTransform(parent_prim).GetInverse()).T
        origin = joint.find("origin")
        roll, pitch, yaw = map(float, origin.get("rpy").split())
        cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
        expected = np.eye(4)
        expected[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                            [-sp, cp*sr, cp*cr]]
        expected[:3, 3] = list(map(float, origin.get("xyz").split()))
        if not np.allclose(actual, expected, atol=2e-5):
            raise RuntimeError(f"USD/URDF transform mismatch: {parent} -> {child}")
        if joint.get("type") != "fixed":
            usd_joint = UsdPhysics.RevoluteJoint(stage.GetPrimAtPath(f"{ROBOT_PATH}/Physics/{joint.get('name')}"))
            axis = {"X": Gf.Vec3d(1, 0, 0), "Y": Gf.Vec3d(0, 1, 0), "Z": Gf.Vec3d(0, 0, 1)}[usd_joint.GetAxisAttr().Get()]
            axis = Gf.Rotation(Gf.Quatd(usd_joint.GetLocalRot1Attr().Get())).TransformDir(axis)
            if not np.allclose(axis, list(map(float, joint.find("axis").get("xyz").split())), atol=2e-5):
                raise RuntimeError(f"USD/URDF joint axis mismatch: {joint.get('name')}")
            count += 1
    print(f"Validated shared USD/URDF link transforms and {count} joint axes", flush=True)
    # Inspect rendered geometry, not just joints/colliders: the old asset
    # passed physics checks despite both wheel visuals being invisible.
    for side in ("left", "right"):
        for mesh in ("dual_wheel_tire", "dual_wheel_core", f"dual_wheel_{side}_hub"):
            prim = stage.GetPrimAtPath(f"{ROOT_PATH}/{side}_wheel/{mesh}")
            if (not prim or not prim.IsA(UsdGeom.Mesh)
                    or UsdGeom.Imageable(prim).ComputeVisibility() == UsdGeom.Tokens.invisible):
                raise RuntimeError(f"Missing or hidden DD wheel visual: {side}/{mesh}")
    print("Validated visible DD tires, cores, and hubs", flush=True)


def create_control_graph(og, namespace, controllers):
    """Start NVIDIA's in-process controller manager and publish the shared clock."""
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": CONTROL_GRAPH, "evaluator_name": "execution"},
        {keys.CREATE_NODES: [
            ("Tick", "omni.graph.action.OnPlaybackTick"),
            ("Time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("Clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("ControllerManager", "isaacsim.ros2.control.ROS2ControlManager"),
        ], keys.CONNECT: [
            ("Tick.outputs:tick", "Clock.inputs:execIn"),
            ("Tick.outputs:tick", "ControllerManager.inputs:execIn"),
            ("Context.outputs:context", "Clock.inputs:context"),
            ("Time.outputs:simulationTime", "Clock.inputs:timeStamp"),
        ], keys.SET_VALUES: [
            # Joints live under /Physics, beside /Geometry; use their common parent.
            ("ControllerManager.inputs:targetPrim", [ROBOT_PATH]),
            ("ControllerManager.inputs:controllerConfig", str(controllers.resolve())),
            ("ControllerManager.inputs:namespace", namespace),
            ("ControllerManager.inputs:publishRobotDescription", False),
            ("ControllerManager.inputs:useSimTime", True),
            ("Context.inputs:useDomainIDEnvVar", True),
            ("Time.inputs:resetOnStop", False),
            ("Clock.inputs:topicName", "/clock"),
        ]},
    )
    print(f"Native Isaac ros2_control: /{namespace}/controller_manager; graph {CONTROL_GRAPH}", flush=True)


def main():
    args = parse_args()
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": args.headless, "shutdown_watchdog_timeout": 10.0})
    exit_code = 0
    pose_publisher = None
    try:
        import isaacsim.core.experimental.utils.app as app_utils
        from isaacsim.core.simulation_manager import SimulationManager
        import omni.graph.core as og
        import omni.usd
        from pxr import Gf, UsdGeom, UsdLux, UsdPhysics

        for extension in ("isaacsim.ros2.bridge", "isaacsim.ros2.control"):
            app_utils.enable_extension(extension)
        from xlerobot_isaac.native_compat import apply_isaac_620_compatibility
        apply_isaac_620_compatibility()
        if not args.headless:
            app_utils.enable_extension("omni.graph.window.action")
        app.update()
        omni.usd.get_context().new_stage()
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        world = UsdGeom.Xform.Define(stage, "/World")
        stage.SetDefaultPrim(world.GetPrim())
        root_path = compose_robot(stage, args.asset, args.description, args.sensors)
        if args.state_log:
            from pxr import PhysxSchema, PhysicsSchemaTools
            import omni.physx
            for prim in stage.Traverse():
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
        physics = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
        physics.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
        physics.CreateGravityMagnitudeAttr(9.81)
        ground = UsdGeom.Cube.Define(stage, "/World/Ground")
        ground.CreateSizeAttr(1.0)
        ground.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
        ground.AddScaleOp().Set(Gf.Vec3d(20, 20, 0.1))
        UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
        if args.navigation_scene:
            from xlerobot_isaac.navigation_scene import add_navigation_obstacles
            add_navigation_obstacles(stage)
        light = UsdLux.DomeLight.Define(stage, "/World/Light")
        light.CreateIntensityAttr(1000.0)
        if not args.headless:
            from omni.kit.viewport.utility import get_active_viewport
            camera = UsdGeom.Camera.Define(stage, "/World/InspectionCamera")
            camera.CreateFocalLengthAttr(25.0)
            camera.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(
                Gf.Vec3d(-2, -2, 1.6), Gf.Vec3d(0, 0, 0.6), Gf.Vec3d(0, 0, 1)).GetInverse())
            viewport = get_active_viewport()
            if viewport:
                viewport.camera_path = camera.GetPath()
        # Register graphs brought in by the USD reference before editing them.
        app.update()
        create_control_graph(og, args.namespace, args.controllers)
        app.update()
        if not args.headless:
            show_control_graph(stage)
        SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
        app_utils.play()
        running = True
        if args.navigation_scene:
            from xlerobot_isaac.navigation_scene import GroundTruthPublisher
            pose_publisher = GroundTruthPublisher(args.namespace, args.description)

        def stop(*_):
            nonlocal running
            running = False

        signal.signal(signal.SIGINT, stop)
        signal.signal(signal.SIGTERM, stop)
        print("XLeRobot native ros2_control running; ROS_DOMAIN_ID is inherited from the environment.", flush=True)
        step = 0
        state_view = None
        if args.state_log:
            args.state_log.write_text("")
        while app.is_running() and running and (not args.max_steps or step < args.max_steps):
            started = time.monotonic()
            app.update()
            step += 1
            if pose_publisher and SimulationManager.get_simulation_time() > 0:
                if state_view is None:
                    state_view = SimulationManager.get_physics_simulation_view().create_articulation_view(root_path)
                pose_publisher.publish(state_view.get_root_transforms().numpy()[0],
                                       SimulationManager.get_simulation_time())
            if args.state_log and step % 15 == 0 and SimulationManager.get_simulation_time() > 0:
                if state_view is None:
                    state_view = SimulationManager.get_physics_simulation_view().create_articulation_view(root_path)
                sample = {"sim": SimulationManager.get_simulation_time(),
                          "body": state_view.get_root_transforms().numpy().tolist()[0],
                          "names": list(state_view.shared_metatype.dof_names),
                          "position": state_view.get_dof_positions().numpy().tolist()[0],
                          "velocity": state_view.get_dof_velocities().numpy().tolist()[0],
                          "target": state_view.get_dof_velocity_targets().numpy().tolist()[0]}
                headers, contacts = omni.physx.get_physx_simulation_interface().get_contact_report()
                sample["contacts"] = [
                    {"a": str(PhysicsSchemaTools.intToSdfPath(h.collider0)),
                     "b": str(PhysicsSchemaTools.intToSdfPath(h.collider1)),
                     "impulses": [list(contacts[i].impulse) for i in range(h.contact_data_offset, h.contact_data_offset + h.num_contact_data)]}
                    for h in headers]
                with args.state_log.open("a") as stream:
                    stream.write(json.dumps(sample) + "\n")
            # Limit real-time speed while allowing slower rendering/physics.
            time.sleep(max(0.0, 1.0 / 60.0 - (time.monotonic() - started)))
        app_utils.stop()
    except Exception:
        exit_code = 1
        traceback.print_exc()
    finally:
        if pose_publisher is not None:
            pose_publisher.close()
        app.close(wait_for_replicator=False, exit_code=exit_code)


if __name__ == "__main__":
    main()
