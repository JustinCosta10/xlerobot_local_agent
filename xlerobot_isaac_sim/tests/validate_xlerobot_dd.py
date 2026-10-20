#!/usr/bin/env python3
"""Check the actual launch composition with USD bindings, without a GPU or D455."""
import hashlib
from pathlib import Path
import runpy

from pxr import Usd, UsdGeom, UsdPhysics, UsdShade


def main():
    root = Path(__file__).resolve().parents[2]
    asset = root / "xlerobot_isaac_sim/xlerobot_dd.usd"
    description = root / "ros2_ws/src/xlerobot_description/urdf/kinematics.urdf"
    runner = runpy.run_path(str(root / "ros2_ws/src/xlerobot_isaac/scripts/run_isaac.py"))
    digest = hashlib.sha256(asset.read_bytes()).hexdigest()
    stage = Usd.Stage.CreateInMemory()
    body = runner["compose_robot"](stage, asset, description)
    assert not stage.GetPrimAtPath(runner["ROBOT_PATH"] + "/Graphs/cmd_vel").IsActive()
    assert not stage.GetPrimAtPath(runner["ROBOT_PATH"] + "/Graphs/realsense_ros2").IsActive()
    assert not stage.GetPrimAtPath(runner["SENSOR_PATH"]).IsActive()
    assert stage.GetPrimAtPath(runner["ROBOT_PATH"]).GetVariantSet("Sensor").GetVariantSelection() == "None"
    assert all("http" not in layer.identifier for layer in stage.GetUsedLayers())
    for side in ("left", "right"):
        wheel = stage.GetPrimAtPath(f"{body}/{side}_wheel")
        assert abs(UsdPhysics.MassAPI(wheel).GetMassAttr().Get() - 0.2) < 1e-6
        collider = UsdGeom.Cylinder(stage.GetPrimAtPath(f"{wheel.GetPath()}/cylinder"))
        assert collider.GetRadiusAttr().Get() == 0.0635
        assert not UsdPhysics.CollisionAPI(collider.GetPrim()).GetCollisionEnabledAttr().Get()
        rolling = UsdGeom.Cylinder(stage.GetPrimAtPath(f"{wheel.GetPath()}/rolling_collision"))
        assert UsdPhysics.CollisionAPI(rolling.GetPrim()).GetCollisionEnabledAttr().Get()
        margin = rolling.GetPrim().GetAttribute("physxConvexGeometry:margin").Get()
        assert abs(rolling.GetRadiusAttr().Get() + margin - collider.GetRadiusAttr().Get()) < 1e-9
        assert abs(rolling.GetHeightAttr().Get() + 2 * margin - collider.GetHeightAttr().Get()) < 1e-9
        assert rolling.GetAxisAttr().Get() == collider.GetAxisAttr().Get()
        material, _ = UsdShade.MaterialBindingAPI(collider.GetPrim()).ComputeBoundMaterial("physics")
        rolling_material, _ = UsdShade.MaterialBindingAPI(rolling.GetPrim()).ComputeBoundMaterial("physics")
        assert rolling_material.GetPath() == material.GetPath()
        assert material.GetPrim().GetAttribute("physics:dynamicFriction").Get() > 0.79
        drive = UsdPhysics.DriveAPI(stage.GetPrimAtPath(
            f"{runner['ROBOT_PATH']}/Physics/{side}_wheel_joint"), "angular")
        assert drive.GetStiffnessAttr().Get() == 0
        assert drive.GetDampingAttr().Get() == 10000
        assert abs(drive.GetMaxForceAttr().Get() - 0.980665) < 1e-6
    print("PASS equivalent wheel contact dimensions, materials, and preserved drives; legacy drive graph excluded; sensors disabled")
    # Check camera graph wiring offline without loading the external D455 asset.
    graph = runner["ROBOT_PATH"] + "/Graphs/realsense_ros2"
    stage.GetPrimAtPath(graph).SetActive(True)
    runner["configure_camera_graph"](stage)
    for name in ("ClockPublisher", "RobotTfCompute", "RobotTfPublisher",
                 "ChassisToBaseTfPublisher", "HeadTiltToHeadCameraTfPublisher"):
        assert not stage.GetAttributeAtPath(f"{graph}/{name}.inputs:execIn").GetConnections()
    for name in ("CameraMountTfCompute", "CameraMountTfPublisher", "ColorRenderProduct",
                 "DepthRenderProduct", "ColorImagePublisher", "DepthImagePublisher", "PointCloudPublisher"):
        assert stage.GetPrimAtPath(f"{graph}/{name}").IsActive()
        assert stage.GetAttributeAtPath(f"{graph}/{name}.inputs:execIn").GetConnections()
    assert stage.GetAttributeAtPath(graph + "/Context.inputs:useDomainIDEnvVar").Get()
    print("PASS camera graph retains streams and mount TF; duplicate clock/robot TF execution disconnected")
    # Reproduce the reported regression: valid physics with invisible wheels.
    UsdGeom.Imageable(stage.GetPrimAtPath(body + "/left_wheel")).MakeInvisible()
    try:
        runner["validate_model"](stage, description)
    except RuntimeError as error:
        assert "hidden DD wheel visual" in str(error)
    else:
        raise AssertionError("Invisible wheels were accepted")
    assert hashlib.sha256(asset.read_bytes()).hexdigest() == digest
    print("PASS hidden-wheel regression detected; saved model unchanged by composition")


if __name__ == "__main__":
    main()
