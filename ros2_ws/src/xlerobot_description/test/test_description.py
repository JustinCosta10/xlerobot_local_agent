from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

PACKAGE = Path(__file__).resolve().parents[1]


def test_joint_contract_and_tree():
    robot = ET.parse(PACKAGE / "urdf/kinematics.urdf").getroot()
    joints = robot.findall("joint")
    movable = {j.get("name") for j in joints if j.get("type") != "fixed"}
    config = yaml.safe_load((PACKAGE / "config/joints.yaml").read_text())["joints"]
    assert len(movable) == 16
    assert movable == set(config)
    parents = {j.find("child").get("link"): j.find("parent").get("link") for j in joints}
    assert len(parents) == len(joints)
    for link in parents:
        visited = set()
        while link in parents:
            assert link not in visited
            visited.add(link)
            link = parents[link]
        assert link == "base_link"
    assert {name for name, value in config.items() if value["command_interface"] == "velocity"} == {"left_wheel_joint", "right_wheel_joint"}


def test_head_and_wheel_frames_at_zero_configuration():
    robot = ET.parse(PACKAGE / "urdf/kinematics.urdf").getroot()
    transforms = {"base_link": np.eye(4)}
    for joint in robot.findall("joint"):
        origin = joint.find("origin")
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_euler("xyz", list(map(float, origin.get("rpy").split()))).as_matrix()
        matrix[:3, 3] = list(map(float, origin.get("xyz").split()))
        transforms[joint.find("child").get("link")] = transforms[joint.find("parent").get("link")] @ matrix
    np.testing.assert_allclose(transforms["left_wheel"][:3, 3], [0, -0.225, 0], atol=1e-6)
    np.testing.assert_allclose(transforms["right_wheel"][:3, 3], [0, 0.225, 0], atol=1e-6)
    np.testing.assert_allclose(transforms["head_tilt_link"][:3, 3], [-0.048, -0.002, 1.08765], atol=1e-6)
    np.testing.assert_allclose(transforms["head_tilt_link"][:3, :3], np.eye(3), atol=1e-6)
    controllers = yaml.safe_load((PACKAGE.parent / "xlerobot_control/config/controllers.yaml").read_text())
    drive = controllers["/**/base_controller"]["ros__parameters"]
    for side, sign in (("left", 1), ("right", -1)):
        joint = robot.find(f"joint[@name='{drive[side + '_wheel_names'][0]}']")
        link = joint.find("child").get("link")
        pose = transforms[link]
        assert sign * pose[1, 3] > 0  # Controller sides agree with ROS +Y left.
        axis = pose[:3, :3] @ np.array(list(map(float, joint.find("axis").get("xyz").split())))
        np.testing.assert_allclose(axis, [0, 1, 0], atol=1e-6)  # Positive rotation rolls toward +X.
        collision = robot.find(f"link[@name='{link}']/collision/geometry/cylinder")
        assert float(collision.get("radius")) == drive["wheel_radius"] == 0.0635
    assert drive["wheel_separation"] == 0.45


def test_mesh_references_are_portable_and_limits_valid():
    robot = ET.parse(PACKAGE / "urdf/kinematics.urdf").getroot()
    for mesh in robot.findall(".//mesh"):
        assert mesh.get("filename").startswith("package://xlerobot_description/meshes/")
    for joint in robot.findall("joint"):
        if joint.get("type") == "revolute":
            limit = joint.find("limit")
            assert float(limit.get("lower")) < float(limit.get("upper"))
            assert float(limit.get("velocity")) > 0


def test_generated_model_matches_saved_dd_source():
    import hashlib
    import runpy
    source = PACKAGE.parents[2] / "xlerobot_isaac_sim/source/xlerobot_dd.urdf"
    generated = runpy.run_path(str(PACKAGE / "scripts/generate_description.py"))["generate"](source)
    assert generated[0] == (PACKAGE / "urdf/kinematics.urdf").read_text()
    config = yaml.safe_load((PACKAGE / "config/joints.yaml").read_text())
    assert config["source_sha256"] == hashlib.sha256(source.read_bytes()).hexdigest()
    assert generated[1] == config["joints"]
    for mesh in generated[2]:
        assert (source.parent / "meshes" / mesh).is_file()


def test_backend_selects_one_hardware_owner():
    import xacro
    for backend, plugin in (("isaac", None), ("mock", "mock_components/GenericSystem")):
        root = ET.fromstring(xacro.process_file(
            str(PACKAGE / "urdf/robot.urdf.xacro"),
            mappings={"backend": backend, "namespace": "test_robot"}).toxml())
        control = root.find("ros2_control")
        if plugin is None:
            assert control is None  # Isaac synthesizes its own native hardware block.
        else:
            assert control.find("hardware/plugin").text == plugin
            assert len(control.findall("joint")) == 16
