#!/usr/bin/env python3
"""Generate the ROS description from the saved xlerobot_dd URDF.

The DD source owns geometry and kinematics; this adapter adds the ROS axle
frame and supplies conservative command limits for unspecified URDF limits.
"""

import argparse
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yaml


def numbers(value):
    return np.array([float(x) for x in value.split()])


def fmt(values):
    return " ".join(f"{float(x):.12g}" for x in values)


def generate(source):
    robot = ET.parse(source).getroot()
    if robot.tag != "robot":
        raise ValueError("Expected an XLeRobot DD URDF")
    world = robot.find("link[@name='world']")
    anchor = robot.find("joint[@name='joint_0']")
    if world is None or anchor is None or anchor.find("child").get("link") != "chassis":
        raise ValueError("Expected the source world -> chassis spawn transform")
    robot.remove(world)
    robot.remove(anchor)
    robot.insert(0, ET.Comment("Generated from source/xlerobot_dd.urdf; regenerate after source changes."))
    robot.insert(1, ET.Element("link", name="base_link"))
    wheels = [robot.find(f"joint[@name='{side}_wheel_joint']") for side in ("left", "right")]
    axle = np.mean([numbers(j.find("origin").get("xyz")) for j in wheels], axis=0)
    # Positive authored wheel motion travels toward chassis -X. ROS +X faces
    # that direction; named right_wheel is on the physical left in this frame.
    base_rotation = Rotation.from_euler("z", np.pi)
    base = ET.Element("joint", name="base_to_chassis", type="fixed")
    ET.SubElement(base, "parent", link="base_link")
    ET.SubElement(base, "child", link="chassis")
    ET.SubElement(base, "origin", xyz=fmt(base_rotation.apply(-axle)), rpy=fmt([0, 0, np.pi]))
    robot.insert(2, base)
    joints = {}
    for joint in robot.findall("joint"):
        if joint.get("type") == "fixed":
            continue
        limit = joint.find("limit")
        wheel = joint.get("name") in {"left_wheel_joint", "right_wheel_joint"}
        if float(limit.get("velocity", "0")) <= 0:
            limit.set("velocity", "1.0")
        if float(limit.get("effort", "0")) <= 0:
            limit.set("effort", "2.94")
        info = {"command_interface": "velocity" if wheel else "position",
                "velocity": float(limit.get("velocity"))}
        if not wheel:
            info.update(lower=float(limit.get("lower")), upper=float(limit.get("upper")))
        joints[joint.get("name")] = info
    meshes = set()
    prefix = "package://xlerobot_description/meshes/"
    for mesh in robot.findall(".//mesh"):
        if not mesh.get("filename").startswith(prefix):
            raise ValueError(f"Unexpected mesh URI: {mesh.get('filename')}")
        meshes.add(mesh.get("filename").removeprefix(prefix))
    ET.indent(robot)
    return ET.tostring(robot, encoding="unicode") + "\n", joints, sorted(meshes)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    workspace = Path(__file__).resolve().parents[3]
    parser.add_argument("--source", type=Path, default=workspace.parent / "xlerobot_isaac_sim/source/xlerobot_dd.urdf")
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    description, joints, meshes = generate(args.source)
    (args.output / "urdf").mkdir(parents=True, exist_ok=True)
    (args.output / "config").mkdir(exist_ok=True)
    (args.output / "urdf/kinematics.urdf").write_text(description)
    (args.output / "config/joints.yaml").write_text(yaml.safe_dump({"source_sha256": hashlib.sha256(args.source.read_bytes()).hexdigest(), "joints": joints}, sort_keys=False))
    (args.output / "meshes.cmake").write_text("set(XLEROBOT_MESHES\n" + "".join(f'  "{mesh}"\n' for mesh in meshes) + ")\n")


if __name__ == "__main__":
    main()
