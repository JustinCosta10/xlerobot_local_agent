#!/usr/bin/env python3
"""Generate the ROS description from the saved xlerobot_dd URDF.

The legacy MJCF converter remains available with an explicit --source.
The DD source owns geometry and kinematics; this adapter adds the ROS axle
frame and supplies conservative command limits for unspecified URDF limits.
"""

import argparse
import hashlib
from pathlib import Path
import warnings
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yaml


def numbers(value):
    return np.array([float(x) for x in value.split()])


def fmt(values):
    return " ".join(f"{float(x):.12g}" for x in values)


def rotation(attributes):
    if "euler" in attributes:
        return Rotation.from_euler("XYZ", numbers(attributes["euler"]))
    if "quat" in attributes:
        w, x, y, z = numbers(attributes["quat"])
        return Rotation.from_quat([x, y, z, w])
    if "zaxis" in attributes:
        target = numbers(attributes["zaxis"])
        target /= np.linalg.norm(target)
        cross = np.cross([0., 0., 1.], target)
        if np.linalg.norm(cross) < 1e-12:
            return Rotation.identity() if target[2] > 0 else Rotation.from_rotvec([np.pi, 0, 0])
        return Rotation.from_rotvec(cross / np.linalg.norm(cross) * np.arccos(target[2]))
    if any(key in attributes for key in ("axisangle", "xyaxes")):
        raise ValueError("Unsupported orientation representation")
    return Rotation.identity()


def origin(parent, attributes):
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", message="Gimbal lock detected")
        rpy = rotation(attributes).as_euler("xyz")
    ET.SubElement(parent, "origin", xyz=attributes.get("pos", "0 0 0"), rpy=fmt(rpy))


def generate_mjcf(source):
    mjcf = ET.parse(source).getroot()
    compiler = mjcf.find("compiler")
    if compiler.get("angle") != "radian" or compiler.get("eulerseq", "xyz") != "xyz":
        raise ValueError("Expected MJCF radians and intrinsic xyz Euler convention")
    defaults = {}

    def collect(element, inherited):
        values = {tag: dict(attrs) for tag, attrs in inherited.items()}
        for child in element:
            if child.tag != "default":
                values.setdefault(child.tag, {}).update(child.attrib)
        defaults[element.get("class", "main")] = values
        for child in element.findall("default"):
            collect(child, values)

    collect(mjcf.find("default"), {})
    meshes = {m.get("name"): m.attrib for m in mjcf.findall("asset/mesh")}
    materials = {m.get("name"): m.get("rgba") for m in mjcf.findall("asset/material")}
    robot = ET.Element("robot", name="xlerobot")
    robot.append(ET.Comment("Generated from xlerobot.xml; edit the source and regenerate."))
    ET.SubElement(robot, "link", name="base_link")
    joints = {}
    used_meshes = set()

    def body(element, parent):
        name = element.get("name")
        link = ET.SubElement(robot, "link", name=name)
        body_joints = element.findall("joint")
        if len(body_joints) > 1:
            raise ValueError(f"Multiple joints on {name} are not supported")
        joint_xml = body_joints[0] if body_joints else None
        joint_name = joint_xml.get("name") if joint_xml is not None else name + "_fixed_joint"
        joint_type = "fixed"
        if joint_xml is not None:
            if joint_xml.get("type", "hinge") != "hinge" or np.any(numbers(joint_xml.get("pos", "0 0 0"))):
                raise ValueError(f"Unsupported joint: {joint_name}")
            joint_type = "revolute" if "range" in joint_xml.attrib else "continuous"
        joint = ET.SubElement(robot, "joint", name=joint_name, type=joint_type)
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=name)
        attributes = element.attrib
        if parent == "base_link":
            # Diff-drive odometry describes the axle midpoint. The MJCF root
            # position is a world spawn pose, not a robot-relative transform.
            wheels = [element.find(f"body[@name='{side}_wheel']") for side in ("left", "right")]
            axle = np.mean([numbers(wheel.get("pos")) for wheel in wheels], axis=0)
            attributes = {**attributes, "pos": fmt(-axle)}
        origin(joint, attributes)
        if joint_xml is not None:
            ET.SubElement(joint, "axis", xyz=joint_xml.get("axis", "0 0 1"))
            wheel = joint_type == "continuous"
            limit = {"effort": "2.0" if wheel else "3.35", "velocity": "6.0" if wheel else "1.0"}
            info = {"command_interface": "velocity" if wheel else "position", "velocity": float(limit["velocity"])}
            if not wheel:
                lower, upper = numbers(joint_xml.get("range"))
                limit.update(lower=str(lower), upper=str(upper))
                info.update(lower=float(lower), upper=float(upper))
            for actuator in mjcf.findall("actuator/position"):
                if actuator.get("joint") == joint_name:
                    attrs = {**defaults[actuator.get("class", "main")].get("position", {}), **actuator.attrib}
                    limit["effort"] = str(max(abs(numbers(attrs.get("forcerange", "-2.94 2.94")))))
            ET.SubElement(joint, "limit", **limit)
            joints[joint_name] = info
        inertia = element.find("inertial")
        if inertia is not None:
            inertial = ET.SubElement(link, "inertial")
            origin(inertial, inertia.attrib)
            ET.SubElement(inertial, "mass", value=inertia.get("mass"))
            values = numbers(inertia.get("fullinertia", inertia.get("diaginertia", "0 0 0")))
            if len(values) == 3:
                values = np.r_[values, 0., 0., 0.]
            ET.SubElement(inertial, "inertia", **dict(zip(
                ("ixx", "iyy", "izz", "ixy", "ixz", "iyz"), map(str, values))))
        for i, geom in enumerate(element.findall("geom")):
            cls = geom.get("class", "main")
            attrs = {**defaults[cls].get("geom", {}), **geom.attrib}
            collision = attrs.get("contype", "0") != "0"
            for tag in (["collision"] if cls == "collision" else ["visual", "collision"] if collision else ["visual"]):
                item = ET.SubElement(link, tag, name=f"{name}_{tag}_{i}")
                origin(item, attrs)
                geometry = ET.SubElement(item, "geometry")
                kind = attrs.get("type", "sphere")
                if kind == "mesh":
                    mesh = meshes[attrs["mesh"]]
                    used_meshes.add(mesh["file"])
                    ET.SubElement(geometry, "mesh", filename="package://xlerobot_description/meshes/" + mesh["file"], scale=mesh.get("scale", "1 1 1"))
                elif kind == "box":
                    ET.SubElement(geometry, "box", size=fmt(numbers(attrs["size"]) * 2))
                elif kind == "cylinder":
                    radius, half_length = numbers(attrs["size"])
                    ET.SubElement(geometry, "cylinder", radius=str(radius), length=str(half_length * 2))
                elif kind == "sphere":
                    ET.SubElement(geometry, "sphere", radius=attrs["size"].split()[0])
                else:
                    raise ValueError(f"Unsupported geom type: {kind}")
                if tag == "visual":
                    material = ET.SubElement(item, "material", name=f"{name}_material_{i}")
                    ET.SubElement(material, "color", rgba=attrs.get("rgba", materials.get(attrs.get("material"), "0.6 0.6 0.6 1")))
        for child in element.findall("body"):
            body(child, name)

    body(mjcf.find("worldbody/body"), "base_link")
    ET.indent(robot)
    return ET.tostring(robot, encoding="unicode") + "\n", joints, sorted(used_meshes)


def generate(source):
    robot = ET.parse(source).getroot()
    if robot.tag == "mujoco":
        return generate_mjcf(source)
    if robot.tag != "robot":
        raise ValueError("Expected an XLeRobot URDF or MJCF")
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
