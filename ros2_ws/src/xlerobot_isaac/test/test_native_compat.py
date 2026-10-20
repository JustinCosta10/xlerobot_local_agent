import xml.etree.ElementTree as ET

import pytest

from xlerobot_isaac.native_compat import prune_control_component


def robot(edges):
    root = ET.Element("robot")
    for name in sorted({link for _, parent, child in edges for link in (parent, child)}):
        ET.SubElement(root, "link", name=name)
    for name, parent, child in edges:
        joint = ET.SubElement(root, "joint", name=name)
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=child)
    return root


def test_preserves_fixed_ancestors_of_independent_driven_branches():
    root = robot([("wheel", "chassis", "tire"), ("mount", "chassis", "arm_base"),
                  ("shoulder", "arm_base", "arm"), ("head_mount", "chassis", "top"),
                  ("pan", "top", "head"), ("unused", "world", "extra")])
    prune_control_component(root, ["wheel", "shoulder", "pan"])
    assert {j.get("name") for j in root.findall("joint")} == {
        "wheel", "mount", "shoulder", "head_mount", "pan"}
    children = {j.find("child").get("link") for j in root.findall("joint")}
    assert {link.get("name") for link in root.findall("link")} - children == {"chassis"}


def test_disconnected_controlled_articulations_are_rejected():
    root = robot([("wheel", "chassis", "tire"), ("shoulder", "base", "arm")])
    with pytest.raises(RuntimeError, match="disconnected"):
        prune_control_component(root, ["wheel", "shoulder"])
