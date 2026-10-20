"""Compatibility fix for Isaac 6.2's ros2.control 0.1.6 URDF pruning.

That release starts a separate tree at each driven branch and removes fixed
ancestors connecting the arms/head to the chassis. Preserve the entire connected
component containing the controlled joints instead. This changes the exported
URDF only; NVIDIA's hardware plugin and physics control loop remain untouched.
"""


def prune_control_component(root, control_joint_names):
    joints = list(root.findall("joint"))
    edges = {}
    by_name = {}
    for joint in joints:
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        edges.setdefault(parent, set()).add(child)
        edges.setdefault(child, set()).add(parent)
        by_name[joint.get("name")] = (parent, child)
    controlled = {link for name in control_joint_names for link in by_name.get(name, ())}
    if not controlled:
        return
    reachable, pending = set(), [next(iter(controlled))]
    while pending:
        link = pending.pop()
        if link not in reachable:
            reachable.add(link)
            pending.extend(edges.get(link, ()))
    if not controlled <= reachable:
        raise RuntimeError("Controlled USD joints belong to disconnected URDF trees")
    for joint in joints:
        if by_name[joint.get("name")][0] not in reachable:
            root.remove(joint)
    for link in list(root.findall("link")):
        if link.get("name") not in reachable:
            root.remove(link)


def apply_isaac_620_compatibility():
    import omni.kit.app
    from isaacsim.ros2.control import urdf_synth

    manager = omni.kit.app.get_app().get_extension_manager()
    ext_id = manager.get_enabled_extension_id("isaacsim.ros2.control")
    version = manager.get_extension_dict(ext_id)["package"]["version"]
    if version == "0.1.6":
        urdf_synth._prune_to_control_tree = prune_control_component
        print("Applied ros2.control 0.1.6 fixed-ancestor URDF export correction", flush=True)
