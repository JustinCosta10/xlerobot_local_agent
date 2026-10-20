"""Optional flat navigation scene and simulation-only localization measurements."""

import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation


def base_pose(chassis_pose, base_to_chassis):
    """Convert PhysX chassis [xyz, xyzw] to the URDF axle frame in world."""
    world_to_chassis = np.eye(4)
    world_to_chassis[:3, :3] = Rotation.from_quat(chassis_pose[3:]).as_matrix()
    world_to_chassis[:3, 3] = chassis_pose[:3]
    world_to_base = world_to_chassis @ np.linalg.inv(base_to_chassis)
    return world_to_base[:3, 3], Rotation.from_matrix(world_to_base[:3, :3]).as_quat()


def add_navigation_obstacles(stage):
    """A small room with visible obstacles ahead of the robot (world -X)."""
    from pxr import Gf, UsdGeom, UsdPhysics

    for name, xyz, size, color in (
        ("FrontWall", (-5.0, 0.0, 0.75), (0.15, 8.0, 1.5), (0.55, 0.6, 0.65)),
        ("BackWall", (2.0, 0.0, 0.75), (0.15, 8.0, 1.5), (0.55, 0.6, 0.65)),
        ("LeftWall", (-1.5, -4.0, 0.75), (7.0, 0.15, 1.5), (0.55, 0.6, 0.65)),
        ("RightWall", (-1.5, 4.0, 0.75), (7.0, 0.15, 1.5), (0.55, 0.6, 0.65)),
        ("CenterBox", (-2.4, 0.0, 0.45), (0.6, 0.6, 0.9), (0.8, 0.35, 0.12)),
        ("SideBox", (-3.7, -1.8, 0.3), (0.7, 0.7, 0.6), (0.2, 0.5, 0.8)),
    ):
        cube = UsdGeom.Cube.Define(stage, f"/World/Navigation/{name}")
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*xyz))
        cube.AddScaleOp().Set(Gf.Vec3d(*size))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())


class GroundTruthPublisher:
    """Publish measurements, leaving TF ownership to the ROS localization node."""

    def __init__(self, namespace, description):
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.context import Context

        self.context = Context()
        rclpy.init(args=[], context=self.context)
        self.node = rclpy.create_node("sim_ground_truth", namespace=namespace, context=self.context)
        self.publisher = self.node.create_publisher(PoseStamped, "sim_ground_truth", 10)
        origin = ET.parse(description).getroot().find("joint[@name='base_to_chassis']/origin")
        self.base_to_chassis = np.eye(4)
        self.base_to_chassis[:3, :3] = Rotation.from_euler(
            "xyz", [float(x) for x in origin.get("rpy").split()]).as_matrix()
        self.base_to_chassis[:3, 3] = [float(x) for x in origin.get("xyz").split()]
        self.previous_time = -1.0

    def publish(self, chassis_pose, sim_time):
        from geometry_msgs.msg import PoseStamped

        if sim_time - self.previous_time < 1.0 / 30.0:
            return
        self.previous_time = sim_time
        xyz, xyzw = base_pose(chassis_pose, self.base_to_chassis)
        message = PoseStamped()
        message.header.frame_id = "map"
        nanoseconds = round(sim_time * 1e9)
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(nanoseconds, 10**9)
        message.pose.position.x, message.pose.position.y, message.pose.position.z = map(float, xyz)
        (message.pose.orientation.x, message.pose.orientation.y,
         message.pose.orientation.z, message.pose.orientation.w) = map(float, xyzw)
        self.publisher.publish(message)

    def close(self):
        self.node.destroy_node()
        self.context.shutdown()
