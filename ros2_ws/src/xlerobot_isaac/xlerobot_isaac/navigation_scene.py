"""Optional flat navigation scene and simulation-only localization measurements."""

import xml.etree.ElementTree as ET
from pathlib import Path

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

    add_navigation_textures(stage)


def add_navigation_textures(stage):
    """Paint the room's inner surfaces with locally packaged, matte textures.

    Visual quads sit 0.5 mm above the existing collision surfaces. The ground
    and wall colliders retain their dimensions, contact materials and friction.
    UVs use physical dimensions so bricks and aggregate keep a consistent scale.
    """
    from ament_index_python.packages import get_package_share_directory
    from pxr import Gf, Sdf, UsdGeom, UsdShade

    textures = Path(get_package_share_directory("xlerobot_isaac")) / "textures"

    def material(name, filename, roughness):
        texture_path = textures / filename
        if not texture_path.is_file():
            raise RuntimeError(f"Missing navigation texture: {texture_path}; rebuild xlerobot_isaac")
        path = f"/World/Navigation/Looks/{name}"
        result = UsdShade.Material.Define(stage, path)
        surface = UsdShade.Shader.Define(stage, path + "/Surface")
        surface.CreateIdAttr("UsdPreviewSurface")
        surface.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
        surface.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        result.CreateSurfaceOutput().ConnectToSource(surface.ConnectableAPI(), "surface")
        uv = UsdShade.Shader.Define(stage, path + "/UV")
        uv.CreateIdAttr("UsdPrimvarReader_float2")
        uv.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        image = UsdShade.Shader.Define(stage, path + "/Albedo")
        image.CreateIdAttr("UsdUVTexture")
        image.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(texture_path)))
        image.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
        for axis in ("S", "T"):
            image.CreateInput("wrap" + axis, Sdf.ValueTypeNames.Token).Set("repeat")
        image.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(uv.ConnectableAPI(), "result")
        surface.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
            image.ConnectableAPI(), "rgb")
        return result

    wall = material("PaintedBrick", "painted_brick_albedo.png", 0.9)
    floor = material("WornConcrete", "worn_concrete_albedo.png", 0.95)

    def quad(name, origin, horizontal, vertical, tile_meters, surface, offset=(0.0, 0.0)):
        origin, horizontal, vertical = map(np.asarray, (origin, horizontal, vertical))
        points = [origin, origin + horizontal, origin + horizontal + vertical, origin + vertical]
        mesh = UsdGeom.Mesh.Define(stage, f"/World/Navigation/Surfaces/{name}")
        mesh.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in points])
        mesh.CreateFaceVertexCountsAttr([4])
        mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        mesh.CreateDoubleSidedAttr(True)
        u, v = np.linalg.norm(horizontal) / tile_meters, np.linalg.norm(vertical) / tile_meters
        s, t = offset
        UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
        ).Set([(s, t), (s + u, t), (s + u, t + v), (s, t + v)])
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(surface)

    quad("Floor", (-10, -10, 0.0005), (20, 0, 0), (0, 20, 0), 2.0, floor)
    # Different phase offsets reduce identical views on opposing walls.
    quad("FrontWall", (-4.9245, 4, 0), (0, -8, 0), (0, 0, 1.5), 0.9, wall)
    quad("BackWall", (1.9245, -4, 0), (0, 8, 0), (0, 0, 1.5), 0.9, wall, (0.37, 0.0))
    quad("LeftWall", (-5, -3.9245, 0), (7, 0, 0), (0, 0, 1.5), 0.9, wall, (0.61, 0.0))
    quad("RightWall", (2, 3.9245, 0), (-7, 0, 0), (0, 0, 1.5), 0.9, wall, (0.13, 0.0))


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
        # cuVSLAM's map has its own origin; simulator coordinates are independent.
        message.header.frame_id = "sim_world"
        nanoseconds = round(sim_time * 1e9)
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(nanoseconds, 10**9)
        message.pose.position.x, message.pose.position.y, message.pose.position.z = map(float, xyz)
        (message.pose.orientation.x, message.pose.orientation.y,
         message.pose.orientation.z, message.pose.orientation.w) = map(float, xyzw)
        self.publisher.publish(message)

    def close(self):
        self.node.destroy_node()
        self.context.shutdown()
