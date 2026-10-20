from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from xlerobot_isaac.navigation_scene import base_pose


def test_axle_pose_uses_shared_urdf_and_physics_orientation():
    description = Path(__file__).parents[2] / "xlerobot_description/urdf/kinematics.urdf"
    origin = ET.parse(description).getroot().find("joint[@name='base_to_chassis']/origin")
    b_c = np.eye(4)
    b_c[:3, :3] = Rotation.from_euler("xyz", list(map(float, origin.get("rpy").split()))).as_matrix()
    b_c[:3, 3] = list(map(float, origin.get("xyz").split()))
    xyz, q = base_pose([0, 0, 0.38, 0, 0, 0, 1], b_c)
    np.testing.assert_allclose(xyz, [-0.15, 0, 0.0635], atol=1e-6)
    np.testing.assert_allclose(Rotation.from_quat(q).apply([1, 0, 0]), [-1, 0, 0], atol=1e-6)
    chassis_rotation = Rotation.from_euler("z", np.pi / 2)
    xyz, q = base_pose([2, 3, 0.38, *chassis_rotation.as_quat()], b_c)
    np.testing.assert_allclose(xyz, [2, 2.85, 0.0635], atol=1e-6)
    np.testing.assert_allclose(Rotation.from_quat(q).apply([1, 0, 0]), [0, -1, 0], atol=1e-6)
