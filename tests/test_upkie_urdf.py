"""Check MJCF-to-URDF rotations without ROS or a running simulator."""

import importlib.util
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "description/urdf/robots/upkie"
SPEC = importlib.util.spec_from_file_location("upkie_urdf_generator", MODEL / "generate_upkie_urdf.py")
GENERATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GENERATOR)


def quaternion_matrix(quaternion):
    w, x, y, z = np.asarray(quaternion) / np.linalg.norm(quaternion)
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - w*z), 2 * (x*z + w*y)],
        [2 * (x*y + w*z), 1 - 2 * (x*x + z*z), 2 * (y*z - w*x)],
        [2 * (x*z - w*y), 2 * (y*z + w*x), 1 - 2 * (x*x + y*y)],
    ])


def rpy_matrix(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp, cp*sr, cp*cr],
    ])


class UpkieUrdfTest(unittest.TestCase):
    def test_exact_and_near_gimbal_lock(self):
        for sign in (-1, 1):
            for offset in (0.0, -1.0e-7, 1.0e-7):
                pitch = sign * math.pi / 2 + offset
                cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
                for roll, yaw in ((0.7, -0.4), (-1.2, 2.1), (0.0, 0.0)):
                    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
                    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
                    q = [cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
                         cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy]
                    with self.subTest(sign=sign, offset=offset, roll=roll, yaw=yaw):
                        np.testing.assert_allclose(rpy_matrix(GENERATOR.quat_to_rpy(q)),
                                                   quaternion_matrix(q), atol=1.0e-8, rtol=0)

    def test_all_mjcf_quaternions_round_trip(self):
        for element in ET.parse(GENERATOR.MJCF).iter():
            if "quat" not in element.attrib:
                continue
            q = GENERATOR.floats(element.get("quat"), 4)
            with self.subTest(tag=element.tag, name=element.get("name", element.get("mesh"))):
                np.testing.assert_allclose(rpy_matrix(GENERATOR.quat_to_rpy(q)),
                                           quaternion_matrix(q), atol=1.0e-12, rtol=0)

    def test_generated_visual_collision_and_joint_poses_match(self):
        urdf = ET.parse(MODEL / "upkie.urdf")
        mjcf = ET.parse(GENERATOR.MJCF)
        for body in mjcf.findall(".//body"):
            link = urdf.find(f"link[@name='{body.get('name')}']")
            for kind, class_name in (("visual", "visual"), ("collision", "collision")):
                geoms = body.findall(f"geom[@class='{class_name}']")
                nodes = link.findall(kind)
                self.assertEqual(len(nodes), len(geoms))
                for geom, node in zip(geoms, nodes):
                    with self.subTest(body=body.get("name"), kind=kind, mesh=geom.get("mesh")):
                        self.assert_pose_matches(node.find("origin"), geom)
            for joint in body.findall("joint"):
                node = urdf.find(f"joint[@name='{joint.get('name')}']")
                self.assert_pose_matches(node.find("origin"), body)
        self.assert_pose_matches(urdf.find("joint[@name='imu_joint']/origin"),
                                 mjcf.find("worldbody/body/site[@name='imu']"))

    def assert_pose_matches(self, origin, source):
        np.testing.assert_allclose(np.fromstring(origin.get("xyz"), sep=" "),
                                   GENERATOR.floats(source.get("pos", "0 0 0"), 3), atol=1.0e-9, rtol=0)
        np.testing.assert_allclose(rpy_matrix(np.fromstring(origin.get("rpy"), sep=" ")),
                                   quaternion_matrix(GENERATOR.floats(source.get("quat", "1 0 0 0"), 4)),
                                   atol=1.0e-8, rtol=0)


if __name__ == "__main__":
    unittest.main()
