"""Pineapple module composition, unchanged baseline settings and matching robot interfaces."""

import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]
DESCRIPTION = ROOT / "description"
MJCF = DESCRIPTION / "mujoco"
CONTROLLER = "humanoid_whole_body_inverse_dynamics_mpc_controller"
ROBOTS = ("pineapple", "pineapple_manipulator")
LEGS = ["L_hip_joint", "L_thigh_joint", "L_calf_joint", "L_wheel_joint",
        "R_hip_joint", "R_thigh_joint", "R_calf_joint", "R_wheel_joint"]


def config(robot):
    return yaml.safe_load((ROOT / "config" / robot / "ros2_controllers.yaml").read_text())


def numeric_parameters(value, prefix=""):
    """Ignore identifiers/paths while keeping every numeric and boolean tuning value."""
    if isinstance(value, dict):
        return {key: number for name, item in value.items()
                for key, number in numeric_parameters(item, prefix + "." + name).items()}
    if isinstance(value, list):
        return {key: number for index, item in enumerate(value)
                for key, number in numeric_parameters(item, prefix + f"[{index}]").items()}
    return {prefix: value} if isinstance(value, (int, float, bool)) else {}


class PineappleVariantsTest(unittest.TestCase):
    def test_baseline_numeric_parameters_are_identical(self):
        self.assertEqual(numeric_parameters(config(ROBOTS[0])), numeric_parameters(config(ROBOTS[1])))
        for robot in ROBOTS:
            p = config(robot)[CONTROLLER]["ros__parameters"]
            self.assertEqual(p["robot"]["jointNames"], LEGS)
            self.assertEqual(p["ocs2"]["model"]["robotName"], robot)
        p = config("pineapple")[CONTROLLER]["ros__parameters"]
        self.assertNotIn("fixedJointNames", p["ocs2"]["model"])
        self.assertTrue(p["paths"]["libFolder"].endswith("/chassis_only"))

    def test_stand_up_is_one_unchanged_shared_baseline(self):
        armless = ROOT / "config/pineapple/stand_up_trajectory.yaml"
        original = ROOT / "config/pineapple_manipulator/stand_up_trajectory.yaml"
        self.assertTrue(armless.is_symlink())
        self.assertEqual(armless.resolve(), original.resolve())
        a = yaml.safe_load((ROOT / "config/pineapple/posture_table.yaml").read_text())["posture_table"]
        b = yaml.safe_load((ROOT / "config/pineapple_manipulator/posture_table.yaml").read_text())["posture_table"]
        self.assertEqual(a, b)

    def test_urdf_and_spawn_dimensions(self):
        for robot, count in (("pineapple", 8), ("pineapple_manipulator", 14)):
            with self.subTest(robot=robot):
                urdf = ET.parse(DESCRIPTION / "urdf/robots" / robot / (robot + ".urdf"))
                joints = [j.get("name") for j in urdf.findall("joint") if j.get("type") != "fixed"]
                self.assertEqual(len(joints), count)
                pose = yaml.safe_load((ROOT / "config" / robot / "initial_pose.yaml").read_text())
                self.assertEqual(set(pose[robot + "_joints"]), set(joints))
                self.assertIn(robot + "_base", pose)
                for joint in LEGS:
                    self.assertIn(joint, joints)
                self.assertEqual(urdf.find("./link[@name='arm_base']") is not None,
                                 robot == "pineapple_manipulator")

    def test_urdf_generation_for_both_variants(self):
        generator = DESCRIPTION / "urdf/robots/pineapple/generate_pineapple_urdf.py"
        with tempfile.TemporaryDirectory() as directory:
            for robot in ROBOTS:
                output = Path(directory) / (robot + ".urdf")
                subprocess.run([sys.executable, str(generator), "--robot", robot, "--output", str(output)],
                               check=True, capture_output=True)
                self.assertEqual(output.read_bytes(), (DESCRIPTION / "urdf/robots" / robot / output.name).read_bytes())

    def test_shared_scene_and_composable_body_modules(self):
        for robot in ROBOTS:
            model = ET.parse(MJCF / "robots" / robot / (robot + ".xml"))
            modules = [e.get("file") for e in model.findall("./worldbody/body/include")]
            self.assertEqual(modules[0], "../../common/pineapple/body.xml")
            self.assertEqual(len(modules), 1 if robot == "pineapple" else 2)
            if robot == "pineapple_manipulator":
                self.assertEqual(modules[1], "../../common/pineapple_arm/body.xml")
            scene = ET.parse(MJCF / "robots" / robot / "scene.xml")
            self.assertEqual([e.get("file") for e in scene.findall("include")],
                             ["../../common/scenes/pineapple_scene.xml", robot + ".xml"])
            self.assertIsNone(scene.find("option"))
            self.assertIsNone(scene.find("worldbody"))

    @unittest.skipIf(importlib.util.find_spec("mujoco") is None, "Python MuJoCo is not installed")
    def test_compiled_physical_dimensions_and_mass(self):
        import mujoco
        for robot, nq, nv, nu, mass in (("pineapple", 15, 14, 8, 10.92),
                                        ("pineapple_manipulator", 23, 22, 14, 14.919995)):
            with self.subTest(robot=robot):
                model = mujoco.MjModel.from_xml_path(str(MJCF / "robots" / robot / "scene.xml"))
                self.assertEqual((model.nq, model.nv, model.nu), (nq, nv, nu))
                self.assertAlmostEqual(sum(model.body_mass), mass, places=6)
                self.assertEqual(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "arm_base") >= 0,
                                 robot == "pineapple_manipulator")


if __name__ == "__main__":
    unittest.main()
