"""Pineapple module composition, unchanged baseline settings and matching robot interfaces."""

import importlib.util
import hashlib
import math
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

    def test_stand_up_is_a_validated_arm_free_plan(self):
        armless = ROOT / "config/pineapple/stand_up_trajectory.yaml"
        original = ROOT / "config/pineapple_manipulator/stand_up_trajectory.yaml"
        self.assertFalse(armless.is_symlink())
        self.assertNotEqual(armless.read_bytes(), original.read_bytes())
        self.assertEqual(hashlib.sha256(original.read_bytes()).hexdigest(),
                         "65008be561ffbd77acb0e0b348a7dc290841cc96fc52c10ae3746718290c1136")
        data = yaml.safe_load(armless.read_text())
        generation = data["generation"]
        self.assertEqual(generation["robot"], "pineapple")
        self.assertAlmostEqual(generation["model_mass_kg"], 10.92, places=6)
        urdf = DESCRIPTION / "urdf/robots/pineapple/pineapple.urdf"
        self.assertEqual(generation["urdf_sha256"], hashlib.sha256(urdf.read_bytes()).hexdigest())
        for key in ("max_constraint_violation", "terminal_dynamics_error", "contact_error", "initial_reference_error"):
            self.assertTrue(math.isfinite(generation[key]))
            self.assertLessEqual(generation[key], 1e-5)
        self.assertEqual(len(generation["phase_durations"]), 2)
        self.assertTrue(generation["symmetric"])
        self.assertEqual(generation["max_yaw_change"], 0)
        self.assertEqual(generation["contact_regularization"], 1e-3)
        geometry = yaml.safe_load((ROOT / "config/pineapple/geometry.yaml").read_text())["geometry"]
        self.assertEqual(data["stand_up"]["knee_contacts"], geometry["knee_housings"])
        for key in ("stand_up", "sit_down"):
            plan = data[key]
            self.assertEqual(plan["joint_names"], LEGS)
            self.assertEqual(plan["wheel_radius"], geometry["wheel"]["radius"])
            self.assertTrue(all(b > a for a, b in zip(plan["time"], plan["time"][1:])))
            self.assertAlmostEqual(plan["time"][-1], plan["handover_time"])
            for field, width in (("position", 8), ("velocity", 8), ("effort", 8),
                                 ("state", 28), ("feedback", 224)):
                self.assertEqual(len(plan[field]), len(plan["time"]))
                self.assertTrue(all(len(row) == width and all(math.isfinite(x) for x in row)
                                    for row in plan[field]))
        self.assertLess(data["stand_up"]["mpc_start_time"], data["stand_up"]["handover_time"])
        self.assertAlmostEqual(data["sit_down"]["state"][0][2], 0.3, places=6)

    def test_arm_free_seed_matches_the_export_and_preserves_full_dynamics_checks(self):
        import json
        seed = json.loads((ROOT / "config/pineapple/stand_up_seed.json").read_text())
        generation = yaml.safe_load((ROOT / "config/pineapple/stand_up_trajectory.yaml").read_text())["generation"]
        self.assertEqual(seed["return_status"], "Solve_Succeeded")
        for key in ("robot", "model_mass_kg", "max_constraint_violation", "terminal_dynamics_error", "contact_error"):
            self.assertEqual(seed[key], generation[key])
        self.assertEqual(seed["duration"], generation["phase_durations"])
        self.assertEqual(len(seed["initial_pose_tolerance"]), 14)
        for field in ("x", "v", "a"):
            rows = seed[field]
            self.assertEqual(len(rows), 14)
            self.assertTrue(all(len(row) == 61 for row in rows))
            self.assertTrue(all(abs(value) < 1e-9 for i in (5, 6, 10) for value in rows[i]))
            for left, right in ((7, 11), (8, 12), (9, 13)):
                self.assertTrue(all(abs(a - b) < 1e-9 for a, b in zip(rows[left], rows[right])))
        self.assertTrue(all(abs(yaw - seed["x"][3][0]) < 1e-9 for yaw in seed["x"][3]))
        for index, row in enumerate(seed["torque"]):
            self.assertTrue(all(abs(value) <= [19, 19, 28, 7.7][index % 4] + 1e-5 for value in row))
        urdf = ET.parse(DESCRIPTION / "urdf/robots/pineapple/pineapple.urdf")
        for index, name in enumerate(LEGS, 6):
            limit = urdf.find(f"./joint[@name='{name}']/limit")
            lo, hi, speed = (float(limit.get(key)) for key in ("lower", "upper", "velocity"))
            self.assertTrue(all(lo - 1e-6 <= value <= hi + 1e-6 for value in seed["x"][index]))
            self.assertTrue(all(abs(value) <= speed + 1e-6 for value in seed["v"][index]))

    def test_locomotion_posture_table_remains_the_original_baseline(self):
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
    def test_stand_up_helpers_select_the_matching_physical_model(self):
        path = ROOT / "config/pineapple_manipulator/stand_up_to/rollout.py"
        spec = importlib.util.spec_from_file_location("pineapple_rollout", path)
        rollout = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(rollout)
        for robot, mass, arms in (("pineapple", 10.92, 0), ("pineapple_manipulator", 14.919995, 6)):
            with self.subTest(robot=robot):
                rollout.configure(robot)
                model = rollout.Rollout()
                self.assertAlmostEqual(model.m.body_subtreemass[1], mass, places=6)
                self.assertEqual(len(model.arm), arms)
                self.assertEqual(len(rollout.FRAMES), 7 if arms == 0 else 9)
                self.assertEqual(rollout.GEOMETRY.parent.name, robot)
                model.spawn_idle(settle=0.01)
                self.assertEqual(model.state().shape, (28,))
        with self.assertRaises(ValueError):
            rollout.configure("pineapple_arm")

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
