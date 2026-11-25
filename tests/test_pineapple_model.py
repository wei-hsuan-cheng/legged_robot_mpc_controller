"""Pineapple model files: the URDF is the generator's output, and the MPC configuration agrees with the model."""

import importlib.util
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]
URDF_DIR = ROOT / "description/urdf/robots/pineapple"
CONFIG = ROOT / "config/pineapple"
CONTROLLER = "humanoid_whole_body_inverse_dynamics_mpc_controller"
LEGS = ["L_hip_joint", "L_thigh_joint", "L_calf_joint", "L_wheel_joint",
        "R_hip_joint", "R_thigh_joint", "R_calf_joint", "R_wheel_joint"]
ARM = ["arm_joint", "arm_base_joint", "upper_arm_joint", "fore_arm_joint", "5dof_joint", "gripper_case_joint"]


def params():
    return yaml.safe_load((CONFIG / "ros2_controllers.yaml").read_text())[CONTROLLER]["ros__parameters"]


class PineappleModelTest(unittest.TestCase):
    def test_urdf_is_generated_from_the_mjcf(self):
        with tempfile.TemporaryDirectory() as directory:
            script = Path(directory) / "urdf/robots/pineapple/generate_pineapple_urdf.py"
            script.parent.mkdir(parents=True)
            shutil.copy(URDF_DIR / "generate_pineapple_urdf.py", script)
            mujoco = Path(directory) / "mujoco/robots/pineapple"
            mujoco.mkdir(parents=True)
            shutil.copy(ROOT / "description/mujoco/robots/pineapple/pineapple.xml", mujoco)
            subprocess.run([sys.executable, str(script)], check=True, capture_output=True)
            self.assertEqual((script.parent / "pineapple.urdf").read_text(), (URDF_DIR / "pineapple.urdf").read_text())

    def test_urdf_joints(self):
        robot = ET.parse(URDF_DIR / "pineapple.urdf").getroot()
        movable = [j.get("name") for j in robot.findall("joint") if j.get("type") != "fixed"]
        self.assertEqual(sorted(movable), sorted(LEGS + ARM))
        p = params()
        self.assertEqual(p["robot"]["jointNames"], LEGS)
        self.assertEqual(p["ocs2"]["model"]["fixedJointNames"], ARM)
        self.assertEqual(p["ocs2"]["model"]["contactParentJointNames"], ["L_wheel_joint", "R_wheel_joint"])

    def test_posture_table_and_spawn_pose(self):
        p = params()
        reference = p["ocs2"]["reference"]
        table = yaml.safe_load((CONFIG / "posture_table.yaml").read_text())["posture_table"]
        geometry = yaml.safe_load((CONFIG / "geometry.yaml").read_text())["geometry"]
        self.assertEqual(table["joint_names"], LEGS)
        self.assertEqual(table["wheel_radius"], geometry["wheel"]["radius"])
        self.assertLessEqual(table["heights"][0], reference["minBaseHeight"])
        self.assertGreaterEqual(table["heights"][-1], reference["maxBaseHeight"])
        default = table["joint_states"][table["heights"].index(reference["defaultBaseHeight"])]
        self.assertEqual(default, reference["defaultJointState"])
        pose = yaml.safe_load((CONFIG / "initial_pose.yaml").read_text())
        self.assertEqual([pose["pineapple_joints"][n] for n in LEGS], default)
        self.assertEqual(sorted(pose["pineapple_joints"]), sorted(LEGS + ARM))
        for row in table["joint_states"]:
            self.assertEqual(row[:4], row[4:])  # same axes on both sides: right equals left

    @unittest.skipIf(importlib.util.find_spec("mujoco") is None, "Python MuJoCo is not installed")
    def test_posture_table_reproduces(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "table.yaml"
            subprocess.run([sys.executable, str(CONFIG / "compute_posture_table.py"), "--output", str(output)],
                           check=True, capture_output=True)
            self.assertEqual(output.read_text(), (CONFIG / "posture_table.yaml").read_text())


if __name__ == "__main__":
    unittest.main()
