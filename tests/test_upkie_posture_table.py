"""Check the Upkie posture table file and its controller configuration without running ROS."""

import subprocess
import sys
from pathlib import Path
import tempfile
import unittest

import numpy as np
import yaml

try:
    import mujoco
except ImportError:
    mujoco = None

ROOT = Path(__file__).resolve().parents[1]
TABLE = ROOT / "config/upkie/posture_table.yaml"


class UpkiePostureTableTest(unittest.TestCase):

    def setUp(self):
        config = yaml.safe_load((ROOT / "config/upkie/ros2_controllers.yaml").read_text())
        self.params = config["humanoid_whole_body_inverse_dynamics_mpc_controller"]["ros__parameters"]
        self.table = yaml.safe_load(TABLE.read_text())["posture_table"]

    def test_controller_loads_the_shipped_file(self):
        reference = self.params["ocs2"]["reference"]
        self.assertEqual(reference["postureTableFile"],
                         "$(find-pkg-share legged_robot_mpc_controller)/config/upkie/posture_table.yaml")
        # The controller rejects inline arrays next to the file.
        self.assertNotIn("postureHeights", reference)
        self.assertNotIn("postureJointStates", reference)

    def test_table_covers_the_commanded_and_sit_down_heights(self):
        heights = np.array(self.table["heights"])
        states = np.array(self.table["joint_states"])
        self.assertEqual(self.table["joint_names"], self.params["robot"]["jointNames"])
        self.assertEqual(states.shape, (heights.size, len(self.table["joint_names"])))
        self.assertTrue(np.all(np.diff(heights) > 0))
        self.assertTrue(np.all(np.isfinite(states)))
        reference = self.params["ocs2"]["reference"]
        for height in (reference["minBaseHeight"], reference["maxBaseHeight"], self.params["bringUp"]["sitDownHeight"]):
            self.assertGreaterEqual(height, heights[0])
            self.assertLessEqual(height, heights[-1])
        # Symmetric postures: right joints mirror the left ones, wheels at zero.
        np.testing.assert_array_equal(states[:, 3:5], -states[:, 0:2])
        np.testing.assert_array_equal(states[:, [2, 5]], 0.0)

    @unittest.skipIf(mujoco is None, "Python MuJoCo is not installed")
    def test_generator_reproduces_the_shipped_table(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / "posture_table.yaml"
            subprocess.run([sys.executable, str(ROOT / "config/upkie/compute_posture_table.py"), "--output", str(output)],
                           check=True, capture_output=True)
            self.assertEqual(yaml.safe_load(output.read_text())["posture_table"], self.table)


if __name__ == "__main__":
    unittest.main()
