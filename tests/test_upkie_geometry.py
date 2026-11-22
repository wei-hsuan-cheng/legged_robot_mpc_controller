"""Check that every Upkie consumer takes its wheel and knee geometry from config/upkie/geometry.yaml, without ROS."""

import importlib.util
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
GEOMETRY_PATH = "$(find-pkg-share legged_robot_mpc_controller)/config/upkie/geometry.yaml"


def load_helper():
    spec = importlib.util.spec_from_file_location("upkie_geometry", ROOT / "config/upkie/upkie_geometry.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class UpkieGeometryTest(unittest.TestCase):

    def setUp(self):
        self.helper = load_helper()
        self.geometry = self.helper.load()
        self.config = yaml.safe_load((ROOT / "config/upkie/ros2_controllers.yaml").read_text())

    def test_matches_the_simulated_robot(self):
        self.helper.check_mjcf(self.geometry)
        mjcf = ET.parse(self.helper.MJCF).getroot()
        for side in ("left", "right"):
            tire = mjcf.find(f".//geom[@name='{side}_foot_collision']")
            # The tire geom sits on the wheel link's axle (z), at the geometry's tire centre.
            np.testing.assert_allclose([float(v) for v in tire.get("pos").split()],
                                       self.geometry["wheel"]["center_offset"], atol=1e-9)

    def test_estimator_and_mpc_read_the_file(self):
        kf = self.config["humanoid_linear_kf"]["ros__parameters"]
        self.assertEqual(kf["geometryFile"], GEOMETRY_PATH)
        self.assertNotIn("wheel", kf)
        mpc = self.config["humanoid_whole_body_inverse_dynamics_mpc_controller"]["ros__parameters"]
        self.assertEqual(mpc["paths"]["geometryFile"], GEOMETRY_PATH)
        # The controller refuses these next to the file.
        self.assertFalse({"radius", "axleAxis"} & set(mpc["ocs2"]["constraints"]["wheelContact"]))
        self.assertNotIn("frameTranslation", mpc["ocs2"]["model"]["contactGeometry"])

    def test_generated_files_were_made_for_this_geometry(self):
        wheel = self.geometry["wheel"]
        plan = yaml.safe_load((ROOT / "config/upkie/stand_up_trajectory.yaml").read_text())["stand_up"]
        self.assertEqual(plan["wheel_radius"], wheel["radius"])
        self.assertEqual(plan["wheel_half_width"], wheel["half_width"])
        self.assertEqual(plan["knee_contacts"], self.geometry["knee_housings"])
        table = yaml.safe_load((ROOT / "config/upkie/posture_table.yaml").read_text())["posture_table"]
        self.assertEqual(table["wheel_radius"], wheel["radius"])


if __name__ == "__main__":
    unittest.main()
