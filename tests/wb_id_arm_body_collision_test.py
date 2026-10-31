#!/usr/bin/env python3
"""Offline parameter/frame and collision-envelope regression tests."""

from pathlib import Path
import sys
import unittest

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent / "tuning"))
from analyze_arm_body_collision import CONTROLLER, ROOT, SphereReplay


class ArmBodyCollisionTest(unittest.TestCase):
    def load(self, robot):
        params = yaml.safe_load((ROOT / f"config/{robot}/ros2_controllers.yaml").read_text())[CONTROLLER]["ros__parameters"]
        return params, SphereReplay(robot, params)

    def test_nominal_posture_has_positive_margin(self):
        for robot in ("g1", "h2"):
            with self.subTest(robot=robot):
                _, replay = self.load(robot)
                self.assertGreater(replay.evaluate(replay.nominal)[0], replay.config["minimumDistance"])

    def test_pairs_are_arm_versus_body_only(self):
        for robot in ("g1", "h2"):
            _, replay = self.load(robot)
            names = [s["name"] for s in replay.arms + replay.bodies]
            self.assertEqual(len(names), len(set(names)))
            for s in replay.arms:
                self.assertTrue(any(x in s["linkName"] for x in ("shoulder", "elbow", "wrist", "hand")))
            for s in replay.bodies:
                self.assertIn(s["linkName"], ("torso_link", "pelvis", "head_link", "head_pitch_link", "head_yaw_link"))

    def test_inward_arm_target_is_detected(self):
        for robot in ("g1", "h2"):
            _, replay = self.load(robot)
            target = replay.nominal | {"left_shoulder_roll_joint": -0.4}
            self.assertLess(replay.evaluate(target)[0], 0)

    def test_configured_spheres_are_finite_and_positive(self):
        for robot in ("g1", "h2"):
            _, replay = self.load(robot)
            for sphere in replay.arms + replay.bodies:
                self.assertEqual(len(sphere["center"]), 3)
                self.assertTrue(np.isfinite(sphere["center"]).all())
                self.assertGreater(sphere["radius"], 0)
            self.assertGreater(replay.config["mu"], 0)
            self.assertGreater(replay.config["delta"], 0)
            self.assertIn(replay.config["implementation"], ("rbf", "polynomial"))

    def test_centroidal_does_not_enable_new_term(self):
        config = yaml.safe_load((ROOT / "config/g1/ros2_controllers.yaml").read_text())
        centroidal = config["humanoid_centroidal_mpc_controller"]["ros__parameters"]
        self.assertNotIn("armSelfCollision", centroidal["ocs2"]["constraints"])


if __name__ == "__main__":
    unittest.main()
