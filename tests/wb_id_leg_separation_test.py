"""Reference-math and wiring checks; simulator tests exercise compiled AD code."""

import math
from pathlib import Path
import unittest

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
NAME = "humanoid_whole_body_inverse_dynamics_mpc_controller"


def margin(left, right, yaw, minimum=.1):
    return np.array([-math.sin(yaw), math.cos(yaw), 0.]) @ (np.asarray(left)-right) - minimum


class LegSeparationTest(unittest.TestCase):
    def test_nominal_width_and_crossing(self):
        self.assertAlmostEqual(margin([0, .14, 0], [0, -.14, 0], 0), .18)
        self.assertLess(margin([0, -.05, 0], [0, .05, 0], 0), 0)

    def test_lifting_or_moving_forward_does_not_hide_crossing(self):
        self.assertAlmostEqual(margin([1, -.05, .5], [0, .05, 0], 0), -.2)

    def test_translation_and_common_heading_changes_do_not_change_margin(self):
        left, right = np.array([.2, .15, .1]), np.array([-.1, -.14, 0])
        expected = margin(left, right, .4)
        for angle in (-2., -.5, 0., 1., 2.):
            c, s = math.cos(angle), math.sin(angle)
            rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            shift = np.array([3, -7, 2])
            self.assertAlmostEqual(margin(rotation@left+shift, rotation@right+shift, .4+angle), expected)

    def test_gradient_is_finite_at_touching_and_corrects_lateral_order(self):
        epsilon = 1e-6
        derivative = (margin([0, .1+epsilon, .2], [0, 0, 0], 0) -
                      margin([0, .1-epsilon, .2], [0, 0, 0], 0)) / (2*epsilon)
        self.assertAlmostEqual(derivative, 1.)

    def test_opt_in_schema_builder_and_separate_ad_cache(self):
        schema = yaml.safe_load((ROOT / f"src/humanoid_whole_body_inverse_dynamics_mpc/{NAME}_parameter.yaml").read_text())[NAME]
        self.assertEqual(schema['ocs2']['constraints']['legSelfCollision']['minimumLateralSeparation']['default_value'], 0.)
        h2 = yaml.safe_load((ROOT / 'config/h2/ros2_controllers.yaml').read_text())[NAME]['ros__parameters']
        self.assertEqual(h2['ocs2']['constraints']['legSelfCollision']['minimumLateralSeparation'], .1)
        g1 = yaml.safe_load((ROOT / 'config/g1/ros2_controllers.yaml').read_text())[NAME]['ros__parameters']
        self.assertEqual(g1['ocs2']['constraints']['legSelfCollision']['minimumLateralSeparation'], .09)
        builder = (ROOT / 'src/humanoid_whole_body_inverse_dynamics_mpc/whole_body_inverse_dynamics_mpc_config_builder.cpp').read_text()
        self.assertIn('minimumLateralSeparation = p.constraints.legSelfCollision.minimumLateralSeparation', builder)
        source = (ROOT / 'src/core/humanoid_common_mpc/src/constraint/FootCollisionConstraint.cpp').read_text()
        self.assertIn('costName += "_lateral_v1"', source)
        self.assertIn('lateralSeparation ? 17 : 16', source)
        self.assertIn('if (cfg_.minimumLateralSeparation > 0.0) return true;', source)
        self.assertIn('parameters[3] * constraintValues.head(16)', source)


if __name__ == '__main__':
    unittest.main()
