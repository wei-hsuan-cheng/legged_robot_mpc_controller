#!/usr/bin/env python3
"""Reference-math and source-wiring checks; no compiled C++ cost or simulator is run."""

import math
import unittest

import numpy as np

from wb_id_cost_activation_contract_test import BUILDER, INTERFACE, PARAMS, ROOT, SCHEMA, block


SOURCE = (ROOT / "src/core/humanoid_wb_id_mpc/src/cost/WbIdTorsoTiltCost.cpp").read_text()


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * cross + (1 - math.cos(angle)) * cross @ cross


def residual(orientation, omega, free_angle=math.radians(5), up=None):
    """Independent reference for the implemented four-component residual."""
    up = np.array([0., 0., 1.]) if up is None else np.asarray(up)
    cosine = float(up @ orientation[:, 2])
    safe_cosine = max(-1 + 1e-9, min(cosine, math.cos(free_angle)))
    tilt_excess = math.acos(safe_cosine) - free_angle
    omega = np.asarray(omega)
    return np.r_[math.sqrt(10) * tilt_excess, math.sqrt(5) * (omega - up * (up @ omega))]


def cost(*args, **kwargs):
    r = residual(*args, **kwargs)
    return 0.5 * float(r @ r)


class TorsoTiltCostTest(unittest.TestCase):
    def test_upright_and_free_lean_have_zero_position_penalty(self):
        for axis in ([1, 0, 0], [0, 1, 0], [1, 1, 0]):
            for angle in (-5, -2, 0, 2, 5):
                self.assertAlmostEqual(cost(rotation(axis, math.radians(angle)), [0, 0, 0]), 0, places=12)

    def test_tilt_excess_has_the_documented_weight(self):
        expected = 0.5 * 10 * math.radians(10)**2
        for axis in ([1, 0, 0], [0, 1, 0]):
            self.assertAlmostEqual(cost(rotation(axis, math.radians(15)), [0, 0, 0]), expected)

    def test_yaw_angle_and_world_vertical_spin_are_unpenalized(self):
        for yaw in (-math.pi, -0.7, 0, 1.5, math.pi):
            self.assertAlmostEqual(cost(rotation([0, 0, 1], yaw), [0, 0, 3]), 0, places=12)

    def test_common_yaw_gauge_change_leaves_cost_unchanged(self):
        orientation = rotation([1, 2, 0], 0.4)
        omega = np.array([0.1, -0.2, 0.7])
        for yaw in (-2, -0.5, 0, 1, 2):
            change = rotation([0, 0, 1], yaw)
            self.assertAlmostEqual(cost(change @ orientation, change @ omega), cost(orientation, omega))

    def test_damping_only_penalizes_velocity_perpendicular_to_gravity(self):
        self.assertAlmostEqual(cost(np.eye(3), [1, 2, 7]), 0.5 * 5 * (1 + 4))
        change = rotation([1, 2, 3], 0.6)
        up = change @ np.array([0., 0., 1.])
        self.assertAlmostEqual(cost(change, change @ np.array([1, 2, 7]), up=up), 0.5 * 5 * (1 + 4))

    def test_bending_remains_finite_and_widening_corridor_removes_penalty(self):
        bend = rotation([0, 1, 0], math.radians(60))
        self.assertTrue(math.isfinite(cost(bend, [0, 0, 0])))
        self.assertGreater(cost(bend, [0, 0, 0]), 0)
        self.assertAlmostEqual(cost(bend, [0, 0, 0], free_angle=math.radians(70)), 0, places=12)

    def test_clamp_and_upright_numerical_jacobian_remain_finite(self):
        for cosine in (-1 - 1e-12, -1, 1, 1 + 1e-12):
            orientation = np.eye(3)
            orientation[2, 2] = cosine
            self.assertTrue(np.all(np.isfinite(residual(orientation, [0, 0, 0]))))
        epsilon = 1e-7
        jacobian = (residual(rotation([1, 0, 0], epsilon), [0, 0, 0]) -
                    residual(rotation([1, 0, 0], -epsilon), [0, 0, 0])) / (2 * epsilon)
        np.testing.assert_allclose(jacobian, 0)

    def test_configuration_and_constructor_are_wired(self):
        costs = PARAMS["ocs2"]["costs"]
        tilt = costs["torsoTiltCost"]
        self.assertIs(tilt["activate"], True)
        self.assertEqual(tilt["linkName"], "torso_link")
        self.assertIs(costs["taskSpaceCosts"]["torso"]["activate"], False)
        self.assertIs(SCHEMA["ocs2"]["costs"]["torsoTiltCost"]["activate"]["default_value"], False)
        for key in tilt:
            self.assertIn(f"config.torsoTiltCost.{key} = p.costs.torsoTiltCost.{key};", BUILDER)
        guard = block(INTERFACE, "if (config_.torsoTiltCost.activate)")
        self.assertIn('costPtr->add("torsoTiltCost"', guard)
        self.assertIn("std::make_unique<WbIdTorsoTiltCost>", guard)
        self.assertNotIn("ConstraintPtr", guard)

    def test_cpp_residual_has_no_pose_reference_or_yaw_target(self):
        self.assertNotIn("getDesiredState", SOURCE)
        self.assertNotIn("getDesiredInput", SOURCE)
        self.assertNotIn(".translation()", SOURCE)
        self.assertIn("pinocchioInterface.getModel().gravity.linear()", SOURCE)
        self.assertIn("up.dot(placement.rotation().col(2))", SOURCE)
        self.assertIn("angularVelocity - up * up.dot(angularVelocity)", SOURCE)
        self.assertIn("CppAD::CondExpLt", SOURCE)
        self.assertIn("CppAD::CondExpGt", SOURCE)
        self.assertIn("return parameters_;", SOURCE)
        self.assertIn('"wb_id_torso_tilt_v1_" + config.linkName', SOURCE)


if __name__ == "__main__":
    unittest.main()
