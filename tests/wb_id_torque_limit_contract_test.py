#!/usr/bin/env python3
"""URDF/configuration and source-wiring checks; these do not execute the C++ OCP."""

from copy import deepcopy
import math
import unittest
import xml.etree.ElementTree as ET

from wb_id_cost_activation_contract_test import BUILDER, INTERFACE, PARAMS, ROOT, SCHEMA, block
from wb_id_parameter_contract_test import NAME, check_keys


class TorqueLimitContractTest(unittest.TestCase):
    def test_urdf_is_the_default_without_a_duplicated_limit_array(self):
        # Explicit empty YAML arrays become unset ROS overrides in Jazzy; omit the key instead.
        self.assertNotIn("limits", PARAMS["ocs2"]["constraints"]["jointTorqueLimits"])
        self.assertEqual(SCHEMA["ocs2"]["constraints"]["jointTorqueLimits"]["limits"]["default_value"], [])
        urdf = ET.parse(ROOT / "description/urdf/robots/g1/g1_29dof.urdf").getroot()
        joints = {joint.attrib["name"]: joint for joint in urdf.findall("joint")}
        for name in PARAMS["robot"]["jointNames"]:
            effort = float(joints[name].find("limit").attrib["effort"])
            self.assertTrue(math.isfinite(effort) and effort > 0, name)

    def test_full_yaml_override_is_still_declared_and_forwarded(self):
        params = deepcopy(PARAMS)
        params["ocs2"]["constraints"]["jointTorqueLimits"]["limits"] = [12.0] * len(params["robot"]["jointNames"])
        check_keys(params, SCHEMA)
        override = block(BUILDER, "if (!p.constraints.jointTorqueLimits.limits.empty())")
        self.assertIn("checkJointArraySize", override)
        self.assertIn("config.torqueLimits = toVector(p.constraints.jointTorqueLimits.limits);", override)

    def test_only_an_empty_override_uses_urdf_effort(self):
        fallback = block(INTERFACE, "if (config_.torqueLimits.size() == 0)")
        self.assertIn("config_.torqueLimits = model.effortLimit.tail(nj);", fallback)
        self.assertIn("if (config_.torqueLimits.size() != nj)", INTERFACE)
        self.assertIn("!std::isfinite(config_.torqueLimits(i)) || config_.torqueLimits(i) <= 0.0", INTERFACE)
        self.assertLess(INTERFACE.index("model.effortLimit.tail(nj)"), INTERFACE.index("if (setupOCP)"))

    def test_ocp_and_clamp_share_the_resolved_limits(self):
        header = (ROOT / "include/humanoid_wb_id_mpc/WbIdMpcInterface.h").read_text()
        controller = (ROOT / f"src/humanoid_whole_body_inverse_dynamics_mpc/{NAME}.cpp").read_text()
        self.assertIn("getTorqueLimits() const { return config_.torqueLimits; }", header)
        self.assertIn("torque_limit_ = mpc_interface_->getTorqueLimits();", controller)
        self.assertIn("clamp_torque(command.feedforward);", controller)
        self.assertIn("config_.torqueLimits, torqueWindowPtr_", INTERFACE)
        self.assertIn("std::make_unique<RelaxedBarrierPenalty>(config_.torqueLimitBarrierConfig)", INTERFACE)


if __name__ == "__main__":
    unittest.main()
