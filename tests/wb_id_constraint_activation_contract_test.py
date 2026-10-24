#!/usr/bin/env python3
"""Static WB-ID constraint wiring checks, not C++ solver or walking validation."""

from copy import deepcopy
from itertools import product
import math
import unittest
import xml.etree.ElementTree as ET

from wb_id_cost_activation_contract_test import BUILDER, INTERFACE, PARAMS, ROOT, SCHEMA, block
from wb_id_parameter_contract_test import NAME, check_keys


FLAGS = {
    "inverseDynamics": ("activateInverseDynamics", ("rneaBaseDynamics", "rneaJointTorques")),
    "footMotionEquality": ("activateFootMotionEquality", ("_stanceAcceleration", "_swingVerticalAcceleration")),
    "swingFootZeroWrench": ("activateSwingFootZeroWrench", ("_zeroWrench",)),
    "frictionCone": ("activateFrictionCone", ("_frictionForceCone",)),
    "contactMomentXY": ("activateContactMomentXY", ("_contactMomentXY",)),
    "jointPositionLimits": ("activateJointPositionLimits", ("jointPositionLimits",)),
    "jointVelocityLimits": ("activateJointVelocityLimits", ("jointVelocityLimits",)),
    "jointTorqueLimits": ("activateJointTorqueLimits", ("jointTorqueLimits",)),
    "legSelfCollision": ("activateLegSelfCollision", ("FootCollisionSoftConstraint",)),
}


class ConstraintActivationContractTest(unittest.TestCase):
    def test_every_constraint_has_an_enabled_default_and_builder_mapping(self):
        header = (ROOT / "include/humanoid_wb_id_mpc/WbIdMpcInterface.h").read_text()
        self.assertEqual(set(PARAMS["ocs2"]["constraints"]), set(FLAGS))
        for name, (flag, _) in FLAGS.items():
            with self.subTest(constraint=name):
                self.assertIs(PARAMS["ocs2"]["constraints"][name]["activate"], True)
                entry = SCHEMA["ocs2"]["constraints"][name]["activate"]
                self.assertEqual(entry["type"], "bool")
                self.assertIs(entry["default_value"], True)
                self.assertIn(f"bool {flag} = true;", header)
                self.assertIn(f"config.{flag} = p.constraints.{name}.activate;", BUILDER)

    def test_all_switch_combinations_are_declared_parameters(self):
        for values in product((False, True), repeat=len(FLAGS)):
            params = deepcopy(PARAMS)
            for name, value in zip(FLAGS, values):
                params["ocs2"]["constraints"][name]["activate"] = value
            check_keys(params, SCHEMA)

    def test_registration_and_construction_are_inside_their_guards(self):
        for name, (flag, terms) in FLAGS.items():
            with self.subTest(constraint=name):
                guard = block(INTERFACE, f"if (config_.{flag}")
                for term in terms:
                    self.assertIn(f'"{term}"', guard)
                    self.assertEqual(INTERFACE.count(f'"{term}"'), 1)

    def test_position_rename_retains_the_existing_urdf_bound_reader(self):
        for tree in (PARAMS, SCHEMA):
            self.assertNotIn("jointLimits", tree["ocs2"]["constraints"])
        self.assertIn("costs.jointLimitsBarrierConfig.mu = p.constraints.jointPositionLimits.mu;", BUILDER)
        self.assertIn("factory.getJointLimitsConstraint()", block(INTERFACE, "if (config_.activateJointPositionLimits)"))
        factory = (ROOT / "src/core/humanoid_common_mpc/src/HumanoidCostConstraintFactory.cpp").read_text()
        self.assertIn("readPinocchioJointLimits", block(factory, "HumanoidCostConstraintFactory::getJointLimitsConstraint()"))
        reader = (ROOT / "src/core/humanoid_common_mpc/src/pinocchio_model/pinocchioUtils.cpp").read_text()
        self.assertIn("model.upperPositionLimit.tail", reader)
        self.assertIn("model.lowerPositionLimit.tail", reader)

    def test_velocity_bounds_still_use_scaled_urdf_limits(self):
        guard = block(INTERFACE, "if (config_.activateJointVelocityLimits")
        self.assertIn("config_.velocityScaling * pinocchioInterfacePtr_->getModel().velocityLimit.tail(nj)", guard)
        urdf = ET.parse(ROOT / "description/urdf/robots/g1/g1_29dof.urdf").getroot()
        joints = {joint.attrib["name"]: joint for joint in urdf.findall("joint")}
        for name in PARAMS["robot"]["jointNames"]:
            limits = joints[name].find("limit").attrib
            self.assertLess(float(limits["lower"]), float(limits["upper"]))
            self.assertTrue(math.isfinite(float(limits["velocity"])) and float(limits["velocity"]) > 0)

    def test_disabled_rnea_does_not_execute_free_policy_torques_or_disable_clipping(self):
        controller = (ROOT / f"src/humanoid_whole_body_inverse_dynamics_mpc/{NAME}.cpp").read_text()
        self.assertIn("feedforward_ == Feedforward::PolicyTorque && parameters_.ocs2.constraints.inverseDynamics.activate", controller)
        action = block(controller, "HumanoidWholeBodyInverseDynamicsMpcController::compute_mpc_joint_action(")
        self.assertIn("control_model_->getGeneralizedCoordinates(observation.state)", action)
        self.assertIn("clamp_torque(command.feedforward);", action)
        self.assertNotIn("constraints.jointTorqueLimits.activate", action)

    def test_scaling_names_defaults_validation_and_shared_torque_bounds(self):
        self.assertNotIn("scale", SCHEMA["ocs2"]["constraints"]["jointVelocityLimits"])
        for group, name in (("jointVelocityLimits", "velocityScaling"), ("jointTorqueLimits", "torqueScaling")):
            entry = SCHEMA["ocs2"]["constraints"][group][name]
            self.assertEqual(entry["default_value"], 1.0)
            self.assertEqual(entry["validation"], {"gt": [0.0], "lt_eq": [1.0]})
            self.assertEqual(PARAMS["ocs2"]["constraints"][group][name], 1.0)
            self.assertIn(f"config.{name} = p.constraints.{group}.{name};", BUILDER)
            self.assertIn(f"!std::isfinite(config_.{name})", INTERFACE)
            self.assertIn(f"config_.{name} <= 0.0", INTERFACE)
            self.assertIn(f"config_.{name} > 1.0", INTERFACE)
        self.assertIn("config_.torqueLimits *= config_.torqueScaling;", INTERFACE)
        self.assertLess(INTERFACE.index("config_.torqueLimits *= config_.torqueScaling;"), INTERFACE.index("if (setupOCP)"))


if __name__ == "__main__":
    unittest.main()
