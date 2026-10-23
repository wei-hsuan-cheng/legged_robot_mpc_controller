#!/usr/bin/env python3
"""Static optional-cost wiring checks; these do not execute the C++ OCP."""

from copy import deepcopy
from itertools import product
import unittest

import yaml

from wb_id_parameter_contract_test import NAME, ROOT, UniqueLoader, check_keys


FOLDER = ROOT / "src/humanoid_whole_body_inverse_dynamics_mpc"
SCHEMA = yaml.load((FOLDER / f"{NAME}_parameter.yaml").read_text(), Loader=UniqueLoader)[NAME]
PARAMS = yaml.load((ROOT / "config/g1/ros2_controllers.yaml").read_text(), Loader=UniqueLoader)[NAME]["ros__parameters"]
BUILDER = (FOLDER / "whole_body_inverse_dynamics_mpc_config_builder.cpp").read_text()
INTERFACE = (ROOT / "src/core/humanoid_wb_id_mpc/src/WbIdMpcInterface.cpp").read_text()


def block(source, marker):
    start = source.index("{", source.index(marker))
    depth = 1
    for end in range(start + 1, len(source)):
        depth += (source[end] == "{") - (source[end] == "}")
        if depth == 0:
            return source[start + 1:end]
    raise AssertionError(f"Unclosed block: {marker}")


class CostActivationContractTest(unittest.TestCase):
    def test_schema_defaults_preserve_enabled_costs(self):
        costs = SCHEMA["ocs2"]["costs"]
        for entry in (costs["taskSpaceFootCost"], costs["taskSpaceCosts"]["__map_names"], costs["legTorqueCost"]):
            self.assertEqual(entry["activate"]["type"], "bool")
            self.assertIs(entry["activate"]["default_value"], True)
        self.assertNotIn("taskSpaceFootCostWeights", costs)

    def test_all_switch_combinations_are_valid_yaml_parameters(self):
        for foot, torso, leg in product((False, True), repeat=3):
            with self.subTest(foot=foot, torso=torso, leg=leg):
                params = deepcopy(PARAMS)
                costs = params["ocs2"]["costs"]
                costs["taskSpaceFootCost"]["activate"] = foot
                costs["taskSpaceCosts"]["torso"]["activate"] = torso
                costs["legTorqueCost"]["activate"] = leg
                check_keys(yaml.safe_load(yaml.safe_dump(params)), SCHEMA)

    def test_builder_maps_flags_and_only_parses_enabled_weights(self):
        for name, field, weights in (
            ("activateTaskSpaceFootCost", "taskSpaceFootCost", "p.costs.taskSpaceFootCost.weights.data()"),
            ("activateLegTorqueCost", "legTorqueCost", "p.costs.legTorqueCost.leftWeights"),
        ):
            self.assertIn(f"config.{name} = p.costs.{field}.activate;", BUILDER)
            self.assertIn(weights, block(BUILDER, f"if (config.{name})"))

    def test_named_costs_skip_before_kinematics_construction(self):
        self.assertIn("continue;", block(BUILDER, "if (!entry.activate)"))
        self.assertLess(BUILDER.index("if (!entry.activate)"), BUILDER.index("if (entry.weights.size()"))
        self.assertIn("taskSpaceCost.activate = entry.activate;", BUILDER)
        self.assertIn("continue;", block(INTERFACE, "if (!taskSpaceCost.activate)"))
        self.assertLess(INTERFACE.index("if (!taskSpaceCost.activate)"),
                        INTERFACE.index("PinocchioEndEffectorKinematicsCppAd eeKinematics"))

    def test_foot_and_load_constructors_are_inside_activation_guards(self):
        for flag, constructor in (
            ("activateTaskSpaceFootCost", "std::make_unique<CentroidalMpcEndEffectorFootCost>"),
            ("activateLegTorqueCost", "factory.getExternalTorqueQuadraticCost(i)"),
        ):
            guard = block(INTERFACE, f"if (config_.{flag})")
            self.assertIn(constructor, guard)
            self.assertEqual(INTERFACE.count(constructor), 1)
            self.assertNotIn("equalityConstraintPtr", guard)
            self.assertNotIn("softConstraintPtr", guard)

    def test_disabling_extra_costs_keeps_main_costs_and_constraints(self):
        remaining = INTERFACE
        for flag in ("activateTaskSpaceFootCost", "activateLegTorqueCost"):
            remaining = remaining.replace(block(remaining, f"if (config_.{flag})"), "")
        for name in ("stateInputQuadraticCost", "baseMotionTrackingCost", "jointTrackingCost", "terminalCost",
                     "rneaBaseDynamics", "rneaJointTorques", "_stanceAcceleration", "_swingVerticalAcceleration",
                     "_zeroWrench", "_frictionForceCone", "_contactMomentXY"):
            self.assertIn(f'"{name}"', remaining)


if __name__ == "__main__":
    unittest.main()
