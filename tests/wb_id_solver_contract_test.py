#!/usr/bin/env python3
"""Check that WB-ID exposes only its implemented SQP/execution settings."""

from pathlib import Path
import unittest

import yaml

from wb_id_parameter_contract_test import check_keys, UniqueLoader


ROOT = Path(__file__).resolve().parents[1]
NAME = "humanoid_whole_body_inverse_dynamics_mpc_controller"
FOLDER = ROOT / "src/controller"


class SolverContract(unittest.TestCase):
    def test_robot_configs_match_schema_without_unused_settings(self):
        schema = yaml.load((FOLDER / "parameters.yaml").read_text(), Loader=UniqueLoader)[NAME]
        for robot in ("g1", "h2"):
            with self.subTest(robot=robot):
                config = yaml.load((ROOT / f"config/{robot}/ros2_controllers.yaml").read_text(), Loader=UniqueLoader)
                params = config[NAME]["ros__parameters"]
                check_keys(params, schema)
                for tree in (params, schema):
                    solver = tree["ocs2"]["solver"]
                    self.assertEqual(set(solver["rollout"]), {"policyTimeOffset"})
                    self.assertNotIn("inequalityConstraintMu", solver["sqp"])
                    self.assertNotIn("inequalityConstraintDelta", solver["sqp"])

    def test_remaining_sqp_parameters_are_mapped(self):
        builder = (FOLDER / "config_builder.cpp").read_text()
        for name in ("nThreads", "dt", "sqpIteration", "deltaTol", "g_max", "g_min",
                     "projectStateInputEqualityConstraints", "printSolverStatistics", "printSolverStatus",
                     "printLinesearch", "useFeedbackPolicy", "integratorType", "threadPriority"):
            self.assertIn(f"p.solver.sqp.{name}", builder)
        for name in ("inequalityConstraintMu", "inequalityConstraintDelta", "rolloutSettings"):
            self.assertNotIn(name, builder)
        self.assertNotIn("p.solver.rollout", builder)

    def test_execution_uses_policy_interpolation_not_ode_rollout(self):
        controller = (FOLDER / "controller.cpp").read_text()
        self.assertIn("mrt_interface_->evaluatePolicy(", controller)
        self.assertEqual(controller.count("parameters_.ocs2.solver.rollout.policyTimeOffset"), 2)
        self.assertNotIn("initRollout(", controller)
        self.assertNotIn("rolloutPolicy(", controller)
        interface = (ROOT / "src/mpc/wb_id/WbIdMpcInterface.cpp").read_text()
        header = (ROOT / "include/legged_robot_mpc_controller/mpc/wb_id/WbIdMpcInterface.h").read_text()
        for source in (interface, header):
            for name in ("TimeTriggeredRollout", "rolloutPtr_", "rolloutSettings", "ddpSettings"):
                self.assertNotIn(name, source)

    def test_schema_only_advertises_supported_solver(self):
        schema = yaml.safe_load((FOLDER / "parameters.yaml").read_text())[NAME]
        self.assertEqual(schema["ocs2"]["mpc"]["solverType"]["validation"]["one_of<>"], [["sqp"]])


if __name__ == "__main__":
    unittest.main()
