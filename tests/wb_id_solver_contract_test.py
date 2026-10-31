#!/usr/bin/env python3
"""Check that WB-ID exposes only its implemented SQP/execution settings."""

from pathlib import Path
import unittest

import yaml

from wb_id_parameter_contract_test import check_keys, UniqueLoader


ROOT = Path(__file__).resolve().parents[1]
NAME = "humanoid_whole_body_inverse_dynamics_mpc_controller"
FOLDER = ROOT / "src/humanoid_whole_body_inverse_dynamics_mpc"


class SolverContract(unittest.TestCase):
    def test_robot_configs_match_schema_without_unused_settings(self):
        schema = yaml.load((FOLDER / f"{NAME}_parameter.yaml").read_text(), Loader=UniqueLoader)[NAME]
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
        builder = (FOLDER / "whole_body_inverse_dynamics_mpc_config_builder.cpp").read_text()
        for name in ("nThreads", "dt", "sqpIteration", "deltaTol", "g_max", "g_min",
                     "projectStateInputEqualityConstraints", "printSolverStatistics", "printSolverStatus",
                     "printLinesearch", "useFeedbackPolicy", "integratorType", "threadPriority"):
            self.assertIn(f"p.solver.sqp.{name}", builder)
        for name in ("inequalityConstraintMu", "inequalityConstraintDelta", "rolloutSettings"):
            self.assertNotIn(name, builder)
        self.assertNotIn("p.solver.rollout", builder)

    def test_execution_uses_policy_interpolation_not_ode_rollout(self):
        controller = (FOLDER / f"{NAME}.cpp").read_text()
        self.assertIn("mrt_interface_->evaluatePolicy(", controller)
        self.assertEqual(controller.count("parameters_.ocs2.solver.rollout.policyTimeOffset"), 2)
        self.assertNotIn("initRollout(", controller)
        self.assertNotIn("rolloutPolicy(", controller)
        interface = (ROOT / "src/core/humanoid_wb_id_mpc/src/WbIdMpcInterface.cpp").read_text()
        header = (ROOT / "include/humanoid_wb_id_mpc/WbIdMpcInterface.h").read_text()
        for source in (interface, header):
            for name in ("TimeTriggeredRollout", "rolloutPtr_", "rolloutSettings", "ddpSettings"):
                self.assertNotIn(name, source)

    def test_schema_only_advertises_supported_solver(self):
        schema = yaml.safe_load((FOLDER / f"{NAME}_parameter.yaml").read_text())[NAME]
        self.assertEqual(schema["ocs2"]["mpc"]["solverType"]["validation"]["one_of<>"], [["sqp"]])

    def test_centroidal_also_removes_unused_solver_and_rollout_settings(self):
        config = yaml.safe_load((ROOT / "config/g1/ros2_controllers.yaml").read_text())
        ocs2 = config["humanoid_centroidal_mpc_controller"]["ros__parameters"]["ocs2"]
        self.assertNotIn("inequalityConstraintMu", ocs2["sqp"])
        self.assertNotIn("inequalityConstraintDelta", ocs2["sqp"])
        self.assertNotIn("rollout", ocs2)
        controller = (ROOT / "src/humanoid_centroidal_mpc/humanoid_centroidal_mpc_controller.cpp").read_text()
        self.assertNotIn("initRollout(", controller)
        self.assertIn("parameters_.control.policyTimeOffset", controller)


if __name__ == "__main__":
    unittest.main()
