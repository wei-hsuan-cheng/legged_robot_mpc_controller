#!/usr/bin/env python3
"""Check the opt-in WB-ID geometric-grid wiring without ROS."""

from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
NAME = "humanoid_whole_body_inverse_dynamics_mpc_controller"
FOLDER = ROOT / "src/humanoid_whole_body_inverse_dynamics_mpc"


class TimeGridContract(unittest.TestCase):
    def test_uniform_is_schema_default(self):
        schema = yaml.safe_load((FOLDER / f"{NAME}_parameter.yaml").read_text())[NAME]
        grid = schema["ocs2"]["solver"]["sqp"]["timeGrid"]
        self.assertEqual(set(grid), {"activate", "maxStep", "growthFactor", "fineHorizon"})
        self.assertIs(grid["activate"]["default_value"], False)

    def test_mapping_and_independent_physical_torque_window(self):
        builder = (FOLDER / "whole_body_inverse_dynamics_mpc_config_builder.cpp").read_text()
        self.assertIn("if (p.solver.sqp.timeGrid.activate)", builder)
        for setting, parameter in (("dtMax", "maxStep"), ("dtGrowth", "growthFactor"), ("dtFineHorizon", "fineHorizon")):
            self.assertIn(f"sqp.{setting} = p.solver.sqp.timeGrid.{parameter}", builder)
        self.assertNotIn("sqp.dtFineHorizon < config.torqueWindowDuration", builder)
        self.assertNotIn("fineHorizon must cover", builder)
        self.assertIn("- 0.5) * p.solver.sqp.dt", builder)

    def test_torque_window_activation_remains_time_based(self):
        window = (ROOT / "include/humanoid_wb_id_mpc/constraint/WbIdTorqueWindow.h").read_text()
        self.assertIn("if (duration_ < 0.0)", window)
        self.assertIn("return time < horizonStart_.load(std::memory_order_acquire) + duration_;", window)
        self.assertNotIn("fineHorizon", window)

    def test_h2_preserves_near_term_resolution(self):
        params = yaml.safe_load((ROOT / "config/h2/ros2_controllers.yaml").read_text())[NAME]["ros__parameters"]["ocs2"]
        sqp = params["solver"]["sqp"]
        grid = sqp["timeGrid"]
        self.assertGreaterEqual(grid["maxStep"], sqp["dt"])
        window = (params["constraints"]["inverseDynamics"]["torqueConstraintNodes"] - .5) * sqp["dt"]
        self.assertGreaterEqual(grid["fineHorizon"], window)


if __name__ == "__main__":
    unittest.main()
