#!/usr/bin/env python3
"""Check that the WB-ID MPC controller expose only supported SQP paths."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml

from wb_id_parameter_contract_test import check_keys, UniqueLoader


ROOT = Path(__file__).resolve().parents[1]
CONTROLLERS = (
    ("controller", "humanoid_whole_body_inverse_dynamics_mpc_controller", "WbIdMpcInterface"),
)


class SqpOnlyContract(unittest.TestCase):
    def test_schemas_only_allow_sqp(self):
        for folder, name, _ in CONTROLLERS:
            with self.subTest(controller=name):
                schema = yaml.safe_load((ROOT / f"src/{folder}/parameters.yaml").read_text())[name]
                self.assertEqual(schema["ocs2"]["mpc"]["solverType"]["validation"]["one_of<>"], [["sqp"]])

    def test_robot_config_keys_and_live_solver_settings(self):
        for robot in ("g1", "h2"):
            config = yaml.load((ROOT / f"config/{robot}/ros2_controllers.yaml").read_text(), Loader=UniqueLoader)
            for folder, name, _ in CONTROLLERS:
                if name not in config:
                    continue
                with self.subTest(robot=robot, controller=name):
                    schema = yaml.load((ROOT / f"src/{folder}/parameters.yaml").read_text(), Loader=UniqueLoader)[name]
                    params = config[name]["ros__parameters"]
                    check_keys(params["ocs2"], schema["ocs2"], "ocs2.")
                    ocs2 = params["ocs2"]
                    self.assertEqual(ocs2["mpc"]["solverType"], "sqp")
                    sqp = ocs2.get("solver", ocs2)["sqp"]
                    self.assertIn("integratorType", sqp)
                    for key in ("inequalityConstraintMu", "inequalityConstraintDelta"):
                        self.assertNotIn(key, sqp)

    def test_no_unused_rollout_or_ddp_interface_plumbing(self):
        for folder, name, interface in CONTROLLERS:
            core = "wb_id"
            paths = (ROOT / f"include/legged_robot_mpc_controller/mpc/{core}/{interface}.h",
                     ROOT / f"src/mpc/{core}/{interface}.cpp",
                     ROOT / f"src/{folder}/controller.cpp")
            for path in paths:
                with self.subTest(path=path):
                    source = path.read_text()
                    for unused in ("ocs2_ddp", "ddpSettings", "rolloutSettings", "rolloutPtr_",
                                   "TimeTriggeredRollout", "initRollout(", "rolloutPolicy("):
                        self.assertNotIn(unused, source)

    def test_controllers_keep_sqp_and_policy_evaluation(self):
        for folder, name, _ in CONTROLLERS:
            source = (ROOT / f"src/{folder}/controller.cpp").read_text()
            self.assertIn("std::make_unique<ocs2::SqpMpc>", source)
            self.assertIn("mrt_interface_->evaluatePolicy(", source)

    def test_no_direct_ddp_build_dependency(self):
        self.assertNotIn("ocs2_ddp", (ROOT / "CMakeLists.txt").read_text())
        manifest = ET.parse(ROOT / "package.xml").getroot()
        self.assertNotIn("ocs2_ddp", [entry.text for entry in manifest.findall("depend")])


if __name__ == "__main__":
    unittest.main()
