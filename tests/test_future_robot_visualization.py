"""Future mesh ghost configuration and worker wiring checks without ROS."""

from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
WB_ID = "humanoid_whole_body_inverse_dynamics_mpc_controller"
CENTROIDAL = "humanoid_centroidal_mpc_controller"


class FutureRobotVisualizationTest(unittest.TestCase):
    def test_legacy_parameters_in_both_schemas(self):
        for controller, folder in ((WB_ID, "humanoid_whole_body_inverse_dynamics_mpc"),
                                   (CENTROIDAL, "humanoid_centroidal_mpc")):
            schema = yaml.safe_load((ROOT / "src" / folder / (controller + "_parameter.yaml")).read_text())
            params = schema[controller]["visualization"]
            self.assertTrue(params["optimizedStateRobotModelsActivate"]["default_value"])
            self.assertEqual(params["optimizedStateRobotModelsInterval"]["default_value"], 0.2)
            self.assertEqual(params["optimizedStateRobotModelsInterval"]["validation"], {"gt": [0.0]})

    def test_robot_configs_use_same_temporal_spacing(self):
        for robot, controllers in (("g1", (WB_ID, CENTROIDAL)), ("h2", (WB_ID,))):
            config = yaml.safe_load((ROOT / "config" / robot / "ros2_controllers.yaml").read_text())
            for controller in controllers:
                params = config[controller]["ros__parameters"]["visualization"]
                self.assertTrue(params["optimizedStateRobotModelsActivate"])
                self.assertEqual(params["optimizedStateRobotModelsInterval"], 0.2)
        launch = (ROOT / "launch/robot.launch.py").read_text()
        self.assertNotIn("optimizedStateRobotModels", launch)

    def test_rviz_topics_and_qos(self):
        displays = yaml.safe_load((ROOT / "config/rviz/humanoid.rviz").read_text())["Visualization Manager"]["Displays"]
        for name, topic in (("Optimized State Ghosts", "optimizedStateGhosts"),
                            ("Terminal State Ghost", "terminalStateGhost"),
                            ("Terminal Target Ghost", "terminalTargetGhost")):
            matches = [display for display in displays if display["Name"] == name]
            if name != "Optimized State Ghosts" and not matches:
                continue  # Terminal-only displays are optional in saved user layouts.
            self.assertEqual(len(matches), 1)
            display = matches[0]
            self.assertEqual(display["Class"], "rviz_default_plugins/MarkerArray")
            self.assertEqual(display["Topic"]["Value"], "/performance_visualization/" + topic)
            self.assertEqual(display["Topic"]["Reliability Policy"], "Best Effort")
            if name == "Optimized State Ghosts":
                self.assertTrue(display["Enabled"])

    def test_shared_worker_uses_consistent_policy_snapshot(self):
        worker = (ROOT / "src/visualization/performance_visualization.cpp").read_text()
        self.assertIn("if (!has_subscribers_.load", worker)
        self.assertIn("Snapshot{times, states, targets}", worker)
        self.assertIn("robot_model_visualization_->publish(snapshot->times, snapshot->states, snapshot->targets)", worker)
        source = (ROOT / "src/visualization/future_robot_model_visualization.cpp").read_text()
        self.assertIn("mpc_robot_model_->getGeneralizedCoordinates(state)", source)
        self.assertIn("times.back() - times.front()", source)
        self.assertIn("std::round(horizon / settings_.interval)", source)
        self.assertIn("rclcpp::QoS(1).best_effort()", source)
        self.assertIn("0.765F, 0.665F, 0.905F", source)
        self.assertIn("0.22 + (0.08 - 0.22) * progress", source)

    def test_centroidal_handoff_is_on_policy_owner_thread(self):
        source = (ROOT / "src/humanoid_centroidal_mpc/humanoid_centroidal_mpc_controller.cpp").read_text()
        worker = source[source.index("void HumanoidCentroidalMpcController::solver_worker()") :]
        self.assertIn("performance_visualization_->update_visualization(", worker)
        prefix = source[:source.index("void HumanoidCentroidalMpcController::solver_worker()")]
        self.assertNotIn("performance_visualization_->update_visualization(", prefix)
        self.assertLess(worker.index("mrt_interface_->updatePolicy()"),
                        worker.index("performance_visualization_->update_visualization("))


if __name__ == "__main__":
    unittest.main()
