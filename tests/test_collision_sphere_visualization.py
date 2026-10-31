"""Collision marker configuration and worker wiring checks without ROS."""

from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
CONTROLLER = "humanoid_whole_body_inverse_dynamics_mpc_controller"


class CollisionSphereVisualizationTest(unittest.TestCase):
    def test_yaml_controls_and_schema(self):
        schema = yaml.safe_load((ROOT / "src/humanoid_whole_body_inverse_dynamics_mpc" /
                                 (CONTROLLER + "_parameter.yaml")).read_text())[CONTROLLER]
        controls = schema["visualization"]["collisionSpheres"]
        self.assertFalse(controls["activate"]["default_value"])
        self.assertEqual(controls["alpha"]["validation"], {"gt": [0.0], "lt_eq": [1.0]})
        for robot in ("g1", "h2"):
            params = yaml.safe_load((ROOT / f"config/{robot}/ros2_controllers.yaml").read_text())[CONTROLLER]["ros__parameters"]
            self.assertTrue(params["visualization"]["collisionSpheres"]["activate"])
            self.assertEqual(params["visualization"]["collisionSpheres"]["alpha"], 0.25)
        self.assertNotIn("collisionSpheres", (ROOT / "launch/robot.launch.py").read_text())

    def test_shared_geometry_from_ocp_config(self):
        source = (ROOT / "src/humanoid_whole_body_inverse_dynamics_mpc" /
                  (CONTROLLER + ".cpp")).read_text()
        self.assertIn("mpc_config.costConstraintConfig.footCollisionConfig", source)
        self.assertIn("spheres.activate && mpc_config.activateLegSelfCollision", source)
        self.assertIn("spheres.activate && mpc_config.armSelfCollision.activate", source)
        for frame in ("leftAnkleFrame", "rightAnkleFrame", "leftFootCenterFrame", "rightFootCenterFrame",
                      "leftFootFrame1", "rightFootFrame1", "leftFootFrame2", "rightFootFrame2",
                      "leftKneeFrame", "rightKneeFrame"):
            self.assertIn("leg." + frame, source)
        self.assertIn("sphere.name, sphere.linkName, sphere.center, sphere.radius", source)
        self.assertIn("observation.state);", source)

    def test_subscriber_driven_worker_and_qos(self):
        source = (ROOT / "src/visualization/collision_sphere_visualization.cpp").read_text()
        worker = (ROOT / "src/visualization/performance_visualization.cpp").read_text()
        self.assertIn(".best_effort().durability_volatile()", source)
        self.assertIn("if (hasSubscribers())", source)
        self.assertIn("collision_sphere_visualization_->hasSubscribers()", worker)
        self.assertIn("collision_sphere_visualization_->publish(snapshot->observed_state)", worker)
        self.assertIn("2.0 * sphere.radius", source)
        self.assertIn(".act(sphere.center)", source)
        self.assertIn("marker.lifetime", source)

    def test_rviz_display(self):
        displays = yaml.safe_load((ROOT / "config/rviz/humanoid.rviz").read_text())["Visualization Manager"]["Displays"]
        display, = [d for d in displays if d["Name"] == "Collision Spheres"]
        self.assertTrue(display["Enabled"])
        self.assertEqual(display["Topic"]["Value"], "/performance_visualization/collisionSpheres")
        self.assertEqual(display["Topic"]["Reliability Policy"], "Best Effort")
        self.assertEqual(display["Topic"]["Durability Policy"], "Volatile")


if __name__ == "__main__":
    unittest.main()
