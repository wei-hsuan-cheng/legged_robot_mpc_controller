"""Configuration checks; physical flight is verified by native replay tests."""

from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]


class RunningGaitTest(unittest.TestCase):
    def test_gait_heights_and_optional_schema_defaults(self):
        for robot, heights in (("g1", (.08, .09, .10)), ("h2", (.10, .15, .20))):
            with self.subTest(robot=robot):
                config = yaml.safe_load((ROOT / "config" / robot / "ros2_controllers.yaml").read_text())
                gait = config["humanoid_whole_body_inverse_dynamics_mpc_controller"]["ros__parameters"]["ocs2"]["gait"]
                for name, height in zip(("walk", "trot", "run"), heights):
                    self.assertEqual(gait["templates"][name]["swingHeight"], height)
                self.assertEqual(gait["swingTrajectory"]["swingHeight"], heights[0])
        for relative in (
            "src/humanoid_whole_body_inverse_dynamics_mpc/humanoid_whole_body_inverse_dynamics_mpc_controller_parameter.yaml",
            "src/humanoid_centroidal_mpc/humanoid_centroidal_mpc_controller_parameter.yaml",
        ):
            schema = yaml.safe_load((ROOT / relative).read_text())
            params = next(iter(schema.values()))
            self.assertEqual(params["ocs2"]["gait"]["templates"]["__map_names"]["swingHeight"]["default_value"], -1.)

    def test_flight_timing_and_supported_transition_gaits(self):
        for robot, support, flight in (("g1", .25, .08), ("h2", .26, .04)):
            with self.subTest(robot=robot):
                config = yaml.safe_load((ROOT / "config" / robot / "ros2_controllers.yaml").read_text())
                gait = config["humanoid_whole_body_inverse_dynamics_mpc_controller"]["ros__parameters"]["ocs2"]["gait"]
                run = gait["templates"]["run"]
                self.assertEqual(run["modeSequence"], ["LF", "FLY", "RF", "FLY"])
                durations = [b-a for a, b in zip(run["switchingTimes"], run["switchingTimes"][1:])]
                for actual, expected in zip(durations, (support, flight, support, flight)):
                    self.assertAlmostEqual(actual, expected)
                expected_ladder = ["stance", "walk", "trot", "run"]
                self.assertEqual(gait["transitions"]["ladder"], expected_ladder)
                expected_bands = {"stance": [-.1, .03], "walk": [.02, .45], "trot": [.40, .75], "run": [.70, 10.]}
                for name, band in expected_bands.items():
                    self.assertEqual(gait["transitions"][name]["linearVelocity"], band)
                self.assertNotIn("slower_trot", gait["transitions"])
                for name in ("slow_walk", "walk"):
                    self.assertNotIn("FLY", gait["templates"][name]["modeSequence"])
                self.assertEqual(gait["templates"]["slow_walk"]["switchingTimes"], [0, .4, .55, .95, 1.1])

    def test_speed_hysteresis_does_not_trigger_running_for_pure_yaw(self):
        for robot in ("g1", "h2"):
            with self.subTest(robot=robot):
                config = yaml.safe_load((ROOT / "config" / robot / "ros2_controllers.yaml").read_text())
                params = config["humanoid_whole_body_inverse_dynamics_mpc_controller"]["ros__parameters"]
                transition = params["ocs2"]["gait"]["transitions"]
                entry = transition["trot"]["linearVelocity"][1]
                exit_ = transition["run"]["linearVelocity"][0]
                self.assertLess(exit_, entry)
                self.assertEqual(entry, .75)
                self.assertEqual(exit_, .70)
                bands = [transition[name]["linearVelocity"] for name in transition["ladder"]]
                for previous, following in zip(bands, bands[1:]):
                    self.assertLess(following[0], previous[1])
                    self.assertGreater(following[1], previous[1])
                self.assertGreater(transition["run"]["linearVelocity"][1], 1.0)
                self.assertGreater(transition["walk"]["yawRate"][1], params["ocs2"]["reference"]["maxRotationVelocity"])
                first_moving = transition[transition["ladder"][1]]
                self.assertLess(first_moving["yawRate"][0], transition["stance"]["yawRate"][1])
                self.assertGreater(params["ocs2"]["constraints"]["legSelfCollision"]["minimumLateralSeparation"], 0)

    def test_h2_running_foothold_reference(self):
        schema = yaml.safe_load((ROOT / "src/humanoid_whole_body_inverse_dynamics_mpc/"
                                 "humanoid_whole_body_inverse_dynamics_mpc_controller_parameter.yaml").read_text())
        defaults = next(iter(schema.values()))["ocs2"]["costs"]["captureFootPlacement"]
        self.assertFalse(defaults["enabled"]["default_value"])
        h2 = yaml.safe_load((ROOT / "config/h2/ros2_controllers.yaml").read_text())
        params = h2["humanoid_whole_body_inverse_dynamics_mpc_controller"]["ros__parameters"]
        capture = params["ocs2"]["costs"]["captureFootPlacement"]
        self.assertEqual(set(capture), {key for key in defaults})
        self.assertTrue(capture["enabled"])
        self.assertEqual(capture["gain"], 0.0)
        self.assertEqual(capture["stanceTimeGain"], 0.5)
        self.assertTrue(capture["currentSwingOnly"])
        self.assertEqual(params["ocs2"]["constraints"]["inverseDynamics"]["torqueConstraintNodes"], 10)
        self.assertGreaterEqual(params["ocs2"]["reference"]["maxDisplacementVelocityX"], 2.0)
        g1 = yaml.safe_load((ROOT / "config/g1/ros2_controllers.yaml").read_text())
        self.assertNotIn("captureFootPlacement",
                         g1["humanoid_whole_body_inverse_dynamics_mpc_controller"]["ros__parameters"]["ocs2"]["costs"])

    def test_inclusive_speed_boundary_tolerates_filter_roundoff(self):
        source = (ROOT / "src/core/humanoid_common_mpc/src/reference_manager/ProceduralMpcMotionManager.cpp").read_text()
        self.assertIn("constexpr scalar_t speedBoundaryTolerance = 1e-9;", source)
        self.assertIn("std::abs(velCommandVec(0)) >= cfg.maxLinVelCmd - speedBoundaryTolerance", source)
        self.assertIn("std::abs(velCommandVec(1)) >= cfg.maxLinVelCmd - speedBoundaryTolerance", source)
        # Reference boundary arithmetic, not an invocation of the compiled C++ guard.
        for speed in (.75, .75 - 1e-14, -.75, 1.0):
            self.assertTrue(abs(speed) >= .75 - 1e-9)
        self.assertFalse(.749 >= .75 - 1e-9)


if __name__ == "__main__":
    unittest.main()
