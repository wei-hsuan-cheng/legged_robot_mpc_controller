"""Check optional stand-up data/export contracts without running ROS."""

import importlib.util
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np
import yaml

try:
    import mujoco
except ImportError:
    mujoco = None

ROOT = Path(__file__).resolve().parents[1]


class UpkieStandUpTest(unittest.TestCase):
    def test_shipped_trajectory_covers_handover(self):
        plan = yaml.safe_load((ROOT / "config/upkie/stand_up_trajectory.yaml").read_text())["stand_up"]
        time = np.array(plan["time"])
        self.assertEqual(time[0], 0)
        self.assertTrue(np.all(np.isfinite(time)))
        self.assertTrue(np.all(np.diff(time) > 0))
        self.assertGreater(plan["handover_time"], 0)
        self.assertLessEqual(plan["handover_time"], time[-1])
        for field in ("position", "velocity", "effort"):
            data = np.array(plan[field])
            self.assertEqual(data.shape, (len(time), 6))
            self.assertTrue(np.all(np.isfinite(data)))

    def test_shipped_contact_aware_feedback_contract(self):
        plan = yaml.safe_load((ROOT / "config/upkie/stand_up_trajectory.yaml").read_text())["stand_up"]
        count = len(plan["time"])
        state = np.array(plan["state"])
        gains = np.array(plan["feedback"])
        self.assertEqual(state.shape, (count, 24))
        self.assertEqual(gains.shape, (count, 6 * 24))
        self.assertTrue(np.all(np.isfinite(state)))
        self.assertTrue(np.all(np.isfinite(gains)))
        np.testing.assert_array_equal(state[:, 6:12], plan["position"])
        np.testing.assert_array_equal(state[:, 18:24], plan["velocity"])
        self.assertGreater(plan["mpc_start_time"], 0)
        self.assertLess(plan["mpc_start_time"], plan["handover_time"])
        self.assertGreater(plan["minimum_knee_clearance"], 0)
        self.assertGreater(plan["clearance_hold_time"], 0)
        self.assertEqual({c["frame"] for c in plan["knee_contacts"]}, {"left_femur", "right_femur"})
        for contact in plan["knee_contacts"]:
            self.assertGreater(contact["radius"], 0)
            self.assertGreater(contact["half_width"], 0)
            self.assertEqual(len(contact["center"]), 3)

    @unittest.skipIf(mujoco is None, "Python MuJoCo is not installed")
    def test_export_includes_liftoff_between_regular_samples(self):
        spec = importlib.util.spec_from_file_location("upkie_stand_up", ROOT / "config/upkie/plan_stand_up.py")
        planner = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(planner)
        q, v, tau = np.arange(6) / 10, np.zeros(6), np.ones(6) / 10
        lift = dict(t=.0065, xi=.01, com_offset=.03, relative_velocity=-.1,
                    pitch=-.7, pitch_rate=-.6, com_height=.1, base_height=.09)
        rollout = SimpleNamespace(
            stand_up=lambda *args, **kwargs: (lift, .8, [(0., q, v, tau), (.005, q, v, tau)]),
            joints=lambda: (q + .01, v), d=SimpleNamespace(ctrl=tau), act=list(range(6)),
            spawn_idle=lambda **kwargs: .1346, slip=0., self_penetration=0., max_tilt=.7,
            spawn_penetration=0., spawn_tilt=.7)
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / "plan.yaml"
            planner.export(rollout, planner.PARAMETERS, output)
            plan = yaml.safe_load(output.read_text())["stand_up"]
        self.assertEqual(plan["time"], [0., .005, .0065])
        self.assertEqual(plan["handover_time"], plan["time"][-1])
        np.testing.assert_allclose(plan["position"][-1], q + .01)
        np.testing.assert_allclose(plan["effort"][-1], tau)

    @unittest.skipIf(mujoco is None, "Python MuJoCo is not installed")
    def test_com_velocity_is_refreshed_without_subtree_sensor(self):
        spec = importlib.util.spec_from_file_location("upkie_stand_up", ROOT / "config/upkie/plan_stand_up.py")
        planner = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(planner)
        rollout = planner.Rollout()
        rollout.spawn_idle(settle=0.)
        rollout.d.qvel[0] = .3
        # A rigid translation moves the CoM and tire at the same speed.
        _, _, relative_velocity = rollout.divergent_component()
        self.assertAlmostEqual(rollout.d.subtree_linvel[1, 0], .3, places=8)
        self.assertAlmostEqual(relative_velocity, 0., places=8)


if __name__ == "__main__":
    unittest.main()
