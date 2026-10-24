"""Scene export and reference-math checks; not a closed-loop terrain walking test."""

import importlib.util
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SCENES = ROOT / "description/mujoco"
spec = importlib.util.spec_from_file_location("terrain_scene_model", ROOT / "launch/terrain_scene_model.py")
terrain = importlib.util.module_from_spec(spec)
spec.loader.exec_module(terrain)

try:
    import mujoco
except ImportError:
    mujoco = None


def surface_height(model, xy):
    height = model["ground_height"]
    for region in model["regions"]:
        delta = np.asarray(xy) - region["center"]
        c, s = math.cos(region["yaw"]), math.sin(region["yaw"])
        local = np.array([[c, s], [-s, c]]) @ delta
        if np.all(np.abs(local) <= np.asarray(region["half_extents"]) + 1e-6):
            n = region["normal"]
            height = max(height, region["height"] - np.dot(n[:2], delta) / n[2])
    return height


class TerrainReferenceTest(unittest.TestCase):
    def test_flat_scene_has_no_phantom_staircase(self):
        self.assertEqual(terrain.scene_regions(SCENES / "scene_flat.xml")["regions"], [])

    def test_ramp_has_matching_height_and_upward_normal(self):
        model = terrain.scene_regions(SCENES / "scene_ramp.xml")
        self.assertEqual(len(model["regions"]), 1)
        ramp = model["regions"][0]
        self.assertAlmostEqual(np.linalg.norm(ramp["normal"]), 1)
        self.assertLess(ramp["normal"][0], 0)
        self.assertGreater(surface_height(model, [4, 0]), surface_height(model, [2, 0]))
        self.assertEqual(surface_height(model, [0, 0]), 0)

    def test_uneven_field_uses_box_tops_and_background_floor(self):
        with tempfile.TemporaryDirectory() as directory:
            scene = Path(directory) / "scene.xml"
            scene.write_text('<mujoco><worldbody><geom type="plane" size="0 0 1"/></worldbody>'
                             f'<include file="{SCENES / "terrains/uneven_terrain_10x10.xml"}"/></mujoco>')
            model = terrain.scene_regions(scene)
        self.assertEqual(len(model["regions"]), 440)
        self.assertEqual(surface_height(model, [0, 0]), 0)
        for region in model["regions"]:
            np.testing.assert_allclose(region["normal"], [0, 0, 1])
            self.assertAlmostEqual(surface_height(model, region["center"]), max(0, region["height"]))

    @unittest.skipIf(mujoco is None, "MuJoCo Python package not installed")
    def test_exported_ramp_matches_compiled_mujoco_and_rays(self):
        scene = SCENES / "scene_ramp.xml"
        model = mujoco.MjModel.from_xml_path(str(scene))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        exported = terrain.scene_regions(scene)
        index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ramp_surface")
        rotation = data.geom_xmat[index].reshape(3, 3)
        top = data.geom_xpos[index] + rotation[:, 2] * model.geom_size[index, 2]
        np.testing.assert_allclose(exported["regions"][0]["normal"], rotation[:, 2], atol=1e-9)
        self.assertAlmostEqual(exported["regions"][0]["height"], top[2])
        for x in (1.7, 2, 3, 4, 5.3):
            distance = mujoco.mj_ray(model, data, np.array([x, 0., 2.]), np.array([0., 0., -1.]),
                                     None, 1, -1, np.array([-1], dtype=np.int32))
            self.assertGreater(distance, 0)
            self.assertAlmostEqual(surface_height(exported, [x, 0]), 2 - distance, places=7)

    def test_dynamic_geometry_is_excluded_and_unsupported_static_geometry_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            scene = Path(directory) / "scene.xml"
            scene.write_text('<mujoco><worldbody><geom type="plane"/>'
                             '<body><freejoint/><geom type="sphere"/></body></worldbody></mujoco>')
            self.assertEqual(terrain.scene_regions(scene)["regions"], [])
            scene.write_text('<mujoco><worldbody><geom type="sphere"/></worldbody></mujoco>')
            with self.assertRaises(ValueError):
                terrain.scene_regions(scene)

    def test_smooth_support_velocity_matches_height_derivative(self):
        duration, height = 0.6, 0.05
        def position(t):
            s = np.clip(t / duration, 0, 1)
            return 0.5 * height * s*s * (3 - 2*s)
        for t in (0.05, 0.2, 0.3, 0.55):
            s = t / duration
            velocity = 0.5 * height * 6*s*(1-s) / duration
            numeric = (position(t+1e-6) - position(t-1e-6)) / 2e-6
            self.assertAlmostEqual(velocity, numeric, places=8)

    def test_normal_and_cache_wiring(self):
        constraint = (ROOT / "src/core/humanoid_wb_id_mpc/src/constraint/WbIdFootAccelerationConstraintCppAd.cpp").read_text()
        self.assertIn("getContactNormal(contactIndex_, time)", constraint)
        self.assertIn("p.segment<3>(11)", constraint)
        self.assertIn("_wb_id_stance_acceleration_v2", constraint)
        friction = (ROOT / "src/core/humanoid_common_mpc/src/constraint/FrictionForceConeConstraint.cpp").read_text()
        self.assertEqual(friction.count("computeLocalForceDerivatives(rotation)"), 2)
        self.assertIn("getContactNormal(contactPointIndex_, time)", friction)
        self.assertNotIn("is not implemented", friction)
        reference = (ROOT / "src/core/humanoid_common_mpc/src/reference_manager/SwitchedModelReferenceManager.cpp").read_text()
        self.assertIn("getAnticipatedSupportVelocity(knotTime)(2)", reference)
        self.assertIn("originalTargets.getDesiredState(time)", reference)


if __name__ == "__main__":
    unittest.main()
