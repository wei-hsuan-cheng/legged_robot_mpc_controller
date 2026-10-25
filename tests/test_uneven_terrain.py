"""Seeded terrain geometry, bounds, reproducibility and format agreement."""

import copy
import importlib.util
import math
from pathlib import Path
import statistics
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "description/mujoco/terrains/generate_uneven_terrain.py"
spec = importlib.util.spec_from_file_location("generate_uneven_terrain", SCRIPT)
terrain = importlib.util.module_from_spec(spec)
spec.loader.exec_module(terrain)

try:
    import mujoco
    import numpy as np
except ImportError:
    mujoco = None


def defaults():
    return yaml.safe_load((ROOT / "config/terrains/uneven_terrain_params.yaml").read_text())["uneven_terrain"]


class UnevenTerrainTest(unittest.TestCase):
    def test_default_neighbor_distribution_and_envelope(self):
        config = defaults()
        heights = terrain.generate_heights(config)
        differences = terrain.adjacent_differences(heights)
        self.assertEqual([len(heights), len(heights[0])], config["grid"])
        self.assertGreaterEqual(min(differences), .02 - 1e-10)
        self.assertLessEqual(max(differences), .05 + 1e-10)
        self.assertGreater(statistics.pstdev(differences), .002)
        self.assertGreater(len({round(d, 4) for d in differences}), 30)
        self.assertGreaterEqual(min(map(min, heights)), config["height"]["min"] - 1e-10)
        self.assertLessEqual(max(map(max, heights)), config["height"]["max"] + 1e-10)

    def test_many_seeds_shapes_and_distributions(self):
        for distribution in ("triangular", "uniform"):
            for seed in range(30):
                with self.subTest(distribution=distribution, seed=seed):
                    config = defaults()
                    config["seed"] = seed
                    config["grid"] = [7, 11]
                    config["size"] = [4., 6.]
                    config["height"]["adjacent_difference"].update(min=.01, max=.04, mode=.025, distribution=distribution)
                    heights = terrain.generate_heights(config)
                    for difference in terrain.adjacent_differences(heights):
                        self.assertGreaterEqual(difference, .01 - 1e-10)
                        self.assertLessEqual(difference, .04 + 1e-10)
                    self.assertTrue(terrain.generate_blocks(config, heights))

    def test_seed_and_profile_control_the_result(self):
        config = defaults()
        first = terrain.generate_heights(config)
        self.assertEqual(first, terrain.generate_heights(copy.deepcopy(config)))
        config["seed"] += 1
        self.assertNotEqual(first, terrain.generate_heights(config))
        config = defaults()
        config["profile"]["strength"] = 0.
        self.assertNotEqual(first, terrain.generate_heights(config))
        config = defaults()
        config["profile"]["wavelength"] = [2., 3.]
        self.assertNotEqual(first, terrain.generate_heights(config))

    def test_clear_patch_is_exact_and_field_keeps_its_extent(self):
        config = defaults()
        config["clear_patch"] = {"center": [1., -.4], "size": [.7, .9]}
        blocks = terrain.generate_blocks(config, terrain.generate_heights(config))
        area = sum(b["size"][0] * b["size"][1] for b in blocks)
        self.assertAlmostEqual(area, math.prod(config["size"]) - .7 * .9)
        for axis in (0, 1):
            self.assertAlmostEqual(min(b["pos"][axis] - b["size"][axis] / 2 for b in blocks), -config["size"][axis] / 2)
            self.assertAlmostEqual(max(b["pos"][axis] + b["size"][axis] / 2 for b in blocks), config["size"][axis] / 2)
        cx, cy = config["clear_patch"]["center"]
        for b in blocks:
            overlap_x = min(b["pos"][0] + b["size"][0] / 2, cx + .35) - max(b["pos"][0] - b["size"][0] / 2, cx - .35)
            overlap_y = min(b["pos"][1] + b["size"][1] / 2, cy + .45) - max(b["pos"][1] - b["size"][1] / 2, cy - .45)
            self.assertTrue(overlap_x < 1e-10 or overlap_y < 1e-10)
        config["clear_patch"]["size"] = [0., 0.]
        config["gap"] = .01
        blocks = terrain.generate_blocks(config, terrain.generate_heights(config))
        self.assertEqual(len(blocks), math.prod(config["grid"]))
        self.assertAlmostEqual(blocks[0]["size"][0], config["size"][0] / config["grid"][0] - .01)

    def test_mjcf_and_urdf_agree_on_pose_geometry_and_material(self):
        config = defaults()
        config["base"] = {"pos": [2., -3., .1], "yaw": .7}
        blocks = terrain.generate_blocks(config, terrain.generate_heights(config))
        mjcf = terrain.build_mjcf(config, blocks)
        urdf = terrain.build_urdf(config, blocks)
        body = mjcf.find("./worldbody/body")
        self.assertEqual(body.get("pos"), "2 -3 0.1")
        self.assertEqual(urdf.find("./joint/origin").get("rpy"), "0 0 0.7")
        for block, geom, visual in zip(blocks, body.findall("geom"), urdf.findall("./link/visual")):
            self.assertEqual(geom.get("material"), "studio_terrain")
            self.assertEqual(geom.get("pos"), visual.find("origin").get("xyz"))
            for half, size in zip(map(float, geom.get("size").split()), map(float, visual.find("./geometry/box").get("size").split())):
                self.assertAlmostEqual(2 * half, size)
            self.assertAlmostEqual(block["pos"][2] - block["size"][2] / 2, -config["depth"])

    def test_generated_files_are_current_and_cli_is_relocatable(self):
        config = defaults()
        blocks = terrain.generate_blocks(config, terrain.generate_heights(config))
        for relative, root in (("description/mujoco/terrains/uneven_terrain.xml", terrain.build_mjcf(config, blocks)),
                               ("description/urdf/terrains/uneven_terrain.urdf", terrain.build_urdf(config, blocks))):
            self.assertEqual((ROOT / relative).read_text(), terrain.xml_text(root))
        self.assertFalse((SCRIPT.parent / "uneven_terrain_10x10.xml").exists())
        with tempfile.TemporaryDirectory() as tmp:
            command = [sys.executable, str(SCRIPT), "--mjcf", str(Path(tmp) / "field.xml"), "--urdf", str(Path(tmp) / "field.urdf")]
            subprocess.run(command, cwd=tmp, check=True, capture_output=True)
            subprocess.run([*command, "--check"], cwd=tmp, check=True, capture_output=True)

    def test_invalid_or_infeasible_settings_fail(self):
        invalid = [
            ("grid", [0, 20]), ("grid", [2.5, 20]), ("size", [0., 10.]), ("gap", .5),
            ("height.max", .03), ("height.jitter", .02), ("height.adjacent_difference.min", -.01),
            ("height.adjacent_difference.max", .01), ("height.adjacent_difference.distribution", "normal"),
            ("height.adjacent_difference.mode", .08), ("base.pos", [0., 0., -.01]),
            ("profile.strength", 2.), ("profile.amplitude", .5), ("profile.wavelength", [0., 1.]),
            ("clear_patch.size", [-1., 1.]), ("clear_patch.size", [20., 20.]), ("depth", 0.), ("seed", True),
        ]
        for path, value in invalid:
            with self.subTest(parameter=path):
                config = defaults()
                target = config
                keys = path.split(".")
                for key in keys[:-1]:
                    target = target[key]
                target[keys[-1]] = value
                with self.assertRaises(ValueError):
                    terrain.generate_heights(config)

    @unittest.skipIf(mujoco is None, "mujoco Python package is not installed")
    def test_compiled_center_yaw_and_rectangular_grid(self):
        config = defaults()
        config["base"] = {"pos": [2., -3., .1], "yaw": .7}
        config["size"], config["grid"] = [4., 6.], [7, 11]
        blocks = terrain.generate_blocks(config, terrain.generate_heights(config))
        root = terrain.build_mjcf(config, blocks)
        ET.SubElement(ET.SubElement(root, "asset"), "material", name="studio_terrain", rgba="0.5 0.5 0.5 1")
        model = mujoco.MjModel.from_xml_string(terrain.xml_text(root))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        c, s = math.cos(.7), math.sin(.7)
        rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        for block in blocks[::13]:
            geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, block["name"])
            np.testing.assert_allclose(data.geom_xpos[geom], np.array(config["base"]["pos"]) + rotation @ block["pos"], atol=1e-10)
            np.testing.assert_allclose(data.geom_xmat[geom].reshape(3, 3), rotation, atol=1e-10)

    @unittest.skipIf(mujoco is None, "mujoco Python package is not installed")
    def test_compiled_surface_matches_tile_tops_and_clear_floor(self):
        config = defaults()
        blocks = terrain.generate_blocks(config, terrain.generate_heights(config))
        model = mujoco.MjModel.from_xml_path(str(ROOT / "description/mujoco/robots/g1/scene_uneven.xml"))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        groups = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
        # Keep only terrain/floor in the ray group, excluding the robot at the origin.
        for geom_id in range(model.ngeom):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
            model.geom_group[geom_id] = 0 if name == "floor" or name.startswith("terrain_") else 1
        for block in blocks[::17]:
            x, y = block["pos"][:2]
            distance = mujoco.mj_ray(model, data, np.array([x, y, 1.]), np.array([0., 0., -1.]),
                                     groups, 1, -1, np.array([-1], dtype=np.int32))
            self.assertAlmostEqual(1. - distance, block["pos"][2] + block["size"][2] / 2)
        distance = mujoco.mj_ray(model, data, np.array([0., 0., 1.]), np.array([0., 0., -1.]),
                                 groups, 1, -1, np.array([-1], dtype=np.int32))
        self.assertAlmostEqual(distance, 1.)


if __name__ == "__main__":
    unittest.main()
