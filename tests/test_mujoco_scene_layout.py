"""Scene composition checks; optional MuJoCo/xacro checks need their Python packages."""

import ast
import importlib.util
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SCENES = ROOT / "description" / "mujoco"
G1_SCENES = SCENES / "robots" / "g1"
ROBOT = ROOT / "description" / "urdf" / "robots" / "g1"
spec = importlib.util.spec_from_file_location("mujoco_scene_creator", SCENES / "mujoco_scene_creator.py")
creator = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = creator
spec.loader.exec_module(creator)

try:
    import mujoco
except ImportError:
    mujoco = None

try:
    import xacro
    import xacro.substitution_args
except ImportError:
    xacro = None


class SceneLayoutTest(unittest.TestCase):
    def test_launch_description_paths(self):
        launch = (ROOT / "launch/robot.launch.py").read_text()
        ast.parse(launch)
        self.assertIn('"robots", robot_name', launch)
        for robot in ("g1", "h2"):
            self.assertTrue((SCENES / "robots" / robot / "scene.xml").is_file())
            self.assertTrue((ROOT / "config" / robot / "ros2_controllers.yaml").is_file())
            self.assertFalse((ROOT / "launch" / f"{robot}.launch.py").exists())

    def test_include_and_asset_paths(self):
        for path in (ROOT / "description").rglob("*.xml"):
            with self.subTest(path=path):
                root = ET.parse(path)
                for include in root.findall(".//include"):
                    self.assertTrue((path.parent / include.get("file")).is_file())
                for texture in root.findall(".//texture[@file]"):
                    self.assertTrue((SCENES / texture.get("file")).is_file())
        for name in ("g1", "h2"):
            directory = SCENES / "robots" / name
            robot = ET.parse(directory / ("g1_29dof.xml" if name == "g1" else "h2_31dof.xml"))
            mesh_dir = directory / robot.find("compiler").get("meshdir")
            for mesh in robot.findall("./asset/mesh"):
                self.assertTrue((mesh_dir / mesh.get("file")).is_file())

    def test_urdf_mesh_uris(self):
        robot = ET.parse(ROBOT / "g1_29dof.urdf")
        prefix = "package://legged_robot_mpc_controller/"
        for mesh in robot.findall(".//mesh"):
            uri = mesh.get("filename")
            self.assertTrue(uri.startswith(prefix))
            self.assertTrue((ROOT / uri[len(prefix):]).is_file())

    def test_background_floor_physics_match(self):
        floors = [ET.parse(SCENES / "common" / "backgrounds" / f"background_{name}.xml")
                  .find("./worldbody/geom") for name in ("day", "night")]
        physics = lambda geom: {key: value for key, value in geom.attrib.items() if key != "material"}
        self.assertEqual(physics(floors[0]), physics(floors[1]))

    def test_none_removes_every_environment_module(self):
        for name, module in creator.TERRAIN_FILES.items():
            with self.subTest(terrain=name):
                source = creator.build_scene(str(G1_SCENES / "scene.xml"),
                                             "common/backgrounds/background_night.xml", None, [name], "test")
                with tempfile.TemporaryDirectory() as tmp:
                    reference = Path(tmp) / "scene.xml"
                    source.write(reference)
                    result = creator.build_scene(str(reference), "common/backgrounds/background_day.xml",
                                                 None, ["none"], "test")
                includes = [elem.get("file") for elem in result.findall("include")]
                self.assertNotIn(Path(module).name, [Path(p).name for p in includes])
                self.assertEqual(includes[0], "g1_29dof.xml")
                self.assertEqual((reference.parent / includes[1]).resolve(),
                                 SCENES / "common/backgrounds/background_day.xml")

    @unittest.skipIf(mujoco is None, "mujoco Python package is not installed")
    def test_all_scenes_and_generated_terrains_compile(self):
        scenes = list(SCENES.glob("robots/*/scene*.xml"))
        self.assertEqual(len(scenes), 8)
        for path in scenes:
            with self.subTest(scene=path.name):
                model = mujoco.MjModel.from_xml_path(str(path))
                self.assertEqual(model.opt.timestep, 0.0001)
                self.assertEqual(model.nu, 31 if path.parent.name == "h2" else 29)
        for background in ("day", "night"):
            for terrain in ["none", *creator.TERRAIN_FILES]:
                with self.subTest(background=background, terrain=terrain):
                    tree = creator.build_scene(str(G1_SCENES / "scene.xml"),
                                              f"common/backgrounds/background_{background}.xml", None, [terrain], "test")
                    entry = str(G1_SCENES / "layout_test.xml")
                    model = mujoco.MjModel.from_xml_path(entry, {entry: ET.tostring(tree.getroot())})
                    self.assertEqual(model.opt.timestep, 0.0001)

    @unittest.skipIf(mujoco is None, "mujoco Python package is not installed")
    def test_generator_cli_in_relocated_description(self):
        with tempfile.TemporaryDirectory() as tmp:
            description = Path(tmp) / "description"
            shutil.copytree(ROOT / "description", description)
            scenes = description / "mujoco"
            shared_background = scenes / "common" / "backgrounds" / "background_night.xml"
            before = shared_background.read_bytes()
            for robot in ("g1", "h2"):
                subprocess.run([sys.executable, str(scenes / "mujoco_scene_creator.py"),
                                "--scene-dir", str(scenes), "--robot-name", robot,
                                "--terrain", "rough", "--reflectance", "0.02",
                                "--output", "scene_test.xml"], check=True, capture_output=True, text=True)
                self.assertEqual(before, shared_background.read_bytes())
                self.assertTrue((scenes / "common" / "backgrounds" / "background_night_scene_test.xml").is_file())
                model = mujoco.MjModel.from_xml_path(str(scenes / "robots" / robot / "scene_test.xml"))
                flat = mujoco.MjModel.from_xml_path(str(scenes / "robots" / robot / "scene_flat.xml"))
                rough_count = len(ET.parse(scenes / "terrains/uneven_terrain.xml").findall(".//geom"))
                self.assertEqual(model.ngeom, flat.ngeom + rough_count)

    @unittest.skipIf(xacro is None, "xacro Python package is not installed")
    def test_stairs_xacros_find_shared_parameters(self):
        terrain = SCENES / "terrains"
        generated = ET.fromstring(xacro.process_file(str(terrain / "stairs.xml.xacro")).toxml())
        committed = ET.parse(terrain / "stairs.xml")
        self.assertEqual([elem.attrib for elem in generated.findall(".//geom")],
                         [elem.attrib for elem in committed.findall(".//geom")])
        rviz = ET.fromstring(xacro.process_file(str(ROOT / "description/urdf/terrains/stairs.urdf.xacro")).toxml())
        self.assertEqual(len(rviz.findall("./link/visual")), len(generated.findall(".//geom")))

    @unittest.skipIf(xacro is None, "xacro Python package is not installed")
    def test_ramp_and_boxes_xacros_find_shared_parameters(self):
        # Committed MJCF must be the current xacro output, like stairs.xml.
        for module, tag in (("terrains/ramp", "mesh"), ("objects/boxes", "body")):
            generated = ET.fromstring(xacro.process_file(str(SCENES / f"{module}.xml.xacro")).toxml())
            committed = ET.parse(SCENES / f"{module}.xml")
            for kind in (tag, "geom"):
                self.assertEqual([elem.attrib for elem in generated.iter(kind)],
                                 [elem.attrib for elem in committed.iter(kind)], module)
        ramp = ET.fromstring(xacro.process_file(str(ROOT / "description/urdf/terrains/ramp.urdf.xacro")).toxml())
        mesh = ramp.find("./link/visual/geometry/mesh")
        self.assertTrue(mesh.get("filename").endswith("description/meshes/terrains/unit_wedge.stl"))
        self.assertTrue((ROOT / "description/meshes/terrains/unit_wedge.stl").is_file())
        boxes = ET.fromstring(xacro.process_file(str(ROOT / "description/urdf/objects/boxes.urdf.xacro")).toxml())
        self.assertEqual(len(boxes.findall("./link/visual")), len(committed.findall(".//body")))

    @unittest.skipIf(mujoco is None, "MuJoCo Python package is not installed")
    def test_ramp_is_a_wedge_from_its_parameters(self):
        import math
        import numpy as np
        import yaml
        params = yaml.safe_load((ROOT / "config/terrains/ramp_params.yaml").read_text())["ramp"]
        model = mujoco.MjModel.from_xml_path(str(G1_SCENES / "scene_ramp.xml"))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        rise = params["length"] * math.tan(math.radians(params["incline_deg"]))
        geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ramp_surface")
        corners = data.geom_xpos[geom] + model.mesh_vert[model.mesh_vertadr[model.geom_dataid[geom]]:][
            :model.mesh_vertnum[model.geom_dataid[geom]]] @ data.geom_xmat[geom].reshape(3, 3).T
        np.testing.assert_allclose(corners[:, 2].min(), params["base"]["pos"][2], atol=1e-6)  # MuJoCo recentres mesh vertices
        np.testing.assert_allclose(corners[:, 2].max(), params["base"]["pos"][2] + rise, atol=1e-6)  # MuJoCo recentres mesh vertices
        np.testing.assert_allclose(np.ptp(corners[:, 0]), params["length"], atol=1e-6)  # MuJoCo recentres mesh vertices
        np.testing.assert_allclose(np.ptp(corners[:, 1]), params["width"], atol=1e-6)  # MuJoCo recentres mesh vertices

    @unittest.skipIf(xacro is None, "xacro Python package is not installed")
    def test_robot_control_xacro(self):
        # Replace package discovery, not xacro expansion, for a ROS-independent path check.
        with patch("xacro.substitution_args._eval_find", return_value=str(ROOT)):
            document = xacro.process_file(str(ROBOT / "g1.ros2_control.xacro"), mappings={
                "initial_pose_file": str(ROOT / "config" / "g1" / "initial_pose.yaml"),
            })
        robot = ET.fromstring(document.toxml())
        self.assertIsNotNone(robot.find("./link[@name='pelvis']"))
        self.assertIsNotNone(robot.find("./ros2_control/hardware/plugin"))

    def test_format_separation(self):
        description = ROOT / "description"
        self.assertEqual({p.name for p in description.iterdir() if p.is_dir()}, {"meshes", "mujoco", "urdf"})
        self.assertFalse(list(SCENES.rglob("*.urdf")))
        self.assertFalse(list(SCENES.rglob("*.urdf.xacro")))
        self.assertFalse(list(SCENES.rglob("*.yaml")))
        for path in description.rglob("*.STL"):
            self.assertTrue(path.is_relative_to(description / "meshes"))

    @unittest.skipIf(xacro is None, "xacro Python package is not installed")
    def test_shared_stairs_parameters_in_relocated_install(self):
        with tempfile.TemporaryDirectory() as tmp:
            package = Path(tmp)
            shutil.copytree(ROOT / "description", package / "description")
            shutil.copytree(ROOT / "config", package / "config")
            mjcf = xacro.process_file(str(package / "description/mujoco/terrains/stairs.xml.xacro"))
            urdf = xacro.process_file(str(package / "description/urdf/terrains/stairs.urdf.xacro"))
            self.assertEqual(len(ET.fromstring(mjcf.toxml()).findall(".//geom")),
                             len(ET.fromstring(urdf.toxml()).findall("./link/visual")))
            for module in ("terrains/ramp", "objects/boxes"):
                mjcf = xacro.process_file(str(package / f"description/mujoco/{module}.xml.xacro"))
                urdf = xacro.process_file(str(package / f"description/urdf/{module}.urdf.xacro"))
                self.assertEqual(len(ET.fromstring(mjcf.toxml()).findall(".//geom")),
                                 len(ET.fromstring(urdf.toxml()).findall("./link/visual")), module)


if __name__ == "__main__":
    unittest.main()
