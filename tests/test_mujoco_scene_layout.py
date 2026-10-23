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
ROBOT = ROOT / "description" / "robots" / "g1"
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
        launch = ast.parse((ROOT / "launch" / "g1.launch.py").read_text())
        checked = 0
        for node in ast.walk(launch):
            if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Name):
                continue
            if node.func.id != "PathJoinSubstitution" or not isinstance(node.args[0], ast.List):
                continue
            parts = [elem.value for elem in node.args[0].elts if isinstance(elem, ast.Constant)]
            if parts and parts[0] == "description":
                self.assertTrue(ROOT.joinpath(*parts).exists(), parts)
                checked += 1
        self.assertEqual(checked, 5)

    def test_include_and_asset_paths(self):
        for path in (ROOT / "description").rglob("*.xml"):
            with self.subTest(path=path):
                root = ET.parse(path)
                for include in root.findall(".//include"):
                    self.assertTrue((path.parent / include.get("file")).is_file())
                for texture in root.findall(".//texture[@file]"):
                    self.assertTrue((SCENES / texture.get("file")).is_file())
        robot = ET.parse(ROBOT / "mujoco" / "g1_29dof.xml")
        mesh_dir = SCENES / robot.find("compiler").get("meshdir")
        for mesh in robot.findall("./asset/mesh"):
            self.assertTrue((mesh_dir / mesh.get("file")).is_file())

    def test_urdf_mesh_uris(self):
        robot = ET.parse(ROBOT / "urdf" / "g1_29dof.urdf")
        prefix = "package://legged_robot_mpc_controller/"
        for mesh in robot.findall(".//mesh"):
            uri = mesh.get("filename")
            self.assertTrue(uri.startswith(prefix))
            self.assertTrue((ROOT / uri[len(prefix):]).is_file())

    def test_background_floor_physics_match(self):
        floors = [ET.parse(SCENES / "backgrounds" / f"background_{name}.xml")
                  .find("./worldbody/geom") for name in ("day", "night")]
        physics = lambda geom: {key: value for key, value in geom.attrib.items() if key != "material"}
        self.assertEqual(physics(floors[0]), physics(floors[1]))

    def test_none_removes_every_environment_module(self):
        for name, module in creator.TERRAIN_FILES.items():
            with self.subTest(terrain=name):
                source = creator.build_scene(str(SCENES / "scene.xml"),
                                             "backgrounds/background_night.xml", None, [name], "test")
                with tempfile.TemporaryDirectory() as tmp:
                    reference = Path(tmp) / "scene.xml"
                    source.write(reference)
                    result = creator.build_scene(str(reference), "backgrounds/background_day.xml",
                                                 None, ["none"], "test")
                includes = [elem.get("file") for elem in result.findall("include")]
                self.assertNotIn(module, includes)
                self.assertEqual(includes, ["../robots/g1/mujoco/g1_29dof.xml",
                                            "backgrounds/background_day.xml"])

    @unittest.skipIf(mujoco is None, "mujoco Python package is not installed")
    def test_all_scenes_and_generated_terrains_compile(self):
        for path in SCENES.glob("scene*.xml"):
            with self.subTest(scene=path.name):
                model = mujoco.MjModel.from_xml_path(str(path))
                self.assertEqual(model.opt.timestep, 0.0001)
                self.assertEqual(model.nu, 29)
        for background in ("day", "night"):
            for terrain in ["none", *creator.TERRAIN_FILES]:
                with self.subTest(background=background, terrain=terrain):
                    tree = creator.build_scene(str(SCENES / "scene.xml"),
                                              f"backgrounds/background_{background}.xml", None, [terrain], "test")
                    entry = str(SCENES / "layout_test.xml")
                    model = mujoco.MjModel.from_xml_path(entry, {entry: ET.tostring(tree.getroot())})
                    self.assertEqual(model.opt.timestep, 0.0001)

    @unittest.skipIf(mujoco is None, "mujoco Python package is not installed")
    def test_generator_cli_in_relocated_description(self):
        with tempfile.TemporaryDirectory() as tmp:
            description = Path(tmp) / "description"
            shutil.copytree(ROOT / "description", description)
            scenes = description / "mujoco"
            shared_background = scenes / "backgrounds" / "background_night.xml"
            before = shared_background.read_bytes()
            subprocess.run([sys.executable, str(scenes / "mujoco_scene_creator.py"),
                            "--scene-dir", str(scenes), "--terrain", "rough", "--reflectance", "0.02",
                            "--output", "scene_test.xml"], check=True, capture_output=True, text=True)
            self.assertEqual(before, shared_background.read_bytes())
            self.assertTrue((scenes / "backgrounds" / "background_night_scene_test.xml").is_file())
            model = mujoco.MjModel.from_xml_path(str(scenes / "scene_test.xml"))
            self.assertEqual(model.ngeom, 512)

    @unittest.skipIf(xacro is None, "xacro Python package is not installed")
    def test_stairs_xacros_find_shared_parameters(self):
        terrain = SCENES / "terrains"
        generated = ET.fromstring(xacro.process_file(str(terrain / "stairs.xml.xacro")).toxml())
        committed = ET.parse(terrain / "stairs.xml")
        self.assertEqual([elem.attrib for elem in generated.findall(".//geom")],
                         [elem.attrib for elem in committed.findall(".//geom")])
        rviz = ET.fromstring(xacro.process_file(str(terrain / "stairs.urdf.xacro")).toxml())
        self.assertEqual(len(rviz.findall("./link/visual")), len(generated.findall(".//geom")))

    @unittest.skipIf(xacro is None, "xacro Python package is not installed")
    def test_robot_control_xacro(self):
        # Replace package discovery, not xacro expansion, for a ROS-independent path check.
        with patch("xacro.substitution_args._eval_find", return_value=str(ROOT)):
            document = xacro.process_file(str(ROBOT / "urdf" / "g1.ros2_control.xacro"), mappings={
                "initial_pose_file": str(ROOT / "config" / "g1" / "initial_pose.yaml"),
            })
        robot = ET.fromstring(document.toxml())
        self.assertIsNotNone(robot.find("./link[@name='pelvis']"))
        self.assertIsNotNone(robot.find("./ros2_control/hardware/plugin"))


if __name__ == "__main__":
    unittest.main()
