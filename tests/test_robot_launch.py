"""Launch substitution tests; run inside the sourced ROS workspace, without starting nodes."""

import importlib.util
from pathlib import Path
import tempfile
import unittest
import subprocess
import xml.etree.ElementTree as ET

import yaml

try:
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, OpaqueFunction
except ImportError:
    LaunchContext = None


ROOT = Path(__file__).resolve().parents[1]
WB_ID = 'humanoid_whole_body_inverse_dynamics_mpc_controller'
CENTROIDAL = 'humanoid_centroidal_mpc_controller'


@unittest.skipIf(LaunchContext is None, 'ROS launch is not installed')
class RobotLaunchTest(unittest.TestCase):
    def resolve(self, robot, controller=WB_ID, **overrides):
        spec = importlib.util.spec_from_file_location('robot_launch', ROOT / 'launch/robot.launch.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        description = module.generate_launch_description()
        context = LaunchContext()
        context.launch_configurations.update(robotName=robot, mpcControllerName=controller, **overrides)
        for action in description.entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
        for action in description.entities:
            if isinstance(action, OpaqueFunction):
                for result in action.execute(context):
                    result.execute(context)
        return context, description

    def test_robot_defaults_and_yaml_height_are_shared_with_gui(self):
        for robot, height in (('g1', .795), ('h2', 1.02)):
            context, description = self.resolve(robot)
            values = context.launch_configurations
            config = yaml.safe_load(Path(values['controllersParamsFile']).read_text())
            reference = config[WB_ID]['ros__parameters']['ocs2']['reference']
            self.assertEqual(reference['defaultBaseHeight'], height)
            self.assertEqual(float(values['guiRefBaseHeight']), height)
            for mode, argument in (('stair_climbing', 'stairClimbingFile'), ('terrain_walking', 'terrainWalkingFile')):
                self.assertEqual(Path(values[argument]).parts[-4:], ('config', 'terrains', mode, f'{robot}.yaml'))
                self.assertTrue(Path(values[argument]).is_file())
            for argument in ('urdfFile', 'controllersFile', 'initialPoseFile'):
                self.assertTrue(Path(values[argument]).is_file())
                self.assertIn(robot, Path(values[argument]).parts)
            self.assertEqual(values['libFolder'], f'auto_generated/{robot}')
            self.assertEqual(values['mujocoModelFile'], 'scene.xml')
            self.assertEqual(values['refBaseHeight'], '')
            self.assertEqual(float(values['guiMaxVx']), reference['maxDisplacementVelocityX'])

    def test_height_override_does_not_modify_the_source_yaml(self):
        for robot in ('g1', 'h2'):
            source = ROOT / 'config' / robot / 'ros2_controllers.yaml'
            before = source.read_bytes()
            context, description = self.resolve(robot, refBaseHeight='0.9')
            values = context.launch_configurations
            resolved = yaml.safe_load(Path(values['controllersParamsFile']).read_text())
            self.assertEqual(resolved[WB_ID]['ros__parameters']['ocs2']['reference']['defaultBaseHeight'], .9)
            self.assertEqual(float(values['guiRefBaseHeight']), .9)
            self.assertNotEqual(Path(values['controllersParamsFile']), source)
            self.assertEqual(source.read_bytes(), before)

    def test_scene_path_remains_unrestricted(self):
        for robot in ('g1', 'h2'):
            context, description = self.resolve(robot, mujocoModelFile='/tmp/my_scene.xml')
            self.assertEqual(context.launch_configurations['mujocoModelFile'], '/tmp/my_scene.xml')
            self.assertFalse(any(isinstance(action, DeclareLaunchArgument) and action.name == 'environmentDescription'
                                 for action in description.entities))

    def test_environment_expands_and_every_piece_combines(self):
        def expand(path):
            root = ET.fromstring(subprocess.check_output(['xacro', str(path)], text=True))
            self.assertEqual(root.tag, 'robot')
            links = [link.get('name') for link in root.findall('./link')]
            self.assertEqual(len(links), len(set(links)))
            self.assertEqual(links.count('world'), 1)
            return root
        self.assertGreater(len(expand(ROOT / 'description/urdf/env.urdf.xacro').findall('./link/visual')), 0)
        pieces = ['terrains/stairs.urdf.xacro', 'terrains/ramp.urdf.xacro', 'terrains/uneven_terrain.urdf.xacro',
                  'objects/boxes.urdf.xacro']
        includes = ''.join(f'<xacro:include filename="{ROOT / "description/urdf" / piece}"/>' for piece in pieces)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'env.urdf.xacro'
            path.write_text('<robot name="environment" xmlns:xacro="http://www.ros.org/wiki/xacro">'
                            f'<link name="world"/>{includes}</robot>')
            root = expand(path)
        roots = {joint.find('parent').get('link') for joint in root.findall('./joint')}
        for link in ('stairs_base', 'ramp_base', 'uneven_terrain', 'purple_box'):
            self.assertIn(link, [element.get('name') for element in root.findall('./link')])
        self.assertIn('world', roots)

    def test_custom_controller_yaml_supplies_gui_height(self):
        config = yaml.safe_load((ROOT / 'config/h2/ros2_controllers.yaml').read_text())
        config[WB_ID]['ros__parameters']['ocs2']['reference']['defaultBaseHeight'] = .98
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'controllers.yaml'
            path.write_text(yaml.safe_dump(config, sort_keys=False))
            context, description = self.resolve('h2', controllersFile=str(path))
            self.assertEqual(float(context.launch_configurations['guiRefBaseHeight']), .98)

    def test_controller_selection_and_invalid_inputs(self):
        context, description = self.resolve('g1', CENTROIDAL)
        self.assertEqual(float(context.launch_configurations['guiRefBaseHeight']), .795)
        with self.assertRaisesRegex(ValueError, 'not configured'):
            self.resolve('h2', CENTROIDAL)
        with self.assertRaisesRegex(ValueError, 'Mid-360'):
            self.resolve('h2', enableLidar='true')
        for height in ('nan', 'inf', '0', '-1'):
            with self.assertRaisesRegex(ValueError, 'finite and positive'):
                self.resolve('h2', refBaseHeight=height)


if __name__ == '__main__':
    unittest.main()
