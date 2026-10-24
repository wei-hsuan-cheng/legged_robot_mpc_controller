"""Launch substitution tests; run inside the sourced ROS workspace, without starting nodes."""

import importlib.util
from pathlib import Path
import tempfile
import unittest

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
