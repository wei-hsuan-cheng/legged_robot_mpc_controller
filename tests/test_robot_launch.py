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


@unittest.skipIf(LaunchContext is None, 'ROS launch is not installed')
class RobotLaunchTest(unittest.TestCase):
    def resolve(self, robot, controller=WB_ID, **overrides):
        spec = importlib.util.spec_from_file_location('robot_launch', ROOT / 'launch/robot.launch.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        description = module.generate_launch_description()
        context = LaunchContext()
        context.launch_configurations.update(robot_name=robot, mpc_controller_name=controller, **overrides)
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
            config = yaml.safe_load(Path(values['controllers_params_file']).read_text())
            reference = config[WB_ID]['ros__parameters']['ocs2']['reference']
            self.assertEqual(reference['defaultBaseHeight'], height)
            self.assertEqual(float(values['gui_ref_base_height']), height)
            self.assertEqual(float(values['gui_min_base_height']), reference['minBaseHeight'])
            self.assertEqual(float(values['gui_max_base_height']), reference['maxBaseHeight'])
            self.assertAlmostEqual(reference['minBaseHeight'], height - .4)
            self.assertAlmostEqual(reference['maxBaseHeight'], height + .4)
            for mode, argument in (('stair_climbing', 'stair_climbing_file'), ('terrain_walking', 'terrain_walking_file')):
                self.assertEqual(Path(values[argument]).parts[-4:], ('config', 'terrains', mode, f'{robot}.yaml'))
                self.assertTrue(Path(values[argument]).is_file())
            for argument in ('urdf_file', 'controllers_file', 'initial_pose_file'):
                self.assertTrue(Path(values[argument]).is_file())
                self.assertIn(robot, Path(values[argument]).parts)
            self.assertEqual(values['lib_folder'], f'auto_generated/{robot}')
            self.assertEqual(values['mujoco_model_file'], 'scene.xml')
            self.assertEqual(values['ref_base_height'], '')
            self.assertEqual(float(values['gui_max_vx']), reference['maxDisplacementVelocityX'])

    def test_height_override_does_not_modify_the_source_yaml(self):
        for robot in ('g1', 'h2'):
            source = ROOT / 'config' / robot / 'ros2_controllers.yaml'
            before = source.read_bytes()
            context, description = self.resolve(robot, ref_base_height='0.9')
            values = context.launch_configurations
            resolved = yaml.safe_load(Path(values['controllers_params_file']).read_text())
            self.assertEqual(resolved[WB_ID]['ros__parameters']['ocs2']['reference']['defaultBaseHeight'], .9)
            self.assertEqual(float(values['gui_ref_base_height']), .9)
            self.assertNotEqual(Path(values['controllers_params_file']), source)
            self.assertEqual(source.read_bytes(), before)

    def test_upkie_gui_height_range_comes_from_yaml(self):
        context, _ = self.resolve('upkie')
        values = context.launch_configurations
        config = yaml.safe_load((ROOT / 'config/upkie/ros2_controllers.yaml').read_text())
        reference = config[WB_ID]['ros__parameters']['ocs2']['reference']
        self.assertEqual(float(values['gui_ref_base_height']), reference['defaultBaseHeight'])
        self.assertEqual((float(values['gui_min_base_height']), float(values['gui_max_base_height'])),
                         (reference['minBaseHeight'], reference['maxBaseHeight']))
        context, _ = self.resolve('upkie', ref_base_height='0.28')
        self.assertEqual(float(context.launch_configurations['gui_ref_base_height']), .28)
        self.assertEqual(float(context.launch_configurations['gui_max_base_height']), reference['maxBaseHeight'])
        with self.assertRaisesRegex(ValueError, 'outside'):
            self.resolve('upkie', ref_base_height='0.38')

    def test_command_frame_comes_from_yaml(self):
        context, _ = self.resolve('upkie')
        values = context.launch_configurations
        self.assertEqual(values['command_frame'], 'base_forward')
        self.assertAlmostEqual(float(values['command_frame_yaw']), 3.141592653589793)
        context, _ = self.resolve('h2')
        self.assertEqual(context.launch_configurations['command_frame'], '')

    def test_upkie_starts_kneeling_with_the_planned_stand_up(self):
        # Plain launch: the linear KF drives, the robot spawns kneeling and plays the planned stand-up.
        context, _ = self.resolve('upkie')
        values = context.launch_configurations
        self.assertEqual((values['floating_base_source'], values['floating_base_estimator']),
                         ('state_estimator', 'humanoid_linear_kf'))
        pose = yaml.safe_load(Path(values['initial_pose_file']).read_text())
        config = yaml.safe_load(Path(values['controllers_params_file']).read_text())
        # Kneeling: spawned below the standing height.
        self.assertLess(pose['upkie_base']['z'], config[WB_ID]['ros__parameters']['ocs2']['reference']['defaultBaseHeight'])
        stand_up = Path(config[WB_ID]['ros__parameters']['bringUp']['standUpTrajectoryFile'])
        self.assertEqual(stand_up.parts[-3:], ('config', 'upkie', 'stand_up_trajectory.yaml'))
        self.assertTrue(stand_up.is_file())

    def test_pineapple_starts_seated_with_the_planned_stand_up(self):
        # As Upkie: the linear KF drives, the robot spawns seated and plays the planned stand-up.
        context, _ = self.resolve('pineapple')
        values = context.launch_configurations
        self.assertEqual((values['floating_base_source'], values['floating_base_estimator']),
                         ('state_estimator', 'humanoid_linear_kf'))
        self.assertEqual(values['gt_body_frame'], 'base_link')
        self.assertEqual(values['command_frame'], '')
        self.assertEqual(values['posture_controller'], '/' + WB_ID)
        pose = yaml.safe_load(Path(values['initial_pose_file']).read_text())
        config = yaml.safe_load(Path(values['controllers_params_file']).read_text())[WB_ID]['ros__parameters']
        reference = config['ocs2']['reference']
        self.assertLess(pose['pineapple_base']['z'], reference['minBaseHeight'])
        stand_up = Path(config['bringUp']['standUpTrajectoryFile'])
        self.assertEqual(stand_up.parts[-3:], ('config', 'pineapple', 'stand_up_trajectory.yaml'))
        self.assertTrue(stand_up.is_file())
        plan = yaml.safe_load(stand_up.read_text())['stand_up']
        self.assertEqual(plan['joint_names'], config['robot']['jointNames'])
        table = yaml.safe_load(Path(reference['postureTableFile']).read_text())['posture_table']
        self.assertLessEqual(table['heights'][0], config['bringUp']['sitDownHeight'])
        self.assertEqual(table['joint_states'][table['heights'].index(reference['defaultBaseHeight'])],
                         reference['defaultJointState'])

    def test_scene_path_remains_unrestricted(self):
        for robot in ('g1', 'h2'):
            context, description = self.resolve(robot, mujoco_model_file='/tmp/my_scene.xml')
            self.assertEqual(context.launch_configurations['mujoco_model_file'], '/tmp/my_scene.xml')
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
        expand(ROOT / 'description/urdf/env.urdf.xacro')  # A flat environment can contain only world.
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
            context, description = self.resolve('h2', controllers_file=str(path))
            self.assertEqual(float(context.launch_configurations['gui_ref_base_height']), .98)

    def test_controller_selection_and_invalid_inputs(self):
        for robot in ('g1', 'h2', 'upkie', 'pineapple', 'pineapple_manipulator'):
            _, description = self.resolve(robot)
            argument = next(action for action in description.entities
                            if isinstance(action, DeclareLaunchArgument) and action.name == 'mpc_controller_name')
            self.assertEqual(list(argument.choices), [WB_ID])
        with self.assertRaisesRegex(ValueError, 'Mid-360'):
            self.resolve('h2', enable_lidar='true')
        for height in ('nan', 'inf', '0', '-1'):
            with self.assertRaisesRegex(ValueError, 'finite and positive'):
                self.resolve('h2', ref_base_height=height)

    def test_arguments_are_snake_case_and_old_names_are_rejected(self):
        _, description = self.resolve('upkie')
        names = [action.name for action in description.entities if isinstance(action, DeclareLaunchArgument)]
        self.assertEqual([name for name in names if name != name.lower()], [])
        # ros2 launch accepts undeclared names silently: robotName:=upkie would otherwise start G1.
        with self.assertRaisesRegex(ValueError, 'robotName:= -> robot_name:='):
            self.resolve('upkie', robotName='upkie')

    def test_model_error_file_is_a_name_under_config_or_a_path(self):
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('legged_robot_mpc_controller'))
        template = share / 'config/model_error/template.yaml'
        self.assertEqual(yaml.safe_load(template.read_text())['model_error'], {'seed': 0})  # as shipped: no error
        context, _ = self.resolve('upkie')
        self.assertEqual(context.launch_configurations['model_error_path'], '')  # the nominal plant
        context, _ = self.resolve('upkie', model_error_file='template.yaml')
        self.assertEqual(context.launch_configurations['model_error_path'], str(template))
        with tempfile.NamedTemporaryFile(suffix='.yaml') as stream:
            context, _ = self.resolve('upkie', model_error_file=stream.name)
            self.assertEqual(context.launch_configurations['model_error_path'], stream.name)
        with self.assertRaises(ValueError):
            self.resolve('upkie', model_error_file='missing.yaml')


if __name__ == '__main__':
    unittest.main()
