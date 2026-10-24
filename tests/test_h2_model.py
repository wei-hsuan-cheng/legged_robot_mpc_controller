"""H2 model/configuration checks, independent of a ROS installation."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
from unittest.mock import patch

import numpy as np
import yaml

from wb_id_parameter_contract_test import NAME, UniqueLoader, check_keys

try:
    import mujoco
except ImportError:
    mujoco = None

try:
    import xacro
except ImportError:
    xacro = None

ROOT = Path(__file__).resolve().parents[1]
CONFIG = yaml.load((ROOT / 'config/h2/ros2_controllers.yaml').read_text(), Loader=UniqueLoader)
PARAMS = CONFIG[NAME]['ros__parameters']
URDF = ET.parse(ROOT / 'description/urdf/robots/h2/h2_31dof.urdf').getroot()


class H2ModelTest(unittest.TestCase):
    def test_wb_id_only_and_schema_compatible(self):
        self.assertNotIn('humanoid_centroidal_mpc_controller', CONFIG)
        self.assertNotIn('humanoid_centroidal_mpc_controller', CONFIG['controller_manager']['ros__parameters'])
        schema = yaml.safe_load((ROOT / f'src/humanoid_whole_body_inverse_dynamics_mpc/{NAME}_parameter.yaml').read_text())[NAME]
        check_keys(PARAMS, schema)
        names = PARAMS['robot']['jointNames']
        self.assertEqual(len(names), 25)
        for estimator in ('humanoid_inekf', 'humanoid_linear_kf'):
            self.assertEqual(CONFIG[estimator]['ros__parameters']['jointNames'], names)
        for stage in ('q', 'qFinal'):
            for field in ('jointPositions', 'jointVelocities'):
                self.assertEqual(len(PARAMS['ocs2']['costs'][stage][field]), len(names))
        for field in ('jointAccelerations', 'jointTorques'):
            self.assertEqual(len(PARAMS['ocs2']['costs']['r'][field]), len(names))
        for field in ('kp', 'kd'):
            self.assertEqual(len(PARAMS['control']['jointEffortPd'][field]), len(names))

    def test_head_uses_existing_cost_and_both_joints_are_active(self):
        costs = PARAMS['ocs2']['costs']
        self.assertEqual(costs['taskSpaceCosts']['names'], ['head'])
        self.assertTrue(costs['taskSpaceCosts']['head']['activate'])
        self.assertEqual(costs['taskSpaceCosts']['head']['linkName'], 'head_yaw_link')
        self.assertTrue(costs['torsoTiltCost']['activate'])
        for name in ('head_pitch_joint', 'head_yaw_joint'):
            self.assertIn(name, PARAMS['robot']['jointNames'])

    def test_g1_cost_weights_are_copied_by_name(self):
        g1 = yaml.safe_load((ROOT / 'config/g1/ros2_controllers.yaml').read_text())[NAME]['ros__parameters']
        for section, field in [('ocs2.costs.q', 'jointPositions'), ('ocs2.costs.q', 'jointVelocities'),
                               ('ocs2.costs.qFinal', 'jointPositions'), ('ocs2.costs.qFinal', 'jointVelocities'),
                               ('ocs2.costs.r', 'jointAccelerations'), ('ocs2.costs.r', 'jointTorques')]:
            old, new = g1, PARAMS
            for part in section.split('.'):
                old, new = old[part], new[part]
            old = dict(zip(g1['robot']['jointNames'], old[field]))
            new = dict(zip(PARAMS['robot']['jointNames'], new[field]))
            for name in old.keys() & new.keys():
                self.assertEqual(old[name], new[name], (section, field, name))

    def test_meshes_and_actuator_limits_exist(self):
        prefix = 'package://legged_robot_mpc_controller/'
        for mesh in URDF.findall('.//mesh'):
            self.assertTrue((ROOT / mesh.attrib['filename'].removeprefix(prefix)).is_file())
        movable = [j for j in URDF.findall('joint') if j.attrib['type'] != 'fixed']
        self.assertEqual(len(movable), 31)
        for joint in movable:
            self.assertGreater(float(joint.find('limit').attrib['effort']), 0)
        self.assertEqual(PARAMS['ocs2']['model']['contactParentJointNames'],
                         ['left_ankle_pitch_joint', 'right_ankle_pitch_joint'])

    @unittest.skipIf(xacro is None, 'xacro is not installed')
    def test_h2_hardware_xacro_expands(self):
        with patch('xacro.substitution_args._eval_find', return_value=str(ROOT)):
            doc = xacro.process_file(str(ROOT / 'description/urdf/robots/h2/h2.ros2_control.xacro'), mappings={
                'initial_pose_file': str(ROOT / 'config/h2/initial_pose.yaml'),
            })
        robot = ET.fromstring(doc.toxml())
        joints = robot.findall('ros2_control/joint')
        self.assertEqual(len(joints), 31)
        self.assertIsNotNone(robot.find("link[@name='pelvis']"))
        self.assertIsNotNone(robot.find("link[@name='head_yaw_link']"))
        self.assertEqual(robot.find("ros2_control/sensor[@name='pelvis_imu']/param[@name='mujoco_quat_sensor_name']").text,
                         'imu_quat')

    def test_joint_order_and_hardware_limits(self):
        children = {}
        for joint in URDF.findall('joint'):
            children.setdefault(joint.find('parent').attrib['link'], []).append(joint)
        locked = set(PARAMS['ocs2']['model']['fixedJointNames'])
        active = []

        def visit(link):
            for joint in sorted(children.get(link, []), key=lambda j: j.attrib['name']):
                if joint.attrib['type'] != 'fixed' and joint.attrib['name'] not in locked:
                    active.append(joint.attrib['name'])
                visit(joint.find('child').attrib['link'])

        visit('pelvis')
        self.assertEqual(active, PARAMS['robot']['jointNames'])
        mjcf = ET.parse(ROOT / 'description/mujoco/robots/h2/h2_31dof.xml').getroot()
        motors = {m.attrib['joint']: m for m in mjcf.findall('actuator/motor')}
        macro = ET.parse(ROOT / 'description/urdf/robots/h2/h2.ros2_control_macro.xacro').getroot()
        hardware = {j.attrib['joint_name']: j for j in macro.iter()
                    if j.tag.endswith('}h2_joint_ros2_control')}
        for joint in URDF.findall('joint'):
            if joint.attrib['type'] == 'fixed':
                continue
            name = joint.attrib['name']
            effort = float(joint.find('limit').attrib['effort'])
            self.assertEqual(float(hardware[name].attrib['effort_limit']), effort)
            np.testing.assert_allclose(np.fromstring(motors[name].attrib['ctrlrange'], sep=' '),
                                       [-effort, effort])

    @unittest.skipIf(mujoco is None, 'mujoco is not installed')
    def test_mass_and_nominal_contact_geometry(self):
        model = mujoco.MjModel.from_xml_path(str(ROOT / 'description/mujoco/robots/h2/scene_flat.xml'))
        self.assertEqual(model.nu, 31)
        urdf_mass = sum(float(m.attrib['value']) for m in URDF.findall('.//inertial/mass'))
        self.assertAlmostEqual(float(sum(model.body_mass)), urdf_mass, places=4)
        data = mujoco.MjData(model)
        data.qpos[:] = model.qpos0
        data.qpos[2] = 1.02
        for name, q in zip(PARAMS['robot']['jointNames'], PARAMS['ocs2']['reference']['defaultJointState']):
            joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            data.qpos[model.jnt_qposadr[joint]] = q
        mujoco.mj_forward(model, data)
        offset = np.array(PARAMS['ocs2']['model']['contactGeometry']['frameTranslation'])
        for side, sign in [('left', 1), ('right', -1)]:
            body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, side + '_ankle_pitch_link')
            rotation = data.xmat[body].reshape(3, 3)
            np.testing.assert_allclose(data.xpos[body] + rotation @ offset,
                                       [data.subtree_com[1, 0], sign * .14, 0], atol=1e-6)
            np.testing.assert_allclose(rotation[:, 2], [0, 0, 1], atol=1e-6)


if __name__ == '__main__':
    unittest.main()
