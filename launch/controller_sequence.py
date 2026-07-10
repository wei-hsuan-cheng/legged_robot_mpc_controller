import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import (
    LoadController,
    ConfigureController,
    SwitchController,
    ListControllers,
)


def response_error_string(result):
    if result is None:
        return ''
    return getattr(result, 'error_string', '')

class ControllerSequencer(Node):
    def __init__(self, controller_manager, robot_description_topic, controller_name, timeout_sec, use_sim_time, configure_timeout_sec=3600.0):
        super().__init__(
            'controller_sequencer',
            parameter_overrides=[Parameter('use_sim_time', value=use_sim_time)],
            automatically_declare_parameters_from_overrides=True,
        )
        self.controller_manager = controller_manager
        self.robot_description_topic = robot_description_topic
        self.controller_name = controller_name
        self.timeout_sec = timeout_sec
        self.configure_timeout_sec = configure_timeout_sec
        self.robot_description_msg = None

        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, robot_description_topic, self._desc_cb, qos)

        self.load_cli = self.create_client(LoadController, f'{controller_manager}/load_controller')
        self.config_cli = self.create_client(ConfigureController, f'{controller_manager}/configure_controller')
        self.switch_cli = self.create_client(SwitchController, f'{controller_manager}/switch_controller')
        self.list_cli = self.create_client(ListControllers, f'{controller_manager}/list_controllers')
        self.mujoco_start_cli = self.create_client(Trigger, '/mujoco_ros2_control/start')

    def _desc_cb(self, msg):
        self.robot_description_msg = msg

    def wait_for_service(self, client, name):
        if not client.wait_for_service(timeout_sec=self.timeout_sec):
            self.get_logger().error(f'Timeout waiting for {name}')
            return False
        return True

    def list_controllers(self):
        req = ListControllers.Request()
        future = self.list_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout_sec)
        result = future.result()
        if result is None:
            self.get_logger().error('Failed to list controllers')
            return {}
        return {ctrl.name: ctrl.state for ctrl in result.controller}

    def ensure_loaded(self, name):
        controllers = self.list_controllers()
        if name in controllers:
            return True

        req = LoadController.Request()
        req.name = name
        future = self.load_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout_sec)
        result = future.result()
        if result is None or not result.ok:
            err = response_error_string(result)
            self.get_logger().error(f'Failed to load {name}: {err}')
            return False
        return True

    def ensure_configured(self, name):
        controllers = self.list_controllers()
        state = controllers.get(name)
        if state in (None, 'unconfigured'):
            req = ConfigureController.Request()
            req.name = name
            future = self.config_cli.call_async(req)
            # Configuring the MPC controller runs CppAD code generation, which can take
            # many minutes on the first run (and on every run with recompileLibraries=true).
            # Wait patiently with heartbeat logs instead of the short service timeout.
            deadline = time.time() + self.configure_timeout_sec
            while rclpy.ok() and not future.done() and time.time() < deadline:
                rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
                if not future.done():
                    elapsed = int(self.configure_timeout_sec - (deadline - time.time()))
                    self.get_logger().info(
                        f'Still configuring {name} ({elapsed}s elapsed; CppAD codegen may be running)...')
            result = future.result() if future.done() else None
            if result is None or not result.ok:
                err = response_error_string(result)
                self.get_logger().error(f'Failed to configure {name}: {err}')
                return False
        return True

    def activate(self, name):
        controllers = self.list_controllers()
        if controllers.get(name) == 'active':
            return True

        req = SwitchController.Request()
        req.activate_controllers = [name]
        req.deactivate_controllers = []
        if hasattr(req, 'start_controllers'):
            req.start_controllers = []
        if hasattr(req, 'stop_controllers'):
            req.stop_controllers = []
        if hasattr(req, 'start_asap'):
            req.start_asap = False
        if hasattr(req, 'activate_asap'):
            req.activate_asap = False
        if hasattr(req, 'timeout'):
            req.timeout = Duration(sec=int(self.timeout_sec))
        if hasattr(SwitchController.Request, 'BEST_EFFORT'):
            req.strictness = SwitchController.Request.BEST_EFFORT
        else:
            req.strictness = 0

        future = self.switch_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout_sec)
        result = future.result()
        if result is None or not result.ok:
            self.get_logger().error(f'Failed to activate {name}')
            return False
        return True

    def wait_for_robot_description(self):
        deadline = time.time() + self.timeout_sec
        while self.robot_description_msg is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.robot_description_msg is None:
            self.get_logger().error(f'Timeout waiting for {self.robot_description_topic}')
            return False
        return True

    def republish_robot_description(self):
        if self.robot_description_msg is None:
            return
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        pub = self.create_publisher(String, self.robot_description_topic, qos)
        pub.publish(self.robot_description_msg)
        rclpy.spin_once(self, timeout_sec=0.1)

    def start_mujoco(self):
        if not self.mujoco_start_cli.wait_for_service(timeout_sec=self.timeout_sec):
            self.get_logger().error('Timeout waiting for /mujoco_ros2_control/start')
            return False
        future = self.mujoco_start_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout_sec)
        result = future.result()
        if result is None or not result.success:
            self.get_logger().error('Failed to start MuJoCo simulation')
            return False
        self.get_logger().info(result.message)
        return True

def wait_for_controller_manager(node, manager):
    # Ensure all controller_manager services are online before sequencing.
    if not node.wait_for_service(node.load_cli, f'{manager}/load_controller'):
        return False
    if not node.wait_for_service(node.config_cli, f'{manager}/configure_controller'):
        return False
    if not node.wait_for_service(node.switch_cli, f'{manager}/switch_controller'):
        return False
    if not node.wait_for_service(node.list_cli, f'{manager}/list_controllers'):
        return False
    return True

def start_joint_state_broadcaster(node):
    # Guarantee robot_description is available before configure to avoid executor errors.
    if not node.ensure_loaded('joint_state_broadcaster'):
        return False
    node.republish_robot_description()
    time.sleep(0.2)
    if not node.ensure_configured('joint_state_broadcaster'):
        return False
    if not node.activate('joint_state_broadcaster'):
        return False
    return True

def start_robot_state_broadcaster(node):
    # Guarantee robot_description is available before configure to avoid executor errors.
    if not node.ensure_loaded('robot_state_broadcaster'):
        return False
    node.republish_robot_description()
    time.sleep(0.2)
    if not node.ensure_configured('robot_state_broadcaster'):
        return False
    if not node.activate('robot_state_broadcaster'):
        return False
    return True

def start_ros2_controllers(node, controller_list, activate=True):
    for name in controller_list:
        if not node.ensure_loaded(name):
            return False
        if not node.ensure_configured(name):
            return False
        if activate and not node.activate(name):
            return False
    return True

def prepare_servo_controller(node):
    # Servo stays inactive by design; only load+configure here.
    if not node.ensure_loaded('servo_controller'):
        return False
    if not node.ensure_configured('servo_controller'):
        return False
    return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--controller-manager', default='/controller_manager')
    parser.add_argument('--robot-description-topic', default='/robot_description')
    parser.add_argument('--controller-name', default='humanoid_wb_mpc_controller')
    parser.add_argument('--timeout', type=float, default=120.0)
    parser.add_argument(
        '--configure-timeout', type=float, default=3600.0,
        help='Timeout for controller configure calls; the MPC controller runs CppAD codegen here.')
    parser.add_argument(
        '--use-mujoco-sim',
        action='store_true',
        help='Adjust startup sequencing for MuJoCo-backed ros2_control.',
    )
    parser.add_argument(
        '--use-sim-time',
        action='store_true',
        help='Set use_sim_time:=true for /clock-based simulation.',
    )
    args = parser.parse_args()

    rclpy.init()
    node = ControllerSequencer(
        controller_manager=args.controller_manager,
        robot_description_topic=args.robot_description_topic,
        controller_name=args.controller_name,
        timeout_sec=args.timeout,
        use_sim_time=args.use_sim_time,
        configure_timeout_sec=args.configure_timeout,
    )

    if not wait_for_controller_manager(node, args.controller_manager):
        rclpy.shutdown()
        return 1

    if not node.wait_for_robot_description():
        rclpy.shutdown()
        return 1

    # Load ros2 controllers in sequence.
    if not start_joint_state_broadcaster(node):
        rclpy.shutdown()
        return 1
    
    active_controller_list = []
    if args.controller_name and args.controller_name.lower() != 'none':
        active_controller_list.append(args.controller_name)
    else:
        node.get_logger().info('No MPC controller requested, starting joint_state_broadcaster only')

    inactive_controller_list = [
    ]
    
    if active_controller_list:
        # Load + configure builds the MPC problem (CppAD codegen happens here).
        if not start_ros2_controllers(node, active_controller_list, activate=False):
            rclpy.shutdown()
            return 1
        # Activation may be refused while the controller is still a migration
        # scaffold; warn and continue so the simulation still starts.
        for name in active_controller_list:
            if not node.activate(name):
                node.get_logger().warn(
                    f'Controller {name} refused activation (migration guard); continuing without it')
    
    if not start_ros2_controllers(node, inactive_controller_list, activate=False):
        rclpy.shutdown()
        return 1

    if args.use_mujoco_sim and not node.start_mujoco():
        rclpy.shutdown()
        return 1

    node.get_logger().info('Controller sequence complete')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
