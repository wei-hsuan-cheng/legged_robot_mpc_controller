#!/usr/bin/env python3
"""Exercise real controller posture services, repeat cycles and a latched fall in MuJoCo (Upkie or Pineapple)."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from ocs2_msgs.msg import WalkingVelocityCommand

from mujoco_scene import write_scene

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "launch"))
from push_recovery_test import WrenchPublisher, WrenchSequence

CONTROLLER = "/humanoid_whole_body_inverse_dynamics_mpc_controller"


class Probe(Node):
    def __init__(self, joystick=False, reference=None):
        super().__init__("upkie_posture_cycle_probe")
        # Command height and the teleop profile: the robot YAML's reference (Upkie's historical 0.22/0.2/0.33).
        reference = reference or dict(defaultBaseHeight=0.22, minBaseHeight=0.2, maxBaseHeight=0.33)
        self.height = reference["defaultBaseHeight"]
        self.phase = "unknown"
        self.transitions = []
        self.samples = []
        self.joints = None
        self.gt = None
        self.stage = "cycles"
        self.teleop = None
        self.joy_pressure = 0.0
        self.joy_posture = False
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(WalkingVelocityCommand, "/humanoid/walking_velocity_command", qos)
        self.joy_publisher = self.create_publisher(Joy, "/codex/upkie/joy", qos)
        if joystick:
            # The real (C++) teleop adapter, driven by the synthetic Joy below.
            self.teleop = subprocess.Popen([
                "ros2", "run", "legged_robot_teleop", "base_twist", "--ros-args",
                "-p", "joy_topic:=/codex/upkie/joy",
                "-p", f"posture_controller:={CONTROLLER}",
                "-p", f"reference_height:={reference['defaultBaseHeight']}",
                "-p", f"minimum_height:={reference['minBaseHeight']}",
                "-p", f"maximum_height:={reference['maxBaseHeight']}",
                "-p", "scale_forward:=1.0",
                "-p", "scale_lateral:=0.0",
            ], start_new_session=True)  # its own process group: `ros2 run` does not pass SIGINT on to base_twist
        self.create_subscription(String, CONTROLLER + "/motion_state", self.state, qos)
        self.create_subscription(Odometry, "/mujoco/ground_truth/odom", self.observe, qos)
        self.create_subscription(JointState, "/joint_states", self.observe_joints, qos)
        self.posture_clients = {name: self.create_client(Trigger, CONTROLLER + "/" + name)
                        for name in ("stand_up", "sit_down", "toggle_stand")}

    def state(self, message):
        if message.data != self.phase:
            self.transitions.append([self.get_clock().now().nanoseconds * 1e-9, message.data])
            print("STATE", self.transitions[-1], flush=True)
        self.phase = message.data

    def observe_joints(self, message):
        self.joints = message

    def observe(self, message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        tilt = math.acos(max(-1.0, min(1.0, 1 - 2 * (q.x * q.x + q.y * q.y))))
        self.gt = message
        row = dict(time=message.header.stamp.sec + 1e-9 * message.header.stamp.nanosec,
                   phase=self.phase, stage=self.stage, position=[p.x, p.y, p.z],
                   quaternion_wxyz=[q.w, q.x, q.y, q.z], tilt=tilt,
                   speed=math.hypot(message.twist.twist.linear.x, message.twist.twist.linear.y))
        if self.joints is not None:
            row.update(names=list(self.joints.name), q=list(self.joints.position),
                       v=list(self.joints.velocity), effort=list(self.joints.effort))
        self.samples.append(row)

    def command(self, vx=0.0, yaw=0.0):
        if self.teleop is not None:
            message = Joy()
            message.axes = [0.0, vx, yaw, 0.0, -self.joy_pressure, 0.0]
            message.buttons = [int(i == 2 and self.joy_posture) for i in range(15)]
            self.joy_publisher.publish(message)
            return
        message = WalkingVelocityCommand()
        message.desired_pelvis_height = self.height
        message.linear_velocity_x = vx
        message.angular_velocity_z = yaw
        self.publisher.publish(message)

    def toggle_posture(self):
        self.joy_pressure = 0.0
        self.joy_posture = False
        self.pump(.15)
        self.joy_posture = True
        self.pump(.2)
        self.joy_posture = False
        self.pump(.1)

    def pump(self, duration, vx=0.0, yaw=0.0):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.command(vx, yaw)
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait(self, phase, timeout=45.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.pump(0.02)
            if self.phase == phase:
                return
            if self.phase in ("fallen", "FAULT (damping)") and self.phase != phase:
                raise RuntimeError(f"Expected {phase}, got {self.phase}")
        raise TimeoutError(f"Expected {phase}, still {self.phase}")

    def request(self, name, expected=True):
        client = self.posture_clients[name]
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f"Missing {name} service")
        future = client.call_async(Trigger.Request())
        end = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < end:
            self.pump(0.02)
        if not future.done():
            raise TimeoutError(name)
        result = future.result()
        print("SERVICE", name, result.success, result.message, flush=True)
        if result.success != expected:
            raise RuntimeError(f"Unexpected {name} response: {result}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-name", choices=("upkie", "pineapple"), default="upkie")
    parser.add_argument("--prefix", default="/tmp/codex_upkie_posture")
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument("--source", choices=("ground_truth_state", "state_estimator"), default="state_estimator")
    parser.add_argument("--fall-test", action="store_true")
    parser.add_argument("--joystick", action="store_true", help="Drive the real teleop adapter with synthetic Joy, including Square")
    parser.add_argument("--trajectory", type=Path, help="Stand-up YAML override for candidate acceptance")
    parser.add_argument("--yaw-command", type=float, default=0.0, help="Normalized yaw command while driving between cycles")
    args = parser.parse_args()
    if not -1.0 <= args.yaw_command <= 1.0:
        parser.error("--yaw-command must lie in [-1, 1]")
    prefix = Path(args.prefix)
    robot = args.robot_name
    scene = write_scene(str(prefix) + ".xml", robot=robot, terrain="none")
    robot_config = yaml.safe_load((Path(__file__).resolve().parents[1] /
                                   f"config/{robot}/ros2_controllers.yaml").read_text())
    # Pineapple drives the teleop/command height from its YAML; Upkie keeps the historical values.
    reference = robot_config[CONTROLLER.lstrip("/")]["ros__parameters"]["ocs2"]["reference"] if robot != "upkie" else None
    cmd = ["ros2", "launch", "legged_robot_mpc_controller", "robot.launch.py", f"robot_name:={robot}",
           "mpc_freq:=200", "mrt_freq:=1000", "rviz:=false", "mujoco_headless:=true", "base_command_device:=none",
           f"mujoco_model_file:={scene}", f"floating_base_source:={args.source}",
           "floating_base_estimator:=" + ("humanoid_linear_kf" if args.source == "state_estimator" else "none")]
    if args.trajectory:
        config = robot_config
        config[CONTROLLER.lstrip("/")]["ros__parameters"]["bringUp"]["standUpTrajectoryFile"] = str(args.trajectory.resolve())
        config_path = Path(str(prefix) + ".yaml")
        config_path.write_text(yaml.safe_dump(config, sort_keys=False))
        cmd.append("controllers_file:=" + str(config_path))
    process = None
    probe = None
    results = dict(verdict="FAIL", cycles=0, source=args.source)
    rclpy.init()
    try:
        with Path(str(prefix) + ".log").open("w") as log:
            process = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            probe = Probe(args.joystick, reference)
            probe.wait("idle", 90)
            probe.pump(4)
            assert probe.phase == "idle", "Must wait for the operator"
            for cycle in range(args.cycles):
                if args.joystick:
                    probe.toggle_posture()
                else:
                    probe.request("toggle_stand" if cycle % 2 else "stand_up")
                probe.wait("standing up")
                probe.request("toggle_stand", expected=False)
                probe.wait("locomotion")
                probe.pump(4)
                probe.joy_pressure = 1.0
                probe.pump(.2)
                probe.pump(2, vx=0.15, yaw=args.yaw_command)
                probe.joy_pressure = 0.0
                if args.joystick:
                    probe.toggle_posture()
                else:
                    probe.request("sit_down" if cycle % 2 else "toggle_stand")
                probe.wait("sitting down")
                probe.request("stand_up", expected=False)
                probe.wait("idle")
                probe.pump(4)
                assert probe.phase == "idle", "Sit-down must not auto-restart"
                results["cycles"] += 1
            if args.fall_test:
                probe.request("stand_up")
                probe.wait("locomotion")
                probe.pump(5)
                probe.stage = "injected_fall"
                # A sideways shove that tips the robot over: 150 N on Upkie's 5.7 kg base, 400 N on Pineapple's 15 kg.
                link, force = ("base", 150) if robot == "upkie" else ("base_link", 400)
                sequence = WrenchSequence([dict(link=link, force=[0, force, 0], duration=0.5)])
                sender = WrenchPublisher(probe, sequence)
                start = time.monotonic()
                while time.monotonic() - start < 0.6:
                    sender.send(time.monotonic() - start)
                    probe.pump(0.01)
                sender.stop()
                probe.wait("fallen")
                for name in ("stand_up", "sit_down", "toggle_stand"):
                    probe.request(name, expected=False)
                probe.pump(3)
                assert probe.phase == "fallen"
                results["fall_latched"] = True
            results["verdict"] = "PASS"
    except Exception as exc:
        results["error"] = str(exc)
        raise
    finally:
        if probe is not None:
            results["transitions"] = probe.transitions
            active = [s for s in probe.samples if s["phase"] == "locomotion" and s["stage"] == "cycles"]
            results["max_locomotion_tilt"] = max((s["tilt"] for s in active), default=None)
            Path(str(prefix) + ".replay.json").write_text(json.dumps(probe.samples))
            if probe.teleop is not None:
                os.killpg(probe.teleop.pid, signal.SIGINT)
                try:
                    probe.teleop.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(probe.teleop.pid, signal.SIGKILL)
            probe.destroy_node()
        Path(str(prefix) + ".json").write_text(json.dumps(results, indent=2))
        print(json.dumps(results, indent=2), flush=True)
        if process is not None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
