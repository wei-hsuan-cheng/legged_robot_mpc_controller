#!/usr/bin/env python3
"""Publishes command_type "joint" MpcTargets: sine arm-joint targets for the G1.

The humanoid MPC parses this internally (mpc_targets_parser): the states must
cover exactly the tracked arm joints (shoulder + elbow, any order via
joint_names). Each publication replaces the previous external target; the arms
revert to the built-in posture + gait swing after a {command_type: "default"}
message.

Each message carries a single-sample trajectory (constant target); the sine is
produced by republishing at publish_rate, so no clock synchronization with the
simulation is required.
"""

import math
from typing import List

import rclpy
from rclpy.node import Node

from ocs2_msgs.msg import MpcInput, MpcState, MpcTargets, MpcTargetTrajectories


DEFAULT_JOINT_NAMES = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
]

# Gentle counter-phase arm swing around a raised-elbow posture.
DEFAULT_CENTER = [0.2, 0.0, 0.0, 0.5, 0.2, 0.0, 0.0, 0.5]
DEFAULT_AMPLITUDE = [0.2, 0.0, 0.0, 0.15, 0.2, 0.0, 0.0, 0.15]
DEFAULT_PHASE = [0.0, 0.0, 0.0, 0.0, math.pi, 0.0, 0.0, math.pi]


def _as_list(value, fallback: List):
    return list(fallback) if value is None else list(value)


class JointTrackingTargetPublisher(Node):
    def __init__(self):
        super().__init__("joint_tracking_target_publisher")

        self.declare_parameter("topic", "/humanoid/mpc_targets")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("sine_frequency", 0.25)
        self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)
        self.declare_parameter("center", DEFAULT_CENTER)
        self.declare_parameter("amplitude", DEFAULT_AMPLITUDE)
        self.declare_parameter("phase", DEFAULT_PHASE)

        self.topic = self.get_parameter("topic").value
        self.publish_rate = max(1e-6, float(self.get_parameter("publish_rate").value))
        self.sine_frequency = float(self.get_parameter("sine_frequency").value)
        self.joint_names = _as_list(self.get_parameter("joint_names").value, DEFAULT_JOINT_NAMES)
        self.center = _as_list(self.get_parameter("center").value, DEFAULT_CENTER)
        self.amplitude = _as_list(self.get_parameter("amplitude").value, DEFAULT_AMPLITUDE)
        self.phase = _as_list(self.get_parameter("phase").value, DEFAULT_PHASE)

        self.start_time = self.get_clock().now()
        self.publisher = self.create_publisher(MpcTargets, self.topic, 1)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish)

        self.get_logger().info(
            f"Publishing joint MpcTargets to {self.topic} for {len(self.joint_names)} arm joints"
        )

    def publish(self):
        t = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        omega = 2.0 * math.pi * self.sine_frequency
        target = [
            self.center[i] + self.amplitude[i] * math.sin(omega * t + self.phase[i])
            for i in range(len(self.joint_names))
        ]

        state = MpcState()
        state.value = [float(q) for q in target]
        target_input = MpcInput()
        target_input.value = []

        trajectory = MpcTargetTrajectories()
        trajectory.time_trajectory = [0.0]
        trajectory.state_trajectory = [state]
        trajectory.input_trajectory = [target_input]

        msg = MpcTargets()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.command_type = "joint"
        msg.joint_names = list(self.joint_names)
        msg.target_trajectories = [trajectory]
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointTrackingTargetPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()