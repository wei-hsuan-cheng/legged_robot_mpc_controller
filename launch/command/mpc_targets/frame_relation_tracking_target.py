#!/usr/bin/env python3
"""Publishes command_type "frame_relation" MpcTargets: relative-pose hand targets.

Convention (matching mpc_controllers): source_frame is the reference (root)
frame the pose is expressed in - a robot frame such as "pelvis" or a global
frame ("world") - and target_frame is the tracked leaf frame (a hand). States
are [position(3), quaternion x y z w] of target expressed in source. Each
(source, target) pair must be declared in costs.frameRelationTracking
(sourceFrames/targetFrames, default pelvis -> left/right_rubber_hand). Optional
weights (6 per pair: position xyz, orientation xyz) override the configured
defaults. Send {command_type: "default"} to revert to the built-in arm-swing
reference.

Each message carries a single-sample trajectory (constant pose); motion is
produced by republishing at publish_rate, so no clock synchronization with the
simulation is required.
"""

import math
from typing import List

import rclpy
from rclpy.node import Node

from ocs2_msgs.msg import MpcInput, MpcState, MpcTargets, MpcTargetTrajectories
from std_msgs.msg import Float64MultiArray


DEFAULT_FRAME_PAIRS = [
    {
        "source_frame": "pelvis",
        "target_frame": "left_rubber_hand",
        "center_position": [0.241, 0.152, 0.195],
        "amplitude": [0.2, 0.0, 0.2],
        "phase": [1.5708, 0.0, 0.0],
        "quaternion": [0.0, 0.0, 0.0, 1.0],
        "weights": [100.0, 100.0, 100.0, 25.0, 25.0, 25.0],
    },

    {
        "source_frame": "pelvis",
        "target_frame": "right_rubber_hand",
        "center_position": [0.241, -0.152, 0.295],
        "amplitude": [0.2, 0.0, 0.2],
        "phase": [0.0, 0.0, 1.5708],
        "quaternion": [0.0, 0.0, 0.0, 1.0],
        "weights": [100.0, 100.0, 100.0, 25.0, 25.0, 25.0],
    },

]


class FrameRelationTrackingTargetPublisher(Node):
    def __init__(self):
        super().__init__("frame_relation_tracking_target_publisher")

        self.declare_parameter("topic", "/humanoid/mpc_targets")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("sine_frequency", 0.5)

        self.topic = self.get_parameter("topic").value
        self.publish_rate = max(1e-6, float(self.get_parameter("publish_rate").value))
        self.sine_frequency = float(self.get_parameter("sine_frequency").value)
        self.frame_pairs = DEFAULT_FRAME_PAIRS

        self.start_time = self.get_clock().now()
        self.publisher = self.create_publisher(MpcTargets, self.topic, 1)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish)

        pairs = ", ".join(f"{p['source_frame']}->{p['target_frame']}" for p in self.frame_pairs)
        self.get_logger().info(f"Publishing frame_relation MpcTargets to {self.topic}: {pairs}")

    def pose_target(self, pair, t: float) -> List[float]:
        omega = 2.0 * math.pi * self.sine_frequency
        position = [
            pair["center_position"][i] + pair["amplitude"][i] * math.sin(omega * t + pair["phase"][i])
            for i in range(3)
        ]
        return position + list(pair["quaternion"])

    def publish(self):
        t = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

        msg = MpcTargets()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.command_type = "frame_relation"

        for pair in self.frame_pairs:
            state = MpcState()
            state.value = [float(v) for v in self.pose_target(pair, t)]
            target_input = MpcInput()
            target_input.value = []

            trajectory = MpcTargetTrajectories()
            trajectory.time_trajectory = [0.0]
            trajectory.state_trajectory = [state]
            trajectory.input_trajectory = [target_input]

            weights = Float64MultiArray()
            weights.data = [float(w) for w in pair["weights"]]

            msg.source_frames.append(pair["source_frame"])
            msg.target_frames.append(pair["target_frame"])
            msg.frame_relation_tracking_weights.append(weights)
            msg.target_trajectories.append(trajectory)

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FrameRelationTrackingTargetPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()