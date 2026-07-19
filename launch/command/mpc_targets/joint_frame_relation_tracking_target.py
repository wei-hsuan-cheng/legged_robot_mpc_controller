#!/usr/bin/env python3
"""Publishes command_type "joint_frame_relation" MpcTargets: both channels at once.

Msg convention (parsed internally by mpc_targets_parser): the arm joint
trajectory comes first in target_trajectories, followed by one pose trajectory
per source/target frame pair (source = reference frame, target = tracked leaf).
Here the arms hold a fixed posture while the left hand additionally tracks a
pelvis-relative pose; both are soft costs, so they balance on the left arm.
Send {command_type: "default"} to revert.
"""

import rclpy
from rclpy.node import Node

from ocs2_msgs.msg import MpcInput, MpcState, MpcTargets, MpcTargetTrajectories
from std_msgs.msg import Float64MultiArray


JOINT_NAMES = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
]
JOINT_TARGET = [0.2, 0.0, 0.0, 0.5, 0.3, 0.0, 0.0, 0.6]

# Left hand pose expressed in the pelvis frame (source = reference/root frame,
# target = tracked leaf frame; the pair must be declared in
# costs.frameRelationTracking sourceFrames/targetFrames).
SOURCE_FRAME = "pelvis"
TARGET_FRAME = "left_rubber_hand"
HAND_POSE = [0.32, 0.15, 0.20, 0.0, 0.0, 0.0, 1.0]  # [pos, quat xyzw] of target in source
HAND_WEIGHTS = [200.0, 200.0, 200.0, 20.0, 20.0, 20.0]


def single_sample_trajectory(values):
    state = MpcState()
    state.value = [float(v) for v in values]
    target_input = MpcInput()
    target_input.value = []
    trajectory = MpcTargetTrajectories()
    trajectory.time_trajectory = [0.0]
    trajectory.state_trajectory = [state]
    trajectory.input_trajectory = [target_input]
    return trajectory


class JointFrameRelationTargetPublisher(Node):
    def __init__(self):
        super().__init__("joint_frame_relation_target_publisher")
        self.declare_parameter("topic", "/humanoid/mpc_targets")
        self.declare_parameter("publish_rate", 5.0)
        self.topic = self.get_parameter("topic").value
        rate = max(1e-6, float(self.get_parameter("publish_rate").value))

        self.publisher = self.create_publisher(MpcTargets, self.topic, 1)
        self.timer = self.create_timer(1.0 / rate, self.publish)
        self.get_logger().info(f"Publishing joint_frame_relation MpcTargets to {self.topic}")

    def publish(self):
        weights = Float64MultiArray()
        weights.data = [float(w) for w in HAND_WEIGHTS]

        msg = MpcTargets()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.command_type = "joint_frame_relation"
        msg.joint_names = list(JOINT_NAMES)
        msg.source_frames = [SOURCE_FRAME]
        msg.target_frames = [TARGET_FRAME]
        msg.frame_relation_tracking_weights = [weights]
        # Joint trajectory first, then one trajectory per frame pair.
        msg.target_trajectories = [
            single_sample_trajectory(JOINT_TARGET),
            single_sample_trajectory(HAND_POSE),
        ]
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointFrameRelationTargetPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
