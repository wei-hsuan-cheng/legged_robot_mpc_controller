#!/usr/bin/env python3
"""Verify real ghost topics in a headless, short flat-ground walking test."""

import argparse
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray
import yaml


ROOT = Path(__file__).resolve().parents[1]
TOPICS = ("optimizedStateGhosts", "terminalStateGhost", "terminalTargetGhost")


class GhostProbe(Node):
    def __init__(self):
        super().__init__("future_robot_ghost_probe")
        self.stamp = None
        self.messages = {}
        self.qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, "/mujoco/ground_truth/odom", self.observe, self.qos)

    def observe(self, msg):
        self.stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def connect(self):
        for topic in TOPICS:
            self.create_subscription(MarkerArray, "/performance_visualization/" + topic,
                                     lambda msg, key=topic: self.messages.__setitem__(key, msg), self.qos)


def validate(messages, expected_count, frame):
    terminal = [m for m in messages["terminalStateGhost"].markers if m.action == m.ADD]
    target = [m for m in messages["terminalTargetGhost"].markers if m.action == m.ADD]
    future = [m for m in messages["optimizedStateGhosts"].markers if m.action == m.ADD]
    assert terminal and target and future, "Empty ghost model"
    assert len(future) == expected_count * len(terminal), (len(future), len(terminal), expected_count)
    mesh_count = 0
    for topic, array in messages.items():
        assert array.markers[0].action == array.markers[0].DELETEALL, "Missing previous-marker cleanup"
        ids = set()
        for marker in array.markers[1:]:
            assert marker.header.frame_id == frame
            assert (marker.ns, marker.id) not in ids
            ids.add((marker.ns, marker.id))
            q, p = marker.pose.orientation, marker.pose.position
            assert all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w))
            assert abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) < 1e-5
            assert min(marker.scale.x, marker.scale.y, marker.scale.z) > 0
            if marker.type == marker.MESH_RESOURCE:
                mesh_count += 1
                prefix = "package://legged_robot_mpc_controller/"
                assert marker.mesh_resource.startswith(prefix), marker.mesh_resource
                assert (ROOT / marker.mesh_resource.removeprefix(prefix)).is_file()
            expected_color = (0.180, 0.220, 0.300) if topic == "terminalStateGhost" else (0.765, 0.665, 0.905)
            assert all(abs(a-b) < 1e-6 for a, b in zip(
                (marker.color.r, marker.color.g, marker.color.b), expected_color))
    assert mesh_count > 0, "No mesh markers published"
    assert abs(future[0].color.a - 0.22) < 1e-6
    assert abs(future[-1].color.a - (0.08 if expected_count > 1 else 0.22)) < 1e-6
    assert abs(terminal[0].color.a - 0.40) < 1e-6
    assert abs(target[0].color.a - 0.30) < 1e-6
    return dict(ghost_count=expected_count, visuals_per_robot=len(terminal),
                future_markers=len(future), terminal_target_markers=len(target))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-name", choices=["g1", "h2"], required=True)
    parser.add_argument("--horizon", type=float, default=1.0)
    parser.add_argument("--prefix", default="/tmp/future_robot_ghosts")
    args = parser.parse_args()
    if not math.isfinite(args.horizon) or args.horizon <= 0:
        parser.error("horizon must be finite and positive")
    params = yaml.safe_load((ROOT / "config" / args.robot_name / "ros2_controllers.yaml").read_text())[
        "humanoid_whole_body_inverse_dynamics_mpc_controller"]["ros__parameters"]
    count = max(1, math.floor(args.horizon / params["visualization"]["optimizedStateRobotModelsInterval"] + 0.5))
    command = [sys.executable, str(ROOT / "tests/wb_id_walk_test.py"),
               "--robot-name", args.robot_name, "--source", "state_estimator",
               "--estimator", "humanoid_linear_kf", "--axis", "vx", "--speed", "0.10",
               "--settle", "4", "--ramp", "0", "--hold", "6", "--rest", "4", "--mpc-rate", "90",
               "--set", f"ocs2.mpc.timeHorizon={args.horizon}", "--prefix", args.prefix + "_walk"]
    rclpy.init()
    probe = GhostProbe()
    process = None
    summary = {}
    try:
        with Path(args.prefix + "_driver.log").open("w") as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 600
            ready_stamp = None
            connected = False
            while time.monotonic() < deadline:
                rclpy.spin_once(probe, timeout_sec=0.02)
                launch_log = Path(args.prefix + "_walk.log")
                ready = launch_log.exists() and "-> locomotion" in launch_log.read_text(errors="replace")
                if ready and probe.stamp is not None and ready_stamp is None:
                    ready_stamp = probe.stamp
                if ready_stamp is not None and not connected and probe.stamp - ready_stamp >= 1.0:
                    # No RViz or ghost subscription yet: these topics should have zero traffic.
                    info = probe.get_publishers_info_by_topic("/performance_visualization/optimizedStateGhosts")
                    assert info, "Ghost publisher not configured"
                    assert all(i.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT for i in info)
                    for topic in TOPICS:
                        assert probe.count_subscribers("/performance_visualization/" + topic) == 0
                    probe.connect()
                    connected = True
                if len(probe.messages) == len(TOPICS):
                    summary = validate(probe.messages, count, params["visualization"]["frameId"])
                    summary.update(robot=args.robot_name, horizon=args.horizon, qos="best_effort")
                    break
                if process.poll() is not None:
                    raise RuntimeError("Walking test exited before ghost topics were verified; inspect driver log")
            assert summary, "Timed out waiting for valid future robot ghosts"
            # Finish the short walking test normally so its owned simulator also shuts down.
            deadline = time.monotonic() + 180
            while process.poll() is None and time.monotonic() < deadline:
                rclpy.spin_once(probe, timeout_sec=0.05)
            assert process.poll() == 0, "Headless walking test failed or timed out"
            Path(args.prefix + ".json").write_text(json.dumps(summary, indent=2) + "\n")
            print(json.dumps(summary, indent=2))
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=30)
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
