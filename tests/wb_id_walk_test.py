#!/usr/bin/env python3
"""Headless, simulation-time WB-ID walking regression on obstacle-free ground.

Run from a sourced workspace. Each run owns and cleans up its launch process
group, records odometry to CSV, and saves the launch log and a JSON verdict.
Commands are specified in m/s (vx/vy) or rad/s (yaw) and normalized using the tested configuration.
"""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from ocs2_msgs.msg import WalkingVelocityCommand
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "launch"))
from push_recovery_test import FEEDBACK_TOPIC, WrenchPublisher, WrenchSequence
from geometry_msgs.msg import WrenchStamped


CONTROLLER = "humanoid_whole_body_inverse_dynamics_mpc_controller"


def attitude(q):
    return (
        math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)),
        math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))),
        math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
    )


class Probe(Node):
    def __init__(self, writer, record_joints=False):
        super().__init__("wb_id_walk_probe")
        self.gt = None
        self.last_wall = time.monotonic()
        self.phase = "startup"
        self.writer = writer
        self.samples = []
        self.walk_samples = []
        self.joints = None
        self.estimate = None
        self.replay = []
        self.wrenches = []
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(WalkingVelocityCommand, "/humanoid/walking_velocity_command", qos)
        self.create_subscription(Odometry, "/mujoco/ground_truth/odom", self.observe, qos)
        self.create_subscription(WrenchStamped, FEEDBACK_TOPIC, self.observe_wrench, qos)
        if record_joints:
            self.create_subscription(JointState, "/joint_states", self.observe_joints, qos)
            self.create_subscription(Odometry, "/humanoid/state_estimate/odom", self.observe_estimate, qos)

    def observe_wrench(self, msg):
        f, t = msg.wrench.force, msg.wrench.torque
        self.wrenches.append((msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec,
                              msg.header.frame_id, f.x, f.y, f.z, t.x, t.y, t.z))

    def observe_estimate(self, msg):
        q = msg.pose.pose.orientation
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        self.estimate = dict(time=msg.header.stamp.sec + 1e-9*msg.header.stamp.nanosec,
                             rpy=attitude(q), linear=[v.x, v.y, v.z], angular=[w.x, w.y, w.z])

    def observe_joints(self, msg):
        self.joints = (list(msg.name), list(msg.position), list(msg.velocity), list(msg.effort))

    def observe(self, msg):
        self.last_wall = time.monotonic()
        p, v = msg.pose.pose.position, msg.twist.twist.linear
        roll, pitch, yaw = attitude(msg.pose.pose.orientation)
        stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        self.gt = (stamp, p.x, p.y, p.z, roll, pitch, yaw, v.x, v.y, v.z, msg.twist.twist.angular.z)
        self.writer.writerow((self.phase, *self.gt))
        if self.joints is not None:
            q = msg.pose.pose.orientation
            self.replay.append(dict(phase=self.phase, time=stamp, position=[p.x, p.y, p.z],
                                    quaternion_wxyz=[q.w, q.x, q.y, q.z], names=self.joints[0],
                                    q=self.joints[1], v=self.joints[2], effort=self.joints[3],
                                    estimate=self.estimate))
        if self.phase != "startup":
            self.samples.append(self.gt)
        if self.phase == "walk":
            self.walk_samples.append(self.gt)

    def command(self, vx, vy, height, yaw=0.0):
        msg = WalkingVelocityCommand()
        msg.linear_velocity_x = vx
        msg.linear_velocity_y = vy
        msg.desired_pelvis_height = height
        msg.angular_velocity_z = yaw
        self.publisher.publish(msg)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--axis", choices=["vx", "vy", "yaw"], default="vx")
    parser.add_argument("--speed", type=float, default=0.25)
    parser.add_argument("--hold", type=float, default=25.0)
    parser.add_argument("--ramp", type=float, default=3.0)
    parser.add_argument("--settle", type=float, default=5.0)
    parser.add_argument("--rest", type=float, default=5.0)
    parser.add_argument("--height", type=float, help="Override YAML nominal pelvis height [m]")
    parser.add_argument("--mpc-rate", type=int, default=100)
    parser.add_argument("--launch-file", default="robot.launch.py")
    parser.add_argument("--robot-name", choices=["g1", "h2"], default="g1")
    parser.add_argument("--scene", default="scene_flat.xml")
    parser.add_argument("--minimum-height", type=float, default=0.38)
    parser.add_argument("--startup-timeout", type=float, default=600.0)
    parser.add_argument("--record-joints", action="store_true", help="Save latest joint sample with each base pose for collision analysis and render replay")
    parser.add_argument("--wrench-sequence", type=Path, help="Optional MuJoCo wrench sequence YAML")
    parser.add_argument("--wrench-delay", type=float, default=8.0, help="First pulse after MPC takeover [simulation s]")
    parser.add_argument("--max-yaw-error", type=float, default=0.5, help="Maximum uncommanded heading change [rad]")
    parser.add_argument("--swing-duration", type=float, help="Override slow_walk single support [s] in the temporary YAML")
    parser.add_argument("--double-support", type=float, default=0.15)
    parser.add_argument("--source", choices=["ground_truth_state", "state_estimator"], default="ground_truth_state")
    parser.add_argument("--estimator", default="humanoid_inekf")
    parser.add_argument("--prefix", default="/tmp/wb_id_walk")
    parser.add_argument("--config", type=Path, help="Controller YAML (default: config/<robot-name>/ros2_controllers.yaml)")
    parser.add_argument("--set", action="append", default=[], metavar="KEY=JSON", help="Override a dotted controller parameter in a temporary YAML")
    args = parser.parse_args()
    sequence = WrenchSequence.load(args.wrench_sequence) if args.wrench_sequence else None
    if args.wrench_delay < 0 or not math.isfinite(args.wrench_delay):
        parser.error("wrench delay must be finite and nonnegative")
    total_duration = args.settle + 2 * args.ramp + args.hold + args.rest
    if sequence and args.wrench_delay + sequence.duration > total_duration:
        parser.error("wrench sequence and recovery pause must fit inside the walking test")
    args.config = args.config or Path(__file__).resolve().parents[1] / "config" / args.robot_name / "ros2_controllers.yaml"
    height_override = args.height
    if args.hold <= 0 or args.ramp < 0 or args.speed == 0:
        parser.error("hold must be positive, ramp nonnegative, and speed nonzero")
    config = yaml.safe_load(args.config.read_text())
    params = config[CONTROLLER]["ros__parameters"]
    for entry in args.set:
        key, value = entry.split("=", 1)
        parent = params
        parts = key.split(".")
        for part in parts[:-1]:
            parent = parent[part]
        if parts[-1] not in parent:
            parser.error(f"Unknown parameter: {key}")
        parent[parts[-1]] = json.loads(value)
    args.height = float(params["ocs2"]["reference"]["defaultBaseHeight"]) if args.height is None else args.height
    if args.swing_duration is not None:
        if args.swing_duration <= 0 or args.double_support <= 0:
            parser.error("swing duration and double support must be positive")
        swing, double_support = args.swing_duration, args.double_support
        params["ocs2"]["gait"]["templates"]["slow_walk"]["switchingTimes"] = [
            0.0, swing, swing + double_support, 2 * swing + double_support, 2 * (swing + double_support)]
    prefix = Path(args.prefix).resolve()
    prefix.parent.mkdir(parents=True, exist_ok=True)
    config_path = Path(str(prefix) + ".yaml")
    config_path.write_text(yaml.safe_dump(config, sort_keys=False))
    limits = params["ocs2"]["reference"]
    scale = limits[{"vx": "maxDisplacementVelocityX", "vy": "maxDisplacementVelocityY", "yaw": "maxRotationVelocity"}[args.axis]]
    if abs(args.speed) > scale:
        parser.error("speed exceeds the configured command limit")
    normalized_speed = args.speed / scale
    launch_cmd = [
        "ros2", "launch", "legged_robot_mpc_controller", args.launch_file,
        f"robotName:={args.robot_name}",
        f"mpcControllerName:={CONTROLLER}", f"mujocoModelFile:={args.scene}",
        "rviz:=false", "mujoco_headless:=true", "baseCommandGui:=false",
        f"floatingBaseSource:={args.source}",
        f"floatingBaseEstimator:={args.estimator if args.source == 'state_estimator' else 'none'}",
        f"mpcFreq:={args.mpc_rate}", "mrtFreq:=1000",
        f"controllersFile:={config_path}",
    ]
    if height_override is not None:
        launch_cmd.append(f"refBaseHeight:={args.height}")
    verdict = "TIMEOUT"
    result = vars(args).copy()
    result["config"] = str(args.config)
    result["wrench_sequence"] = str(args.wrench_sequence) if args.wrench_sequence else None
    if sequence:
        result["wrench_events"] = sequence.events
    process = None
    probe = None
    sender = None
    rclpy.init()
    log_path = Path(str(prefix) + ".log")
    try:
        with log_path.open("w") as log, Path(str(prefix) + ".csv").open("w") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(["phase", "time", "x", "y", "z", "roll", "pitch", "yaw", "vx_body", "vy_body", "vz_body", "wz_body"])
            probe = Probe(writer, args.record_joints)
            sender = WrenchPublisher(probe, sequence) if sequence else None
            process = subprocess.Popen(launch_cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + args.startup_timeout
            start = None
            movement_start = None
            last_phase = None
            bad_since = None
            next_report = 0
            while time.monotonic() < deadline:
                rclpy.spin_once(probe, timeout_sec=0.02)
                if process.poll() is not None:
                    verdict = "LAUNCH_EXITED"
                    break
                if start is None:
                    startup_log = log_path.read_text(errors="replace")
                    if any(marker in startup_log for marker in ("while initializing controller", "Failed to configure",
                           "bring-up FAULT", "Failed to parse global arguments", "Caught exception in launch", "init failed:")):
                        verdict = "STARTUP_ERROR"
                        break
                if probe.gt is None:
                    continue
                if time.monotonic() - probe.last_wall > 10:
                    verdict = "STALE_ODOM"
                    break
                if start is None:
                    startup_log = log_path.read_text(errors="replace")
                    ready = "-> locomotion" in startup_log
                    if not ready:
                        probe.command(0.0, 0.0, args.height)
                        continue
                    start = probe.gt[0]
                    deadline = time.monotonic() + 6 * (args.settle + 2 * args.ramp + args.hold + args.rest) + 60
                elapsed = probe.gt[0] - start
                if sender:
                    sender.send(elapsed - args.wrench_delay)
                if elapsed < args.settle:
                    phase, fraction = "settle", 0.0
                elif elapsed < args.settle + args.ramp:
                    phase, fraction = "ramp_in", (elapsed - args.settle) / args.ramp
                elif elapsed < args.settle + args.ramp + args.hold:
                    phase, fraction = "walk", 1.0
                elif elapsed < args.settle + 2 * args.ramp + args.hold:
                    phase, fraction = "ramp_out", 1 - (elapsed - args.settle - args.ramp - args.hold) / args.ramp
                else:
                    phase, fraction = "rest", 0.0
                probe.phase = phase
                if phase != last_phase:
                    print(f"PHASE {phase} t={elapsed:.2f}", flush=True)
                    last_phase = phase
                    if phase in ("ramp_in", "walk") and movement_start is None:
                        movement_start = probe.gt
                command = normalized_speed * fraction
                probe.command(command if args.axis == "vx" else 0.0, command if args.axis == "vy" else 0.0,
                              args.height, command if args.axis == "yaw" else 0.0)
                _, x, y, z, roll, pitch, yaw, *_ = probe.gt
                if elapsed >= next_report:
                    print(f"t={elapsed:.1f} xyz=({x:.3f},{y:.3f},{z:.3f}) rpy=({roll:.3f},{pitch:.3f},{yaw:.3f})", flush=True)
                    next_report = elapsed + 5
                bad = z < args.minimum_height or abs(roll) > 0.8 or abs(pitch) > 0.8
                bad_since = (bad_since if bad_since is not None else elapsed) if bad else None
                if bad_since is not None and elapsed - bad_since > 0.15:
                    verdict = "FALL"
                    break
                if elapsed >= args.settle + 2 * args.ramp + args.hold + args.rest:
                    dx, dy = x - movement_start[1], y - movement_start[2]
                    yaw0 = movement_start[6]
                    displacement = dx * math.cos(yaw0) + dy * math.sin(yaw0) if args.axis == "vx" else -dx * math.sin(yaw0) + dy * math.cos(yaw0)
                    expected = args.speed * (args.hold + args.ramp)
                    cross_axis = -dx * math.sin(yaw0) + dy * math.cos(yaw0) if args.axis == "vx" else dx * math.cos(yaw0) + dy * math.sin(yaw0)
                    yaw_error = max(abs(math.remainder(s[6] - yaw0, 2 * math.pi)) for s in probe.samples)
                    result.update(displacement=displacement, expected_displacement=expected,
                                  cross_axis_displacement=cross_axis, max_observed_yaw_error=yaw_error)
                    verdict = "SUCCESS" if (displacement / expected > 0.5 and yaw_error <= args.max_yaw_error
                                             and abs(cross_axis) < 0.35 * abs(expected)) else "NO_TRACKING"
                    if args.axis == "yaw":
                        movement = [movement_start] + [s for s in probe.samples if s[0] > movement_start[0]]
                        rotation = sum(math.remainder(b[6] - a[6], 2 * math.pi) for a, b in zip(movement, movement[1:]))
                        drift = max(math.hypot(s[1] - movement_start[1], s[2] - movement_start[2]) for s in movement)
                        result.update(rotation=rotation, expected_rotation=expected, max_translation_drift=drift)
                        verdict = "SUCCESS" if 0.5 < rotation / expected < 1.5 and drift < 0.5 else "NO_TRACKING"
                    break
            if start is not None:
                result["sim_seconds"] = probe.gt[0] - start
            if probe.samples:
                result.update(min_z=min(s[3] for s in probe.samples),
                              max_roll=max(abs(s[4]) for s in probe.samples),
                              max_pitch=max(abs(s[5]) for s in probe.samples), final_state=probe.gt)
            if sequence and start is not None:
                verified = []
                for event in sequence.events:
                    begin = start + args.wrench_delay + event["start"]
                    end = start + args.wrench_delay + event["end"]
                    force_norm = math.sqrt(sum(x*x for x in event["force"]))
                    torque_norm = math.sqrt(sum(x*x for x in event["torque"]))
                    # Feedback is body-frame; magnitudes are invariant to rotation.
                    matches = [row for row in probe.wrenches if row[1] == event["link"]
                               and begin - 0.05 <= row[0] <= end + 0.1
                               and abs(math.sqrt(sum(x*x for x in row[2:5])) - force_norm) < 0.01
                               and abs(math.sqrt(sum(x*x for x in row[5:8])) - torque_norm) < 0.01]
                    verified.append(dict(link=event["link"], samples=len(matches),
                                         measured_span=matches[-1][0] - matches[0][0] if matches else 0.0))
                result["wrench_feedback"] = verified
                result["push_verified"] = all(
                    check["samples"] >= 2 and check["measured_span"] >= 0.7 * event["duration"]
                    for check, event in zip(verified, sequence.events))
                recovery = [s for s in probe.samples if s[0] > start + args.wrench_delay + sequence.duration - 1.0]
                if recovery:
                    result["push_final_recovery"] = dict(min_z=min(s[3] for s in recovery),
                                                         max_roll=max(abs(s[4]) for s in recovery),
                                                         max_pitch=max(abs(s[5]) for s in recovery))
                if not result["push_verified"] and verdict == "SUCCESS":
                    verdict = "WRENCH_NOT_VERIFIED"
            if probe.walk_samples:
                count = len(probe.walk_samples)
                result.update(walk_mean_vx=sum(s[7] for s in probe.walk_samples) / count,
                              walk_mean_vy=sum(s[8] for s in probe.walk_samples) / count,
                              walk_mean_wz=sum(s[10] for s in probe.walk_samples) / count,
                              walk_mean_z=sum(s[3] for s in probe.walk_samples) / count)
                first, last = probe.walk_samples[0], probe.walk_samples[-1]
                duration = last[0] - first[0]
                if duration > 0:
                    result.update(walk_pose_mean_vx_world=(last[1] - first[1]) / duration,
                                  walk_pose_mean_vy_world=(last[2] - first[2]) / duration)
    finally:
        if sender is not None:
            for _ in range(5):
                sender.stop()
                rclpy.spin_once(probe, timeout_sec=0.01)
        if probe is not None and sequence:
            with Path(str(prefix) + "_wrenches.csv").open("w") as file:
                writer = csv.writer(file)
                writer.writerow(["time", "link", "fx_body", "fy_body", "fz_body", "mx_body", "my_body", "mz_body"])
                writer.writerows(probe.wrenches)
        if probe is not None and probe.replay:
            Path(str(prefix) + "_replay.json").write_text(json.dumps(probe.replay))
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        if probe is not None:
            probe.destroy_node()
        rclpy.shutdown()
        result["verdict"] = verdict
        Path(str(prefix) + ".json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2), flush=True)
    return 0 if verdict == "SUCCESS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
