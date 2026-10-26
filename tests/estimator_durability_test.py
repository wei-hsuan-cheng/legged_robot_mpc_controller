#!/usr/bin/env python3
"""Long estimator-durability run: does the floating-base estimate drift and degrade control?

Launches one robot headless on an isolated flat scene with state-estimator feedback, holds one
command (stand still, or a constant forward speed plus yaw rate, i.e. a circle) for the requested
duration and records ground truth and the estimate at a decimated rate. It stops early on a fall.

Outputs: <prefix>.csv (20 Hz ground truth + estimate), <prefix>.log (launch output),
<prefix>.yaml (effective controller YAML), <prefix>_scene.xml and <prefix>.json (verdict, minute
status, controller memory). Analyze with tests/tuning/analyze_estimator_durability.py.
Run from a sourced workspace, like tests/wb_id_walk_test.py.
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
from ocs2_msgs.msg import WalkingVelocityCommand
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from mujoco_scene import write_scene

CONTROLLER = "humanoid_whole_body_inverse_dynamics_mpc_controller"
COLUMNS = ["phase", "t", "gt_x", "gt_y", "gt_z", "gt_roll", "gt_pitch", "gt_yaw", "gt_vx", "gt_vy", "gt_vz", "gt_wz",
           "est_t", "est_x", "est_y", "est_z", "est_roll", "est_pitch", "est_yaw", "est_vx", "est_vy", "est_vz", "est_wz"]


def attitude(q):
    return (math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)),
            math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))),
            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def pose_twist(msg):
    p, v, w = msg.pose.pose.position, msg.twist.twist.linear, msg.twist.twist.angular
    stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
    return (stamp, p.x, p.y, p.z, *attitude(msg.pose.pose.orientation), v.x, v.y, v.z, w.z)


class Probe(Node):
    def __init__(self, writer, record_period):
        super().__init__("estimator_durability_probe")
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(WalkingVelocityCommand, "/humanoid/walking_velocity_command", qos)
        self.create_subscription(Odometry, "/mujoco/ground_truth/odom", self.observe_gt, qos)
        self.create_subscription(Odometry, "/humanoid/state_estimate/odom", self.observe_estimate, qos)
        self.writer, self.record_period = writer, record_period
        self.gt = self.estimate = None
        self.last_record = -1e9
        self.last_wall = time.monotonic()
        self.phase = "startup"

    def observe_estimate(self, msg):
        self.estimate = pose_twist(msg)

    def observe_gt(self, msg):
        self.last_wall = time.monotonic()
        self.gt = pose_twist(msg)
        if self.phase != "startup" and self.gt[0] - self.last_record >= self.record_period - 1e-6:
            self.last_record = self.gt[0]
            estimate = self.estimate if self.estimate is not None else (math.nan,) * 11
            self.writer.writerow((self.phase, *self.gt, *estimate))

    def command(self, vx, height, yaw):
        msg = WalkingVelocityCommand()
        msg.linear_velocity_x, msg.linear_velocity_y = vx, 0.0
        msg.desired_pelvis_height, msg.angular_velocity_z = height, yaw
        self.publisher.publish(msg)


def controller_rss_mb():
    try:
        pids = subprocess.run(["pgrep", "-f", "[l]ib/mujoco_ros2_control/mujoco_ros2_control"],
                              capture_output=True, text=True).stdout.split()
        for line in Path(f"/proc/{pids[0]}/status").read_text().splitlines():
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) / 1024.0
    except (IndexError, OSError, ValueError):
        pass
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--robot-name", choices=["g1", "h2"], default="h2")
    parser.add_argument("--estimator", choices=["humanoid_inekf", "humanoid_linear_kf"], default="humanoid_inekf")
    parser.add_argument("--vx", type=float, default=0.0, help="Forward command [m/s]; 0 with --yaw-rate 0 stands still")
    parser.add_argument("--yaw-rate", type=float, default=0.0, help="Yaw-rate command [rad/s]")
    parser.add_argument("--duration", type=float, default=3600.0, help="Hold duration [simulation s]")
    parser.add_argument("--settle", type=float, default=10.0)
    parser.add_argument("--ramp", type=float, default=5.0)
    parser.add_argument("--rest", type=float, default=20.0)
    parser.add_argument("--mpc-rate", type=int, default=90)
    parser.add_argument("--record-rate", type=float, default=20.0)
    parser.add_argument("--minimum-height", type=float, default=0.38)
    parser.add_argument("--startup-timeout", type=float, default=600.0)
    parser.add_argument("--config", type=Path, help="Controller YAML (default: config/<robot-name>/ros2_controllers.yaml)")
    parser.add_argument("--prefix", default="/tmp/estimator_durability")
    args = parser.parse_args()

    prefix = Path(args.prefix).resolve()
    prefix.parent.mkdir(parents=True, exist_ok=True)
    args.config = args.config or Path(__file__).resolve().parents[1] / "config" / args.robot_name / "ros2_controllers.yaml"
    config = yaml.safe_load(args.config.read_text())
    reference = config[CONTROLLER]["ros__parameters"]["ocs2"]["reference"]
    if abs(args.vx) > reference["maxDisplacementVelocityX"] or abs(args.yaw_rate) > reference["maxRotationVelocity"]:
        parser.error("command exceeds the configured normalization limits")
    vx_norm = args.vx / reference["maxDisplacementVelocityX"]
    yaw_norm = args.yaw_rate / reference["maxRotationVelocity"]
    height = float(reference["defaultBaseHeight"])
    config_path = Path(str(prefix) + ".yaml")
    config_path.write_text(yaml.safe_dump(config, sort_keys=False))
    scene = write_scene(Path(str(prefix) + "_scene.xml"), args.robot_name)

    launch = ["ros2", "launch", "legged_robot_mpc_controller", "robot.launch.py",
              f"robotName:={args.robot_name}", f"mpcControllerName:={CONTROLLER}", f"mujocoModelFile:={scene}",
              "rviz:=false", "mujoco_headless:=true", "baseCommandGui:=false",
              "floatingBaseSource:=state_estimator", f"floatingBaseEstimator:={args.estimator}",
              f"mpcFreq:={args.mpc_rate}", "mrtFreq:=1000", f"controllersFile:={config_path}"]
    result = {key: (str(value) if isinstance(value, Path) else value) for key, value in vars(args).items()}
    result.update(minutes=[], verdict="TIMEOUT")
    log_path = Path(str(prefix) + ".log")
    rclpy.init()
    process = probe = None
    try:
        with log_path.open("w") as log, Path(str(prefix) + ".csv").open("w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(COLUMNS)
            probe = Probe(writer, 1.0 / args.record_rate)
            process = subprocess.Popen(launch, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + args.startup_timeout
            start = bad_since = None
            next_status = 0.0
            total = args.settle + 2 * args.ramp + args.duration + args.rest
            while time.monotonic() < deadline:
                rclpy.spin_once(probe, timeout_sec=0.02)
                if process.poll() is not None:
                    result["verdict"] = "LAUNCH_EXITED"
                    break
                if probe.gt is None:
                    continue
                if time.monotonic() - probe.last_wall > 20:
                    result["verdict"] = "STALE_ODOM"
                    break
                if start is None:
                    if "-> locomotion" not in log_path.read_text(errors="replace"):
                        probe.command(0.0, height, 0.0)
                        continue
                    start = probe.gt[0]
                    deadline = time.monotonic() + 3 * total + 120
                elapsed = probe.gt[0] - start
                if elapsed < args.settle:
                    probe.phase, fraction = "settle", 0.0
                elif elapsed < args.settle + args.ramp:
                    probe.phase, fraction = "ramp_in", (elapsed - args.settle) / max(args.ramp, 1e-9)
                elif elapsed < args.settle + args.ramp + args.duration:
                    probe.phase, fraction = "hold", 1.0
                elif elapsed < args.settle + 2 * args.ramp + args.duration:
                    probe.phase, fraction = "ramp_out", 1 - (elapsed - args.settle - args.ramp - args.duration) / max(args.ramp, 1e-9)
                else:
                    probe.phase, fraction = "rest", 0.0
                probe.command(vx_norm * fraction, height, yaw_norm * fraction)
                _, x, y, z, roll, pitch, yaw, *_ = probe.gt
                if elapsed >= next_status:
                    est = probe.estimate
                    status = dict(t=round(elapsed, 1), phase=probe.phase, gt_z=round(z, 4), roll=round(roll, 4),
                                  pitch=round(pitch, 4), gt_xy=[round(x, 3), round(y, 3)],
                                  est_z=round(est[3], 4) if est else None, rss_mb=controller_rss_mb())
                    result["minutes"].append(status)
                    print("STATUS " + json.dumps(status), flush=True)
                    next_status += 60.0
                bad = z < args.minimum_height or abs(roll) > 0.8 or abs(pitch) > 0.8
                bad_since = (bad_since if bad_since is not None else elapsed) if bad else None
                if bad_since is not None and elapsed - bad_since > 0.15:
                    result.update(verdict="FALL", fall_time=elapsed, fall_phase=probe.phase)
                    break
                if elapsed >= total:
                    result["verdict"] = "SURVIVED"
                    break
            if start is not None:
                result["sim_seconds"] = probe.gt[0] - start
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        if probe is not None:
            probe.destroy_node()
        rclpy.shutdown()
        Path(str(prefix) + ".json").write_text(json.dumps(result, indent=2) + "\n")
        print("RESULT " + json.dumps({k: result.get(k) for k in ("verdict", "sim_seconds", "fall_time", "fall_phase")}), flush=True)
    return 0 if result["verdict"] == "SURVIVED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
