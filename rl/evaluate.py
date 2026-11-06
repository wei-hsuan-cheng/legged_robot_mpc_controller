#!/usr/bin/env python3
# Copyright (c) 2026, Wei-Hsuan Cheng. All rights reserved.
"""Deployment validation: one simulation through the ordinary launch - asynchronous MPC in real
time, commands on the walking-velocity topic - MPC alone vs MPC with the trained policy evaluated
in the controller (gaitAdaptation.source policy). Ground truth only scores the runs.

  python3 rl/evaluate.py --policy /tmp/claude_rl/run1/policy.txt --scenario flat push slippery rough --repeats 2

Scenarios: flat (the command SEQUENCE, friction 1.0 and 0.5), push (SEQUENCE with random
pushes: random times, directions and sizes from a seed per repeat, the same seed for both
modes, so the comparison is paired), slippery (SEQUENCE at friction 0.3), rough (SEQUENCE on the
uneven-box terrain, tops 0.6-10 cm), and terrain:<type>:<difficulty> (walking out from the
centre of one rl/terrain.py tile).
"""

import argparse
import json
import math
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
from ocs2_msgs.msg import WalkingVelocityCommand
from rclpy.qos import QoSProfile, ReliabilityPolicy

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tests"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from mujoco_scene import write_scene  # noqa: E402
from mujoco_ros2_control_multienv import compile_model  # noqa: E402

MPC = "humanoid_whole_body_inverse_dynamics_mpc_controller"
# (seconds after the MPC takes over, vx [m/s], vy [m/s], yaw rate [rad/s]); the last entry ends the run.
SEQUENCE = [(0, 0.0, 0.0, 0.0), (2, 0.4, 0.0, 0.0), (7, 0.4, 0.0, 0.3), (11, 0.0, 0.15, 0.0), (15, -0.2, 0.0, 0.0),
            (19, 0.55, 0.0, 0.0), (23, 0.0, 0.0, 0.0), (27, None, None, None)]
PUSH_BODY = "torso_link"
PUSH_DURATION = 0.1           # [s], mujoco_external_wrench_timeout: one message pushes this long
PUSH_INTERVAL = (3.0, 6.0)    # [s] between pushes
PUSH_FORCE = (150.0, 600.0)   # [N] on the torso, in a uniformly random horizontal direction
TERRAIN_SEQUENCE = [(0, 0.0, 0.0, 0.0), (2, 0.3, 0.0, 0.0), (17, 0.0, 0.0, 0.0), (19, None, None, None)]


def random_pushes(seed, start=2.0, end=25.0):
    """(seconds after take-over, unit direction in the world xy plane, force [N])."""
    rng = np.random.default_rng(1000 + seed)
    pushes, at = [], start + rng.uniform(*PUSH_INTERVAL)
    while at < end:
        heading = rng.uniform(-math.pi, math.pi)
        pushes.append((at, (math.cos(heading), math.sin(heading)), rng.uniform(*PUSH_FORCE)))
        at += rng.uniform(*PUSH_INTERVAL)
    return pushes




def run(label, mjb, overlay, out, domain, workspace, height, scale, sequence=SEQUENCE, pushes=(), initial_pose=None,
        ground=None):
    log_path = out / f"{label}.log"
    cmd = ["ros2", "launch", "legged_robot_mpc_controller", "robot.launch.py", "robotName:=h2", "rviz:=false",
           "mujoco_headless:=true", "baseCommandGui:=false", f"mujocoModelFile:={mjb}"]
    if overlay:
        cmd.append(f"controllersOverlay:={overlay}")
    if initial_pose:
        cmd.append(f"initialPoseFile:={initial_pose}")
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST")
    with open(log_path, "w") as log:
        proc = subprocess.Popen(cmd, cwd=workspace, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    rclpy.init()
    node = rclpy.create_node("claude_rl_evaluate")
    gt = {}

    def on_odom(m):
        p, q, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist
        gt.update(t=m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, x=p.x, y=p.y, z=p.z, q=(q.w, q.x, q.y, q.z),
                  v=(v.linear.x, v.linear.y, v.linear.z), w=(v.angular.x, v.angular.y, v.angular.z))
    node.create_subscription(Odometry, "/mujoco/ground_truth/odom", on_odom, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    pub = node.create_publisher(WalkingVelocityCommand, "/humanoid/walking_velocity_command", 10)
    push_pub, pending, applied = None, list(pushes), []  # applied: (time, force [N])
    samples, verdict, t0, fall_time = [], "TIMEOUT", None, math.inf
    deadline = time.monotonic() + 240
    try:
        while time.monotonic() < deadline and proc.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.02)
            if t0 is None:
                if gt and "-> locomotion" in log_path.read_text(errors="ignore"):
                    t0 = gt["t"]
                continue
            e = gt["t"] - t0
            seg = [s for s in sequence if s[0] <= e][-1]
            if pending and push_pub is None:
                topic = next((n for n, _ in node.get_topic_names_and_types() if n.endswith("external_wrench_stamped")), None)
                if topic:
                    push_pub = node.create_publisher(WrenchStamped, topic, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
            if pending and push_pub is not None and e >= pending[0][0]:
                at, (dx, dy), force = pending.pop(0)
                wrench = WrenchStamped()
                wrench.header.frame_id = PUSH_BODY
                wrench.wrench.force.x, wrench.wrench.force.y = force * dx, force * dy
                push_pub.publish(wrench)
                applied.append((at, force))
            w, x, y, z = gt["q"]
            roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
            pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
            ground_z = float(ground(gt["x"], gt["y"])) if ground else 0.0
            if gt["z"] - ground_z < 0.6 or max(abs(roll), abs(pitch)) > math.pi / 6:
                verdict, fall_time = f"FELL at {e:.1f} s", e
                if applied:
                    verdict += f", {e - applied[-1][0]:.1f} s after a {applied[-1][1]:.0f} N push"
                break
            if seg[1] is None:
                verdict = "SURVIVED"
                break
            m = WalkingVelocityCommand()
            m.linear_velocity_x, m.linear_velocity_y = seg[1] / scale[0], seg[2] / scale[1]
            m.angular_velocity_z, m.desired_pelvis_height = seg[3] / scale[2], height
            pub.publish(m)
            # Body-frame GT twist -> heading frame (roll/pitch small): close to the body frame.
            samples.append((e, seg[1], seg[2], seg[3], gt["v"][0], gt["v"][1], gt["w"][2], gt["z"] - ground_z, roll, pitch, gt["x"], gt["y"]))
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(20)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
    a = np.array(samples) if samples else np.zeros((0, 12))
    moving = a[:, 0] > 1.0 if len(a) else np.zeros(0, bool)
    text = log_path.read_text(errors="ignore")
    adaptation = re.findall(r"gait adaptation \(policy\): (period x[\d.]+, swing height [+-][\d.]+ m, apex shift [+-][\d.]+)", text)
    return {
        "run": label, "result": verdict,
        "vel_rmse": round(float(np.sqrt(np.mean((a[moving, 4] - a[moving, 1]) ** 2 + (a[moving, 5] - a[moving, 2]) ** 2))), 4) if moving.any() else None,
        "yaw_rate_rmse": round(float(np.sqrt(np.mean((a[moving, 6] - a[moving, 3]) ** 2))), 4) if moving.any() else None,
        "height_rmse": round(float(np.sqrt(np.mean((a[moving, 7] - height) ** 2))), 4) if moving.any() else None,
        "roll_pitch_rms": round(float(np.sqrt(np.mean(a[moving, 8] ** 2 + a[moving, 9] ** 2))), 4) if moving.any() else None,
        "mpc_rate": re.findall(r"actual=([\d.]+) Hz", text)[-1:] or None,
        "adaptation_samples": adaptation[::4][:8],
        # A push counts as survived once the robot is still up 2 s after it.
        # Per push [time s, force N, recovered]: recovered unless it fell within 2 s.
        "pushes": [[round(at, 1), round(f), fall_time >= at + 2.0] for at, f in applied],
        "distance": round(float(np.hypot(a[-1, 10] - a[0, 10], a[-1, 11] - a[0, 11])), 2) if len(a) else None,
        "max_push_survived_N": round(max([f for at, f in applied if fall_time >= at + 2.0], default=0.0)) if pushes else None,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--policy", type=Path, required=True)
    parser.add_argument("--scenario", nargs="+", default=["flat"],
                        help="flat push slippery rough terrain:<type>:<difficulty>")
    parser.add_argument("--friction", type=float, nargs="+", default=[1.0, 0.5], help="flat scenario")
    parser.add_argument("--repeats", type=int, default=2)
    parser.add_argument("--decision-ticks", type=int, default=2,
                        help="MPC ticks per policy decision, as trained (EnvConfig.action_repeat; 1 for the "
                             "3-action policies flat_v1, robust_run2_1M, robust_run3)")
    parser.add_argument("--out", type=Path, default=Path("/tmp/claude_rl/eval"))
    parser.add_argument("--workspace", type=Path, default=Path.home() / "ocs2_ros2_ws")
    parser.add_argument("--domain", type=int, default=150)
    args = parser.parse_args()
    import yaml
    from ament_index_python.packages import get_package_share_directory
    share = Path(get_package_share_directory("legged_robot_mpc_controller"))
    ref = yaml.safe_load((share / "config/h2/ros2_controllers.yaml").read_text())[MPC]["ros__parameters"]["ocs2"]["reference"]
    scale = (ref["maxDisplacementVelocityX"], ref["maxDisplacementVelocityY"], ref["maxRotationVelocity"])
    args.out.mkdir(parents=True, exist_ok=True)
    overlay = args.out / "policy_overlay.yaml"
    overlay.write_text(yaml.safe_dump({MPC: {"ros__parameters": {"gaitAdaptation": {
        "source": "policy", "policyFile": str(args.policy.resolve()),
        "policyDecisionTicks": args.decision_ticks}}}}))
    flat = write_scene(args.out / "scene_flat.xml", "h2")
    # (name, scene, friction, sequence, pushes, initial pose, ground height, per-repeat pushes)
    cases = []
    for scenario in args.scenario:
        if scenario == "flat":
            cases += [(f"flat_f{f}", flat, f, SEQUENCE, (), None, None, None) for f in args.friction]
        elif scenario == "push":
            cases.append(("push", flat, 1.0, SEQUENCE, (), None, None, random_pushes))
        elif scenario == "slippery":
            cases.append(("slippery_f0.3", flat, 0.3, SEQUENCE, (), None, None, None))
        elif scenario.startswith("terrain:"):
            from terrain import TerrainMap
            _, kind, difficulty = scenario.split(":")
            tile = TerrainMap.single(kind, float(difficulty))
            name = f"{kind}_{float(difficulty):.2f}"
            cases.append((name, tile.write_scene(args.out, name=f"scene_{name}"), 1.0, TERRAIN_SEQUENCE, (), None,
                          tile.height, None))
        elif scenario == "rough":
            # Spawn above the highest box near the origin (10 cm) so no foot starts inside one.
            pose = yaml.safe_load((share / "config/h2/initial_pose.yaml").read_text())
            pose["h2_base"]["z"] = float(pose["h2_base"]["z"]) + 0.10
            pose_file = args.out / "initial_pose_rough.yaml"
            pose_file.write_text(yaml.safe_dump(pose))
            cases.append(("rough", write_scene(args.out / "scene_rough.xml", "h2", "rough"), 1.0, SEQUENCE, (), pose_file,
                          None, None))
    results = []
    for name, scene, friction, sequence, pushes, pose, ground, push_for_repeat in cases:
        mjb = compile_model(scene, args.out / f"{Path(scene).stem}_friction_{friction}.mjb", friction)
        for repeat in range(args.repeats):
            if push_for_repeat:
                pushes = push_for_repeat(repeat)
            for mode, ov in (("mpc_only", None), ("mpc_rl", overlay)):
                r = run(f"{mode}_{name}_{repeat}", mjb, ov, args.out, args.domain, str(args.workspace),
                        float(ref["defaultBaseHeight"]), scale, sequence, pushes, pose, ground)
                r.update(mode=mode, scenario=name, friction=friction, repeat=repeat)
                results.append(r)
                print(json.dumps(r), flush=True)
                (args.out / "results.json").write_text(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
