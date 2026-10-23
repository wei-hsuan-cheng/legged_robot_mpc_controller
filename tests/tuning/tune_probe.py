#!/usr/bin/env python3
"""WB-ID tuning probe: stand -> forward walk -> stop.

usage: tune_probe.py <out_prefix> [speed] [vmax] [height]
env PROFILE: vx (default) | vy | yaw (speed in m/s or rad/s on that axis, vmax its command scale) |
             stops (vx with 4 s walk / 3 s stand cycles during the hold)

Publishes the walking command on a wall-clock schedule (4 s stand, 3 s ramp, 20 s hold, 3 s ramp
down, 5 s stop), watches ground truth for a fall, and writes <out_prefix>_meta.npz: the live
(sim time, wall time) pairs that place the schedule in sim time, the column names of ~/tracking
and ~/solver, and the fall time. The data itself is recorded by ros2 bag (run_case.sh), because a
Python subscriber in the busy container drops most of the 1 kHz streams. See
docs/wb_id_tuning_benchmark.md.
"""
import math
import sys
import threading
import time
import os
import subprocess

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from ocs2_msgs.msg import WalkingVelocityCommand
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

OUT = sys.argv[1]
VX = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
VMAX = float(sys.argv[3]) if len(sys.argv) > 3 else 0.75
HEIGHT = float(sys.argv[4]) if len(sys.argv) > 4 else 0.795
SETTLE, RAMP, HOLD, STOP = 4.0, 3.0, 20.0, 5.0
PROFILE = os.environ.get("PROFILE", "vx")
SHOT_TIMES = [SETTLE + RAMP + 8.0 + 0.2 * k for k in range(4)] if os.environ.get("SHOTS") else []
CTRL = "/humanoid_whole_body_inverse_dynamics_mpc_controller"


class Probe(Node):
    def __init__(self):
        super().__init__("wb_tuning_probe")
        be = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(WalkingVelocityCommand, "/humanoid/walking_velocity_command", be)
        self.create_subscription(Odometry, "/mujoco/ground_truth/odom", self.on_gt, be)
        self.create_subscription(String, CTRL + "/tracking_names", lambda m: setattr(self, "track_names", m.data.split(",")), latched)
        self.create_subscription(String, CTRL + "/solver_names", lambda m: setattr(self, "solver_names", m.data.split(",")), latched)
        self.track_names, self.solver_names = [], []
        self.lock = threading.Lock()
        self.gt_map = []
        self.wall_now = 0.0
        self.fell = None

    def on_gt(self, m):
        q, z = m.pose.pose.orientation, m.pose.pose.position.z
        with self.lock:
            self.gt_map.append((m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, self.wall_now))
        tilt = math.acos(max(-1.0, min(1.0, 1 - 2 * (q.x * q.x + q.y * q.y))))
        if self.fell is None and (z < 0.45 or tilt > 0.8):
            self.fell = self.wall_now

    def command(self, v):
        c = WalkingVelocityCommand()
        if PROFILE == "vy":
            c.linear_velocity_y = v / VMAX
        elif PROFILE == "yaw":
            c.angular_velocity_z = v / VMAX
        else:
            c.linear_velocity_x = v / VMAX
        c.desired_pelvis_height = HEIGHT
        self.pub.publish(c)


def main():
    rclpy.init()
    node = Probe()
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    t0 = time.time()
    total = SETTLE + 2 * RAMP + HOLD + STOP
    shots = list(SHOT_TIMES)
    while True:
        t = time.time() - t0
        node.wall_now = t
        if t > total or (node.fell is not None and t > node.fell + 1.5):
            break
        if t < SETTLE:
            vx = 0.0
        elif t < SETTLE + RAMP:
            vx = VX * (t - SETTLE) / RAMP
        elif t < SETTLE + RAMP + HOLD:
            vx = VX if PROFILE != "stops" or (t - SETTLE - RAMP) % 7.0 < 4.0 else 0.0
        elif t < SETTLE + 2 * RAMP + HOLD:
            vx = VX * (1 - (t - SETTLE - RAMP - HOLD) / RAMP)
        else:
            vx = 0.0
        node.command(vx)
        if shots and t >= shots[0]:
            k = len(SHOT_TIMES) - len(shots)
            subprocess.Popen("id=$(xwininfo -root -tree -display :1 | grep -i ' - rviz' | head -1 | awk '{print $1}'); "
                             "if [ -n \"$id\" ]; then xwd -silent -display :1 -id $id; else xwd -silent -display :1 -root; fi > "
                             + f"{OUT}_shot{k}.xwd", shell=True)
            shots.pop(0)
        time.sleep(0.02)
    ex.shutdown()
    with node.lock:
        gt_map = np.array(node.gt_map)
    gt_map = gt_map[np.argsort(gt_map[:, 0])]
    np.savez(OUT + "_meta.npz", gt_map=gt_map, track_names=np.array(node.track_names), solver_names=np.array(node.solver_names),
             phases=np.array([SETTLE, RAMP, HOLD, STOP, VX]), profile=np.array(PROFILE), fell=np.array(np.nan if node.fell is None else node.fell))
    print(f"PROBE {'FELL at %.1f s after the command started' % (node.fell - SETTLE) if node.fell is not None else 'survived'}")


if __name__ == "__main__":
    main()
