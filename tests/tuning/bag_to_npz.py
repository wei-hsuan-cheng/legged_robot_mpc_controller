#!/usr/bin/env python3
"""Convert a tuning-run rosbag (mcap) plus the probe's meta file into data.npz for plot_tracking.py.

usage: bag_to_npz.py <bag_dir> <meta.npz> <out data.npz>

Python subscribers in the busy simulation container receive only a fraction of the 1 kHz
streams (measured: /joint_states 46 Hz, ~/tracking 80 Hz of 500 Hz), which biases every
high-frequency metric. ros2 bag records them in full; this reads the bag back.
"""
import math
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG, META, OUT = sys.argv[1:4]
CTRL = "/humanoid_whole_body_inverse_dynamics_mpc_controller"

reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=BAG, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
types = {t.name: t.type for t in reader.get_all_topics_and_types()}
msg_type = {name: get_message(kind) for name, kind in types.items()}

js_t, js_q, js_v, js_e, js_names = [], [], [], [], None
gt, track, solver = [], [], []
track_names = solver_names = None
while reader.has_next():
    topic, raw, _ = reader.read_next()
    m = deserialize_message(raw, msg_type[topic])
    if topic == "/joint_states":
        js_names = js_names or list(m.name)
        js_t.append(m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec)
        js_q.append(m.position); js_v.append(m.velocity); js_e.append(m.effort)
    elif topic == "/mujoco/ground_truth/odom":
        p, q, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        gt.append((m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, p.x, p.y, p.z, v.x, v.y, v.z, roll, pitch, yaw))
    elif topic == CTRL + "/tracking":
        track.append(m.data)
    elif topic == CTRL + "/solver":
        solver.append(m.data)
    elif topic == CTRL + "/tracking_names":
        track_names = m.data.split(",")
    elif topic == CTRL + "/solver_names":
        solver_names = m.data.split(",")

meta = np.load(META, allow_pickle=False)
track_names = track_names or list(meta["track_names"])
solver_names = solver_names or list(meta["solver_names"])
gt = np.array(gt)
gt = gt[np.argsort(gt[:, 0])]
# Column 1 = probe wall time of each sample, from the probe's live (sim, wall) pairs: the probe's
# schedule (stand / ramp / hold) is in wall time.
wall = np.interp(gt[:, 0], meta["gt_map"][:, 0], meta["gt_map"][:, 1])
gt = np.column_stack([gt[:, 0], wall, gt[:, 1:]])
np.savez_compressed(OUT, track=np.array(track), track_names=np.array(track_names), solver=np.array(solver),
                    solver_names=np.array(solver_names), gt=gt, js_t=np.array(js_t), js_q=np.array(js_q),
                    js_v=np.array(js_v), js_e=np.array(js_e), js_names=np.array(js_names), phases=meta["phases"],
                    fell=meta["fell"], profile=meta["profile"] if "profile" in meta.files else np.array("vx"))
dur = lambda t: max(1e-9, t[-1] - t[0]) if len(t) > 1 else float("nan")
print(f"bag: joint_states {len(js_t) / dur(np.array(js_t)):.0f} Hz, ground truth {len(gt) / dur(gt[:, 0]):.0f} Hz, "
      f"tracking {len(track) / dur(np.array([r[0] for r in track])) if track else 0:.0f} Hz, solver rows {len(solver)}")
